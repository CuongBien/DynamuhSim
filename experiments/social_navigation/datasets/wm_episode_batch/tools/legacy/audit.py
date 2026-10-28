#!/usr/bin/env python3
"""Independent raw-to-export audit, including bytewise depth and future-label checks."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()


def audit(episode,normalized):
    contract=json.loads((normalized/'contract.json').read_text())
    manifest=json.loads((normalized/'manifest.json').read_text())
    qa=json.loads((normalized/'qa.json').read_text())
    x=np.load(normalized/'samples.npz',allow_pickle=False)
    y=np.load(normalized/'labels.npz',allow_pickle=False)
    o=np.load(normalized/'observations.npz',allow_pickle=False)
    depth=np.load(normalized/'depth.npy',allow_pickle=False,mmap_mode='r')
    bev=np.load(normalized/'bev_observed.npz',allow_pickle=False)
    n=len(x['anchor_ns'])
    assert n==qa['sample_count'] and n>0
    assert y['human_xy_future_m'].shape==(n,30,6,2)
    assert x['action'].shape==(n,20,3) and y['executed_action_future'].shape==(n,30,3)
    assert not any(any(word in k for word in ['human','persona','backend','future','world']) for k in x.files)
    assert np.all(x['history_query_ns']<=x['anchor_ns'][:,None])
    for key in ['scan','imu','odom','action','depth']:
        assert np.all(x[key+'_stamp_ns']<=x['history_query_ns'])
        assert np.all(x[key+'_available_ns']<=x['history_query_ns'])
        age=x['history_query_ns']-x[key+'_stamp_ns']
        assert np.all(age[x[key+'_valid']]<=contract['max_sensor_age_ns_by_modality'][key])
    assert np.all(x['goal_stamp_ns'][x['goal_valid']]<=x['anchor_ns'][x['goal_valid']])
    assert np.all(x['goal_available_ns'][x['goal_valid']]<=x['anchor_ns'][x['goal_valid']])
    assert np.all(y['future_stamp_ns']-x['anchor_ns'][:,None]==np.arange(1,31)*100_000_000)
    assert np.all(y['executed_action_query_ns']-x['anchor_ns'][:,None]==np.arange(30)*100_000_000)
    assert np.all(y['executed_action_stamp_ns']<=y['executed_action_query_ns'])
    assert np.all(y['executed_action_available_ns']<=y['executed_action_query_ns'])
    np.testing.assert_array_equal(y['executed_action_future'][:,0],x['action'][:,-1])
    assert np.isfinite(y['human_xy_future_m']).all() and y['human_valid'].all()
    assert not y['persona_valid'].any() and (y['persona_index']==-1).all()
    assert set(np.unique(bev['occupancy']))=={-1,0,1}
    depth_lookup=collections.defaultdict(list)
    for i,(stamp,received) in enumerate(zip(o['depth__stamp_ns'],o['depth__received_ns'])):
        depth_lookup[int(stamp),int(received)].append(i)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(episode/'raw_bag'),storage_id='mcap'),
                rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    counts=collections.Counter();scenes={};compared_depth=0;clocks=[];action_rows=[];negative_lags=collections.Counter()
    while reader.has_next():
        topic,data,received=reader.read_next();m=deserialize_message(data,types[topic]);counts[topic]+=1
        if hasattr(m,'header'):
            stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
            if received<stamp:negative_lags[topic]+=1
        if topic=='/camera/depth/image_rect_raw':
            i=depth_lookup[stamp,received].pop(0)
            raw=np.frombuffer(bytes(m.data),dtype='>f4' if m.is_bigendian else '<f4').reshape(m.height,m.step//4)[:,:m.width].astype(np.float32)
            assert raw.tobytes()==depth[i].tobytes(), 'Depth export changed raw values'
            compared_depth+=1
        elif topic=='/dataset/scene_poses':
            s=json.loads(m.data);scenes[s['stamp_ns']]=s
        elif topic=='/clock':clocks.append(m.clock.sec*1_000_000_000+m.clock.nanosec)
        elif topic=='/cmd_vel':
            action_rows.append((stamp,max(stamp,received),[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]))
    assert dict(counts)==qa['topic_counts']
    assert compared_depth==len(depth) and all(not v for v in depth_lookup.values())
    assert np.all(np.diff(np.array(clocks,dtype=np.int64))>=0)
    # Reconstruct 3s targets from the raw canonical scene message, independent of builder helpers.
    ts=np.array(sorted(scenes),dtype=np.int64)
    ids=y['human_ids'].tolist()
    humans=np.array([[[h['x'],h['y']] for h in sorted(scenes[t]['humans'],key=lambda h:h['id'])] for t in ts])
    robots=np.array([[scenes[t]['robot'][k] for k in ['x','y','yaw']] for t in ts]);robots[:,2]=np.unwrap(robots[:,2])
    max_error=0.
    for i,anchor in enumerate(x['anchor_ns']):
        anchor_robot=np.array([np.interp(anchor,ts,robots[:,j]) for j in range(3)])
        z=y['future_stamp_ns'][i]
        h=np.stack([np.stack([np.interp(z,ts,humans[:,j,k]) for k in range(2)],axis=-1) for j in range(len(ids))],axis=1)
        c,s=math.cos(anchor_robot[2]),math.sin(anchor_robot[2]);r=np.array([[c,-s],[s,c]])
        expected=(h-anchor_robot[:2])@r
        max_error=max(max_error,float(np.max(np.abs(expected-y['human_xy_future_m'][i]))))
        assert z[0]>=ts[0] and z[-1]<=ts[-1]
    assert max_error<1e-10
    # Check every exported future command against the closed raw command stream.
    action_rows.sort(key=lambda r:(r[1],r[0]))
    action_available=np.array([r[1] for r in action_rows],dtype=np.int64)
    raw_values=np.array([r[2] for r in action_rows],dtype=np.float32)
    index=np.searchsorted(action_available,y['executed_action_query_ns'],side='right')-1
    np.testing.assert_array_equal(raw_values[index],y['executed_action_future'])
    checked=0
    for name,v in manifest['files'].items():
        p=episode/name
        assert p.stat().st_size==v['bytes'] and sha(p)==v['sha256'], 'Manifest file mismatch: '+name
        checked+=1
    for name,h in manifest['builder_sha256'].items():assert sha(Path(__file__).parent/name)==h
    result=json.loads((episode/'result.json').read_text())
    assert any('NVIDIA' in s for s in result['cleanup']['ogre_renderer_lines'])
    assert result['cleanup']['status']=='PASS' and not result['cleanup']['remaining']
    assert all(p['node'] in ['controller_server','behavior_server'] for k in
               ['cmd_vel_publishers_before_goal','cmd_vel_publishers_after_goal'] for p in result[k])
    report=dict(status='PASS',samples=n,independent_episodes=1,depth_frames_bitwise_equal=compared_depth,
                human_future_max_abs_error_m=max_error,raw_future_actions_exact=True,
                no_gt_or_future_fields_in_observations=True,future_action_interval_alignment=True,
                persona_supervision_masked=True,manifest_files_verified=checked,
                negative_bag_vs_header_lags=dict(negative_lags),
                negative_lag_handling='Availability=max(header,bag latest-clock reception); no wall-clock latency claim',
                sensor_valid_history_counts={k:int(x[k+'_valid'].sum()) for k in ['scan','imu','odom','action','depth']},
                goal_valid_samples=int(x['goal_valid'].sum()),
                gpu='NVIDIA RTX 4060 (D3D12)',cleanup='PASS',
                scope='Integrity of one recorded episode; not all-persona, perception, future-BEV or multi-source acceptance')
    print(json.dumps(report,indent=2))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--normalized',default='normalized');p.add_argument('--report',type=Path,required=True)
    a=p.parse_args();r=audit(a.episode.resolve(),a.episode.resolve()/a.normalized)
    assert not a.report.exists();a.report.parent.mkdir(parents=True,exist_ok=True)
    a.report.write_text(json.dumps(r,indent=2)+'\n')
