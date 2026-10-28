#!/usr/bin/env python3
"""Read a closed MCAP episode, export causal windows and separate supervision."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from dataset_core import causal_indices, interpolate_labels, rotation, observed_bev, validate_clock, group_membership

HERE = Path(__file__).resolve().parent
NS = 1_000_000_000


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def pose(p):
    q = p.orientation
    return [p.position.x, p.position.y,
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def read_episode(episode):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(episode/'raw_bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    schemas = {t.name:t.type for t in reader.get_all_topics_and_types()}
    types = {t:get_message(v) for t,v in schemas.items()}
    rows = collections.defaultdict(list)
    counts = collections.Counter()
    frames = collections.defaultdict(set)
    static = collections.defaultdict(set)
    clock = []
    events = []
    while reader.has_next():
        topic, data, received = reader.read_next()
        msg = deserialize_message(data, types[topic])
        counts[topic] += 1
        if hasattr(msg, 'header'):
            frames[topic].add(msg.header.frame_id)
        if topic == '/clock':
            clock.append(ns(msg.clock))
        elif topic == '/tf_static':
            for t in msg.transforms:
                v, q = t.transform.translation, t.transform.rotation
                static[t.child_frame_id].add((t.header.frame_id, v.x,v.y,v.z,q.x,q.y,q.z,q.w))
        elif topic in ['/dataset/events', '/dataset/scene_poses', '/common/ground_truth']:
            value = json.loads(msg.data)
            if topic == '/dataset/events':
                events.append(value)
            else:
                value['_received_ns'] = received
                rows[topic].append(value)
        elif topic in ['/scan','/imu','/odom','/cmd_vel','/camera/depth/image_rect_raw',
                       '/camera/camera_info','/amcl_pose']:
            rows[topic].append((ns(msg.header.stamp), received, msg))
    return rows, dict(counts), schemas, frames, static, np.array(clock,dtype=np.int64), events


def build(episode, output):
    contract = json.loads((HERE/'contract.json').read_text())
    result = json.loads((episode/'result.json').read_text())
    assert result['status']=='MEASURED' and not result['faults'], 'Episode incomplete/adapter fault'
    assert result['recording']['returncode']==0 and result['gazebo_clean_exit']
    assert result['cleanup']['status']=='PASS'
    rows, counts, schemas, frames, static, clocks, events = read_episode(episode)
    required = ['/clock','/scan','/imu','/odom','/tf','/tf_static','/cmd_vel',
                '/camera/depth/image_rect_raw','/camera/camera_info','/common/ground_truth',
                '/dataset/scene_poses','/dataset/events']
    assert all(counts.get(k,0)>0 for k in required), 'Missing full-message topic'
    validate_clock(clocks)
    assert [e['seq'] for e in events]==[1,2,3,4], 'Missing/duplicate episode event'
    by_event = {e['event']:e for e in events}
    start = by_event['goal_accepted']['stamp_ns']
    end = by_event['action_end']['stamp_ns']
    observation_end = by_event['observation_end']['stamp_ns']
    assert observation_end-end>=3*NS
    assert all(len(v)==1 for v in static.values()), 'Conflicting static TF'
    assert frames['/cmd_vel'] == {'base_footprint'}
    assert list(static['base_link'])[0] == ('base_footprint',0.,0.,.04,0.,0.,0.,1.)
    assert list(static['camera_link'])[0] == ('base_link',.12,0.,.08,0.,0.,0.,1.)
    def transform(child):
        parent,*values = list(static[child])[0]
        a = np.eye(4)
        a[:3,:3] = rotation(values[3:])
        a[:3,3] = values[:3]
        return parent,a
    _, camera = transform('camera_link')
    parent,optical = transform('camera_optical_frame')
    assert parent=='camera_link'
    extrinsic = camera @ optical
    assert np.allclose(extrinsic[:3,:3], [[0,0,1],[-1,0,0],[0,-1,0]], atol=1e-9)
    calibration_rows = rows['/camera/camera_info']
    first_info = calibration_rows[0][2]
    assert first_info.header.frame_id=='camera_optical_frame'
    assert all(np.allclose(m.k,first_info.k) and m.header.frame_id==first_info.header.frame_id
               and m.width==first_info.width and m.height==first_info.height
               for _,_,m in calibration_rows), 'Calibration changed within episode'
    assert all(abs(x)<1e-12 for x in first_info.d), 'Distorted depth needs rectification'
    calibration = dict(schema='social_calibration/1', depth_topic='/camera/depth/image_rect_raw',
                       frame_id='camera_optical_frame', width=first_info.width,height=first_info.height,
                       k=list(first_info.k),p=list(first_info.p),d=list(first_info.d),
                       distortion_model=first_info.distortion_model,
                       optical_to_base_link=extrinsic.tolist(), depth_unit='m',
                       valid_depth_range_m=[.28,5.],
                       tf_static={k:list(v)[0] for k,v in static.items()},
                       caveat='Planar labels and base_link-local BEV; SDF/URDF base height is not a validated world 3D calibration')
    output.mkdir(parents=True,exist_ok=False)
    dump(output/'calibration.json',calibration)
    modalities = {}
    depth_frames=[]
    lidar_geometry = None
    for key,topic in [('scan','/scan'),('imu','/imu'),('odom','/odom'),('action','/cmd_vel'),
                      ('depth','/camera/depth/image_rect_raw'),('amcl','/amcl_pose')]:
        ordered=sorted(rows[topic],key=lambda r:(max(r[0],r[1]),r[0]))
        stamp=np.array([r[0] for r in ordered],dtype=np.int64)
        received=np.array([r[1] for r in ordered],dtype=np.int64)
        values=[];valid=[]
        for s,b,m in ordered:
            if key=='scan':
                x=np.array(m.ranges,dtype=np.float32)
                v=np.isfinite(x)&(x>=m.range_min)&(x<=m.range_max)
                values.append(np.where(v,x,0));valid.append(v)
                geom=dict(frame_id=m.header.frame_id,angle_min=m.angle_min,
                          angle_increment=m.angle_increment,angle_max=m.angle_max,
                          range_min=m.range_min,range_max=m.range_max,scan_time=m.scan_time,
                          time_increment=m.time_increment,rays=len(m.ranges))
                if lidar_geometry is None:lidar_geometry=geom
                assert geom==lidar_geometry, 'LiDAR geometry changed'
            elif key=='imu':
                values.append([m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w,
                               m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,
                               m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
            elif key=='odom':
                v=m.twist.twist
                values.append(pose(m.pose.pose)+[v.linear.x,v.linear.y,v.linear.z,
                                                v.angular.x,v.angular.y,v.angular.z])
            elif key=='action':
                values.append([m.twist.linear.x,m.twist.linear.y,m.twist.angular.z])
            elif key=='amcl':values.append(pose(m.pose.pose))
            else:
                assert m.encoding=='32FC1' and m.header.frame_id=='camera_optical_frame'
                assert m.width==first_info.width and m.height==first_info.height
                assert m.step>=m.width*4 and len(m.data)==m.step*m.height
                dtype=np.dtype('>f4' if m.is_bigendian else '<f4')
                data=np.frombuffer(bytes(m.data),dtype=dtype).reshape(m.height,m.step//4)[:,:m.width]
                depth_frames.append(data.astype(np.float32,copy=True))
                values.append([len(depth_frames)-1])
        modalities[key]=dict(stamp_ns=stamp,available_ns=np.maximum(stamp,received),
                             values=np.array(values,dtype=np.float32),received_ns=received)
        if valid:modalities[key]['valid']=np.array(valid)
    dump(output/'lidar_geometry.json',lidar_geometry)
    full_depth=np.stack(depth_frames)
    np.save(output/'depth.npy',full_depth,allow_pickle=False)
    flat={key+'__'+field:array for key,v in modalities.items() for field,array in v.items()}
    np.savez(output/'observations.npz',**flat)
    # Canonical scene poses are simulator labels ONLY, never observation features.
    scene = {r['stamp_ns']:r for r in rows['/dataset/scene_poses']}
    scene = [scene[t] for t in sorted(scene)]
    ids=sorted(h['id'] for h in scene[-1]['humans'])
    assert ids==list(range(1,7)) and all(sorted(h['id'] for h in s['humans'])==ids for s in scene)
    times=np.array([s['stamp_ns'] for s in scene],dtype=np.int64)
    human_xy=np.array([[[h['x'],h['y']] for h in sorted(s['humans'],key=lambda h:h['id'])] for s in scene])
    robot_xyyaw=np.array([[s['robot'][k] for k in ['x','y','yaw']] for s in scene])
    robot_xyyaw[:,2]=np.unwrap(robot_xyyaw[:,2])
    assert all(s['persona'] is None and not s['persona_valid'] for s in scene)
    with (output/'human_states.jsonl').open('w') as f:
        for s in scene:f.write(json.dumps(s,allow_nan=False)+'\n')
    groups=sorted(rows['/common/ground_truth'],key=lambda r:r['time'])
    group_times=np.array([round(g['time']*NS) for g in groups],dtype=np.int64)
    group_ids=[];group_modes=[]
    for state in groups:
        gids,gmodes=group_membership(state['groups'],ids)
        group_ids.append(gids)
        group_modes.append(gmodes)
    group_ids=np.array(group_ids,dtype=np.int32);group_modes=np.array(group_modes,dtype=np.int8)
    anchors=np.arange(((start+2*NS+99_999_999)//100_000_000)*100_000_000,
                      end-3*NS+1,100_000_000,dtype=np.int64)
    history=np.array(contract['history_offsets_ns'],dtype=np.int64)
    future=np.array(contract['future_offsets_ns'],dtype=np.int64)
    action_offsets=np.array(contract['future_action_offsets_ns'],dtype=np.int64)
    accepted=[];rejected=[];input_data=collections.defaultdict(list);label_data=collections.defaultdict(list)
    for anchor in anchors:
        queries=anchor+history
        ix={};ok={}
        for key in ['scan','imu','odom','action','depth']:
            m=modalities[key]
            ix[key],ok[key]=causal_indices(m['stamp_ns'],m['available_ns'],queries,
                                         contract['max_sensor_age_ns_by_modality'][key])
        if not all(ok[k].all() for k in ['imu','odom','action']) or not all(
                ok[k].mean()>=.9 and ok[k][-1] for k in ['scan','depth']):
            rejected.append(dict(anchor_ns=int(anchor),reason='history_missing_or_stale',
                                 valid={k:int(v.sum()) for k,v in ok.items()}));continue
        label_queries=anchor+future
        xy,valid=interpolate_labels(times,human_xy,label_queries,
                                    contract['max_scene_interpolation_gap_ns'])
        anchor_pose,anchor_valid=interpolate_labels(times,robot_xyyaw,np.array([anchor]),
                                                    contract['max_scene_interpolation_gap_ns'])
        current_xy,current_valid=interpolate_labels(times,human_xy,np.array([anchor]),
                                                    contract['max_scene_interpolation_gap_ns'])
        act=modalities['action']
        action_queries=anchor+action_offsets
        fi,fv=causal_indices(act['stamp_ns'],act['available_ns'],action_queries,150_000_000)
        if not(valid.all() and anchor_valid.all() and current_valid.all() and fv.all()):
            rejected.append(dict(anchor_ns=int(anchor),reason='future_label_or_action_gap'));continue
        robot=anchor_pose[0];c,s=math.cos(robot[2]),math.sin(robot[2]);rot=np.array([[c,-s],[s,c]])
        input_data['anchor_ns'].append(anchor)
        input_data['history_query_ns'].append(queries)
        for key in ix:
            m=modalities[key];index=ix[key]
            input_data[key+'_stamp_ns'].append(m['stamp_ns'][index])
            input_data[key+'_available_ns'].append(m['available_ns'][index])
            input_data[key+'_valid'].append(ok[key])
            if key=='depth':input_data['depth_index'].append(m['values'][index,0].astype(np.int32))
            elif key=='scan':
                input_data['scan_ranges_m'].append(m['values'][index])
                input_data['scan_ray_valid'].append(m['valid'][index]&ok[key][:,None])
            elif key=='odom':
                v=m['values'][index].copy();origin=v[-1,:3].copy()
                co,si=np.cos(origin[2]),np.sin(origin[2])
                v[:,:2]=(v[:,:2]-origin[:2])@np.array([[co,-si],[si,co]])
                v[:,2]=np.arctan2(np.sin(v[:,2]-origin[2]),np.cos(v[:,2]-origin[2]))
                input_data['odom_local_pose_and_twist'].append(v)
            else:input_data[key].append(m['values'][index])
        amcl=modalities['amcl'];ai,av=causal_indices(amcl['stamp_ns'],amcl['available_ns'],np.array([anchor]),NS)
        ap=amcl['values'][ai[0]];co,si=np.cos(ap[2]),np.sin(ap[2])
        goal=(np.array([-20.,0.])-ap[:2])@np.array([[co,-si],[si,co]])
        input_data['goal_xy_from_amcl'].append(goal if av[0] else np.zeros(2))
        input_data['goal_valid'].append(av[0])
        input_data['goal_stamp_ns'].append(amcl['stamp_ns'][ai[0]])
        input_data['goal_available_ns'].append(amcl['available_ns'][ai[0]])
        label_data['future_stamp_ns'].append(label_queries)
        label_data['human_xy_future_m'].append((xy-robot[:2])@rot)
        label_data['human_xy_at_anchor_m'].append((current_xy[0]-robot[:2])@rot)
        label_data['human_valid'].append(np.ones((len(future),len(ids)),dtype=bool))
        label_data['executed_action_query_ns'].append(action_queries)
        label_data['executed_action_future'].append(act['values'][fi])
        label_data['executed_action_stamp_ns'].append(act['stamp_ns'][fi])
        label_data['executed_action_available_ns'].append(act['available_ns'][fi])
        gi=np.searchsorted(group_times,label_queries,side='right')-1
        assert np.all(gi>=0)
        label_data['group_id_future'].append(group_ids[gi])
        label_data['group_mode_future'].append(group_modes[gi])
        label_data['persona_index'].append(np.full(len(ids),-1,dtype=np.int8))
        label_data['persona_valid'].append(np.zeros(len(ids),dtype=bool))
        accepted.append(int(anchor))
    assert accepted, 'No causal complete sample'
    inputs={k:np.array(v) for k,v in input_data.items()}
    labels={k:np.array(v) for k,v in label_data.items()}
    labels['human_ids']=np.array(ids,dtype=np.int32)
    for key in ['scan','imu','odom','action','depth']:
        assert np.all(inputs[key+'_stamp_ns']<=inputs['history_query_ns'])
        assert np.all(inputs[key+'_available_ns']<=inputs['history_query_ns'])
    assert np.all(labels['future_stamp_ns']>inputs['anchor_ns'][:,None])
    np.savez(output/'samples.npz',**inputs)
    np.savez(output/'labels.npz',**labels)
    depth_ids=sorted(set(inputs['depth_index'][:,-1].tolist()))
    bevs=np.stack([observed_bev(full_depth[i],first_info.k,extrinsic) for i in depth_ids])
    np.savez(output/'bev_observed.npz',depth_index=np.array(depth_ids,dtype=np.int32),occupancy=bevs)
    # Preview uses actual depth-derived BEV; no image generation.
    from PIL import Image, ImageDraw
    di=depth_ids[len(depth_ids)//2];b=bevs[depth_ids.index(di)]
    rgb=np.empty((*b.shape,3),dtype=np.uint8)
    rgb[b==-1]=[80,80,80];rgb[b==0]=[235,235,235];rgb[b==1]=[220,70,40]
    preview=Image.fromarray(rgb[::-1,::-1]).resize((512,512),resample=Image.Resampling.NEAREST)
    canvas=Image.new('RGB',(512,560),'white');canvas.paste(preview,(0,48))
    ImageDraw.Draw(canvas).text((8,8),'Actual depth BEV: gray unknown / white free / red occupied',fill='black')
    ImageDraw.Draw(canvas).text((8,25),'x forward upward; y left to left; 0.1m/cell; no RGB input',fill='black')
    canvas.save(output/'bev_preview.png')
    all_lengths={k:len(v['stamp_ns']) for k,v in modalities.items()}
    qa=dict(status='PASS',schema='social_episode/1',episode=episode.name,
            sample_count=len(accepted),candidate_count=len(anchors),rejected_count=len(rejected),
            actual_goal_reached=result['actual_goal_reached'],navigation_status=result['navigation_status'],
            goal_xy_error_world_m=result['goal_xy_error_world_m'],
            episode_goal_start_ns=start,episode_action_end_ns=end,observation_end_ns=observation_end,
            sample_anchor_range_ns=[accepted[0],accepted[-1]],
            human_ids=ids,persona_valid_count=0,depth_frames=len(full_depth),
            depth_shape=list(full_depth.shape),topic_counts=counts,modality_counts=all_lengths,
            clock_monotonic=True,static_tf_unique=True,optical_axes_checked=True,
            no_future_observations=True,full_future_labels=True,raw_readback=True,
            group_modes={'0':'none','1':'walking','2':'standing'},
            bev_unknown_cells=int((bevs==-1).sum()),bev_free_cells=int((bevs==0).sum()),
            bev_occupied_cells=int((bevs==1).sum()),
            m1='3s future human supervision + executed 3D robot actions; sensor tracker association pending',
            m2='observed current depth-only BEV; future BEV target definition pending',
            m3='masked; no validated H1-H5 supervision',
            scope='One pilot, not a train/validation dataset or persona acceptance',
            transport_timing='Conservative availability=max(header_stamp, bag latest-clock reception stamp); not wall-clock latency metrology',
            optional_mcap_warning='Action status definition absent in self-description; local Jazzy CDR readback succeeded; core topics retain definitions',
            rejected=rejected)
    dump(output/'qa.json',qa)
    dump(output/'contract.json',contract)
    dump(output/'events.json',events)
    dump(output/'arrays.json',dict(inputs={k:dict(shape=list(v.shape),dtype=str(v.dtype)) for k,v in inputs.items()},
                                   labels={k:dict(shape=list(v.shape),dtype=str(v.dtype)) for k,v in labels.items()}))
    dump(output/'sample_example.json',dict(anchor_ns=accepted[0],history_query_ns=inputs['history_query_ns'][0].tolist(),
              action_history=inputs['action'][0].tolist(),human_ids=ids,
              human_future_1st_step_m=labels['human_xy_future_m'][0,0].tolist(),
              persona=[None]*len(ids),persona_valid=[False]*len(ids),
              note='Human XY is LABEL ONLY; observations contain no simulator human states/backend identifier'))
    manifest=dict(schema='social_episode/1',episode=episode.name,split='pilot_only',seed=result['seed'],
                  simulator='Gazebo Harmonic',backend='SocNavGym',response_variant=result['response_variant'],
                  runtime_world_sha256=result['runtime_world_sha256'],runtime_nav_params_sha256=result['runtime_nav_params_sha256'],
                  source_head=result['head'],source_sha256=result['source_sha256'],
                  camera_runtime_corrections=result['camera_runtime_corrections'],
                  builder_sha256={p.name:sha(p) for p in [Path(__file__),HERE/'dataset_core.py',HERE/'contract.json']},
                  schemas=schemas,frames={k:sorted(v) for k,v in frames.items()},
                  files={str(p.relative_to(episode)):dict(bytes=p.stat().st_size,sha256=sha(p))
                         for root in [episode/'raw_bag',episode/'config_snapshot',output] for p in sorted(root.rglob('*')) if p.is_file()},
                  group_mode=result['scenario']+'_fixture_with_native_lifecycle',persona=None,persona_valid=False,
                  holdout='HuNavSim excluded from train/tuning',
                  training_ready=False,remaining=['validated persona mechanisms','sensor-track association',
                  'future BEV specification','multi-source split and diversity audit'])
    dump(output/'manifest.json',manifest)
    print(json.dumps({k:qa[k] for k in ['status','sample_count','candidate_count','rejected_count','depth_frames','goal_xy_error_world_m','m1','m2','m3']},indent=2))
    return qa


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--episode',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    build(args.episode.resolve(),(args.output or args.episode/'structured').resolve())
