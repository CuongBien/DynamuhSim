#!/usr/bin/env python3
"""Independent timing/projection/mask checks against the unchanged closed episode."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
import numpy as np
from sensor import Tracker,candidates,odom_at,project


def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for data in iter(lambda:f.read(1<<20),b''):h.update(data)
    return h.hexdigest()

def audit(bundle,sensor,labels,out):
    out.mkdir(parents=True,exist_ok=False)
    inv=json.loads((bundle/'bundle_manifest.json').read_text())['files']
    assert all((bundle/n).stat().st_size==v['bytes'] and sha(bundle/n)==v['sha256'] for n,v in inv.items())
    manifest=json.loads((sensor/'sensor_manifest.json').read_text())
    label_manifest=json.loads((labels/'label_manifest.json').read_text())
    for m in [manifest,label_manifest]:
        assert m['source_bundle_manifest_sha256']==sha(bundle/'bundle_manifest.json')
        assert all(sha(Path(__file__).with_name(n))==v for n,v in m['source_sha256'].items())
    assert label_manifest['sensor_manifest_sha256']==sha(sensor/'sensor_manifest.json')
    norm=bundle/'normalized';obs=np.load(norm/'observations.npz',allow_pickle=False);samples=np.load(norm/'samples.npz',allow_pickle=False)
    x=np.load(sensor/'track_inputs.npz',allow_pickle=False);current=np.load(sensor/'current_bev_inputs.npz',allow_pickle=False)
    y=np.load(labels/'future_bev_labels.npz',allow_pickle=False);hints=np.load(labels/'association_hints.npz',allow_pickle=False)
    assert not any(s in k for k in x.files for s in ['human','persona','future','world','gt_'])
    assert not x['association_accepted'].any() and not x['semantic_person_valid'].any() and not hints['association_accepted'].any() and not y['training_eligible'].any()
    assert np.all(x['xy_vxy'][~x['observed_valid']]==0)
    assert np.all(x['depth_available_ns']<=x['history_query_ns']) and np.all(x['depth_stamp_ns']<=x['history_query_ns'])
    assert np.all(x['anchor_odom_stamp_ns']<=x['anchor_ns'])
    assert np.all(current['source_odom_stamp_ns']<=current['depth_stamp_ns'])
    assert np.all(current['anchor_odom_stamp_ns']<=current['anchor_ns'])
    assert set(np.unique(y['occupancy'])).issubset({-1,0,1})
    np.testing.assert_array_equal(y['cell_observed_valid'],(y['occupancy']!=-1)&y['frame_valid'][:,:,None,None])
    assert np.all(y['depth_stamp_ns']>y['anchor_ns'][:,None])
    assert np.all(abs(y['depth_time_error_ns'])<=50_000_000)
    stamps=obs['depth__stamp_ns']
    # Brute nearest measurement check, separate from binary-search implementation.
    for i in range(len(y['anchor_ns'])):
        for j in range(3):assert int(np.argmin(abs(stamps-y['query_ns'][i,j])))==y['depth_index'][i,j]
    scene=[json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()]
    times=np.array([r['stamp_ns'] for r in scene]);poses=np.array([[r['robot'][k] for k in ['x','y','yaw']] for r in scene]);poses[:,2]=np.unwrap(poses[:,2])
    def pose(t):return np.array([np.interp(t,times,poses[:,j]) for j in range(3)])
    # Independent pinhole and SE2 arithmetic, no geometry.warp/raster imports.
    depth=np.load(norm/'depth.npy',mmap_mode='r',allow_pickle=False);cal=json.loads((norm/'calibration.json').read_text());k=cal['k'];tr=np.array(cal['optical_to_base_link']);endpoints=0
    for i in [0,len(x['anchor_ns'])//2,len(x['anchor_ns'])-1]:
        np.testing.assert_allclose(y['anchor_pose_world_gt'][i],pose(y['anchor_ns'][i]),atol=1e-12)
        for j in range(3):
            p=pose(y['depth_stamp_ns'][i,j]);a=pose(y['anchor_ns'][i]);np.testing.assert_allclose(y['future_pose_world_gt'][i,j],p,atol=1e-12)
            d=depth[y['depth_index'][i,j]];v,u=np.mgrid[0:d.shape[0]:4,0:d.shape[1]:4];z=d[::4,::4];ok=np.isfinite(z)&(z>=.28)&(z<=5);z=z[ok]
            optical=np.column_stack(((u[ok]-k[2])*z/k[0],(v[ok]-k[5])*z/k[4],z));q=optical@tr[:3,:3].T+tr[:3,3]
            world_x=np.cos(p[2])*q[:,0]-np.sin(p[2])*q[:,1]+p[0]
            world_y=np.sin(p[2])*q[:,0]+np.cos(p[2])*q[:,1]+p[1]
            bx=np.cos(a[2])*(world_x-a[0])+np.sin(a[2])*(world_y-a[1])
            by=-np.sin(a[2])*(world_x-a[0])+np.cos(a[2])*(world_y-a[1])
            good=(bx>=0)&(bx<6.4)&(by>=-3.2)&(by<3.2)&(q[:,2]>=.03)&(q[:,2]<=1.8)
            cells=np.floor(np.column_stack((bx[good],by[good]+3.2))/.1).astype(int)
            assert np.all(y['occupancy'][i,j,cells[:,0],cells[:,1]]==1);endpoints+=len(cells)
    # Replay a real prefix without loading any GT; future depth/odom cannot influence its IDs.
    frames=[json.loads(l) for l in (sensor/'sensor_tracks.jsonl').read_text().splitlines()]
    tracker=Tracker();checked=0
    for i in range(min(100,len(frames))):
        stamp=obs['depth__stamp_ns'][i]
        # Restrict all arrays to delivered data before each current measurement.
        available=(obs['odom__stamp_ns']<=stamp)&(obs['odom__available_ns']<=stamp)
        od=odom_at(stamp,obs['odom__stamp_ns'][available],obs['odom__available_ns'][available],obs['odom__values'][available])
        det=candidates(project(depth[i],k,tr)) if od else []
        actual=tracker.update(int(stamp),det,od['pose']) if od else []
        assert actual==frames[i]['tracks'];checked+=1
    # Label association is injective in every frame; unknown is not a negative person label.
    grouped={};paired_distances=[]
    person_ids=[h['id'] for h in sorted(scene[0]['humans'],key=lambda h:h['id'])]
    person_xy=np.array([[[h['x'],h['y']] for h in sorted(r['humans'],key=lambda h:h['id'])] for r in scene])
    for line in (labels/'association_diagnostics.jsonl').read_text().splitlines():
        row=json.loads(line)
        if row['geometric_pair_valid']:
            key=row['depth_index'];grouped.setdefault(key,[]).append(row['gt_human_id'])
            frame=frames[key];track=next(t for t in frame['tracks'] if t['track_id']==row['track_id'])
            stamp=frame['stamp_ns'];r=pose(stamp);j=person_ids.index(row['gt_human_id'])
            human=np.array([np.interp(stamp,times,person_xy[:,j,c]) for c in range(2)])
            dx,dy=human-r[:2]
            local=np.array([np.cos(r[2])*dx+np.sin(r[2])*dy,-np.sin(r[2])*dx+np.cos(r[2])*dy])
            distance=float(np.linalg.norm(local-track['xy_base']))
            assert distance<=.60 and abs(distance-row['distance_m'])<1e-10
            paired_distances.append(distance)
    assert all(len(v)==len(set(v)) for v in grouped.values())
    pairing=json.loads((labels/'association_summary.json').read_text())
    assert len(paired_distances)==pairing['geometrically_paired_rows']
    assert abs(float(np.median(paired_distances))-pairing['distance_median_m'])<1e-10
    result=dict(status='PASS',scope='Offline geometry/timing QA only; perception and training acceptance remain false',
        original_bundle_files_unchanged=len(inv),samples=len(x['anchor_ns']),future_bev_frames=int(y['frame_valid'].sum()),
        independent_occupied_endpoints_checked=endpoints,actual_sensor_prefix_frames_replayed=checked,
        future_frame_selection_brute_force_checked=int(y['depth_index'].size),
        association_distances_independently_checked=len(paired_distances),
        current_bev_valid=int(current['valid'].sum()),input_gt_leakage=False,unknown_cells_masked=True,
        association_semantics_accepted=False,training_ready=False)
    for root in [sensor,labels]:
        files={str(p.relative_to(root)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in root.rglob('*') if p.is_file()}
        (out/(root.name+'_inventory.json')).write_text(json.dumps(files,indent=2)+'\n')
    (out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--sensor',type=Path,required=True);p.add_argument('--labels',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();audit(a.bundle.resolve(),a.sensor.resolve(),a.labels.resolve(),a.output.resolve())
