#!/usr/bin/env python3
"""Offline sensor branch: reads ONLY depth/calibration/odom/sample-query arrays."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from sensor import CONFIG,Tracker,candidates,odom_at,project,to_local,rot
from geometry import raster,warp


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def run(bundle,out):
    out.mkdir(parents=True,exist_ok=False)
    norm=bundle/'normalized';obs=np.load(norm/'observations.npz',allow_pickle=False)
    samples=np.load(norm/'samples.npz',allow_pickle=False)
    depth=np.load(norm/'depth.npy',mmap_mode='r',allow_pickle=False)
    cal=json.loads((norm/'calibration.json').read_text());transform=np.array(cal['optical_to_base_link']);k=cal['k']
    stamps=obs['depth__stamp_ns'];available=obs['depth__available_ns']
    assert np.all(np.diff(stamps)>0) and np.all(np.diff(available)>=0)
    os,oa,ov=[obs['odom__'+f] for f in ['stamp_ns','available_ns','values']]
    tracker=Tracker();frames=[]
    for i,(stamp,ready) in enumerate(zip(stamps,available)):
        pose=odom_at(stamp,os,oa,ov)
        detections=candidates(project(depth[i],k,transform)) if pose else []
        tracks=tracker.update(int(stamp),detections,pose['pose']) if pose else []
        frames.append(dict(depth_index=i,stamp_ns=int(stamp),available_ns=int(ready),
            odom_valid=pose is not None,odom_stamp_ns=pose['stamp_ns'] if pose else -1,
            odom_available_ns=pose['available_ns'] if pose else -1,tracks=tracks))
    with (out/'sensor_tracks.jsonl').open('w') as f:
        for row in frames:f.write(json.dumps(row,allow_nan=False)+'\n')
    n=len(samples['anchor_ns']);slots=16
    ids=np.full((n,slots),-1,np.int32);values=np.zeros((n,20,slots,4),np.float32)
    valid=np.zeros((n,20,slots),bool);hitcount=np.zeros((n,20,slots),np.int32)
    bevs=np.full((n,64,64),-1,np.int8);bev_valid=np.zeros(n,bool)
    odom_stamp=np.full(n,-1,np.int64);source_odom_stamp=np.full(n,-1,np.int64);truncated=0
    for i,anchor in enumerate(samples['anchor_ns']):
        pose=odom_at(anchor,os,oa,ov);indices=samples['depth_index'][i]
        if not pose:continue
        odom_stamp[i]=pose['stamp_ns'];latest=frames[indices[-1]]
        active=[t for t in latest['tracks'] if t['confirmed']]
        active.sort(key=lambda t:(float(np.linalg.norm(t['xy_base'])),t['track_id']))
        truncated+=max(0,len(active)-slots);selected=[t['track_id'] for t in active[:slots]]
        ids[i,:len(selected)]=selected
        for h,di in enumerate(indices):
            row=frames[di]
            assert row['stamp_ns']<=samples['history_query_ns'][i,h] and row['available_ns']<=samples['history_query_ns'][i,h]
            lookup={t['track_id']:t for t in row['tracks'] if t['confirmed']}
            for j,ident in enumerate(selected):
                if ident not in lookup or not samples['depth_valid'][i,h]:continue
                t=lookup[ident];values[i,h,j,:2]=to_local(t['xy_odom'],pose['pose'])
                values[i,h,j,2:]=np.array(t['velocity_odom'])@rot(pose['pose'][2])
                valid[i,h,j]=True;hitcount[i,h,j]=t['hits']
        source_pose=odom_at(latest['stamp_ns'],os,oa,ov)
        if source_pose and samples['depth_valid'][i,-1]:
            p=project(depth[indices[-1]],k,transform)
            warped=warp(p,source_pose['pose'],pose['pose'])
            origin=warp(transform[None,:3,3],source_pose['pose'],pose['pose'])[0]
            bevs[i]=raster(warped,origin);bev_valid[i]=True;source_odom_stamp[i]=source_pose['stamp_ns']
        if i%200==0:print('sensor samples',i,'/',n,flush=True)
    np.savez_compressed(out/'track_inputs.npz',anchor_ns=samples['anchor_ns'],history_query_ns=samples['history_query_ns'],
        depth_stamp_ns=samples['depth_stamp_ns'],depth_available_ns=samples['depth_available_ns'],track_id=ids,
        xy_vxy=values,observed_valid=valid,confirmed_hits=hitcount,
        semantic_person_valid=np.zeros((n,slots),bool),association_accepted=np.zeros((n,slots),bool),
        anchor_odom_stamp_ns=odom_stamp)
    np.savez_compressed(out/'current_bev_inputs.npz',anchor_ns=samples['anchor_ns'],depth_stamp_ns=samples['depth_stamp_ns'][:,-1],
        anchor_odom_stamp_ns=odom_stamp,source_odom_stamp_ns=source_odom_stamp,occupancy=bevs,valid=bev_valid)
    metadata=dict(schema='sensor_candidate_pilot/1',config=CONFIG,frames=len(frames),samples=n,
        candidate_track_ids=tracker.next_id-1,confirmed_rows=sum(t['confirmed'] for f in frames for t in f['tracks']),
        all_candidate_rows=sum(len(f['tracks']) for f in frames),truncated_slots=truncated,
        semantic_person_detector_validated=False,association_accepted=False,training_ready=False,
        frame='causal odom estimate of base_link at anchor; ZOH poses, no simulator GT alignment',
        velocity='Smoothed finite differences of surface-cluster centroids in odom; may include residual ego-motion error',
        current_bev='Depth reprojected using causal odom ZOH delta to estimated anchor; not perfect GT-aligned pose',
        imports='NumPy + SciPy; no ROS, simulator, pretrained weights or GT input',
        source_bundle_manifest_sha256=sha(bundle/'bundle_manifest.json'),
        source_sha256={p.name:sha(p) for p in [Path(__file__),Path(__file__).with_name('sensor.py'),Path(__file__).with_name('geometry.py')]})
    (out/'sensor_manifest.json').write_text(json.dumps(metadata,indent=2)+'\n');print(json.dumps(metadata,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.bundle.resolve(),a.output.resolve())
