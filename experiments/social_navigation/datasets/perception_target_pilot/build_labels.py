#!/usr/bin/env python3
"""LABEL-SIDE only: simulator pairing diagnostics and future observed depth BEV."""
import argparse,hashlib,json
from collections import defaultdict
from pathlib import Path
import numpy as np
from sensor import project,to_local
from geometry import interpolate,nearest_frame,raster,warp
from association import associate


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')

def run(bundle,sensor,out):
    out.mkdir(parents=True,exist_ok=False)
    contract=json.loads(Path(__file__).with_name('contract.json').read_text())
    norm=bundle/'normalized';scene=[json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()]
    times=np.array([r['stamp_ns'] for r in scene],np.int64)
    human_ids=np.array(sorted(h['id'] for h in scene[0]['humans']),np.int32)
    humans=np.array([[[h['x'],h['y']] for h in sorted(r['humans'],key=lambda h:h['id'])] for r in scene])
    robot=np.array([[r['robot'][k] for k in ['x','y','yaw']] for r in scene]);robot[:,2]=np.unwrap(robot[:,2])
    obs=np.load(norm/'observations.npz',allow_pickle=False);samples=np.load(norm/'samples.npz',allow_pickle=False)
    depth=np.load(norm/'depth.npy',mmap_mode='r',allow_pickle=False)
    cal=json.loads((norm/'calibration.json').read_text());transform=np.array(cal['optical_to_base_link']);k=cal['k']
    stamps=obs['depth__stamp_ns'];poses,pvalid=interpolate(times,robot,stamps);hp,hvalid=interpolate(times,humans,stamps)
    frames=[json.loads(l) for l in (sensor/'sensor_tracks.jsonl').read_text().splitlines()]
    assert len(frames)==len(stamps)
    pair_rows=[];mapping=[];distances=[];switches=0;last={};fragments=defaultdict(set)
    confirmed=0;paired=0;fov_count=0;fov_paired=0
    for i,row in enumerate(frames):
        tracks=[t for t in row['tracks'] if t['confirmed']];confirmed+=len(tracks)
        local=to_local(hp[i],poses[i])
        geometric_fov=(local[:,0]>=.28)&(local[:,0]<=5)&(abs(local[:,1])<=local[:,0]*(212/223.401487057))
        fov_count+=int(geometric_fov.sum()) if pvalid[i] and hvalid[i] else 0
        gt_index,error=associate([t['xy_base'] for t in tracks],local) if pvalid[i] and hvalid[i] else (np.full(len(tracks),-1),np.full(len(tracks),np.nan))
        current={}
        for t,g,e in zip(tracks,gt_index,error):
            human=int(human_ids[g]) if g>=0 else -1
            pair_rows.append(dict(depth_index=i,stamp_ns=int(stamps[i]),track_id=t['track_id'],gt_human_id=human,
                geometric_pair_valid=bool(g>=0),distance_m=float(e) if g>=0 else None,semantic_person_valid=False,association_accepted=False))
            if g<0:continue
            paired+=1;distances.append(float(e));current[t['track_id']]=human
            fov_paired+=int(geometric_fov[g]);fragments[human].add(t['track_id'])
            previous=last.get(t['track_id'])
            if previous and stamps[i]-previous[0]<=500_000_000 and previous[1]!=human:switches+=1
            last[t['track_id']]=(stamps[i],human)
        mapping.append(current)
    with (out/'association_diagnostics.jsonl').open('w') as f:
        for row in pair_rows:f.write(json.dumps(row,allow_nan=False)+'\n')
    inputs=np.load(sensor/'track_inputs.npz',allow_pickle=False);n,slots=inputs['track_id'].shape
    gt_id=np.full((n,slots),-1,np.int32);consistent=np.zeros((n,slots),bool);paired_history=np.zeros((n,20,slots),bool)
    for i,indices in enumerate(samples['depth_index']):
        for j,ident in enumerate(inputs['track_id'][i]):
            if ident<0:continue
            anchor_id=mapping[indices[-1]].get(int(ident),-1);gt_id[i,j]=anchor_id
            found=[]
            for h,di in enumerate(indices):
                if not inputs['observed_valid'][i,h,j]:continue
                found.append(mapping[di].get(int(ident),-1));paired_history[i,h,j]=found[-1]==anchor_id and anchor_id>=0
            consistent[i,j]=anchor_id>=0 and len(found)>=10 and all(v==anchor_id for v in found)
    np.savez_compressed(out/'association_hints.npz',anchor_ns=samples['anchor_ns'],track_id=inputs['track_id'],
        gt_human_id=gt_id,geometric_history_consistent=consistent,history_pair_valid=paired_history,
        association_accepted=np.zeros((n,slots),bool))
    summary=dict(scope='Geometric proximity diagnostics on this episode, not precision/recall of a person detector',
        confirmed_candidate_rows=confirmed,geometrically_paired_rows=paired,
        pairing_rate=paired/confirmed if confirmed else None,
        distance_median_m=float(np.median(distances)) if distances else None,
        distance_p95_m=float(np.percentile(distances,95)) if distances else None,
        paired_identity_changes_within_500ms=switches,distinct_candidate_track_ids_per_gt_human={str(k):len(v) for k,v in fragments.items()},
        geometric_camera_wedge_gt_rows=fov_count,paired_gt_rows_inside_wedge=fov_paired,
        wedge_note='XY angular/range eligibility only, not occlusion-aware visibility or a recall denominator',
        window_slots_with_geometric_history_consistency=int(consistent.sum()),
        semantic_person_detector_validated=False,association_accepted=False,training_ready=False)
    dump(out/'association_summary.json',summary)
    offsets=np.array(contract['bev_future_offsets_ns'],np.int64);queries=samples['anchor_ns'][:,None]+offsets
    selected,dvalid=nearest_frame(stamps,queries,contract['bev_future_depth_tolerance_ns'])
    target_stamp=stamps[selected];anchor_pose,anchor_valid=interpolate(times,robot,samples['anchor_ns'])
    future_pose,future_valid=interpolate(times,robot,target_stamp)
    valid=dvalid&future_valid&anchor_valid[:,None]&(target_stamp>samples['anchor_ns'][:,None])
    grids=np.full((n,len(offsets),64,64),-1,np.int8)
    points={int(i):project(depth[i],k,transform) for i in np.unique(selected[valid])}
    for i in range(n):
        for j in range(len(offsets)):
            if not valid[i,j]:continue
            p=warp(points[int(selected[i,j])],future_pose[i,j],anchor_pose[i])
            origin=warp(transform[None,:3,3],future_pose[i,j],anchor_pose[i])[0]
            grids[i,j]=raster(p,origin)
        if i%100==0:print('future BEV labels',i,'/',n,flush=True)
    observed=(grids!=-1)&valid[:,:,None,None]
    np.savez_compressed(out/'future_bev_labels.npz',anchor_ns=samples['anchor_ns'],offset_ns=offsets,
        query_ns=queries,depth_index=selected.astype(np.int32),depth_stamp_ns=target_stamp,depth_time_error_ns=target_stamp-queries,
        anchor_pose_world_gt=anchor_pose,future_pose_world_gt=future_pose,
        occupancy=grids,frame_valid=valid,cell_observed_valid=observed,
        training_eligible=np.zeros(n,bool))
    target_summary=dict(status='BUILT',samples=n,horizons_s=(offsets/1e9).tolist(),valid_frames=int(valid.sum()),
        expected_frames=int(valid.size),unknown_cells=int((grids==-1).sum()),free_cells=int((grids==0).sum()),occupied_cells=int((grids==1).sum()),
        observed_fraction_per_horizon=observed.mean(axis=(0,2,3)).tolist(),
        max_abs_depth_time_error_ns=int(abs(target_stamp-queries).max()),
        target='Future actually observed depth occupancy, anchor-frozen true robot SE2 axes; GT ego poses in label branch only',
        full_world_occupancy=False,unknown_is_loss_masked=True,training_ready=False)
    dump(out/'target_summary.json',target_summary)
    dump(out/'label_manifest.json',dict(schema='perception_labels_pilot/1',source_bundle_manifest_sha256=sha(bundle/'bundle_manifest.json'),
        sensor_manifest_sha256=sha(sensor/'sensor_manifest.json'),scene_sha256=sha(norm/'human_states.jsonl'),
        contract=contract,source_sha256={p.name:sha(p) for p in [Path(__file__),Path(__file__).with_name('sensor.py'),Path(__file__).with_name('geometry.py'),Path(__file__).with_name('association.py'),Path(__file__).with_name('contract.json')]},
        association_accepted=False,training_ready=False))
    print(json.dumps(dict(association=summary,bev=target_summary),indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--sensor',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.bundle.resolve(),a.sensor.resolve(),a.output.resolve())
