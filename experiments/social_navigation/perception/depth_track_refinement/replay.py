#!/usr/bin/env python3
"""Sensor-only, one causal pass, four frozen ablations; no simulator labels read."""
import argparse,json
from pathlib import Path
import numpy as np
from freeze import verify,sha
from snapshots.sensor import Tracker,project,odom_at,candidates
from refinement import enrich,DormantTracker


def run(frozen,dataset,output):
    fixed=verify(frozen);source=fixed['datasets'][dataset];norm=Path(source['normalized'])
    assert all(sha(norm/n)==v for n,v in source['input_sha256'].items())
    output.mkdir(parents=True,exist_ok=False)
    obs=np.load(norm/'observations.npz',allow_pickle=False);depth=np.load(norm/'depth.npy',mmap_mode='r',allow_pickle=False)
    calibration=json.loads((norm/'calibration.json').read_text());transform=np.array(calibration['optical_to_base_link'])
    stamps=obs['depth__stamp_ns'];available=obs['depth__available_ns'];assert np.all(np.diff(stamps)>0) and np.all(np.diff(available)>=0)
    trackers=dict(baseline=Tracker(),shape_only=Tracker(),tracking_only=DormantTracker(),combined=DormantTracker())
    handles={k:(output/(k+'.jsonl')).open('w') for k in trackers};reject_log=(output/'shape_rejections.jsonl').open('w')
    counts={k:dict(candidate_rows=0,confirmed_rows=0,unique_ids=set(),dormant_reacquisitions=0) for k in trackers};rejected_rows=0
    try:
        for i,stamp in enumerate(stamps):
            pose=odom_at(stamp,obs['odom__stamp_ns'],obs['odom__available_ns'],obs['odom__values'])
            original=[];kept=[];rejected=[];all_enriched=[]
            if pose:
                points=project(depth[i],calibration['k'],transform)
                original=candidates(points);kept,rejected=enrich(points);all_enriched=sorted(kept+rejected,key=lambda d:tuple(d['xy']))
            options=dict(baseline=original,shape_only=kept,tracking_only=all_enriched,combined=kept)
            for name,t in trackers.items():
                tracks=t.update(int(stamp),options[name],pose['pose']) if pose else []
                row=dict(depth_index=i,stamp_ns=int(stamp),available_ns=int(available[i]),odom_valid=pose is not None,
                    odom_stamp_ns=pose['stamp_ns'] if pose else -1,odom_available_ns=pose['available_ns'] if pose else -1,tracks=tracks)
                handles[name].write(json.dumps(row,allow_nan=False)+'\n')
                c=counts[name];c['candidate_rows']+=len(tracks);c['confirmed_rows']+=sum(t['confirmed'] for t in tracks);c['unique_ids'].update(t['track_id'] for t in tracks)
                c['dormant_reacquisitions']+=sum(t.get('reacquired_after_gap',False) for t in tracks)
            for d in rejected:
                reject_log.write(json.dumps(dict(depth_index=i,stamp_ns=int(stamp),xy_base=d['xy'].tolist(),shape=d['shape'].tolist(),profile_shift_m=d['profile_shift_m'],profile_bins=d['profile_bins'],reason='tall thin or height-invariant surface prior'),allow_nan=False)+'\n');rejected_rows+=1
            if i%250==0:print(dataset,'sensor frames',i,'/',len(stamps),flush=True)
    finally:
        for f in handles.values():f.close()
        reject_log.close()
    for c in counts.values():c['unique_candidate_ids']=len(c.pop('unique_ids'))
    result=dict(status='REPLAYED',dataset=dataset,frames=len(stamps),frozen_sha256=sha(frozen),source_manifest_sha256=source['manifest_sha256'],
        variants=counts,shape_rejections=rejected_rows,semantic_person_validated=False,association_accepted=False,training_ready=False,
        inputs='Only actual depth/calibration/causal odometry. No GT, future scene, persona or backend feature')
    (output/'replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--frozen',type=Path,required=True);p.add_argument('--dataset',choices=['development','validation'],required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.frozen.resolve(),a.dataset,a.output.resolve())
