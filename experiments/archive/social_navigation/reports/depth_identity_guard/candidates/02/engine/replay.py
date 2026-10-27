#!/usr/bin/env python3
"""Causal sensor replay with all ambiguous proposals retained, no simulator labels."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from freeze import verify,sha
from guard_tracker import IdentityGuard
from refinement import enrich,DormantTracker
from snapshots.sensor import Tracker,candidates,project,odom_at


def run(frozen,dataset,output):
    obj=verify(frozen);source=obj['datasets'][dataset];norm=Path(source['normalized'])
    assert all(sha(norm/n)==h for n,h in source['input_sha256'].items());output.mkdir(parents=True,exist_ok=False)
    o=np.load(norm/'observations.npz',allow_pickle=False);d=np.load(norm/'depth.npy',mmap_mode='r');c=json.loads((norm/'calibration.json').read_text());tr=np.array(c['optical_to_base_link'])
    ts=o['depth__stamp_ns'];available=o['depth__available_ns'];assert np.all(np.diff(ts)>0) and np.all(np.diff(available)>=0)
    trackers=dict(baseline=Tracker(),parent=DormantTracker(),shape_only=Tracker(),combined=IdentityGuard());handles={k:(output/(k+'.jsonl')).open('w') for k in trackers}
    counts={k:dict(proposals=0,confirmed_rows=0,unassigned_rows=0,reacquisitions=0,ids=set()) for k in trackers}
    try:
        for i,t in enumerate(ts):
            od=odom_at(t,o['odom__stamp_ns'],o['odom__available_ns'],o['odom__values']);orig=[];kept=[]
            if od:
                points=project(d[i],c['k'],tr);orig=candidates(points);kept,_=enrich(points)
            choices=dict(baseline=orig,parent=kept,shape_only=kept,combined=kept)
            for name,tracker in trackers.items():
                tracks=tracker.update(int(t),choices[name],od['pose']) if od else []
                assert len(tracks)==len(choices[name]),'Do not hide ambiguous proposals'
                if name=='combined':assert len({v['proposal_index'] for v in tracks})==len(tracks)
                row=dict(depth_index=i,stamp_ns=int(t),available_ns=int(available[i]),odom_valid=od is not None,odom_stamp_ns=od['stamp_ns'] if od else -1,odom_available_ns=od['available_ns'] if od else -1,tracks=tracks)
                handles[name].write(json.dumps(row,allow_nan=False)+'\n');q=counts[name];q['proposals']+=len(tracks);q['confirmed_rows']+=sum(v['confirmed'] for v in tracks);q['unassigned_rows']+=sum(v['track_id']<0 for v in tracks);q['reacquisitions']+=sum(v.get('reacquired_after_gap',False) for v in tracks);q['ids'].update(v['track_id'] for v in tracks if v['track_id']>=0)
            if i%300==0:print(dataset,i,'/',len(ts),flush=True)
    finally:
        for f in handles.values():f.close()
    for q in counts.values():q['unique_assigned_ids']=len(q.pop('ids'))
    result=dict(dataset=dataset,frames=len(ts),frozen_sha256=sha(frozen),role=source['role'],variants=counts,all_sensor_proposals_retained=True,association_accepted=False,training_ready=False)
    (output/'replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--frozen',type=Path,required=True);p.add_argument('--dataset',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.frozen.resolve(),a.dataset,a.output.resolve())
