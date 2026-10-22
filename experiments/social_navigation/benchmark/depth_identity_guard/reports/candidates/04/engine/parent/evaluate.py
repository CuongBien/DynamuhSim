#!/usr/bin/env python3
"""GT-side comparison only; frozen inference must finish before loading scenes."""
import argparse,json
from pathlib import Path
from collections import defaultdict
import numpy as np
from freeze import verify,sha
from snapshots.association import associate
from snapshots.sensor import to_local


def run(frozen,dataset,sensor,output):
    fixed=verify(frozen);source=fixed['datasets'][dataset];norm=Path(source['normalized']);replay=json.loads((sensor/'replay.json').read_text())
    assert replay['frozen_sha256']==sha(frozen) and replay['dataset']==dataset
    assert sha(norm/'human_states.jsonl')==source['scene_sha256']
    output.mkdir(parents=True,exist_ok=False)
    scene=[json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()];times=np.array([r['stamp_ns'] for r in scene]);ids=sorted(h['id'] for h in scene[0]['humans'])
    robot=np.array([[r['robot'][k] for k in ['x','y','yaw']] for r in scene]);robot[:,2]=np.unwrap(robot[:,2])
    people=np.array([[[h['x'],h['y']] for h in sorted(r['humans'],key=lambda h:h['id'])] for r in scene])
    def interpolate(t,array):return np.array([np.interp(t,times,array[:,j]) for j in range(array.shape[1])])
    summary={}
    for name in fixed['config']['comparison_variants']:
        frames=[json.loads(l) for l in (sensor/(name+'.jsonl')).read_text().splitlines()];n=len(frames)
        paired=0;confirmed=0;distances=[];per_human=defaultdict(set);last={};switches=0;reacquired=0;reacquired_paired=0;reacquired_wrong=0;track_last_person={};label_rows=[];matched_pairs=set()
        for row in frames:
            t=row['stamp_ns'];tracks=[v for v in row['tracks'] if v['confirmed']];confirmed+=len(tracks)
            gap_index=np.searchsorted(times,t);lo=max(0,gap_index-1);hi=min(gap_index,len(times)-1)
            valid=times[0]<=t<=times[-1] and (times[hi]==t or times[hi]-times[lo]<=100_000_000)
            if not valid:continue
            r=interpolate(t,robot);h=np.array([interpolate(t,people[:,j,:]) for j in range(len(ids))]);local=to_local(h,r)
            index,error=associate([v['xy_base'] for v in tracks],local)
            for track,j,distance in zip(tracks,index,error):
                k=track['track_id'];person=ids[j] if j>=0 else -1;re_id=track.get('reacquired_after_gap',False)
                label_rows.append(dict(depth_index=row['depth_index'],stamp_ns=t,track_id=k,gt_human_id=person,geometric_pair_valid=bool(j>=0),distance_m=float(distance) if j>=0 else None,reacquired_after_gap=re_id,association_accepted=False))
                if re_id:reacquired+=1
                if j<0:continue
                paired+=1;distances.append(float(distance));per_human[person].add(k);matched_pairs.add((row['depth_index'],person))
                if k in last and t-last[k][0]<=500_000_000 and last[k][1]!=person:switches+=1
                if re_id:
                    reacquired_paired+=1
                    if k in track_last_person and track_last_person[k]!=person:reacquired_wrong+=1
                last[k]=(t,person);track_last_person[k]=person
        with (output/(name+'_pairs_LABEL_ONLY.jsonl')).open('w') as f:
            for row in label_rows:f.write(json.dumps(row,allow_nan=False)+'\n')
        summary[name]=dict(frames=n,confirmed_candidate_rows=confirmed,paired_rows=paired,unpaired_rows=confirmed-paired,
            pairing_rate=paired/confirmed if confirmed else None,distance_median_m=float(np.median(distances)) if distances else None,distance_p95_m=float(np.percentile(distances,95)) if distances else None,
            distinct_ids_per_gt_human={str(h):len(per_human[h]) for h in ids},fragmentation_proxy=sum(max(0,len(per_human[h])-1) for h in ids),
            paired_identity_changes_within_500ms=switches,dormant_reacquisitions_confirmed=reacquired,dormant_reacquisitions_paired=reacquired_paired,dormant_reacquisitions_wrong_known_identity=reacquired_wrong,
            unique_paired_frame_person=len(matched_pairs),semantic_person_validated=False,association_accepted=False,training_ready=False)
    base=summary['baseline'];combined=summary['combined']
    # Acceptance is intentionally limited to this frozen geometric comparison.
    comparison=dict(pair_rows_retained_fraction=combined['paired_rows']/base['paired_rows'] if base['paired_rows'] else None,
        unpaired_row_delta=combined['unpaired_rows']-base['unpaired_rows'],fragmentation_proxy_delta=combined['fragmentation_proxy']-base['fragmentation_proxy'],
        paired_identity_change_delta=combined['paired_identity_changes_within_500ms']-base['paired_identity_changes_within_500ms'],
        geometric_improvement=(combined['paired_rows']>=.95*base['paired_rows'] and combined['unpaired_rows']<base['unpaired_rows'] and combined['fragmentation_proxy']<base['fragmentation_proxy'] and combined['paired_identity_changes_within_500ms']<=base['paired_identity_changes_within_500ms'] and combined['dormant_reacquisitions_wrong_known_identity']==0),
        semantic_or_training_acceptance=False)
    result=dict(dataset=dataset,frozen_sha256=sha(frozen),variants=summary,comparison=comparison,
        metric_scope='Geometric gates only, not semantic precision/recall/MOTA/IDF1; no visibility labels',validation_scope=fixed['config']['validation_limit'],training_ready=False)
    (output/'evaluation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--frozen',type=Path,required=True);p.add_argument('--dataset',choices=['development','validation'],required=True);p.add_argument('--sensor',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.frozen.resolve(),a.dataset,a.sensor.resolve(),a.output.resolve())
