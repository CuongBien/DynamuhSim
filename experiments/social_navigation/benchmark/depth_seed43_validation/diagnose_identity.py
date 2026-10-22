#!/usr/bin/env python3
"""GT-side regression evidence only, never sensor features or inference tuning."""
import argparse,json
from pathlib import Path
HERE=Path(__file__).resolve().parent


def diagnose(output):
    result={}
    for name in ['baseline','combined']:
        pairs=[json.loads(l) for l in (HERE/'generated/evaluation01'/(name+'_pairs_LABEL_ONLY.jsonl')).read_text().splitlines()]
        frames=[json.loads(l) for l in (HERE/'generated/sensor01'/(name+'.jsonl')).read_text().splitlines()];last={};events=[]
        for p in pairs:
            if not p['geometric_pair_valid']:continue
            old=last.get(p['track_id'])
            if old and p['stamp_ns']-old['stamp_ns']<=500_000_000 and old['gt_human_id']!=p['gt_human_id']:
                observations=[]
                for q in pairs:
                    if q['track_id']!=p['track_id'] or not old['stamp_ns']-500_000_000<=q['stamp_ns']<=p['stamp_ns']+500_000_000:continue
                    t=next(t for t in frames[q['depth_index']]['tracks'] if t['track_id']==q['track_id'])
                    observations.append(dict(**q,xy_base=t['xy_base'],extent_m=t['extent_m']))
                events.append(dict(track_id=p['track_id'],previous=old,current=p,window=observations,
                    caveat='Change in geometric GT pairing, not manually validated identity-switch truth'))
            last[p['track_id']]=p
        result[name]=events
    value=dict(seed=43,role='Label-side diagnostic and future regression candidate; no change to frozen inference',events=result,
        next_validation_rule='If these cases are used to tune a future tracker, seed43 becomes development/regression data for that tracker; use a new untouched seed for validation')
    output.parent.mkdir(parents=True,exist_ok=True);assert not output.exists();output.write_text(json.dumps(value,indent=2)+'\n')
    for name,events in result.items():print(name,[(p['previous']['stamp_ns'],p['current']['stamp_ns'],p['previous']['gt_human_id'],p['current']['gt_human_id']) for p in events])
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();diagnose(a.output.resolve())
