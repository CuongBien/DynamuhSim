"""Audit existing phase-1 smoke JSONL, with explicit limitations."""
import argparse
import json
from pathlib import Path


def audit(path, expected=100):
    rows=[json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    issues=[]
    if len(rows)!=expected:issues.append(f'Expected {expected} records, got {len(rows)}')
    if any(r.get('status')!='PASS' for r in rows):issues.append('FAIL records present')
    if [r.get('episode_index') for r in rows]!=list(range(len(rows))):
        issues.append('Episode indices not contiguous')
    if [r.get('epoch') for r in rows]!=list(range(1,len(rows)+1)):
        issues.append('Epochs not contiguous/monotonic')
    if [r.get('seed') for r in rows]!=list(range(1000,1000+len(rows))):
        issues.append('Unexpected seed sequence')
    for r in rows:
        if r.get('status')=='PASS':
            a=r.get('initial_sim_time');b=r.get('final_sim_time')
            if not isinstance(a,(int,float)) or not isinstance(b,(int,float)) or abs((b-a)-.6)> .0021:
                issues.append(f"Episode {r.get('episode_index')}: total 3-step time not .6s")
    return {'file':str(path),'rows':len(rows),
            'pass':sum(r.get('status')=='PASS' for r in rows),
            'fail':sum(r.get('status')=='FAIL' for r in rows),
            'bootstrap_0p4_count':sum(abs(r.get('initial_sim_time',0)-.4)<1e-5 for r in rows),
            'provisional_acceptance':not issues,'issues':issues,
            'limitations':['no nonzero-action robot displacement','no actual human-state reset check',
                           'no external episode provenance','no physical collision truth','no process-leak proof']}


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('paths',nargs='+');a=ap.parse_args()
    for p in a.paths:print(json.dumps(audit(p),indent=2,ensure_ascii=False))
