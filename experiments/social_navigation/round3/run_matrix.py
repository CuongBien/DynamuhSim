#!/usr/bin/env python3
"""Thirty unique exploratory runs; reuse a common baseline across four factors."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
REPORT=HERE/'reports'
parser=argparse.ArgumentParser()
parser.add_argument('--resume',action='store_true')
args=parser.parse_args()
conditions=[('baseline',[7,8,0,0],10,'passing_crossing'),
    ('sparse',[2,3,0,0],10,'passing_crossing'),('dense',[15,15,0,0],10,'passing_crossing'),
    ('narrow',[1,2,0,0],1.5,'passing_crossing'),('medium_width',[2,2,0,0],3,'passing_crossing'),
    ('orca_only',[15,0,0,0],10,'passing_crossing'),('heterogeneous',[5,5,2,3],10,'passing_crossing'),
    ('passing',[7,8,0,0],10,'passing'),('crossing',[7,8,0,0],10,'crossing')]
failures=[]
executed=[]
skipped=[]
def run(condition,seed,blind=False):
    name,counts,width,scenario=condition
    key=f'{name}_s{seed}_{"blind" if blind else "visible"}'
    path=REPORT/f'{key}.json'
    data=json.loads(path.read_text()) if path.exists() else None
    valid_initial=data is not None and data['initial_state']['min_clearance_m']>=0 and data['initial_state']['min_human_human_clearance_m']>=0
    if args.resume and valid_initial and data['status']!='error' and not data.get('render_error'):
        skipped.append(key)
        return
    command=[sys.executable,str(HERE/'run_complexitynav.py'),'--name',name,'--counts',*map(str,counts),
             '--width',str(width),'--scenario',scenario,'--seed',str(seed)]
    if blind: command+=['--invisible-robot']
    if seed==0: command+=['--video']
    with (REPORT/f'{key}.log').open('w') as stream:
        try:
            result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=180)
            if result.returncode: failures.append(dict(name=key,returncode=result.returncode))
        except subprocess.TimeoutExpired:
            failures.append(dict(name=key,error='180s wall timeout'))
    executed.append(key)
    if path.exists():
        result=json.loads(path.read_text())
        print(key,result['status'],result['simulation_seconds'],result['minimum_clearance_m'],flush=True)
    else:
        print(key,'failed before summary',flush=True)
for c in conditions:
    for seed in [0,1,2]: run(c,seed)
for seed in [0,1,2]: run(conditions[0],seed,blind=True)
(REPORT/'matrix_execution.json').write_text(json.dumps(dict(planned=30,failures=failures,executed=executed,skipped=skipped),indent=2))
raise SystemExit(1 if failures else 0)
