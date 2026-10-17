#!/usr/bin/env python3
"""Run native scenes sequentially; --resume reruns only missing/error cases."""
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
failures=[]
executed=[]
skipped=[]

def run_case(scenario,population,human_policy,seed,blind=False):
    name=f'{scenario}_{human_policy}_dwa_n{population}_s{seed}_{"blind" if blind else "visible"}'
    existing=REPORT/f'{name}.json'
    if args.resume and existing.exists() and json.loads(existing.read_text())['status'] in ['success','timeout']:
        skipped.append(name)
        return
    command=[sys.executable,str(HERE/'run_crowdsimplus.py'),'--scenario',scenario,
        '--humans',str(population),'--human-policy',human_policy,'--seed',str(seed)]
    if blind:
        command+=['--invisible-robot']
    if seed==0:
        command+=['--video']
    with (REPORT/f'{name}.log').open('w') as log:
        try:
            completed=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=180)
            if completed.returncode:
                failures.append(dict(name=name,returncode=completed.returncode))
        except subprocess.TimeoutExpired:
            failures.append(dict(name=name,error='180s wall timeout'))
    executed.append(name)
    if existing.exists():
        result=json.loads(existing.read_text())
        print(name,result['status'],result['step_counts'],flush=True)
    else:
        print(name,'FAILED before summary',flush=True)

for scenario,population in [('circle',8),('hallway',6),('bottleneck',3)]:
    for human_policy in ['orca_plus','sfm']:
        for seed in [0,1,2]:
            run_case(scenario,population,human_policy,seed)
for human_policy in ['orca_plus','sfm']:
    for seed in [0,1,2]:
        run_case('circle',8,human_policy,seed,blind=True)
(REPORT/'matrix_execution.json').write_text(json.dumps(dict(failures=failures,
    planned_cases=24,executed=executed,skipped_completed=skipped),indent=2))
raise SystemExit(1 if failures else 0)
