#!/usr/bin/env python3
"""Sequential pilot: control plus two scenes, two seeds, two native backends."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser()
p.add_argument('--seeds',type=int,nargs='+',default=[42,43])
p.add_argument('--rerun',action='store_true')
a=p.parse_args()
jobs=[('empty','standing',42)]
for seed in a.seeds:
    for condition in ['standing','counterflow']:
        backends=['rvo2','socnavgym'] if seed%2==0 else ['socnavgym','rvo2']
        jobs.extend((backend,condition,seed) for backend in backends)
results=[]
for index,(backend,condition,seed) in enumerate(jobs,1):
    name=f'{backend}_{condition}_s{seed}'
    path=HERE/'reports'/f'{name}.json'
    cached=json.loads(path.read_text()) if path.exists() and not a.rerun else None
    if cached and cached.get('protocol_valid'):
        print(f'[{index}/{len(jobs)}] REUSE {name}: {cached["status"]}',flush=True)
        results.append(cached)
        continue
    print(f'[{index}/{len(jobs)}] START {name}',flush=True)
    with (HERE/'reports'/f'{name}_runner.log').open('w') as log:
        completed=subprocess.run([sys.executable,str(HERE/'trial.py'),'--backend',backend,
            '--condition',condition,'--seed',str(seed)],stdout=log,stderr=subprocess.STDOUT)
    result=json.loads(path.read_text()) if path.exists() else dict(name=name,status='ERROR',protocol_valid=False)
    results.append(result)
    print(f'[{index}/{len(jobs)}] DONE {name}: {result["status"]}, valid={result["protocol_valid"]}',flush=True)
    (HERE/'reports/matrix_progress.json').write_text(json.dumps([dict(name=r['name'],status=r['status'],protocol_valid=r['protocol_valid']) for r in results],indent=2))
    if completed.returncode:
        print(f'Harness error; inspect {name}_runner.log before continuing.',flush=True)
        raise SystemExit(completed.returncode)
for seed in a.seeds:
    for condition in ['standing','counterflow']:
        pair=[r for r in results if r.get('seed')==seed and r.get('condition')==condition and r.get('backend')!='empty']
        assert len(pair)==2
        for field in ['scene_sha256','map_sha256','nav_params_sha256']:
            assert pair[0][field]==pair[1][field],(seed,condition,field)
(HERE/'reports/matrix_protocol.json').write_text(json.dumps(dict(status='PASS',jobs=len(jobs),seeds=a.seeds,
    paired_initial_scene_map_and_params_identical=True,protocol_valid_trials=sum(r['protocol_valid'] for r in results)),indent=2))
print('All paired scene/map/controller hashes match.',flush=True)
