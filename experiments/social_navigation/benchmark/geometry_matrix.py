#!/usr/bin/env python3
"""Counterbalanced sequential native/circle-contact ablation; never overwrite pilot."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser()
p.add_argument('--report-dir',required=True,type=Path)
p.add_argument('--seeds',type=int,nargs='+',default=[42,43])
a=p.parse_args()
report=a.report_dir.resolve();report.mkdir(parents=True,exist_ok=True)
jobs=[]
for seed in a.seeds:
    for condition in ['standing','counterflow']:
        policies=['native','circle_contact'] if seed%2==0 else ['circle_contact','native']
        jobs.extend((seed,condition,policy) for policy in policies)
results=[]
for i,(seed,condition,policy) in enumerate(jobs,1):
    label='socnavgym' if policy=='native' else 'socnavgym_circle_contact'
    name=f'{label}_{condition}_s{seed}';path=report/f'{name}.json'
    if path.exists():
        result=json.loads(path.read_text())
        if not result.get('protocol_valid'):raise RuntimeError('Incomplete trial; choose a new output directory: '+name)
        print(f'[{i}/{len(jobs)}] REUSE {name}',flush=True)
    else:
        print(f'[{i}/{len(jobs)}] START {name}',flush=True)
        with (report/f'{name}_runner.log').open('w') as log:
            completed=subprocess.run([sys.executable,str(HERE/'trial.py'),'--backend','socnavgym',
                '--condition',condition,'--seed',str(seed),'--geometry',policy,
                '--diagnostics','--report-dir',str(report)],stdout=log,stderr=subprocess.STDOUT)
        if not path.exists():raise RuntimeError('Trial produced no report: '+name)
        result=json.loads(path.read_text())
        if completed.returncode or not result.get('protocol_valid'):
            print(json.dumps(result),flush=True);raise SystemExit(1)
    results.append(result)
    (report/'progress.json').write_text(json.dumps([dict(name=r['name'],status=r['status'],protocol_valid=r['protocol_valid']) for r in results],indent=2))
    print(f'[{i}/{len(jobs)}] DONE {name}: {result["status"]}; {result.get("sim_seconds",0):.3f} sim s',flush=True)
for seed in a.seeds:
    for condition in ['standing','counterflow']:
        pair=[r for r in results if r['seed']==seed and r['condition']==condition]
        assert len(pair)==2
        for field in ['scene_sha256','map_sha256','nav_params_sha256']:assert pair[0][field]==pair[1][field],field
        native=next(r for r in pair if r['geometry']=='native')
        baseline=json.loads((HERE/'reports'/f'socnavgym_{condition}_s{seed}.json').read_text())
        for field in ['scene_sha256','map_sha256','nav_params_sha256']:assert native[field]==baseline[field],field
(report/'protocol.json').write_text(json.dumps(dict(status='PASS',jobs=len(jobs),seeds=a.seeds,
    pairs_same_initial_scene_map_and_nav_params=True,original_pilot_hashes_match=True,
    variant='circle_contact',added_robot_avoidance=False,swept_collision_guard=False,
    diagnostics_enabled_on_both_variants=True,order='native first on even seeds; circle_contact first on odd seeds'),indent=2))
