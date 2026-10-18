#!/usr/bin/env python3
"""Six sequential paired trials; compare normalized geometry with robot-aware ORCA."""
import argparse,json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',required=True,type=Path);a=p.parse_args()
report=a.report_dir.resolve();report.mkdir(parents=True,exist_ok=True)
jobs=[(42,'counterflow',False),(42,'counterflow',True),(43,'standing',True),(43,'standing',False),(43,'counterflow',True),(43,'counterflow',False)]
results=[]
for i,(seed,condition,enabled) in enumerate(jobs,1):
    label='socnavgym_circle_contact'+('_robot_orca' if enabled else '')
    name=f'{label}_{condition}_s{seed}';path=report/f'{name}.json'
    if path.exists():
        result=json.loads(path.read_text())
        if not result.get('protocol_valid'):raise RuntimeError('Incomplete trial: choose new report directory: '+name)
        print(f'[{i}/{len(jobs)}] REUSE {name}',flush=True)
    else:
        print(f'[{i}/{len(jobs)}] START {name}',flush=True)
        cmd=[sys.executable,str(HERE/'trial.py'),'--backend','socnavgym','--condition',condition,'--seed',str(seed),
            '--geometry','circle_contact','--diagnostics','--report-dir',str(report)]
        if enabled:cmd.append('--robot-avoidance')
        with (report/f'{name}_runner.log').open('w') as log:completed=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
        if not path.exists():raise RuntimeError('No trial report: '+name)
        result=json.loads(path.read_text())
        if completed.returncode or not result.get('protocol_valid'):
            print(json.dumps(result),flush=True);raise SystemExit(1)
    results.append(result)
    (report/'progress.json').write_text(json.dumps([dict(name=r['name'],status=r['status'],protocol_valid=r['protocol_valid']) for r in results],indent=2))
    print(f'[{i}/{len(jobs)}] DONE {name}: {result["status"]}; {result.get("sim_seconds",0):.3f} sim s',flush=True)
for seed,condition in [(42,'counterflow'),(43,'standing'),(43,'counterflow')]:
    pair=[r for r in results if r['seed']==seed and r['condition']==condition];assert len(pair)==2
    baseline=json.loads((HERE/'reports'/f'socnavgym_{condition}_s{seed}.json').read_text())
    for key in ['scene_sha256','map_sha256','nav_params_sha256']:assert pair[0][key]==pair[1][key]==baseline[key],key
    assert all(r['contact_geometry']['effective_robot_radius']==.25 and r['contact_geometry']['human_contact_shape']=='circle' for r in pair)
(report/'protocol.json').write_text(json.dumps(dict(status='PASS',jobs=len(jobs),pairs_same_scene_map_nav2_and_contact_geometry=True,
    comparison='circle_contact vs circle_contact + robot_orca',counterflow_seeds=[42,43],standing_control_seed=43,
    diagnostics_on_both=True,new_robot_controller=False),indent=2))
