#!/usr/bin/env python3
"""Sequential CrowdSimPlus integration pilot with matched SocNavGym adapter controls."""
import argparse,json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',required=True,type=Path);a=p.parse_args()
report=a.report_dir.resolve();report.mkdir(parents=True,exist_ok=True)
jobs=[('crowdsimplus','counterflow',43,False),('crowdsimplus','standing',42,False),('socnavgym','counterflow',43,False),
      ('crowdsimplus','counterflow',42,False),('socnavgym','counterflow',42,False),('crowdsimplus','standing',43,False),('crowdsimplus','counterflow',43,True)]
results=[]
for i,(backend,condition,seed,blind) in enumerate(jobs,1):
    label='socnavgym_circle_contact_robot_orca' if backend=='socnavgym' else 'crowdsimplus'+('_blind' if blind else '')
    name=f'{label}_{condition}_s{seed}';path=report/f'{name}.json'
    if path.exists():
        result=json.loads(path.read_text())
        if not result.get('protocol_valid'):raise RuntimeError('Incomplete trial; use a new directory: '+name)
        print(f'[{i}/{len(jobs)}] REUSE {name}',flush=True)
    else:
        print(f'[{i}/{len(jobs)}] START {name}',flush=True)
        cmd=[sys.executable,str(HERE/'trial.py'),'--backend',backend,'--condition',condition,'--seed',str(seed),'--diagnostics','--report-dir',str(report)]
        if backend=='socnavgym':cmd+=['--geometry','circle_contact','--robot-avoidance']
        if blind:cmd.append('--invisible-robot')
        with (report/f'{name}_runner.log').open('w') as f:completed=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
        if not path.exists():raise RuntimeError('No report: '+name)
        result=json.loads(path.read_text())
        if completed.returncode or not result.get('protocol_valid'):
            print(json.dumps(result),flush=True);raise SystemExit(1)
    results.append(result)
    (report/'progress.json').write_text(json.dumps([dict(name=r['name'],status=r['status'],protocol_valid=r['protocol_valid']) for r in results],indent=2))
    print(f'[{i}/{len(jobs)}] DONE {name}: {result["status"]}; {result.get("sim_seconds",0):.3f} sim s',flush=True)
for seed in [42,43]:
    pair=[r for r in results if r['seed']==seed and r['condition']=='counterflow' and r['robot_visible']]
    assert len(pair)==2
    baseline=json.loads((HERE/'reports'/f'socnavgym_counterflow_s{seed}.json').read_text())
    for key in ['scene_sha256','map_sha256','nav_params_sha256']:assert pair[0][key]==pair[1][key]==baseline[key],key
(report/'protocol.json').write_text(json.dumps(dict(status='PASS',jobs=len(jobs),counterflow_seeds=[42,43],crowdsim_stationary_seeds=[42,43],
    crowdsim_visibility_control_seed=43,common_scene_map_nav2=True,comparison='native CrowdSimPlus human ORCA+ vs modified SocNavGym robot-aware group adapter',
    source_step_scope='human dynamics only; Gazebo/Nav2 owns robot',common_time_step=.1,diagnostics_on_both=True),indent=2))
