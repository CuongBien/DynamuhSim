#!/usr/bin/env python3
"""Sequential native CrowdSimPlus SFM/ORCA+ pilot, same Gazebo fixture and Nav2."""
import argparse,json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args()
report=a.report_dir.resolve();report.mkdir(parents=True,exist_ok=True)
for policy in ['orca_plus','sfm']:
    check=json.loads((report/f'{policy}_validation.json').read_text())
    assert check['status']=='PASS' and check['metadata']['human_policy']==policy
jobs=[('sfm',42),('orca_plus',42),('orca_plus',43),('sfm',43)]
results=[]
for i,(policy,seed) in enumerate(jobs,1):
    label='crowdsimplus'+('_sfm' if policy=='sfm' else '')
    name=f'{label}_counterflow_s{seed}';path=report/f'{name}.json'
    if path.exists():
        result=json.loads(path.read_text());assert result['protocol_valid'],name
        assert result.get('human_policy')==policy,name
        print(f'[{i}/4] REUSE {name}',flush=True)
    else:
        print(f'[{i}/4] START {name}',flush=True)
        cmd=[sys.executable,str(HERE/'trial.py'),'--backend','crowdsimplus','--human-policy',policy,
            '--condition','counterflow','--seed',str(seed),'--diagnostics','--report-dir',str(report)]
        with (report/f'{name}_runner.log').open('w') as f:done=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
        if not path.exists():raise RuntimeError('Missing result: '+name)
        result=json.loads(path.read_text())
        if done.returncode or not result.get('protocol_valid'):raise RuntimeError(json.dumps(result))
    results.append(result)
    (report/'progress.json').write_text(json.dumps([dict(name=x['name'],status=x['status'],protocol_valid=x['protocol_valid']) for x in results],indent=2))
    print(f'[{i}/4] DONE {name}: {result["status"]}; {result["sim_seconds"]:.3f} sim s',flush=True)
for seed in [42,43]:
    pair=[x for x in results if x['seed']==seed]
    baseline=json.loads((HERE/'reports'/f'socnavgym_counterflow_s{seed}.json').read_text())
    assert len(pair)==2
    for key in ['scene_sha256','map_sha256','nav_params_sha256']:
        assert pair[0][key]==pair[1][key]==baseline[key],key
for x in results:
    m=x['crowdsim_metadata'];assert m['time_step']==.1 and m['human_radius']==.36 and m['human_v_pref']==.8 and m['robot_visible']
    assert m['static_line_segments']==168 and m['source_commit']=='d526a07c013c621fa1ad50894b6ba639f61063de'
    if x['human_policy']=='sfm':assert m['sfm_parameters']['is_bottleneck'] is False
(report/'protocol.json').write_text(json.dumps(dict(status='PASS',trials=4,seeds=[42,43],policies=['sfm','orca_plus'],
    common_scene_map_nav2=True,time_step=.1,human_radius=.36,human_v_pref=.8,robot_visible=True,
    static_line_segments=168,comparison='Native human policy only; Gazebo/Nav2 owns robot',
    note='No group model. Original standalone SFM demo uses dt .25; warehouse SFM is_bottleneck=False. Nav2/AMCL/LiDAR noise not jointly seeded.'),indent=2))
