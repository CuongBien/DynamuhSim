#!/usr/bin/env python3
"""Validate native bottleneck behavior, then run four sequential Gazebo trials."""
import argparse,hashlib,json,os,shutil,subprocess,sys,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent
sys.path.insert(0,str(SOCIAL/'crowdsimplus_adapter'))
from crowd_host import prepare_scene,IMAGE,SOURCE,cleanup_container
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);p.add_argument('--validate-only',action='store_true')
p.add_argument('--legacy-kernel',type=Path,default=HERE/'bottleneck/reports/run01/kernel_before.py');a=p.parse_args()
report=a.report_dir.resolve();report.mkdir(parents=True,exist_ok=True)
legacy=a.legacy_kernel.resolve();assert legacy.is_file(),legacy
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
image_id=subprocess.check_output(['docker','image','inspect',IMAGE,'--format','{{.Id}}'],text=True).strip()
assert image_id=='sha256:39929ee8e4fa6d738a381a0b695bf25eaf9fa972cd5b9d8a6854d72f80bd5f3c',image_id
source_hashes={f:sha(SOCIAL/'crowdsimplus_adapter'/f) for f in ['kernel.py','crowd_host.py','bottleneck_scene.py','passage_metrics.py','validate_bottleneck.py']}
fingerprint=dict(sources=source_hashes,legacy_kernel_sha256=sha(legacy),image_id=image_id,layout_sha256=sha(SOCIAL/'warehouse/layout.json'))
jobs=[('sfm',42),('orca_plus',42),('orca_plus',43),('sfm',43)]
# Preserve both previously supported scene modes byte for byte.
for seed in [42,43]:
 for goal_layout in ['shared','individual']:
    check=Path(tempfile.mkdtemp(prefix='bottleneck_scene_regression_',dir=HERE/'generated'))
    prepare_scene(check,'counterflow',seed,goal_layout)
    reference=(HERE/'reports'/f'socnavgym_counterflow_s{seed}.json') if goal_layout=='shared' else (HERE/'individual_goals/reports/run01'/f'fixture_s{seed}.json')
    original=json.loads(reference.read_text());original=original['scene'] if goal_layout=='shared' else original
    assert json.loads((check/'scene.json').read_text())==original,(seed,goal_layout)
print('PREVIOUS SHARED/INDIVIDUAL FIXTURES UNCHANGED',flush=True)
for policy,seed in jobs:
 path=report/f'{policy}_s{seed}_validation.json'
 if path.exists():
    data=json.loads(path.read_text());assert data['status']=='PASS' and data['fingerprint']==fingerprint
    print(f'VALIDATION REUSE {policy} seed {seed}',flush=True);continue
 runtime=Path(tempfile.mkdtemp(prefix=f'bottleneck_validation_{policy}_s{seed}_',dir=HERE/'generated'))
 prepare_scene(runtime,'bottleneck',seed,'individual');shutil.copyfile(legacy,runtime/'legacy_kernel.py')
 shutil.copyfile(HERE/'individual_goals/reports/run01'/f'fixture_s{seed}.json',runtime/'counterflow_scene.json')
 shutil.copyfile(HERE/'crossing/reports/run01'/f'fixture_s{seed}.json',runtime/'crossing_scene.json')
 name=f'dynamuhsim_bottleneck_check_{os.getpid()}'
 cmd=['docker','run','--rm','--name',name,'--network','none','--read-only','--tmpfs','/tmp:rw,nosuid,nodev,size=256m',
      '--cpus','4','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}',
      '-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-v',f'{SOCIAL/"crowdsimplus_adapter"}:/work:ro',
      '-v',f'{runtime}:/runtime','-w','/work',IMAGE,'python','/work/validate_bottleneck.py','--human-policy',policy]
 print(f'VALIDATION START {policy} seed {seed}: {runtime}',flush=True)
 with (runtime/'runner.log').open('w') as f:
    try:done=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=300)
    finally:cleanup_container(name)
 if done.returncode:
    print((runtime/'runner.log').read_text()[-5000:],flush=True);raise SystemExit(1)
 data=json.loads((runtime/'validation.json').read_text());assert data['status']=='PASS' and data['metadata']['source_commit']==SOURCE
 assert (runtime/'benchmark.config').read_bytes()==(HERE/'sfm_comparison/reports/run01'/f'{policy}.config').read_bytes()
 data.update(runtime=str(runtime.relative_to(HERE)),fingerprint=fingerprint)
 path.write_text(json.dumps(data,indent=2));shutil.copyfile(runtime/'benchmark.config',report/f'{policy}_s{seed}.config')
 shutil.copyfile(runtime/'scene.json',report/f'fixture_s{seed}.json')
 print(f'VALIDATION DONE {policy} seed {seed}: {data["checks"][3]}',flush=True)
if a.validate_only:raise SystemExit(0)
control_path=report/'empty_bottleneck_s42.json'
if not control_path.exists():
 print('EMPTY CONTROL START',flush=True)
 with (report/'empty_bottleneck_s42_runner.log').open('w') as f:
  done=subprocess.run([sys.executable,str(HERE/'trial.py'),'--backend','empty','--condition','bottleneck','--seed','42','--diagnostics','--report-dir',str(report)],stdout=f,stderr=subprocess.STDOUT)
 assert done.returncode==0,'Empty control runtime error'
control=json.loads(control_path.read_text());assert control['protocol_valid'] and control['status']=='SUCCEEDED',control
from passage_metrics import PassageCounter
cp=PassageCounter([dict(id=0,x=8.,y=0.)],control['scene']['passage']);rt=HERE/control['runtime'];gate=float((rt/'start_gate').read_text())
for line in (rt/'diagnostics.jsonl').open():
 row=json.loads(line)
 if row['kind']=='gazebo' and gate<=row['data']['time']<=gate+control['sim_seconds']:
  cp.update(row['data']['time']-gate,[dict(id=0,**row['data']['poses']['mecanum_robot'])])
assert cp.counts()['0']>=1,'Empty robot did not traverse actual channel'
print('EMPTY CONTROL PASS: actual channel traversed',flush=True)
results=[]
for i,(policy,seed) in enumerate(jobs,1):
 label='crowdsimplus'+('_sfm' if policy=='sfm' else '')+'_individual_goals';name=f'{label}_bottleneck_s{seed}';path=report/f'{name}.json'
 if path.exists():
    result=json.loads(path.read_text());assert result['protocol_valid'] and result['condition']=='bottleneck' and result['human_policy']==policy
    print(f'[{i}/4] REUSE {name}',flush=True)
 else:
    print(f'[{i}/4] START {name}',flush=True)
    cmd=[sys.executable,str(HERE/'trial.py'),'--backend','crowdsimplus','--human-policy',policy,'--goal-layout','individual',
         '--condition','bottleneck','--seed',str(seed),'--diagnostics','--report-dir',str(report)]
    with (report/f'{name}_runner.log').open('w') as f:done=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
    if not path.exists():raise RuntimeError('Missing trial: '+name)
    result=json.loads(path.read_text())
    if done.returncode or not result.get('protocol_valid'):raise RuntimeError(json.dumps(result))
 results.append(result)
 (report/'progress.json').write_text(json.dumps([dict(name=x['name'],status=x['status'],protocol_valid=x['protocol_valid']) for x in results],indent=2))
 print(f'[{i}/4] DONE {name}: {result["status"]}; {result["sim_seconds"]:.3f} sim s',flush=True)
for seed in [42,43]:
 pair=[x for x in results if x['seed']==seed];baseline=json.loads((HERE/'reports'/f'socnavgym_counterflow_s{seed}.json').read_text())
 assert len(pair)==2 and pair[0]['scene']==pair[1]['scene']
 for x in pair:
    for key in ['map_sha256','nav_params_sha256']:assert x[key]==baseline[key]
    m=x['crowdsim_metadata'];assert m['goal_layout']=='individual' and m['time_step']==.1 and m['human_radius']==.36 and m['human_v_pref']==.8 and m['robot_visible']
    assert m['static_line_segments']==168
    assert x['robot_start']==[8.,0.,0.] and x['robot_goal']==[24.5,0.,0.]
    assert x['runtime_nav_params_sha256']==control['runtime_nav_params_sha256']
    if x['human_policy']=='sfm':assert not m['sfm_parameters']['is_bottleneck']
(report/'protocol.json').write_text(json.dumps(dict(status='PASS',trials=4,empty_controls=1,seeds=[42,43],policies=['sfm','orca_plus'],
 same_scene_within_each_policy_pair=True,same_map_controller_sensor_params_as_previous_pilot=True,
 amcl_initial_pose_only_override=True,empty_control_passed_actual_channel=True,
 fixture_change='Three independent counterflow humans, private x endpoints; real factory throat x12..20, width1.8m; robot start8 goal24.5; no lane/timing constraints.',
 previous_fixture_modes_unchanged=True,external_patrol='native reached_destination criterion, two endpoints per person',
 shared_mode_remains_default=True,clock='simulation',validation_before_gazebo=True,
 robot_noise_jointly_seeded=False,source_commit=SOURCE,image_id=image_id,fingerprint=fingerprint),indent=2))
