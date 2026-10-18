#!/usr/bin/env python3
"""Sequential per-human endpoint validation and Gazebo SFM/ORCA+ pilot."""
import argparse,hashlib,json,os,shutil,subprocess,sys,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent
sys.path.insert(0,str(SOCIAL/'crowdsimplus_adapter'))
from crowd_host import prepare_scene,IMAGE,SOURCE,cleanup_container
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);p.add_argument('--validate-only',action='store_true')
p.add_argument('--legacy-kernel',type=Path,default=HERE/'individual_goals/reports/run01/kernel_before.py');a=p.parse_args()
report=a.report_dir.resolve();report.mkdir(parents=True,exist_ok=True)
legacy=a.legacy_kernel.resolve();assert legacy.is_file(),legacy
kernel_hash=hashlib.sha256((SOCIAL/'crowdsimplus_adapter/kernel.py').read_bytes()).hexdigest()
image_id=subprocess.check_output(['docker','image','inspect',IMAGE,'--format','{{.Id}}'],text=True).strip()
assert image_id=='sha256:39929ee8e4fa6d738a381a0b695bf25eaf9fa972cd5b9d8a6854d72f80bd5f3c',image_id
jobs=[('sfm',42),('orca_plus',42),('orca_plus',43),('sfm',43)]
for policy,seed in jobs:
    path=report/f'{policy}_s{seed}_validation.json'
    if path.exists():
        data=json.loads(path.read_text());assert data['status']=='PASS' and data['bridge_sha256']==kernel_hash
        print(f'VALIDATION REUSE {policy} seed {seed}',flush=True);continue
    runtime=Path(tempfile.mkdtemp(prefix=f'individual_validation_{policy}_s{seed}_',dir=HERE/'generated'))
    prepare_scene(runtime,'counterflow',seed,'individual');shutil.copyfile(legacy,runtime/'legacy_kernel.py')
    name=f'dynamuhsim_individual_check_{os.getpid()}'
    cmd=['docker','run','--rm','--name',name,'--network','none','--read-only','--tmpfs','/tmp:rw,nosuid,nodev,size=256m',
        '--cpus','4','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}',
        '-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-v',f'{SOCIAL/"crowdsimplus_adapter"}:/work:ro',
        '-v',f'{runtime}:/runtime','-w','/work',IMAGE,'python','/work/validate_individual_goals.py','--human-policy',policy]
    if policy=='sfm' and seed==42:cmd.append('--shared-control')
    print(f'VALIDATION START {policy} seed {seed}',flush=True)
    with (runtime/'runner.log').open('w') as f:
        try:done=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=240)
        finally:cleanup_container(name)
    if done.returncode:
        print((runtime/'runner.log').read_text()[-4000:],flush=True);raise SystemExit(1)
    data=json.loads((runtime/'validation.json').read_text());assert data['status']=='PASS' and data['metadata']['source_commit']==SOURCE
    data.update(runtime=str(runtime.relative_to(HERE)),bridge_sha256=kernel_hash,legacy_kernel_sha256=hashlib.sha256(legacy.read_bytes()).hexdigest(),image_id=image_id)
    path.write_text(json.dumps(data,indent=2))
    shutil.copyfile(runtime/'benchmark.config',report/f'{policy}_s{seed}.config')
    shutil.copyfile(runtime/'scene.json',report/f'fixture_s{seed}.json')
    print(f'VALIDATION DONE {policy} seed {seed}: completed legs {data["individual"]["completed_legs"]}',flush=True)
if a.validate_only:raise SystemExit(0)
results=[]
for i,(policy,seed) in enumerate(jobs,1):
    label='crowdsimplus'+('_sfm' if policy=='sfm' else '')+'_individual_goals';name=f'{label}_counterflow_s{seed}';path=report/f'{name}.json'
    if path.exists():
        result=json.loads(path.read_text());assert result['protocol_valid'] and result['goal_layout']=='individual' and result['human_policy']==policy
        print(f'[{i}/4] REUSE {name}',flush=True)
    else:
        print(f'[{i}/4] START {name}',flush=True)
        cmd=[sys.executable,str(HERE/'trial.py'),'--backend','crowdsimplus','--human-policy',policy,'--goal-layout','individual',
            '--condition','counterflow','--seed',str(seed),'--diagnostics','--report-dir',str(report)]
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
        assert x['scene']['initial']==baseline['scene']['initial']
        m=x['crowdsim_metadata'];assert m['goal_layout']=='individual' and m['time_step']==.1 and m['human_radius']==.36 and m['human_v_pref']==.8 and m['robot_visible']
        assert m['static_line_segments']==168
        if x['human_policy']=='sfm':assert not m['sfm_parameters']['is_bottleneck']
(report/'protocol.json').write_text(json.dumps(dict(status='PASS',trials=4,seeds=[42,43],policies=['sfm','orca_plus'],
    initial_poses_unchanged=True,same_scene_within_each_policy_pair=True,same_map_nav2_as_shared_goal_pilot=True,
    only_fixture_change='Per-person endpoint y=-1.4/0/1.4 by initial y order; no lane following constraint',
    external_patrol='native reached_destination criterion, two endpoints per person',
    shared_mode_remains_default=True,clock='simulation',validation_before_gazebo=True,
    robot_noise_jointly_seeded=False,source_commit=SOURCE,image_id=image_id),indent=2))
