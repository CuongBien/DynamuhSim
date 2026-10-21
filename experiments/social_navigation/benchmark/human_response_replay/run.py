#!/usr/bin/env python3
"""Reproduce the bounded offline probes from the committed fixed input bundle."""
import argparse,datetime,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent;REPO=SOCIAL.parents[1]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();R=a.report_dir.resolve();R.mkdir(parents=True,exist_ok=False)
reference=HERE/'reports/run01';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
shutil.copytree(reference/'inputs',R/'inputs');shutil.copyfile(reference/'fixture_checks.json',R/'fixture_checks.json')
for condition in ['counterflow','standing']:
    directory=R/'group_inputs'/condition;directory.mkdir(parents=True)
    for name in ['config.yaml','scene.json']:shutil.copyfile(reference/'group_inputs'/condition/name,directory/name)
image=subprocess.check_output(['docker','image','inspect','dynamuhsim-crowdsimplus:round2','--format','{{.Id}}'],text=True).strip()
assert image=='sha256:39929ee8e4fa6d738a381a0b695bf25eaf9fa972cd5b9d8a6854d72f80bd5f3c'
source={str(path.relative_to(REPO)):sha(path) for path in [Path(__file__).resolve(),HERE/'run_crowd.py',SOCIAL/'crowdsimplus_adapter/kernel.py',SOCIAL/'crowdsimplus_adapter/crowd_host.py']}
protocol=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),image=image,seed=42,time_step=.1,steps=200,sim_seconds=20,plan=[dict(policy=policy,case=case) for policy in ['orca_plus','sfm'] for case in ['far','head_on','crossing']],robot_source='Committed scripted world-frame traces from run01; not Gazebo recordings.',input_sha256={p.name:sha(p) for p in (R/'inputs').iterdir()},source_sha256=source,cpu_limit=2)
(R/'protocol.json').write_text(json.dumps(protocol,indent=2))
name='dynamuhsim_replay_'+str(os.getpid())
command=['docker','run','--rm','--name',name,'--network','none','--read-only','--tmpfs','/tmp:rw,nosuid,nodev,size=256m','--cpus','2','--memory','2g','--user',f'{os.getuid()}:{os.getgid()}',
    '-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','PYTHONDONTWRITEBYTECODE=1','-v',f'{SOCIAL/"crowdsimplus_adapter"}:/adapter:ro','-v',f'{HERE}:/probe:ro','-v',f'{R}:/runtime','dynamuhsim-crowdsimplus:round2','python','/probe/run_crowd.py']
print('Running six native human probes',flush=True)
try:
    with (R/'container.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=600,check=True)
finally:subprocess.run(['docker','rm','-f',name],capture_output=True)
source={str(path.relative_to(REPO)):sha(path) for path in [HERE/'run_groups.py',SOCIAL/'gazebo_adapter/kernel.py',BENCH/'circle_contact.py',BENCH/'robot_orca.py',BENCH/'worker.py']}
(R/'group_protocol.json').write_text(json.dumps(dict(seed=42,modes=['walking','conversation'],variants=['native','circle_contact','robot_orca'],cases=['far','head_on','crossing'],updates=200,dt=.1,robot_source=protocol['robot_source'],input_sha256=protocol['input_sha256'],source_sha256=source),indent=2))
env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',SDL_VIDEODRIVER='dummy',SDL_AUDIODRIVER='dummy',MPLBACKEND='Agg');cpus=set(sorted(os.sched_getaffinity(0))[:2])
print('Running18 group ablation probes',flush=True)
with (R/'group_runner.log').open('w') as log:subprocess.run([str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'run_groups.py'),'--report-dir',str(R)],stdout=log,stderr=subprocess.STDOUT,env=env,timeout=600,check=True,preexec_fn=lambda:os.sched_setaffinity(0,cpus))
for name in ['analyze.py','validate.py']:subprocess.run([sys.executable,str(HERE/name),'--report-dir',str(R)],cwd=REPO,check=True)
