#!/usr/bin/env python3
"""Fixed six-trial cohort. Stop for infrastructure faults, retain all goal outcomes."""
import argparse,datetime,hashlib,json,os,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent;REPO=SOCIAL.parents[1]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args()
report=a.report_dir.resolve();report.mkdir(parents=True,exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
order=[['sfm','orca_plus'],['orca_plus','sfm'],['sfm','orca_plus']]
plan=[dict(round=i,policy=policy,seed=42,name='crowdsimplus'+('_sfm' if policy=='sfm' else '')+'_individual_goals_bottleneck_s42') for i,policies in enumerate(order,1) for policy in policies]
def owned_nodes():
    found=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            env=dict(v.split(b'=',1) for v in (proc/'environ').read_bytes().split(b'\0') if b'=' in v);args=(proc/'cmdline').read_bytes()
        except OSError:continue
        if args and (b'social_navigation/' in env.get(b'ROS_LOG_DIR',b'') or env.get(b'DYNAMUHSIM_EXPERIMENT_SESSION')):found.append(dict(pid=int(proc.name),command=args.replace(b'\0',b' ').decode(errors='replace')))
    return found
def gpu():
    r=subprocess.run(['nvidia-smi','--query-gpu=name,temperature.gpu,power.draw,utilization.gpu,memory.used','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=10)
    return dict(returncode=r.returncode,rows=r.stdout.strip(),stderr=r.stderr.strip())
assert not owned_nodes(),'Existing experiment processes; stop rather than disturb another session'
protocol=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),plan=plan,
    condition='bottleneck',goal_layout='individual',sim_budget_s=75,cooldown_wall_s=15,diagnostics='--navigation-diagnostics',gui=False,
    goal_failure_policy='Keep ABORTED/TIMEOUT; continue remaining fixed trials. No cherry-picked retries or replacement of old cohorts.',
    infrastructure_stop_policy='Stop remaining cohort on child exit, invalid trial, missing diagnostic channels, wrong hardware renderer, cleanup failure or leftover owned nodes.',
    randomness_scope='Seed42 fixes people fixture/native policy seeds only; AMCL/LiDAR/MPPI/physics scheduling are not jointly deterministically seeded.',
    comparison_scope='Six within-seed diagnostic repetitions; not simulator ranking, not failure-rate estimate across general environments. Old 2/4 benchmark and later 2/2 diagnostics remain separate.',
    source_sha256={str(x.relative_to(REPO)):sha(x) for x in [BENCH/'trial.py',BENCH/'diagnostics.py',BENCH/'navigation_diagnostics/capture.py',BENCH/'gpu_runtime/runtime_guard.py',SOCIAL/'warehouse/nav2_warehouse.yaml',SOCIAL/'warehouse/warehouse.pgm',SOCIAL/'crowdsimplus_adapter/kernel.py',SOCIAL/'crowdsimplus_adapter/adapter.cc',SOCIAL/'crowdsimplus_adapter/crowd_host.py',Path(__file__)]})
(report/'protocol.json').write_text(json.dumps(protocol,indent=2));progress=[];stopped=None
for sequence,entry in enumerate(plan,1):
    left=owned_nodes()
    if left:stopped=dict(reason='Owned nodes before trial',processes=left);break
    directory=report/f"round{entry['round']:02d}";directory.mkdir(exist_ok=True)
    command=['bash',str(BENCH/'run.sh'),'trial','--backend','crowdsimplus','--human-policy',entry['policy'],'--goal-layout','individual','--condition','bottleneck','--seed','42','--sim-budget','75','--navigation-diagnostics','--report-dir',str(directory)]
    before=gpu();started=datetime.datetime.now(datetime.timezone.utc).isoformat()
    print(f"START {sequence}/6 round{entry['round']} {entry['policy']}",flush=True)
    with (directory/(entry['name']+'_runner.log')).open('w') as output:
        process=subprocess.run(command,cwd=REPO,stdout=output,stderr=subprocess.STDOUT)
    path=directory/(entry['name']+'.json');result=json.loads(path.read_text()) if path.exists() else {}
    item=dict(**entry,sequence=sequence,started_utc=started,finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),runner_returncode=process.returncode,report=str(path.relative_to(report)),gpu_before=before,gpu_after=gpu(),status=result.get('status','MISSING'),protocol_valid=result.get('protocol_valid',False),remaining_nodes=owned_nodes())
    reasons=[]
    if process.returncode or not result.get('protocol_valid'):reasons.append('Trial process/protocol failed')
    if not result.get('rendering',{}).get('hardware_verified') or 'NVIDIA' not in result.get('rendering',{}).get('renderer',''):reasons.append('NVIDIA renderer not verified')
    cleanup=result.get('process_cleanup',{})
    if cleanup.get('status')!='PASS' or cleanup.get('remaining') or item['remaining_nodes']:reasons.append('Process cleanup failed')
    if not any('NVIDIA' in l for l in cleanup.get('ogre_renderer_lines',[])):reasons.append('Fresh Gazebo Ogre2 renderer not verified')
    for key in ['global_costmap','global_plan','particle_cloud','amcl','scan','odom','gazebo','native']:
        if result.get('diagnostic_counts',{}).get(key,0)==0:reasons.append('Missing '+key)
    item['infrastructure_errors']=reasons;progress.append(item)
    (report/'progress.json').write_text(json.dumps(dict(completed=len(progress),planned=6,attempts=progress,stopped=stopped),indent=2))
    print(f"END {sequence}/6 {entry['policy']} {item['status']} infra_errors={reasons}",flush=True)
    if reasons:stopped=dict(sequence=sequence,reasons=reasons);break
    if sequence<6:time.sleep(15)
(report/'progress.json').write_text(json.dumps(dict(completed=len(progress),planned=6,attempts=progress,stopped=stopped),indent=2))
raise SystemExit(1 if stopped else 0)
