#!/usr/bin/env python3
"""Fixed group demonstration cohort. Stop for infrastructure faults, retain all goal outcomes."""
import argparse,datetime,hashlib,json,os,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent;REPO=SOCIAL.parents[1]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True)
p.add_argument('--conditions',nargs='+',choices=['standing','counterflow'],default=['standing','counterflow'])
p.add_argument('--amcl-observer-qos',choices=['sensor_data','transient_local'],default='sensor_data')
p.add_argument('--amcl-late-join-check',action='store_true')
p.add_argument('--gui-first',action='store_true')
p.add_argument('--continuation-of',type=Path)
a=p.parse_args()
if len(a.conditions)!=len(set(a.conditions)):p.error('Duplicate conditions')
if a.amcl_late_join_check and a.amcl_observer_qos!='transient_local':p.error('Late-join check requires transient_local')
report=a.report_dir.resolve();report.mkdir(parents=True,exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
plan=[dict(variant=variant,condition=condition,gui=(variant=='native' and condition=='standing'),
    name='socnavgym'+('' if variant=='native' else '_circle_contact')+('_robot_orca' if variant=='robot_orca' else '')+f'_{condition}_s42')
    for condition in a.conditions for variant in ['native','circle_contact','robot_orca']]
assert len(plan)==3*len(a.conditions)
if a.gui_first:plan[0]['gui']=True
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
    seed=42,sim_budget_s=75,cooldown_wall_s=15,localization_profile='baseline',shutdown_mode='wsl_d3d12',
    amcl_observer_qos=a.amcl_observer_qos,amcl_late_join_check=a.amcl_late_join_check,qt_qpa_platform=os.environ.get('QT_QPA_PLATFORM'),
    scope='NEW live Gazebo/Nav2 demonstrations of one3-person group. Actual robot input can differ between policies; NOT identical-input replay. One seed, no simulator ranking.',
    capture='--diagnostics plus live observer RGB camera; first standing-native trial opens GUI for manual user check.',
    source_sha256={str(path.relative_to(REPO)):sha(path) for path in [Path(__file__),BENCH/'trial.py',BENCH/'amcl_observer.py',BENCH/'diagnostics.py',BENCH/'worker.py',BENCH/'circle_contact.py',BENCH/'robot_orca.py',BENCH/'shutdown_fix/ordered_stop.py',BENCH/'gpu_runtime/runtime_guard.py',SOCIAL/'gazebo_adapter/kernel.py',SOCIAL/'warehouse/nav2_warehouse.yaml',SOCIAL/'warehouse/warehouse.pgm']})
if a.continuation_of:
    previous=a.continuation_of.resolve()
    protocol['continuation_of']=dict(path=str(previous),sha256=sha(previous),
        amendment='Read-only AMCL observer QoS changed explicitly; original startup error retained. No reset, force-update, robot controller or AMCL parameter change.')
(report/'protocol.json').write_text(json.dumps(protocol,indent=2));progress=[];stopped=None
for sequence,entry in enumerate(plan,1):
    left=owned_nodes()
    if left:stopped=dict(reason='Owned nodes before trial',processes=left);break
    directory=report
    command=['bash',str(BENCH/'run.sh'),'trial','--backend','socnavgym','--condition',entry['condition'],'--seed','42',
        '--sim-budget','75','--diagnostics','--shutdown-mode','wsl_d3d12','--report-dir',str(directory),
        '--amcl-observer-qos',a.amcl_observer_qos]
    if a.amcl_late_join_check:command+=['--amcl-late-join-check']
    if entry['variant']!='native':command+=['--geometry','circle_contact']
    if entry['variant']=='robot_orca':command+=['--robot-avoidance']
    if entry['gui']:command+=['--gui']
    before=gpu();started=datetime.datetime.now(datetime.timezone.utc).isoformat()
    print(f"START {sequence}/{len(plan)} {entry['condition']} {entry['variant']} gui={entry['gui']}",flush=True)
    with (directory/(entry['name']+'_runner.log')).open('w') as output:
        process=subprocess.run(command,cwd=REPO,stdout=output,stderr=subprocess.STDOUT)
    path=directory/(entry['name']+'.json');result=json.loads(path.read_text()) if path.exists() else {}
    item=dict(**entry,sequence=sequence,started_utc=started,finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),runner_returncode=process.returncode,report=str(path.relative_to(report)),gpu_before=before,gpu_after=gpu(),status=result.get('status','MISSING'),protocol_valid=result.get('protocol_valid',False),remaining_nodes=owned_nodes())
    reasons=[]
    if process.returncode or not result.get('protocol_valid'):reasons.append('Trial process/protocol failed')
    if not result.get('rendering',{}).get('hardware_verified') or 'NVIDIA' not in result.get('rendering',{}).get('renderer',''):reasons.append('NVIDIA renderer not verified')
    if not result.get('gazebo_exit',{}).get('clean') or result.get('server_shutdown',{}).get('status')!='STOPPED':reasons.append('Gazebo did not exit cleanly')
    if result.get('localization_profile')!='baseline' or not result.get('amcl_parameters_readback'):reasons.append('Profile not verified')
    if result.get('ros_teardown_error'):reasons.append('ROS teardown exception')
    if any(error in (BENCH/result['runtime']/'worker.log').read_text() for error in ['Traceback','ConnectionResetError','BrokenPipeError']):reasons.append('Worker exception')
    cleanup=result.get('process_cleanup',{})
    if cleanup.get('status')!='PASS' or cleanup.get('remaining') or item['remaining_nodes']:reasons.append('Process cleanup failed')
    if not any('NVIDIA' in l for l in cleanup.get('ogre_renderer_lines',[])):reasons.append('Fresh Gazebo Ogre2 renderer not verified')
    for key in ['amcl','scan','odom','gazebo','native']:
        if result.get('diagnostic_counts',{}).get(key,0)==0:reasons.append('Missing '+key)
    item['infrastructure_errors']=reasons;progress.append(item)
    (report/'progress.json').write_text(json.dumps(dict(completed=len(progress),planned=len(plan),attempts=progress,stopped=stopped),indent=2))
    print(f"END {sequence}/{len(plan)} {entry['condition']} {entry['variant']} {item['status']} infra_errors={reasons}",flush=True)
    if reasons:stopped=dict(sequence=sequence,reasons=reasons);break
    if sequence<len(plan):time.sleep(15)
(report/'progress.json').write_text(json.dumps(dict(completed=len(progress),planned=len(plan),attempts=progress,stopped=stopped),indent=2))
raise SystemExit(1 if stopped else 0)
