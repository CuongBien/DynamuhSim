#!/usr/bin/env python3
"""Sequential frozen capture cohort. Native controller/policy unchanged, no success retries."""
import argparse,datetime,difflib,hashlib,json,os,subprocess,sys,time
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=HERE.parents[3];PARENT=BENCH/'early_avoidance_pilot';DATASET=BENCH/'dataset_pilot';LIFE=BENCH/'depth_proxy_lifecycle'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
def verify():
 f=json.loads((HERE/'capture_frozen.json').read_text());assert all(sha(REPO/n)==h for n,h in f['source_sha256'].items()),'Capture source changed';return f

def prepare():
 dest=HERE/'reports/implementation';dest.mkdir(exist_ok=False);parent=PARENT/'reports/drivers/avoidance_moderate01/driver.py';before=parent.read_text();after=before;edits={"choices=[42,43]":"choices=[45,46,47,48,49]","runtime=HERE/'generated'/out.name;runtime.mkdir(exist_ok=False)":"runtime=Path("+repr(str(HERE/'generated/runtime'))+")/out.name;runtime.mkdir(exist_ok=False)","Conservative Nav2 footprint/inflation/speed pilot; native sensors/standingv3 unchanged":"WM data cohort; same moderate Nav2/native sensors and group policies"}
 for a,b in edits.items():assert after.count(a)==1,(a,after.count(a));after=after.replace(a,b)
 compile(after,str(dest/'driver.py'),'exec');(dest/'driver.py').write_text(after);(dest/'driver_change.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='accepted moderate driver',tofile='WM cohort driver',n=0)))
 source=json.loads((PARENT/'reports/avoidance_moderate01/result.json').read_text())['source_sha256'];files=[REPO/n for n in source]+[HERE/'capture.py',HERE/'protocol.json',HERE/'run.sh',dest/'driver.py',parent,BENCH/'amcl_observer.py',REPO/'python/dynamuhsim/runtime/gpu.py',REPO/'python/dynamuhsim/runtime/shutdown.py',LIFE/'motion_prepare.py',LIFE/'pair_recording.py',LIFE/'quiesce.py',BENCH/'depth_proxy_ablation/generated/build_jazzy/libdepth_proxy_filter.so']+[PARENT/n for n in ['namespace_prepare.py','moderate_config.py','config_runtime.py','generated/check_friction']]
 frozen=dict(schema='wm_capture_freeze/1',git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256={str(q.relative_to(REPO)):sha(q) for q in sorted(set(files))},driver_changes='Allowed seed list, own runtime path and experiment description only; same90s budget/controller/start/goal/native worker/filter')
 dump(HERE/'capture_frozen.json',frozen);(HERE/'generated/runtime').mkdir(exist_ok=True)
 sys.path[:0]=[str(PARENT),str(LIFE)];from namespace_prepare import prepare_camera;sys.path.insert(0,str(REPO/'python'));from dynamuhsim.runtime.gpu import gpu_preflight
 pre=HERE/'generated/preflight';pre.mkdir(exist_ok=False);camera=prepare_camera(pre,REPO);expected='14c15412664eb37dd2f0fb2d61a79b09f98722f204f97d97212f0ec814ad3d9f';assert sha(pre/'mecanum_camera.sdf')==expected;check=subprocess.run([str(PARENT/'generated/check_friction'),str(pre/'mecanum_camera.sdf')],capture_output=True,text=True,check=True);friction=json.loads(check.stdout);assert friction['recognized_gz_base_link_frames']==4;gpu=gpu_preflight(pre);dump(HERE/'reports/preflight.json',dict(status='PASS',camera_sha256=expected,native_friction=friction,gpu=gpu,capture_freeze_sha256=sha(HERE/'capture_frozen.json')));verify();print('CAPTURE SOURCE FROZEN; GPU/friction preflight PASS',flush=True)

def audit_capture(episode):
 out=HERE/'generated/runs'/episode['episode_id'];r=json.loads((out/'result.json').read_text());runtime=REPO/r['runtime'];events=[json.loads(v) for v in (out/'events.jsonl').read_text().splitlines()];start=next(v['stamp_ns'] for v in events if v['event']=='goal_accepted');end=next(v['stamp_ns'] for v in events if v['event']=='action_end');tail=next(v['stamp_ns'] for v in events if v['event']=='observation_end');poses=[json.loads(v) for v in (out/'gazebo_poses.jsonl').read_text().splitlines()];xy=np.array([[v['poses']['mecanum_robot'][k] for k in ['x','y']] for v in poses if 'mecanum_robot' in v['poses'] and start/1e9<=v['time']<=end/1e9]);translation=float(np.linalg.norm(xy-xy[0],axis=1).max()) if len(xy) else 0.;path=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()) if len(xy) else 0.;renderer=r['cleanup'].get('ogre_renderer_lines',[]);model=runtime/'mecanum_camera.sdf';native_cam=sha(model)==json.loads((HERE/'reports/preflight.json').read_text())['camera_sha256'];infra=bool(r['status']=='MEASURED' and not r['faults'] and r['gazebo_clean_exit'] and r['cleanup']['status']=='PASS' and not r['cleanup']['remaining'] and any('NVIDIA' in x for x in renderer) and r['recording']['returncode']==0 and r['recording']['metadata_present'] and native_cam and tail-end>=3_000_000_000 and end-start>=6_000_000_000 and all(sha(REPO/n)==h for n,h in r['source_sha256'].items()))
 accepted=infra and translation>=.3;result=dict(episode=episode,infrastructure_pass=infra,moving_episode_accepted=accepted,max_robot_translation_from_start_m=translation,robot_sampled_path_length_m=path,active_sim_seconds=(end-start)/1e9,future_tail_s=(tail-end)/1e9,navigation_status=r['navigation_status'],actual_goal_reached=r['actual_goal_reached'],goal_error_world_m=r['goal_xy_error_world_m'],renderer=renderer,native_camera_and_friction_unchanged=native_cam,raw_result_sha256=sha(out/'result.json'),raw_bag_bytes=sum(f.stat().st_size for f in (out/'raw_bag').iterdir() if f.is_file()),raw_capture_sha256={n:sha(out/n) for n in ['result.json','events.jsonl','raw_bag/metadata.yaml']},retry_count=0)
 dump(out/'capture_audit.json',result);return result

def cohort():
 verify();plan=json.loads((HERE/'protocol.json').read_text());pre=json.loads((HERE/'reports/preflight.json').read_text());assert pre['status']=='PASS' and pre['capture_freeze_sha256']==sha(HERE/'capture_frozen.json');driver=HERE/'reports/implementation/driver.py';env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join([str(PARENT),str(LIFE),str(DATASET),env.get('PYTHONPATH','')]);ledger=[]
 for episode in plan['episodes']:
  verify();name=episode['episode_id'];out=HERE/'generated/runs'/name
  if out.exists():
   assert (out/'capture_audit.json').exists(),'Preserved incomplete attempt; no automatic retry';a=json.loads((out/'capture_audit.json').read_text());assert a['infrastructure_pass'] and a['raw_result_sha256']==sha(out/'result.json');ledger.append(a);continue
  print('CAPTURE START',name,episode['split'],flush=True);logdir=HERE/'generated/console';logdir.mkdir(exist_ok=True)
  with (logdir/(name+'.log')).open('w') as stream:proc=subprocess.run([sys.executable,str(driver),'--scenario',episode['scenario'],'--seed',str(episode['seed']),'--report-dir',str(out)],env=env,stdout=stream,stderr=subprocess.STDOUT)
  if proc.returncode!=0:dump(HERE/'generated/failed_capture.json',dict(episode=episode,returncode=proc.returncode,console=str(logdir/(name+'.log')),no_auto_retry=True));raise RuntimeError('Infrastructure/worker failure retained; stop cohort')
  a=audit_capture(episode);ledger.append(a);dump(HERE/'generated/capture_ledger.json',ledger);print('CAPTURE CLOSED',name,a['navigation_status'],'translation',round(a['max_robot_translation_from_start_m'],3),'accepted',a['moving_episode_accepted'],flush=True);assert a['infrastructure_pass'],'Infrastructure gate failed; stop, preserve failure'
 verify();dump(HERE/'reports/capture_ledger.json',ledger);print('TEN EPISODES CAPTURED',flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','cohort']);a=p.parse_args();prepare() if a.mode=='prepare' else cohort()
