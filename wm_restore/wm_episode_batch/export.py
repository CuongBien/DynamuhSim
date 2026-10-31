#!/usr/bin/env python3
"""Closed raw MCAP -> portable WM observations/actions/labels, with frozen export math."""
import argparse,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
import numpy as np
from targets import run as target_run
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=HERE.parents[3];DATASET=HERE/'generated/dataset01'
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
def verify():
 frozen=json.loads((HERE/'export_frozen.json').read_text());assert all(sha(REPO/n)==h for n,h in frozen['source_sha256'].items()),'Export source changed'
def freeze():
 files=[HERE/n for n in ['export.py','targets.py','loader.py','test.py','export_protocol.json','protocol.json','capture_frozen.json','follow_export.py']]+list((HERE/'tools/legacy').glob('*'))+[BENCH/'perception_target_pilot/geometry.py',BENCH/'perception_target_pilot/sensor.py'];assert not (HERE/'export_frozen.json').exists();dump(HERE/'export_frozen.json',dict(schema='wm_export_freeze/1',stage='Before first normalization/target build',source_sha256={str(f.relative_to(REPO)):sha(f) for f in sorted(files) if f.is_file()}));print('EXPORT SOURCE FROZEN')
def snapshot(raw,runtime):
 dest=raw/'config_snapshot/runtime';dest.mkdir(parents=True,exist_ok=False)
 for name in ['warehouse.sdf','nav2_runtime.yaml','mecanum_camera.sdf','mecanum_camera.launch.py','ready.json','rendering.json','ogre2_session.log','quiesce_shutdown.json']:
  shutil.copy2(runtime/name,dest/name)
 for name in ['live_parameters.json','config_changes.json']:shutil.copy2(raw/name,dest.parent/name)
 shutil.copy2(HERE/'reports/implementation/driver.py',dest.parent/'actual_capture_driver.py')
def process(episode):
 verify();sid=episode['episode_id'];raw=HERE/'generated/runs'/sid;capture=json.loads((raw/'capture_audit.json').read_text());assert capture['infrastructure_pass'];assert capture['raw_result_sha256']==sha(raw/'result.json');result=json.loads((raw/'result.json').read_text());runtime=REPO/result['runtime'];assert sha(runtime/'warehouse.sdf')==result['runtime_world_sha256'] and sha(runtime/'nav2_runtime.yaml')==result['runtime_nav_params_sha256'];snapshot(raw,runtime);norm=raw/'normalized';logs=HERE/'generated/export_logs';logs.mkdir(exist_ok=True)
 with (logs/(sid+'_build.log')).open('w') as stream:subprocess.run([sys.executable,str(HERE/'tools/legacy/build.py'),'--episode',str(raw),'--output',str(norm)],stdout=stream,stderr=subprocess.STDOUT,check=True)
 with (logs/(sid+'_raw_audit.log')).open('w') as stream:subprocess.run([sys.executable,str(HERE/'tools/legacy/audit.py'),'--episode',str(raw),'--normalized',str(norm),'--report',str(raw/'raw_export_audit.json')],stdout=stream,stderr=subprocess.STDOUT,check=True)
 ep=DATASET/'episodes'/sid;inp=ep/'inputs';lab=ep/'labels';inp.mkdir(parents=True,exist_ok=False);lab.mkdir();(ep/'provenance').mkdir()
 # Plain hardlinks avoid an extra large depth copy; copying the dataset elsewhere remains self-contained.
 mapping={'samples.npz':'samples_SENSOR_ONLY.npz','observations.npz':'observations_SENSOR_ONLY.npz','depth.npy':'depth_SENSOR_ONLY.npy','calibration.json':'calibration_SENSOR_ONLY.json','lidar_geometry.json':'lidar_geometry_SENSOR_ONLY.json'}
 for src,dest in mapping.items():os.link(norm/src,inp/dest)
 os.link(norm/'labels.npz',lab/'future_trajectories_actions_LABEL_ONLY.npz');os.link(norm/'human_states.jsonl',lab/'human_states_LABEL_ONLY.jsonl');stats=target_run(norm,ep,episode,capture);ending=dict(navigation_status=result['navigation_status'],actual_goal_reached=result['actual_goal_reached'],terminated=bool(result['actual_goal_reached']),truncated=not bool(result['actual_goal_reached']),time_limit=result['navigation_status']=='TIMEOUT',reason='task_success' if result['actual_goal_reached'] else result['navigation_status'].lower(),RL_reward_valid=False,contact_ground_truth_valid=False,action_end_sim=result['action_end_sim'],note='Episode collection boundary; RL reward/contact semantics and step/reset interface not validated')
 dump(lab/'episode_end_LABEL_ONLY.json',ending)
 for src,dest in [(raw/'result.json','capture_result.json'),(raw/'capture_audit.json','capture_audit.json'),(raw/'raw_export_audit.json','raw_export_audit.json'),(norm/'manifest.json','legacy_normalizer_manifest.json'),(norm/'qa.json','legacy_normalizer_qa.json')]:shutil.copy2(src,ep/'provenance'/dest)
 summary=dict(**episode,**stats,navigation_status=result['navigation_status'],actual_goal_reached=result['actual_goal_reached'],max_robot_translation_from_start_m=capture['max_robot_translation_from_start_m'],raw_depth_frames=json.loads((raw/'raw_export_audit.json').read_text())['depth_frames_bitwise_equal'],raw_result_sha256=sha(raw/'result.json'),capture_accepted=capture['moving_episode_accepted'],relative_path='episodes/'+sid,raw_path=str(raw.relative_to(REPO)),target_scope='Observed future BEV only; individual semantic/association/persona and RL reward masksfalse')
 dump(ep/'episode.json',summary);dump(ep/'inventory.json',{str(f.relative_to(ep)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in sorted(ep.rglob('*')) if f.is_file()});verify();print('EXPORTED',sid,'windows',stats['windows'],'WM valid',stats['wm_observed_bev_eligible_windows'],flush=True);return summary

def run():
 verify();plan=json.loads((HERE/'protocol.json').read_text());DATASET.mkdir(parents=True,exist_ok=True);(DATASET/'episodes').mkdir(exist_ok=True);(DATASET/'tools').mkdir(exist_ok=True);shutil.copy2(HERE/'loader.py',DATASET/'tools/loader.py');shutil.copy2(HERE/'protocol.json',DATASET/'capture_protocol.json');shutil.copy2(HERE/'export_protocol.json',DATASET/'export_protocol.json');episodes=[]
 for e in plan['episodes']:
  ep=DATASET/'episodes'/e['episode_id']
  if ep.exists():
   assert (ep/'inventory.json').exists(),'Preserve incomplete export; no overwrite';summary=json.loads((ep/'episode.json').read_text())
  else:
   assert (HERE/'generated/runs'/e['episode_id']/'capture_audit.json').exists(),'Capture not closed yet';summary=process(e)
  episodes.append(summary);dump(DATASET/'dataset.json',dict(schema='wm_sensor_action_dataset/1',episodes=episodes,planned_episodes=10,complete=len(episodes)==10,source_capture_freeze_sha256=sha(HERE/'capture_frozen.json'),source_export_freeze_sha256=sha(HERE/'export_frozen.json'),backend='SocNavGym',test_policy=plan['split_rule'],wm_observed_bev_supervision=True,association_valid=False,persona_valid=False,RL_reward_valid=False,hu_nav_sim_used=False))
 dump(HERE/'reports/dataset_summary.json',json.loads((DATASET/'dataset.json').read_text()));print('TEN EPISODES EXPORTED',flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--freeze',action='store_true');a=p.parse_args();freeze() if a.freeze else run()
