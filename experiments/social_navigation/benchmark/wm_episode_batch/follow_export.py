#!/usr/bin/env python3
import json,time
from export import HERE,DATASET,process,dump,sha,verify
import shutil
verify();plan=json.loads((HERE/'protocol.json').read_text());DATASET.mkdir(parents=True,exist_ok=False);(DATASET/'episodes').mkdir();(DATASET/'tools').mkdir();shutil.copy2(HERE/'loader.py',DATASET/'tools/loader.py')
for n in ['protocol.json','export_protocol.json']:shutil.copy2(HERE/n,DATASET/('capture_protocol.json' if n=='protocol.json' else n))
episodes=[]
for e in plan['episodes']:
 raw=HERE/'generated/runs'/e['episode_id'];deadline=time.monotonic()+1500
 while not (raw/'capture_audit.json').exists():
  if (HERE/'generated/failed_capture.json').exists():raise RuntimeError('Capture cohort failed; do not wait or skip')
  if time.monotonic()>deadline:raise RuntimeError('Capture did not close in bounded wait; preserve partial dataset')
  time.sleep(2)
 episodes.append(process(e));dump(DATASET/'dataset.json',dict(schema='wm_sensor_action_dataset/1',episodes=episodes,planned_episodes=10,complete=len(episodes)==10,source_capture_freeze_sha256=sha(HERE/'capture_frozen.json'),source_export_freeze_sha256=sha(HERE/'export_frozen.json'),backend='SocNavGym',split_rule=plan['split_rule'],wm_observed_bev_supervision=True,association_valid=False,persona_valid=False,RL_reward_valid=False,hu_nav_sim_used=False));print('DATASET PROGRESS',len(episodes),'/10',flush=True)
dump(HERE/'reports/dataset_summary.json',json.loads((DATASET/'dataset.json').read_text()));print('TEN EPISODES EXPORTED',flush=True)
