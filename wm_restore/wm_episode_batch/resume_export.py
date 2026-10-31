#!/usr/bin/env python3
"""Resume accepted exports; failed raw remains quarantined, source math unchanged."""
import json,time,shutil
from export import HERE,DATASET,process,dump,sha,verify
from capture import REPO
verify();review=json.loads((HERE/'reports/quarantine_review01.json').read_text());q={v['episode_id']:v for v in review['quarantined']};plan=json.loads((HERE/'protocol.json').read_text());episodes=[]
for e in plan['episodes']:
 sid=e['episode_id'];raw=HERE/'generated/runs'/sid
 if sid in q:assert sha(raw/'result.json')==q[sid]['raw_result_sha256'];continue
 ep=DATASET/'episodes'/sid
 if ep.exists():
  assert (ep/'inventory.json').exists();episodes.append(json.loads((ep/'episode.json').read_text()));continue
 deadline=time.monotonic()+1500
 while not (raw/'capture_audit.json').exists():
  if (HERE/'generated/resume_failure.json').exists():raise RuntimeError('Capture continuation failed')
  if time.monotonic()>deadline:raise RuntimeError('Capture wait timed out')
  time.sleep(2)
 episodes.append(process(e));dump(DATASET/'dataset.json',dict(schema='wm_sensor_action_dataset/1',episodes=episodes,planned_episodes=10,accepted_episodes=len(episodes),quarantined_episodes=review['quarantined'],complete=len(episodes)+len(q)==10,source_capture_freeze_sha256=sha(HERE/'capture_frozen.json'),source_export_freeze_sha256=sha(HERE/'export_frozen.json'),review_amendment_sha256=sha(HERE/'reports/quarantine_review01.json'),backend='SocNavGym',split_rule=plan['split_rule'],wm_observed_bev_supervision=True,association_valid=False,persona_valid=False,RL_reward_valid=False,hu_nav_sim_used=False));print('DATASET PROGRESS',len(episodes),'/9 accepted;1quarantined',flush=True)
shutil.copy2(HERE/'reports/quarantine_review01.json',DATASET/'quarantine_review.json');dump(HERE/'reports/dataset_summary.json',json.loads((DATASET/'dataset.json').read_text()));print('NINE ACCEPTED EPISODES EXPORTED; ONE RAW QUARANTINED',flush=True)
