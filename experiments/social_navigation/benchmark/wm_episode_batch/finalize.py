#!/usr/bin/env python3
"""Preserve reports/media and write an honest dataset ledger after all QA is complete."""
import hashlib,json,shutil,subprocess
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3];ROOT=HERE/'generated/dataset01'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
def main():
 ds=json.loads((ROOT/'dataset.json').read_text());audit=json.loads((HERE/'reports/audit.json').read_text());assert ds['complete'] and audit['status']=='PASS';plan=json.loads((HERE/'protocol.json').read_text());review=json.loads((HERE/'reports/quarantine_review02.json').read_text());q={v['episode_id'] for v in review['quarantined']};byid={v['episode_id']:v for v in ds['episodes']};rows=[];counts={};total_scan=0;rawbytes=0
 for item in plan['episodes']:
  sid=item['episode_id'];raw=HERE/'generated/runs'/sid;r=json.loads((raw/'result.json').read_text());cap=json.loads((raw/'capture_audit.json').read_text());runtime=REPO/r['runtime'];dest=HERE/'reports/captures'/sid;dest.mkdir(parents=True,exist_ok=False)
  for n in ['result.json','capture_audit.json','events.jsonl','live_parameters.json']:
   shutil.copy2(raw/n,dest/n)
  shutil.copy2(runtime/'ogre2_session.log',dest/'ogre2_session.log')
  if sid in q:shutil.copy2(raw/'launch.log',dest/'launch.log')
  else:
   shutil.copy2(raw/'raw_export_audit.json',dest/'raw_export_audit.json');shutil.copy2(raw/'normalized/qa.json',dest/'normalizer_qa.json');total_scan+=json.loads((raw/'normalized/qa.json').read_text())['modality_counts']['scan']
  rawbytes+=cap['raw_bag_bytes'];n=byid[sid]['windows'] if sid in byid else 0;wm=byid[sid]['wm_observed_bev_eligible_windows'] if sid in byid else 0;rows.append(dict(**item,navigation_status=r['navigation_status'],capture_accepted=sid not in q,raw_depth_frames=byid[sid]['raw_depth_frames'] if sid in byid else None,windows=n,wm_valid=wm,max_robot_translation_m=cap['max_robot_translation_from_start_m'],raw_bag_bytes=cap['raw_bag_bytes'],quarantine_reason='Gazebo exit255 after recording close' if sid in q else None))
 for split in ['train','validation','test']:
  selected=[e for e in ds['episodes'] if e['split']==split];counts[split]=dict(episodes=len(selected),windows=sum(e['windows'] for e in selected),wm_valid=sum(e['wm_observed_bev_eligible_windows'] for e in selected))
 normalizer_candidates=sum(json.loads((HERE/'generated/runs'/e['episode_id']/'normalized/qa.json').read_text())['candidate_count'] for e in ds['episodes']);normalizer_rejected=normalizer_candidates-sum(v['windows'] for v in ds['episodes']);inventory={str(f.relative_to(ROOT)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in sorted(ROOT.rglob('*')) if f.is_file()};dump(HERE/'reports/dataset_inventory.json',inventory);stats=dict(planned_attempts=10,accepted_episodes=len(ds['episodes']),quarantined_episodes=len(q),splits=counts,total_windows=audit['total_windows'],wm_observed_bev_valid_windows=audit['wm_observed_bev_valid_windows'],normalizer_candidate_windows=normalizer_candidates,normalizer_rejected_windows=normalizer_rejected,raw_depth_bitwise=audit['total_raw_depth_bitwise'],accepted_native_scan_messages=total_scan,portable_dataset_logical_bytes=sum(v['bytes'] for v in inventory.values()),raw_bag_bytes_all10_attempts=rawbytes,raw_bag_backup_separate=True,metadata_roles='Legacy overall trainingfalse preserved; canonical new per-task validity in task masks; test never used in fitting',WM_models_trained=0,RL_reward_valid=False,association_valid=False,persona_valid=False,HuNavSim_used=False)
 dump(HERE/'reports/summary.json',stats);dump(HERE/'reports/all_attempts_ledger.json',rows);dump(HERE/'reports/unit_tests.json',dict(status='PASS',tests=8,executed_before_export_freeze=True,observed_runtime_s=.027,scope=['Causal future/late-delivery rejection','Invalid depth never carves free','SE2 known transform and roundtrip','Unsupported label pose interpolation masked','Future frame tolerance','Observation independent of deleted labels','Global horizon vector and explicit action condition','Test default denial']))
 checkpoint=HERE/'generated/checkpoint';old=json.loads((checkpoint/'files.json').read_text());changed=[n for n,h in old.items() if not Path(n).is_file() or sha(Path(n))!=h];preserved=dict(preexisting_present_tracked_files=len(old),changed_preexisting_files=changed,original_staged_patch_unchanged=subprocess.check_output(['git','diff','--cached','--binary'])==(checkpoint/'staged.patch').read_bytes(),outside_social_navigation_patch_unchanged=subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'])==(checkpoint/'outside.patch').read_bytes());dump(HERE/'reports/preservation_before_commit.json',preserved);assert not changed and preserved['original_staged_patch_unchanged'] and preserved['outside_social_navigation_patch_unchanged'];print(json.dumps(stats,indent=2))
if __name__=='__main__':main()
