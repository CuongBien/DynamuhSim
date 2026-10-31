#!/usr/bin/env python3
"""Explicitly reviewed continuation; never reattempt a rejected capture."""
import json,os,subprocess,sys
from pathlib import Path
from capture import HERE,BENCH,REPO,PARENT,LIFE,DATASET,verify,sha,dump,audit_capture
review=json.loads((HERE/'reports/quarantine_review01.json').read_text());allowed={v['episode_id']:v for v in review['quarantined']};verify();manifest=json.loads((HERE/'resume_frozen.json').read_text());assert all(sha(REPO/n)==h for n,h in manifest['source_sha256'].items());env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join([str(PARENT),str(LIFE),str(DATASET),env.get('PYTHONPATH','')]);driver=HERE/'reports/implementation/driver.py';plan=json.loads((HERE/'protocol.json').read_text());ledger=[]
for episode in plan['episodes']:
 verify();sid=episode['episode_id'];out=HERE/'generated/runs'/sid
 if out.exists():
  assert (out/'capture_audit.json').exists(),'Incomplete old attempt cannot be retried';audit=json.loads((out/'capture_audit.json').read_text());assert audit['raw_result_sha256']==sha(out/'result.json')
  if not audit['infrastructure_pass']:assert sid in allowed and allowed[sid]['raw_result_sha256']==sha(out/'result.json')
  ledger.append(audit);continue
 print('CAPTURE START',sid,episode['split'],'reviewed continuation',flush=True)
 with (HERE/'generated/console'/(sid+'.log')).open('w') as stream:proc=subprocess.run([sys.executable,str(driver),'--scenario',episode['scenario'],'--seed',str(episode['seed']),'--report-dir',str(out)],env=env,stdout=stream,stderr=subprocess.STDOUT)
 if proc.returncode!=0:dump(HERE/'generated/resume_failure.json',dict(episode_id=sid,returncode=proc.returncode));raise RuntimeError('New failed capture, preserve and stop')
 audit=audit_capture(episode);ledger.append(audit);dump(HERE/'generated/capture_ledger.json',ledger);print('CAPTURE CLOSED',sid,audit['navigation_status'],'translation',round(audit['max_robot_translation_from_start_m'],3),'accepted',audit['moving_episode_accepted'],flush=True);assert audit['infrastructure_pass'],'New infrastructure failure, preserve and stop'
verify();dump(HERE/'reports/capture_ledger.json',ledger);print('TEN PLANNED ATTEMPTS CLOSED; ONE QUARANTINED',flush=True)
