#!/usr/bin/env python3
import json,subprocess,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
assert not (HERE/'reports/v2_execution.json').exists(),'Recorded cohort already exists; use a newly planned output directory'
receipts=[]
for trial in json.loads((HERE/'protocol.json').read_text())['plan']:
    out=HERE/'reports'/trial['name'];cmd=['bash',str(HERE/'run.sh'),'live','--scenario',trial['scenario'],'--seed',str(trial['seed']),'--report-dir',str(out)]
    if trial['gui']:cmd.append('--gui')
    started=time.time()
    with (HERE/'generated'/(trial['name']+'_execution.log')).open('w') as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
    receipt=dict(name=trial['name'],command=cmd,returncode=r.returncode,wall_seconds=time.time()-started)
    receipts.append(receipt);(HERE/'reports/v2_execution.json').write_text(json.dumps(receipts,indent=2)+'\n');print(json.dumps(receipt),flush=True)
    if r.returncode:raise SystemExit('Infrastructure failure; remaining fixed trials not run')
    cmd=['bash',str(HERE/'run.sh'),'analyze','--live',str(out)]
    with (HERE/'generated'/(trial['name']+'_audit.log')).open('w') as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
    receipt['audit_returncode']=r.returncode;(HERE/'reports/v2_execution.json').write_text(json.dumps(receipts,indent=2)+'\n')
    if r.returncode:raise SystemExit('Audit failure; inspect before continuing')
    v=json.loads((out/'validation.json').read_text());print(json.dumps(dict(name=trial['name'],actual_reached=v['actual_goal_reached'],status=v['navigation_status'],gaps=v['actual_swept_human_gaps_m'],moving_group=v['groups']['moving_travel_covered'],standing_group=v['groups']['standing_duration_covered'])),flush=True)
