#!/usr/bin/env python3
"""Run/reuse named v2 fixtures serially; append immutable first execution records."""
import json,subprocess,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3];out=HERE/'reports';plan=json.loads((HERE/'protocol.json').read_text());execution=[]
for item in plan['live_plan']:
    report=out/item['report'];cmd=['bash',str(HERE/'run.sh'),'live','--model','soc_guarded_orca','--case',item['case'],'--report-dir',str(report)]
    if item.get('gui'):cmd+=['--gui']
    if item.get('fault'):cmd+=['--fault-after-updates',str(item['after_updates']),'--fault-mode',item['fault']]
    started=time.monotonic();code=None;ran=not report.exists()
    if ran:
        with (HERE/'generated'/(report.name+'_recorder.log')).open('w') as console:process=subprocess.run(cmd,stdout=console,stderr=subprocess.STDOUT,timeout=420);code=process.returncode
    result=json.loads((report/'result.json').read_text())
    audit_cmd=['bash',str(HERE/'run.sh'),'analyze','--live',str(report)]
    with (HERE/'generated'/(report.name+'_analysis.log')).open('w') as console:audit=subprocess.run(audit_cmd,stdout=console,stderr=subprocess.STDOUT,timeout=90)
    row=dict(report=item['report'],case=item['case'],status=result['status'],command=cmd,simulation_reexecuted=ran,live_exit_code=code,audit_exit_code=audit.returncode,invocation_wall_seconds=time.monotonic()-started);execution.append(row)
    (out/'final_audit.json').write_text(json.dumps(execution,indent=2)+'\n')
    first=report/'invocation.json'
    if not first.exists():first.write_text(json.dumps(row,indent=2)+'\n')
    print(json.dumps(row),flush=True)
    if result.get('cleanup',{}).get('remaining') or result.get('container_remaining'):raise RuntimeError('Owned resources remain')
print('DONE: all6 live sessions audited, no retry or evidence overwrite',flush=True)
