#!/usr/bin/env python3
"""Sequential9-case cohort; preserve errors and never retry an existing output."""
import json,subprocess,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
plan=json.loads((HERE/'protocol.json').read_text());names={'soc_guarded_orca':'soc','crowd_orca_plus':'crowd','complex_geometry_mixed':'complex'};scenes={'crossing_near':'crossing','opposed_near':'opposed','blocked_narrow_near':'narrow'}
execution=[]
for case in plan['cases']:
    for model in plan['models']:
        report=HERE/'reports'/(names[model]+'_'+scenes[case]+'01');start=time.monotonic();cmd=['bash',str(HERE/'run.sh'),'live','--model',model,'--case',case,'--report-dir',str(report)]
        if not report.exists():
            with (HERE/'generated'/(report.name+'_recorder.log')).open('w') as console:response=subprocess.run(cmd,stdout=console,stderr=subprocess.STDOUT,timeout=420)
            code=response.returncode
        else:code=None
        result=json.loads((report/'result.json').read_text()) if (report/'result.json').exists() else dict(status='MISSING')
        audit_cmd=['bash',str(HERE/'run.sh'),'analyze','--live',str(report)]
        with (HERE/'generated'/(report.name+'_analysis.log')).open('w') as log:audit=subprocess.run(audit_cmd,stdout=log,stderr=subprocess.STDOUT,timeout=90)
        row=dict(model=model,case=case,report=str(report.relative_to(REPO)),status=result['status'],error=result.get('error'),live_exit_code=code,simulation_reexecuted=code is not None,audit_exit_code=audit.returncode,invocation_wall_seconds=time.monotonic()-start,command=cmd)
        execution.append(row);(HERE/'reports/execution.json').write_text(json.dumps(execution,indent=2)+'\n');print(json.dumps(row),flush=True)
        if result.get('cleanup',{}).get('remaining') or result.get('container_remaining'):raise RuntimeError('Owned worker/process remains; stop cohort')
print('DONE:9 planned cases attempted; every error retained',flush=True)
