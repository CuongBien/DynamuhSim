#!/usr/bin/env python3
"""Same six locked fixtures, with opt-in response projection and all outcomes saved."""
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
import copy,hashlib,json,time,traceback
from backend import create,COMMON,HERE
out=HERE/'reports/offline01';out.mkdir(exist_ok=False);protocol=json.loads((COMMON/'inputs/protocol.json').read_text());results=[]
(out/'source_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'projection.py',HERE/'responsive_motion.py',HERE/'backend.py',Path(__file__)]},indent=2)+'\n')
for name in protocol['fixtures']:
    case=json.loads((COMMON/'inputs'/(name+'.json')).read_text());runner=create(case);rows=[runner.state()];error=None;started=time.monotonic()
    try:
        for i in range(100):
            robot=copy.deepcopy(case['robot_trace'][i*2]);robot.pop('time');robot['omega']=0.;rows.append(runner.advance(robot))
        status='COMPLETED'
    except Exception as exc:status='ERROR';error=str(exc);(out/(name+'_error.txt')).write_text(traceback.format_exc())
    finally:
        (out/(name+'_trajectory.json')).write_text(json.dumps(rows,allow_nan=False)+'\n');row=dict(case=name,status=status,error=error,steps=len(rows)-1,wall_s=time.monotonic()-started);results.append(row);print(json.dumps(row),flush=True);runner.close()
(out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
