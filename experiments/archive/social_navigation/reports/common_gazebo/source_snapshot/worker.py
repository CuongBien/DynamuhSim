#!/usr/bin/env python3
"""Reuse frozen offline Runner class; measured Gazebo robot, terminal IPC faults."""
import argparse,ast,copy,hashlib,json,math,os,random,socket,sys,time,types
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
try:os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
except AttributeError:pass
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--common',type=Path,required=True);p.add_argument('--model',required=True);p.add_argument('--case',required=True);p.add_argument('--runtime',type=Path,required=True);p.add_argument('--socket',required=True);a=p.parse_args()
common=a.common.resolve();protocol=json.loads((common/'inputs/protocol.json').read_text());model=protocol['models'][a.model];dt=model['dt'];layout=json.loads((common/'inputs/layout.json').read_text());case=json.loads((common/'inputs'/(a.case+'.json')).read_text())
source=common/'model_worker.py';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();locked=json.loads((common/'reports/run01/protocol.json').read_text());expected=next(v for k,v in locked['source_sha256'].items() if k.endswith('/source_snapshot/model_worker.py'));assert sha(source)==expected
for name,digest in protocol['input_sha256'].items():assert sha(common/name)==digest,name
node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Runner')
ns=dict(HERE=common,a=a,protocol=protocol,model=model,dt=dt,layout=layout,sha=sha,Path=Path,np=np,random=random,copy=copy,math=math,sys=sys)
exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),ns)
k=ns['Runner'](case);yaws={h['id']:h['yaw'] for h in case['humans']}
def decorate(humans,turn=False):
    result=copy.deepcopy(humans)
    for h in result:
        hid=h['id'];speed=math.hypot(h['vx'],h['vy'])
        if turn and speed>1e-6:
            delta=math.atan2(math.sin(math.atan2(h['vy'],h['vx'])-yaws[hid]),math.cos(math.atan2(h['vy'],h['vx'])-yaws[hid]));yaws[hid]+=max(-1.57*dt,min(1.57*dt,delta))
        h['yaw']=math.atan2(math.sin(yaws[hid]),math.cos(yaws[hid]));h['speed']=speed
    return result
current=decorate(k.state()['humans']);server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(a.socket);server.listen(1)
(a.runtime/'ready.json').write_text(json.dumps(dict(humans=current,obstacles=42,native_dt=dt,robot_radius=.25,human_radius=.36,epoch=0,model=a.model,case=a.case,runner_sha256=sha(source),metadata=k.meta)))
try:
    conn,_=server.accept()
    with conn,conn.makefile('rb') as stream,(a.runtime/'worker_trajectory.jsonl').open('w',buffering=1) as log:
        base=previous=None
        for line in stream:
            start=time.monotonic();request={}
            try:
                request=json.loads(line);t=float(request['time'])
                if request['frame']!='world' or type(request['epoch']) is not int or request['epoch']!=0:raise ValueError('Invalid frame/epoch')
                if not math.isfinite(t) or not math.isfinite(request['dt']) or abs(request['dt']-dt)>1e-8:raise ValueError('Invalid time/dt')
                if previous is not None and abs(t-previous-dt)>1e-6:raise ValueError('Noncontiguous clock; restart required')
                if base is None:base=t
                if abs(t-base-k.n*dt)>1e-6:raise ValueError('Clock phase mismatch')
                robot=request['robot']
                if any(not math.isfinite(float(robot[key])) for key in ['x','y','yaw','vx','vy','omega']):raise ValueError('Nonfinite robot input')
                before=copy.deepcopy(current);row=k.advance(robot);current=decorate(row['humans'],True);previous=t
                reply=dict(time=t,state_time=t+dt,step=k.n,before=before,humans=current,groups_before=[],groups=[],compute_ms=1000*(time.monotonic()-start))
                log.write(json.dumps(dict(**reply,robot_input=robot),allow_nan=False)+'\n')
            except Exception as error:
                reply=dict(error=str(error),fatal=True,restart_required=True,step=k.n);log.write(json.dumps(reply,allow_nan=False)+'\n')
            conn.sendall((json.dumps(reply,allow_nan=False)+'\n').encode())
            if 'error' in reply:break
finally:server.close();k.close()
