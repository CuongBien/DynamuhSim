#!/usr/bin/env python3
"""Private IPC worker: fault is terminal, current session cannot resume."""
import argparse,copy,json,math,os,socket,sys,time
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
sys.path.insert(0,'/geometry')
from kernel import GeometryKernel
p=argparse.ArgumentParser();p.add_argument('--runtime',type=Path,required=True);p.add_argument('--socket',required=True);p.add_argument('--fault-after-updates',type=int);a=p.parse_args()
fixture=json.loads(Path('/work/fixture.json').read_text());layout=json.loads(Path('/geometry/generated/layout.json').read_text())
k=GeometryKernel(layout,fixture['humans']);dt=fixture['dt'];yaws={h['id']:math.atan2(h['gy']-h['y'],h['gx']-h['x']) for h in fixture['humans']}
def decorated(humans,turn=False):
    result=copy.deepcopy(humans)
    for h in result:
        speed=math.hypot(h['vx'],h['vy']);hid=h['id']
        if turn and speed>1e-6:
            target=math.atan2(h['vy'],h['vx']);delta=math.atan2(math.sin(target-yaws[hid]),math.cos(target-yaws[hid]))
            yaws[hid]+=max(-fixture['max_yaw_rate']*dt,min(fixture['max_yaw_rate']*dt,delta))
        h['yaw']=math.atan2(math.sin(yaws[hid]),math.cos(yaws[hid]));h['speed']=speed
    return result
current=decorated(k.state()['humans']);server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(a.socket);server.listen(1)
(a.runtime/'ready.json').write_text(json.dumps(dict(humans=current,obstacles=len(layout['obstacles']),native_dt=dt,robot_radius=.25,human_radius=.3,epoch=k.epoch,framework='ComplexityNav',variant='roadmap+native_intent+ORCA_projection',fault_after_updates=a.fault_after_updates)))
try:
    connection,_=server.accept()
    with connection,connection.makefile('rb') as stream,(a.runtime/'worker_trajectory.jsonl').open('w',buffering=1) as log:
        base=None;previous=None
        for line in stream:
            started=time.monotonic();request={}
            try:
                request=json.loads(line);t=float(request['time'])
                if request['frame']!='world' or type(request['epoch']) is not int or request['epoch']!=k.epoch:raise ValueError('Invalid frame/reset epoch; fresh session required')
                if not math.isfinite(t) or not math.isfinite(request['dt']) or abs(request['dt']-dt)>1e-8:raise ValueError('Invalid simulation time/planning dt')
                if previous is not None and abs(t-previous-dt)>1e-6:raise ValueError('Reset/noncontiguous clock requires fresh session')
                if base is None:base=t
                if abs(t-base-k.time)>1e-6:raise ValueError('Native/Gazebo clock phase mismatch')
                if a.fault_after_updates is not None and k.step>=a.fault_after_updates:raise RuntimeError('INJECTED worker failure for freeze contract')
                before=copy.deepcopy(current)
                row=k.request(dict(frame='world',epoch=k.epoch,time=k.time,advance=True,robot=request['robot']))
                current=decorated(row['humans'],turn=True);previous=t
                reply=dict(time=t,state_time=t+dt,step=k.step,before=before,humans=current,groups_before=[],groups=[],guard=row['geometry'],compute_ms=1000*(time.monotonic()-started))
                log.write(json.dumps(dict(**reply,robot_input=request['robot']),allow_nan=False)+'\n')
            except Exception as ex:
                reply=dict(error=str(ex),fatal=True,restart_required=True,step=k.step)
                log.write(json.dumps(dict(**reply,request=json.loads(json.dumps(request),parse_constant=lambda value:value),last_safe_endpoint=current),allow_nan=False)+'\n')
            connection.sendall((json.dumps(reply,allow_nan=False)+'\n').encode())
            if 'error' in reply:break
finally:server.close()
