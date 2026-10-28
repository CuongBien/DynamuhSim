#!/usr/bin/env python3
"""Reuse frozen offline Runner class; measured Gazebo robot, terminal IPC faults."""
import argparse,ast,copy,hashlib,json,math,os,random,socket,sys,time
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
try:os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
except AttributeError:pass
import numpy as np
variant=globals().get('WORKER_VARIANT','common')
assert variant in ['common','shared','nav2','response']
gated=variant in ['shared','nav2']
p=argparse.ArgumentParser();p.add_argument('--common',type=Path,required=True);p.add_argument('--model',required=True);p.add_argument('--case',required=True);p.add_argument('--runtime',type=Path,required=True);p.add_argument('--socket',required=True)
if variant!='common':
    p.add_argument('--fault-after-updates',type=int);p.add_argument('--fault-mode',choices=['error','disconnect'],default='error')
a=p.parse_args()
common=a.common.resolve();protocol=json.loads((common/'inputs/protocol.json').read_text());model=protocol['models'][a.model];dt=model['dt'];layout=json.loads((common/'inputs/layout.json').read_text());case=json.loads((common/'inputs'/(a.case+'.json')).read_text())
source=common/'model_worker.py';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();locked=json.loads((common/'reports/run01/protocol.json').read_text());expected=next(v for k,v in locked['source_sha256'].items() if k.endswith('/source_snapshot/model_worker.py'));assert sha(source)==expected
for name,digest in protocol['input_sha256'].items():assert sha(common/name)==digest,name
node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Runner')
ns=dict(HERE=common,a=a,protocol=protocol,model=model,dt=dt,layout=layout,sha=sha,Path=Path,np=np,random=random,copy=copy,math=math,sys=sys)
# Bind the frozen Runner's short imports to the live sibling stage.
if a.model.startswith('soc_'):
    sys.path.insert(0,str(common.parent/'socnavgym_warehouse_lifecycle'))
    import kernel,motion  # Populate canonical modules before the frozen Runner adds its legacy path.
exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),ns)
k=ns['Runner'](case)
if variant=='shared':
    k.meta['robot_controller']='Measured Gazebo robot driven by ROS2 Nav2; zero velocity only in warmup'
    k.meta['robot_observation_scope']='Native observation contract retained; see common_response audit'
elif variant in ['nav2','response']:
    sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'socnavgym_response_fix'))
    from responsive_motion import enable
    enable(k)
yaws={h['id']:h['yaw'] for h in case['humans']}
def decorate(humans,turn=False):
    result=copy.deepcopy(humans)
    for h in result:
        hid=h['id'];speed=math.hypot(h['vx'],h['vy'])
        if turn and speed>1e-6:
            delta=math.atan2(math.sin(math.atan2(h['vy'],h['vx'])-yaws[hid]),math.cos(math.atan2(h['vy'],h['vx'])-yaws[hid]));yaws[hid]+=max(-1.57*dt,min(1.57*dt,delta))
        h['yaw']=math.atan2(math.sin(yaws[hid]),math.cos(yaws[hid]));h['speed']=speed
    return result
current=decorate(k.state()['humans']);server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(a.socket);server.listen(1)
ready=dict(humans=current,obstacles=42,native_dt=dt,robot_radius=.25,human_radius=.36,epoch=0,model=a.model,case=a.case,runner_sha256=sha(source),metadata=k.meta)
if variant!='common':
    ready['response_variant']=a.model+'_native_intent_container' if variant=='shared' else 'per_human_support_planes_dykstra_v2'
    if gated:ready['requires_start_gate']=True
    ready.update(fault_after_updates=a.fault_after_updates,fault_mode=a.fault_mode)
(a.runtime/'ready.json').write_text(json.dumps(ready))
try:
    conn,_=server.accept()
    with conn,conn.makefile('rb') as stream,(a.runtime/'worker_trajectory.jsonl').open('w',buffering=1) as log:
        base=previous=None;transport_step=0
        for line in stream:
            start=time.monotonic();request={}
            try:
                request=json.loads(line);t=float(request['time'])
                if request['frame']!='world' or type(request['epoch']) is not int or request['epoch']!=0:raise ValueError('Invalid frame/epoch')
                if not math.isfinite(t) or not math.isfinite(request['dt']) or abs(request['dt']-dt)>1e-8:raise ValueError('Invalid time/dt')
                if previous is not None and abs(t-previous-dt)>1e-6:raise ValueError('Noncontiguous clock; restart required')
                if base is None:base=t
                phase_step=transport_step if gated else k.n
                if abs(t-base-phase_step*dt)>1e-6:raise ValueError('Clock phase mismatch')
                robot=request['robot']
                if any(not math.isfinite(float(robot[key])) for key in ['x','y','yaw','vx','vy','omega']):raise ValueError('Nonfinite robot input')
                if variant!='common' and a.fault_after_updates is not None and k.n>=a.fault_after_updates:
                    if a.fault_mode=='disconnect':log.write(json.dumps(dict(injected_disconnect=True,step=k.n,request=request))+'\n');raise SystemExit(0)
                    raise RuntimeError('INJECTED worker failure for pause interlock')
                before=copy.deepcopy(current);active=(a.runtime/'start_gate').exists() if gated else True;row=k.advance(robot) if active else {}
                if active:current=decorate(row['humans'],True)
                previous=t
                if gated:
                    transport_step+=1
                    reply=dict(time=t,state_time=t+dt,step=transport_step,native_step=k.n,active_phase=active,before=before,humans=current,groups_before=[],groups=[],motion_guard=row.get('guard'),compute_ms=1000*(time.monotonic()-start))
                else:
                    reply=dict(time=t,state_time=t+dt,step=k.n,before=before,humans=current,groups_before=[],groups=[])
                    if variant!='common':reply['motion_guard']=row.get('guard')
                    reply['compute_ms']=1000*(time.monotonic()-start)
                log.write(json.dumps(dict(**reply,robot_input=robot),allow_nan=False)+'\n')
            except Exception as error:
                reply=dict(error=str(error),fatal=True,restart_required=True,step=k.n)
                if variant=='common':log.write(json.dumps(reply,allow_nan=False)+'\n')
                else:
                    safe_request=json.loads(json.dumps(request),parse_constant=lambda value:value);log.write(json.dumps(dict(**reply,request=safe_request,last_safe_humans=current),allow_nan=False)+'\n')
            conn.sendall((json.dumps(reply,allow_nan=False)+'\n').encode())
            if 'error' in reply:break
finally:server.close();k.close()
