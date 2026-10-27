#!/usr/bin/env python3
"""Reuse frozen offline Runner class; measured Gazebo robot, terminal IPC faults."""
import argparse,ast,copy,hashlib,json,math,os,random,socket,sys,time,types
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
try:os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
except AttributeError:pass
import numpy as np
from fixture import make_kernel
p=argparse.ArgumentParser();p.add_argument('--scenario',choices=['walking','standing'],required=True);p.add_argument('--seed',type=int,required=True);p.add_argument('--runtime',type=Path,required=True);p.add_argument('--socket',required=True);a=p.parse_args()
k=make_kernel(a.scenario,a.seed);dt=.1
current=copy.deepcopy(k.humans());server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(a.socket);server.listen(1)
(a.runtime/'ready.json').write_text(json.dumps(dict(humans=current,groups=k.groups(),obstacles=42,native_dt=dt,robot_radius=.25,human_radius=.36,epoch=0,scenario=a.scenario,seed=a.seed,response_variant='native_initial_groups_continuous_lifecycle_response_v2',reactive_standing=a.scenario=='standing',standing_speed_cap_mps=.2,activation_center_distance_m=1.4,requires_start_gate=True)))
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
                if abs(t-base-transport_step*dt)>1e-6:raise ValueError('Clock phase mismatch')
                robot=request['robot']
                if any(not math.isfinite(float(robot[key])) for key in ['x','y','yaw','vx','vy','omega']):raise ValueError('Nonfinite robot input')
                before=copy.deepcopy(current);groups_before=copy.deepcopy(k.groups());events=len(k.motion.events);active=(a.runtime/'start_gate').exists()
                if active:k.advance(robot);current=copy.deepcopy(k.humans())
                previous=t;transport_step+=1
                reply=dict(time=t,state_time=t+dt,step=transport_step,native_step=k.steps,active_phase=active,before=before,humans=current,groups_before=groups_before,groups=k.groups(),finish_events=k.motion.events[events:],motion_guard=k.motion.steps[-1] if active else None,compute_ms=1000*(time.monotonic()-start))
                log.write(json.dumps(dict(**reply,robot_input=robot),allow_nan=False)+'\n')
            except Exception as error:
                reply=dict(error=str(error),fatal=True,restart_required=True,step=k.steps);safe_request=json.loads(json.dumps(request),parse_constant=lambda value:value);log.write(json.dumps(dict(**reply,request=safe_request,last_safe_humans=current),allow_nan=False)+'\n')
            conn.sendall((json.dumps(reply,allow_nan=False)+'\n').encode())
            if 'error' in reply:break
finally:server.close();k.close()
