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
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'python'))
from dynamuhsim.simulation.encounters import load_fixture,group_goals
encounter=load_fixture(k,a.runtime)
from dynamuhsim.simulation.conversation import load_conversation
conversation=load_conversation(k,a.runtime)
from dynamuhsim.simulation.human_motion_profiles import load_profile
human_motion=load_profile(k,a.runtime)
from dynamuhsim.simulation.goal_clock import load_goal_clock
goal_clock=load_goal_clock(k,a.runtime)
current=copy.deepcopy(k.humans());server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(a.socket);server.listen(1)
from dynamuhsim.runtime.worker_readiness import publish_ready
ready=dict(humans=current,groups=k.groups(),obstacles=42,native_dt=dt,robot_radius=.25,human_radius=.36,epoch=0,scenario=a.scenario,seed=a.seed,response_variant='native_initial_groups_continuous_lifecycle_response_v2',reactive_standing=a.scenario=='standing',standing_speed_cap_mps=.2,activation_center_distance_m=1.4,requires_start_gate=True)
if human_motion is not None:ready['human_motion_profile']=human_motion.profile
if encounter is not None:ready['encounter']=encounter
if goal_clock is not None:ready['goal_clock']=goal_clock.snapshot()
if conversation is not None:ready['human_lifecycle']=conversation.snapshot()
publish_ready(a.runtime,ready)
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
                if active:
                    if goal_clock is not None:goal_clock.begin_step(t,float((a.runtime/'start_gate').read_text()))
                    k.advance(robot);current=copy.deepcopy(k.humans())
                    if goal_clock is not None:goal_clock.end_step(t+dt)
                previous=t;transport_step+=1
                reply=dict(time=t,state_time=t+dt,step=transport_step,native_step=k.steps,active_phase=active,before=before,humans=current,groups_before=groups_before,groups=k.groups(),finish_events=k.motion.events[events:],motion_guard=k.motion.steps[-1] if active else None,compute_ms=1000*(time.monotonic()-start))
                if human_motion is not None:reply["group_orca" if human_motion.profile=="group_orca_v1" else "group_intent"]=human_motion.last
                if human_motion is not None and hasattr(human_motion,"cohesion_last"):reply["group_cohesion"]=human_motion.cohesion_last
                if encounter is not None:
                    reply["encounter_goals"]=group_goals(k)
                    reply["encounter_goal_ages_wall_s"]={str(h.id):time.time()-h.initial_time for g in k.env.moving_interactions for h in g.humans}
                if goal_clock is not None:reply["goal_clock"]=goal_clock.snapshot()
                if conversation is not None:reply["human_lifecycle"]=conversation.snapshot()
                log.write(json.dumps(dict(**reply,robot_input=robot),allow_nan=False)+'\n')
            except Exception as error:
                reply=dict(error=str(error),fatal=True,restart_required=True,step=k.steps);safe_request=json.loads(json.dumps(request),parse_constant=lambda value:value);log.write(json.dumps(dict(**reply,request=safe_request,last_safe_humans=current),allow_nan=False)+'\n')
            conn.sendall((json.dumps(reply,allow_nan=False)+'\n').encode())
            if 'error' in reply:break
finally:server.close();k.close()
