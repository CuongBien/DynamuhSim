#!/usr/bin/env python3
"""Lifecycle repair experiment around full native Gymnasium step with explicit adapter variant."""
import argparse,hashlib,json,math,os,random,sys,time,traceback
from pathlib import Path
os.environ.setdefault('SDL_VIDEODRIVER','dummy');os.environ.setdefault('SDL_AUDIODRIVER','dummy')
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
if hasattr(os,'sched_getaffinity'):os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
import cv2,gymnasium as gym,numpy as np,socnavgym
from PIL import Image
p=argparse.ArgumentParser();p.add_argument('--variant',choices=['fixed','native'],default='fixed');p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seed',type=int,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
random.seed(a.seed);np.random.seed(a.seed)
env=gym.make('SocNavGym-v2',config=str(a.config));env.action_space.seed(a.seed);e=env.unwrapped
from continuous_groups import ContinuousGroups
repair=ContinuousGroups(e) if a.variant=='fixed' else None
rows=[];calls=[];events=[];frames=[];issues=[];step=0;epoch=0;group_refs=[];human_refs={};seen={};expected=None

def active_groups():
    return [('moving',g) for g in e.moving_interactions]+[('static',g) for g in e.static_interactions]

def snapshot():
    global expected
    owners=[('independent_static',h) for h in e.static_humans]+[('independent_dynamic',h) for h in e.dynamic_humans]
    groups=[]
    for kind,g in active_groups():
        if not any(g is old for old in group_refs):group_refs.append(g)
        gid=next(i+1 for i,old in enumerate(group_refs) if g is old)
        members=[int(h.id) for h in g.humans]
        groups.append(dict(observer_group_id=gid,kind=kind,members=sorted(members),x=float(g.x),y=float(g.y)))
        owners.extend((f'{kind}_group_{gid}',h) for h in g.humans)
    ids=[int(h.id) for _,h in owners]
    assert len(ids)==len(set(ids))==12,ids
    if expected is None:expected=set(ids)
    assert set(ids)==expected,ids
    humans=[]
    for owner,h in owners:
        hid=int(h.id)
        if hid not in human_refs:human_refs[hid]=h
        assert human_refs[hid] is h,('Human object replaced within epoch',hid)
        data=dict(id=hid,x=float(h.x),y=float(h.y),yaw=float(h.orientation),speed=float(h.speed),owner=owner)
        assert all(math.isfinite(data[k]) for k in ['x','y','yaw','speed'])
        humans.append(data)
    humans.sort(key=lambda h:h['id'])
    forming=bool(e.crowd_forming)
    pending=sorted(int(i) for i in e.humans_forming_crowd) if forming else []
    assert set(pending)<=expected
    if forming:assert set(pending)<=set(int(h.id) for h in e.dynamic_humans)
    return dict(epoch=epoch,step=step,time=step*e.TIMESTEP,humans=humans,groups=sorted(groups,key=lambda g:g['observer_group_id']),
                forming=forming,pending_members=pending,
                robot=dict(x=float(e.robot.x),y=float(e.robot.y),yaw=float(e.robot.orientation)))

# Read-only wrappers retain the same bound native methods and RNG sequence.
for method in ['form_human_crowd','finish_human_crowd_formation','disperse_static_crowd','disperse_moving_crowd']:
    original=getattr(e,method)
    def trace(*args,_method=method,_original=original,**kwargs):
        before=snapshot();value=_original(*args,**kwargs);after=snapshot()
        jumps=[dict(id=h['id'],position_delta_m=math.hypot(h['x']-before['humans'][i]['x'],h['y']-before['humans'][i]['y']),yaw_delta_rad=math.atan2(math.sin(h['yaw']-before['humans'][i]['yaw']),math.cos(h['yaw']-before['humans'][i]['yaw']))) for i,h in enumerate(after['humans'])]
        calls.append(dict(epoch=epoch,step=step,native_method=_method,args=list(args),kwargs=kwargs,before=before,after=after,deltas=jumps))
        return value
    setattr(e,method,trace)

def signature(s):return [(g['observer_group_id'],g['kind'],g['members']) for g in s['groups']]

def image(s,path=None):
    frame=e.render_without_showing('human',draw_human_gaze=True,draw_human_goal=False).copy()
    cv2.putText(frame,('ADAPTER FIX' if repair else 'NATIVE')+' | SOCNAVGYM 2D | NOT GAZEBO',(15,40),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),2)
    cv2.putText(frame,f'seed {a.seed} | t={s["time"]:.1f}s | groups {len(s["groups"])}',(15,67),cv2.FONT_HERSHEY_SIMPLEX,.60,(0,0,0),1)
    if path:assert cv2.imwrite(str(path),frame)
    return frame

result=dict(variant=a.variant,seed=a.seed,status='ERROR',scope='Standalone14x14m; stationary robot; native lifecycle plus explicitly modified continuous/disc-safe adapter when variant=fixed; no ROS/Gazebo',observer_group_ids='External monotonic labels assigned to native group object identity within one reset epoch; not upstream persistent group IDs')
started=time.monotonic()
try:
    obs,_=env.reset(seed=a.seed);initial=snapshot();rows.append(initial)
    result['initial_observation_in_space']=bool(env.observation_space.contains(obs));invalid=0
    if a.seed==42:frames.append(image(initial))
    for step in range(1,1201):
        before=rows[-1];obs,reward,terminated,truncated,info=env.step(np.zeros(3,dtype=np.float32));after=snapshot()
        invalid+=int(not env.observation_space.contains(obs))
        after.update(terminated=bool(terminated),truncated=bool(truncated));rows.append(after)
        if signature(before)!=signature(after):
            events.append(dict(step=step,time=step*e.TIMESTEP,before=signature(before),after=signature(after)))
            if len(events)<=4:image(after,a.output/f'event_{step:04d}.png')
        if a.seed==42 and step%10==0:frames.append(image(after))
        if terminated or truncated:
            assert step==1200,('Unexpected early native episode end',step)
    result['steps']=1200;result['observation_outside_space_steps']=invalid
    result['events']=events;result['native_call_count']=len(calls)
    # Same live environment reset after membership changes; compare fresh prefix.
    if a.seed==42:
        old_refs=human_refs.copy();pre_reset=rows[-1];epoch=1;step=0;group_refs=[];human_refs={};expected=None
        obs,_=env.reset(seed=a.seed);reset_initial=snapshot()
        assert not reset_initial['groups'] and not reset_initial['forming']
        assert all(human_refs[i] is not old_refs[i] for i in human_refs)
        initial_diff=max(abs(reset_initial['humans'][i][k]-initial['humans'][i][k]) for i in range(12) for k in ['x','y','yaw','speed'])
        repeat=[];max_error=initial_diff;membership_match=True
        for step in range(1,501):
            obs,*_=env.step(np.zeros(3,dtype=np.float32));state=snapshot();repeat.append(state)
            reference=rows[step]
            max_error=max(max_error,max(abs(state['humans'][i][k]-reference['humans'][i][k]) for i in range(12) for k in ['x','y','yaw','speed']))
            membership_match=membership_match and signature(state)==signature(reference) and state['pending_members']==reference['pending_members']
        result['reset_check']=dict(steps=500,max_state_error=max_error,membership_match=membership_match,groups_before=pre_reset['groups'],groups_after=reset_initial['groups'],same_numeric_human_ids=True,new_human_objects=True,namespace='Use (reset_epoch,human_id); numeric IDs are reused by native reset')
        (a.output/'reset_repeat.json').write_text(json.dumps(dict(initial=reset_initial,rows=repeat)))
        assert max_error<1e-9 and membership_match,'Reset replay mismatch retained for diagnosis'
    result['status']='PASS'
except Exception as error:
    result['error']=str(error);traceback.print_exc()
finally:
    if repair:
        (a.output/'repair_events.json').write_text(json.dumps(repair.events))
        (a.output/'repair_steps.json').write_text(json.dumps(repair.steps))
        result['repair_effective_robot_radius']=float(e.robot.radius)
    result['wall_seconds']=time.monotonic()-started;result['recorded_steps']=len(rows)-1
    result['events']=events
    (a.output/'trajectory.json').write_text(json.dumps(rows));(a.output/'native_calls.json').write_text(json.dumps(calls))
    if frames:
        preview=[Image.fromarray(cv2.cvtColor(f,cv2.COLOR_BGR2RGB)).quantize(colors=96) for f in frames[:-1]]
        preview[0].save(a.output/'native_preview_4x.gif',save_all=True,append_images=preview[1:],duration=250,loop=0)
        result['preview']=dict(type='Recorded SocNavGym2D render with adapter variant label, not Gazebo',source_interval_sim_s=1.,playback_speed=4,gif_duration_ms=30000)
    (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    env.close()
print(json.dumps(result),flush=True)
raise SystemExit(0 if result['status']=='PASS' else 1)
