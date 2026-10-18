#!/usr/bin/env python3
"""Validate numerical bridge parity, then observe the native throat fixture."""
import argparse,importlib.util,json,math
from pathlib import Path
from kernel import CrowdKernel
from passage_metrics import PassageCounter
from crowd_sim_plus.envs.utils.action import ActionXY
p=argparse.ArgumentParser();p.add_argument('--human-policy',choices=['sfm','orca_plus'],required=True);a=p.parse_args()
R=Path('/runtime');scene=json.loads((R/'scene.json').read_text());layout=json.loads((R/'layout.json').read_text());checks=[]
k=CrowdKernel(scene,layout,human_policy=a.human_policy);ref=CrowdKernel(scene,layout,human_policy=a.human_policy)
humans,robot,obstacles=ref.env.humans,ref.env.robot,ref.env.static_obstacles
ref.env.reset('test',0,return_stat=True);ref.env.humans=humans;ref.env.robot=robot;ref.env.static_obstacles=obstacles;ref.env.sim_env='social_warehouse'
robot.set(*scene['robot_start'][:2],*scene['robot_goal'][:2],0.,0.,scene['robot_start'][2]);errors=[]
for step in range(30):
 r=ref.env.robot;k.advance(dict(x=r.px,y=r.py,yaw=r.theta,vx=r.vx,vy=r.vy,omega=0.));ref.env.step(ActionXY(.5,0.))
 for h,j in zip(k.env.humans,ref.env.humans):errors.extend(abs(getattr(h,key)-getattr(j,key)) for key in ['px','py','vx','vy','gx','gy'])
assert max(errors)<1e-12 and not any(k.arrivals),max(errors)
checks.append(dict(case='30 updates match full native env.step before first patrol turn',max_state_error=max(errors)))
spec=importlib.util.spec_from_file_location('legacy_kernel',str(R/'legacy_kernel.py'));legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
for condition in ['crossing','counterflow']:
 old_scene=json.loads((R/(condition+'_scene.json')).read_text());old=legacy.CrowdKernel(old_scene,layout,human_policy=a.human_policy);new=CrowdKernel(old_scene,layout,human_policy=a.human_policy);errors=[]
 fixed=dict(x=-14.,y=5.,yaw=0.,vx=0.,vy=0.,omega=0.)
 for step in range(30):
  old.advance(fixed);new.advance(fixed)
  for h,j in zip(old.humans(),new.humans()):errors.extend(abs(h[key]-j[key]) for key in ['x','y','vx','vy','yaw','speed']);errors.extend(abs(x-y) for x,y in zip(h['goal'],j['goal']))
 assert max(errors)<1e-12,max(errors)
 checks.append(dict(case=condition+' fixture dynamics unchanged, 30 updates',max_state_error=max(errors)))
model=CrowdKernel(scene,layout,human_policy=a.human_policy);counter=PassageCounter(model.humans(),scene['passage']);minimum_pair=math.inf;minimum_factory=math.inf;before=model.humans()
with (R/'far_robot_patrol.jsonl').open('w') as f:
 for step in range(200):
  model.advance(fixed);hs=model.humans();counter.update((step+1)*.1,hs)
  assert all(math.isfinite(h[key]) for h in hs for key in ['x','y','vx','vy','speed'])
  minimum_pair=min(minimum_pair,min(math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(hs) for j in hs[i+1:]))
  for h in hs:
   for b in layout['obstacles']:
    qx,qy=abs(h['x']-b['x'])-b['sx']/2,abs(h['y']-b['y'])-b['sy']/2
    minimum_factory=min(minimum_factory,math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36)
  for h,j in zip(hs,before):
   if h['completed_legs']>j['completed_legs']:assert math.hypot(h['x']-j['goal'][0],h['y']-j['goal'][1])<.36+1e-10
  f.write(json.dumps(dict(time=(step+1)*.1,humans=hs,robot=fixed))+'\n');before=hs
checks.append(dict(case='20 s distant robot behavior screen, no forced success criterion',completed_legs=[h['completed_legs'] for h in hs],passages=counter.counts(),minimum_pair_clearance_m=minimum_pair,minimum_factory_clearance_m=minimum_factory))
traces={}
for visible in [True,False]:
 model=CrowdKernel(scene,layout,visible,human_policy=a.human_policy);rows=[]
 for step in range(100):
  robot=dict(x=8.+.5*step*.1,y=0.,yaw=0.,vx=.5,vy=0.,omega=0.);model.advance(robot);rows.append(model.humans())
 traces[str(visible)]=rows
change=max(math.hypot(h['x']-j['x'],h['y']-j['y']) for x,y in zip(traces['True'],traces['False']) for h,j in zip(x,y))
assert change>.001,change
checks.append(dict(case='10 s same scripted robot, visibility response',max_difference_m=change))
with (R/'benchmark.config').open('w') as f:k.config.write(f)
(R/'visibility_traces.json').write_text(json.dumps(traces))
result=dict(status='PASS',seed=scene['seed'],human_policy=a.human_policy,checks=checks,metadata=k.metadata(),note='PASS confirms bridge/finite-state checks, not safe behavior or successful passage. Worker probes are not Nav2 trials; native parameters unchanged.')
(R/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
