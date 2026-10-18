#!/usr/bin/env python3
"""Crossing-native parity, previous-fixture regression and robot-response probe."""
import argparse,importlib.util,json,math
from pathlib import Path
from kernel import CrowdKernel
from crossing_metrics import CrossingCounter
from crowd_sim_plus.envs.utils.action import ActionXY
p=argparse.ArgumentParser();p.add_argument('--human-policy',choices=['orca_plus','sfm'],required=True);a=p.parse_args()
R=Path('/runtime');scene=json.loads((R/'scene.json').read_text());layout=json.loads((R/'layout.json').read_text());checks=[]
assert scene['condition']=='crossing'
k=CrowdKernel(scene,layout,human_policy=a.human_policy);ref=CrowdKernel(scene,layout,human_policy=a.human_policy)
humans,robot,obstacles=ref.env.humans,ref.env.robot,ref.env.static_obstacles
ref.env.reset('test',0,return_stat=True);ref.env.humans=humans;ref.env.robot=robot;ref.env.static_obstacles=obstacles;ref.env.sim_env='social_warehouse'
robot.set(-27.,0.,-14.,0.,0.,0.,0.);errors=[]
for step in range(30):
 r=ref.env.robot;k.advance(dict(x=r.px,y=r.py,yaw=r.theta,vx=r.vx,vy=r.vy,omega=0.));ref.env.step(ActionXY(.5,0.))
 for h,j in zip(k.env.humans,ref.env.humans):errors.extend(abs(getattr(h,key)-getattr(j,key)) for key in ['px','py','vx','vy','gx','gy'])
assert max(errors)<1e-12,max(errors);assert not any(k.arrivals)
checks.append(dict(case='crossing: 30 updates match full upstream env.step before first patrol turn',max_state_error=max(errors)))
spec=importlib.util.spec_from_file_location('legacy_kernel',str(R/'legacy_kernel.py'));legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
legacy_scene=json.loads((R/'legacy_scene.json').read_text())
old=legacy.CrowdKernel(legacy_scene,layout,human_policy=a.human_policy);new=CrowdKernel(legacy_scene,layout,human_policy=a.human_policy)
fixed=dict(x=-14.,y=5.,yaw=0.,vx=0.,vy=0.,omega=0.);errors=[]
for step in range(30):
 old.advance(fixed);new.advance(fixed)
 for h,j in zip(old.humans(),new.humans()):
  errors.extend(abs(h[key]-j[key]) for key in ['x','y','vx','vy','yaw','speed']);errors.extend(abs(x-y) for x,y in zip(h['goal'],j['goal']))
assert max(errors)<1e-12,max(errors)
checks.append(dict(case='previous individual-goal fixture unchanged versus saved kernel, 30 updates',max_state_error=max(errors)))
model=CrowdKernel(scene,layout,human_policy=a.human_policy);counter=CrossingCounter(model.humans());before=model.humans();minimum_pair=math.inf
with (R/'far_robot_patrol.jsonl').open('w') as f:
 for step in range(200):
  model.advance(fixed);hs=model.humans();counter.update((step+1)*.1,hs)
  assert all(math.isfinite(h[key]) for h in hs for key in ['x','y','vx','vy','speed'])
  minimum_pair=min(minimum_pair,min(math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(hs) for j in hs[i+1:]))
  for h,j in zip(hs,before):
   if h['completed_legs']>j['completed_legs']:assert math.hypot(h['x']-j['goal'][0],h['y']-j['goal'][1])<.36+1e-10
  f.write(json.dumps(dict(time=(step+1)*.1,humans=hs,robot=fixed))+'\n');before=hs
legs=[h['completed_legs'] for h in hs];crossings=counter.counts()
assert min(legs)>=2,legs;assert min(crossings.values())>=2,crossings
checks.append(dict(case='fixed distant robot: every person completes >=2 legs and >=2 centerline crossings in 20 sim s',completed_legs=legs,crossings=crossings,minimum_pair_clearance_m=minimum_pair))
traces={}
for visible in [True,False]:
 model=CrowdKernel(scene,layout,visible,human_policy=a.human_policy);rows=[]
 for step in range(80):
  robot=dict(x=-27.+.5*step*.1,y=0.,yaw=0.,vx=.5,vy=0.,omega=0.);model.advance(robot);rows.append(model.humans())
 traces[str(visible)]=rows
change=max(math.hypot(h['x']-j['x'],h['y']-j['y']) for x,y in zip(traces['True'],traces['False']) for h,j in zip(x,y))
assert change>.001,change
checks.append(dict(case='same scripted robot: visible/invisible changes crossing trajectory',max_difference_m=change))
with (R/'benchmark.config').open('w') as f:k.config.write(f)
(R/'visibility_traces.json').write_text(json.dumps(traces))
result=dict(status='PASS',seed=scene['seed'],human_policy=a.human_policy,checks=checks,metadata=k.metadata(),
 note='Worker probes only, not additional Nav2 trials. Native force, timestep, speed and arrival threshold unchanged.')
(R/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
