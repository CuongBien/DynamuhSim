#!/usr/bin/env python3
"""Native parity, legacy default regression and fixed-robot patrol screening."""
import argparse,copy,importlib.util,json,math,sys
from pathlib import Path
from kernel import CrowdKernel
from crowd_sim_plus.envs.utils.action import ActionXY
p=argparse.ArgumentParser();p.add_argument('--human-policy',choices=['orca_plus','sfm'],required=True);p.add_argument('--shared-control',action='store_true');a=p.parse_args()
R=Path('/runtime');scene=json.loads((R/'scene.json').read_text());layout=json.loads((R/'layout.json').read_text())
assert scene['goal_layout']=='individual';checks=[]
k=CrowdKernel(scene,layout,human_policy=a.human_policy);ref=CrowdKernel(scene,layout,human_policy=a.human_policy)
humans,robot,obstacles=ref.env.humans,ref.env.robot,ref.env.static_obstacles
ref.env.reset('test',0,return_stat=True);ref.env.humans=humans;ref.env.robot=robot;ref.env.static_obstacles=obstacles;ref.env.sim_env='social_warehouse'
robot.set(-27.,0.,-14.,0.,0.,0.,0.);errors=[]
for step in range(30):
 r=ref.env.robot;k.advance(dict(x=r.px,y=r.py,yaw=r.theta,vx=r.vx,vy=r.vy,omega=0.));ref.env.step(ActionXY(.5,0.))
 for h,j in zip(k.env.humans,ref.env.humans):errors.extend(abs(getattr(h,key)-getattr(j,key)) for key in ['px','py','vx','vy','gx','gy'])
assert max(errors)<1e-12,max(errors);assert not any(k.arrivals)
checks.append(dict(case='individual fixture: 30 steps match full upstream env.step before any patrol turn',max_state_error=max(errors)))
shared=copy.deepcopy(scene);shared.pop('human_routes');shared.pop('goal_layout')
spec=importlib.util.spec_from_file_location('legacy_kernel',str(R/'legacy_kernel.py'));legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
old=legacy.CrowdKernel(shared,layout,human_policy=a.human_policy);new=CrowdKernel(shared,layout,human_policy=a.human_policy)
fixed=dict(x=-14.,y=5.,yaw=0.,vx=0.,vy=0.,omega=0.);errors=[]
for step in range(30):
 old.advance(fixed);new.advance(fixed)
 for h,j in zip(old.humans(),new.humans()):
  errors.extend(abs(h[key]-j[key]) for key in ['x','y','vx','vy','yaw','speed']);errors.extend(abs(x-y) for x,y in zip(h['goal'],j['goal']))
assert max(errors)<1e-12,max(errors)
checks.append(dict(case='legacy shared fixture: unchanged 30-step dynamics versus saved pre-change kernel',max_state_error=max(errors)))
def screen(fixture,label):
 model=CrowdKernel(fixture,layout,human_policy=a.human_policy);before=model.humans();minimum_pair=math.inf;turns=[]
 with (R/(label+'_patrol.jsonl')).open('w') as f:
  for step in range(300):
   model.advance(fixed);hs=model.humans()
   assert all(math.isfinite(h[key]) for h in hs for key in ['x','y','vx','vy','speed'])
   minimum_pair=min(minimum_pair,min(math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(hs) for j in hs[i+1:]))
   for h,j in zip(hs,before):
    if h['completed_legs']>j['completed_legs']:
     distance=math.hypot(h['x']-j['goal'][0],h['y']-j['goal'][1]);assert distance<.36+1e-10,distance
     turns.append(dict(step=step+1,id=h['id'],distance_to_previous_goal=distance))
   f.write(json.dumps(dict(time=(step+1)*.1,humans=hs,robot=fixed))+'\n');before=hs
 return dict(label=label,simulation_seconds=30.,completed_legs=[h['completed_legs'] for h in hs],
  final_human_speed_mean=sum(h['speed'] for h in hs)/3,minimum_pair_clearance_m=minimum_pair,
  final_goal_distances=[math.hypot(h['x']-h['goal'][0],h['y']-h['goal'][1]) for h in hs],turn_events=turns)
individual=screen(scene,'individual');assert min(individual['completed_legs'])>=2,individual
checks.append(dict(case='fixed robot: every human completes at least two patrol legs in 30 sim seconds',completed_legs=individual['completed_legs']))
control=screen(shared,'shared') if a.shared_control else None
if control:
 checks.append(dict(case='same fixed robot shared-endpoint control measured without imposing a desired failure',completed_legs=control['completed_legs']))
with (R/'benchmark.config').open('w') as f:k.config.write(f)
result=dict(status='PASS',seed=scene['seed'],human_policy=a.human_policy,checks=checks,
 metadata=k.metadata(),individual=individual,shared_control=control,
 note='30-second open-loop worker screen with a fixed distant robot; not additional Nav2/Gazebo trials. No SFM tuning, lane constraint or arrival-threshold change.')
(R/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(status=result['status'],seed=scene['seed'],human_policy=a.human_policy,checks=checks),indent=2))
