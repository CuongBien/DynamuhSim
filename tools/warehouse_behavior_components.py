#!/usr/bin/env python3
"""Bounded small-population fixtures of the real preference/ORCA/projection stack.

These are COMPONENT fixtures in declared local geometry, not Gazebo episodes.
Native formation/dispersal lifecycle is not stepped. All executed endpoints,
robot inputs, traits and events are saved, including failures/partial data.
"""
import argparse,copy,json,math,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=Path(__file__).resolve().parents[1];sys.path[:0]=[str(root/'python'),str(root/'experiments/social_navigation/navigation/socnavgym_group_nav2')]
from fixture import make_kernel
from dynamuhsim.simulation.goal_clock import people
from dynamuhsim.simulation.human_motion_profiles import enable_profile,profile_config
from socnavgym.envs.utils.wall import Wall
specs={
 'independent':dict(positions=[(-2.,0.)],goals=[(4.,0.)]),
 'crossing':dict(positions=[(-2.,0.),(0.,-2.)],goals=[(4.,0.),(0.,4.)]),
 'head_on':dict(positions=[(-2.,0.),(2.,0.)],goals=[(4.,0.),(-4.,0.)]),
 'overtaking':dict(positions=[(-2.,0.),(-.5,0.)],goals=[(5.,0.),(5.,0.)]),
 'group_obstacle':dict(positions=[(-3.,-.72),(-3.62,.36),(-2.38,.36)],goals=[(4.,0.)]*3,group=True,obstacle=True),
 'regrouping':dict(positions=[(-3.,-1.4),(-4.21,.7),(-1.79,.7)],goals=[(5.,0.)]*3,group=True),
 'constrained_robot':dict(positions=[(-2.,0.)],goals=[(4.,0.)],robot='crossing',constrained=True),
 'moving_robot':dict(positions=[(-2.,0.)],goals=[(4.,0.)],robot='crossing'),
 'stopped_robot':dict(positions=[(-2.,0.)],goals=[(4.,0.)],robot='stopped'),
 'conversation':dict(positions=[(-1.,-.72),(-1.62,.36),(-.38,.36)],goals=[(4.,0.)]*3,standing=True),
}
a.output.mkdir(parents=True,exist_ok=False);inventory=[]
for seed in [45,46]:
 for name,spec in specs.items():
  k=make_kernel('walking',seed);e=k.env;hs=people(k)[:len(spec['positions'])];group=e.moving_interactions[0]
  e.static_humans=[];e.dynamic_humans=[];e.moving_interactions=[];e.static_interactions=[];e.h_l_interactions=[]
  for h,pos,goal in zip(hs,spec['positions'],spec['goals']):
   h.x,h.y=pos;h.goal_x,h.goal_y=goal;h.orientation=math.atan2(goal[1]-pos[1],goal[0]-pos[0]);h.speed=0.;h.type='dynamic';h.position_history.clear()
  if spec.get('group') or spec.get('standing'):
   group.humans=hs;group.goal_x,group.goal_y=spec['goals'][0];group.x=sum(h.x for h in hs)/len(hs);group.y=sum(h.y for h in hs)/len(hs);group.type='stationary' if spec.get('standing') else 'moving'
   (e.static_interactions if spec.get('standing') else e.moving_interactions).append(group)
  else:e.dynamic_humans=hs
  boxes=[dict(x=100.,y=100.,sx=1.,sy=1.)]
  if spec.get('obstacle'):boxes=[dict(x=0.,y=0.,sx=1.,sy=1.)]
  if spec.get('constrained'):boxes=[dict(x=0.,y=1.1,sx=12.,sy=.1),dict(x=0.,y=-1.1,sx=12.,sy=.1)]
  e.walls=[Wall(id=100+i,x=b['x'],y=b['y'],orientation=0. if b['sx']>=b['sy'] else math.pi/2,length=max(b['sx'],b['sy']),thickness=min(b['sx'],b['sy'])) for i,b in enumerate(boxes)]
  m=k.motion;m.boxes=boxes;m.velocities={};m.original['handle_collision_and_update']=lambda:None;m.steps=[]
  controller=enable_profile(k,profile_config('social_diversity_v1'));case=a.output/f'{name}_seed{seed}';case.mkdir();status='COMPLETE_COMPONENT_WINDOW';error=None;completed=0
  with (case/'config.json').open('x') as f:json.dump(dict(scenario=name,seed=seed,dt_s=.1,duration_s=16.,fixture=spec,boxes=boxes,mode='COMPONENT_REAL_MOTION_STACK_NATIVE_LIFECYCLE_NOT_STEPPED'),f,indent=2)
  with (case/'trajectory.jsonl').open('x') as f:
   try:
    for step in range(160):
     t=step*.1;m.tick=step+1;m.start={int(h.id):(h.x,h.y,h.orientation) for h in hs}
     robot=dict(x=-20.,y=-20.,yaw=0.,vx=0.,vy=0.,omega=0.)
     if spec.get('robot')=='crossing':robot.update(x=0.,y=-2.+.4*t,yaw=math.pi/2,vy=.4)
     if spec.get('robot')=='stopped':robot.update(x=0.,y=0.)
     m.robot_start=(robot['x'],robot['y']);m.robot_velocity=(robot['vx'],robot['vy']);before=copy.deepcopy(k.humans());m.integrate();completed+=1
     f.write(json.dumps(dict(time=t,state_time=t+.1,mode='COMPONENT',before=before,humans=k.humans(),groups=k.groups(),robot_input=robot,motion_guard=m.steps[-1],behavior=controller.snapshot()),allow_nan=False)+'\n')
   except Exception as exc:status='INCOMPLETE_COMPONENT';error=repr(exc)
  result=dict(scenario=name,seed=seed,status=status,error=error,steps=completed,duration_s=completed*.1,humans=len(hs),raw=str(case/'trajectory.jsonl'),traits=controller.traits)
  with (case/'result.json').open('x') as f:json.dump(result,f,indent=2)
  inventory.append(result);k.close()
with (a.output/'inventory.json').open('x') as f:json.dump(inventory,f,indent=2)
print(json.dumps(inventory,indent=2))
