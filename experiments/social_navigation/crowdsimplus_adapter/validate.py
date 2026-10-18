#!/usr/bin/env python3
"""Compare human-only kernel against upstream full env.step and visibility control."""
import argparse,json,math,sys
from pathlib import Path
from kernel import CrowdKernel
from crowd_sim_plus.envs.utils.action import ActionXY
p=argparse.ArgumentParser();p.add_argument('--human-policy',choices=['orca_plus','sfm'],default='orca_plus');options=p.parse_args()
R=Path('/runtime');scene=json.loads((R/'scene.json').read_text());layout=json.loads((R/'layout.json').read_text());checks=[]
k=CrowdKernel(scene,layout,human_policy=options.human_policy);ref=CrowdKernel(scene,layout,human_policy=options.human_policy)
custom_humans=ref.env.humans;custom_robot=ref.env.robot;obstacles=ref.env.static_obstacles
ref.env.reset('test',0,return_stat=True)
ref.env.humans=custom_humans;ref.env.robot=custom_robot;ref.env.static_obstacles=obstacles;ref.env.sim_env='social_warehouse'
custom_robot.set(-27.,0.,-14.,0.,0.,0.,0.)
errors=[]
for step in range(30):
    r=ref.env.robot
    robot=dict(x=r.px,y=r.py,yaw=r.theta,vx=r.vx,vy=r.vy,omega=0.)
    k.advance(robot);ref.env.step(ActionXY(.5,0.))
    for h,j in zip(k.env.humans,ref.env.humans):
        errors.extend(abs(getattr(h,key)-getattr(j,key)) for key in ['px','py','vx','vy','gx','gy'])
assert max(errors)<1e-12,max(errors)
checks.append(dict(case='30 human updates match upstream full env.step',max_state_error=max(errors)))
trajectories={}
for visible in [True,False]:
    model=CrowdKernel(scene,layout,visible,human_policy=options.human_policy);rows=[]
    for step in range(90):
        model.advance(dict(x=-27.+.5*step*.1,y=0.,yaw=0.,vx=.5,vy=0.,omega=0.));rows.append(model.humans())
    trajectories[str(visible)]=rows
max_change=max(math.hypot(h['x']-j['x'],h['y']-j['y']) for x,y in zip(trajectories['True'],trajectories['False']) for h,j in zip(x,y))
assert max_change>.001,max_change
checks.append(dict(case='same scripted robot: visible versus invisible changes human trajectory',max_difference_m=max_change))
fixture=CrowdKernel(scene,layout,human_policy=options.human_policy);h=fixture.env.humans[0]
h.set(0.,0.,2.,0.,0.,0.,0.);fixture.env.static_obstacles=[[(.4,-1.),(.4,1.)]]
a=fixture.env.constrain_agent_action_exact(h,ActionXY(.8,0.));h.step(a)
assert 0<h.px<.040001,h.px
checks.append(dict(case='native line constraint prevents crossing circle boundary',unconstrained_x=.08,constrained_x=h.px))
standing=dict(scene);standing['condition']='standing';model=CrowdKernel(standing,layout,human_policy=options.human_policy);before=model.humans()
for i in range(20):model.advance(dict(x=-24.,y=0.,yaw=0.,vx=0.,vy=0.,omega=0.))
assert model.humans()==before;checks.append(dict(case='stationary fixture holds exactly',steps=20))
metadata=k.metadata();assert metadata['source_commit']=='d526a07c013c621fa1ad50894b6ba639f61063de'
metadata['python_version']=sys.version
(R/'validation.json').write_text(json.dumps(dict(status='PASS',checks=checks,metadata=metadata),indent=2))
(R/'visibility_traces.json').write_text(json.dumps(trajectories))
with (R/'benchmark.config').open('w') as f:k.config.write(f)
print(json.dumps(dict(status='PASS',checks=checks,metadata=metadata),indent=2))
