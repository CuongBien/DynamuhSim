#!/usr/bin/env python3
"""Closed-form geometry fixtures and a native-kernel distant-robot parity check."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as Obj
import yaml
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'gazebo_adapter'))
from kernel import GroupKernel
from circle_contact import CircleContact, circle_intersects
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args()
checks=[]
h=Obj(name='human',x=0.,y=0.,width=.72)
for angle in [0.,math.pi/4,math.pi/2,math.pi]:
    for distance,expected in [(.60,True),(.62,False)]:
        r=Obj(name='robot',x=distance*math.cos(angle),y=distance*math.sin(angle),radius=.25)
        assert circle_intersects(h,r)==expected
        checks.append(dict(case='circle at direction',angle=angle,distance=distance,expected=expected))
w=Obj(name='wall',x=0.,y=0.,length=2.,thickness=.4,orientation=0.)
for x,y,expected in [(1.2,.4,True),(1.3,.5,False),(0.,.55,True),(0.,.57,False)]:
    h.x,h.y=x,y;assert circle_intersects(h,w)==expected
    checks.append(dict(case='finite wall face/corner',x=x,y=y,expected=expected))
w.orientation=math.pi/2
h.x,h.y=.55,0.;assert circle_intersects(h,w)
h.x=.57;assert not circle_intersects(h,w)
checks.append(dict(case='wall rotated 90 degrees',expected=True))
root=HERE.parent
layout=json.loads((root/'warehouse/layout.json').read_text())
cfg=yaml.safe_load((root/'configs/groups.yaml').read_text())
cfg['env'].update(min_dynamic_humans=0,max_dynamic_humans=0,min_h_h_dynamic_interactions=1,max_h_h_dynamic_interactions=1,
    min_h_h_static_interactions=0,max_h_h_static_interactions=0,set_shape='rectangle',min_map_x=60,max_map_x=60,min_map_y=40,max_map_y=40)
(HERE/'generated').mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix='geometry_validation_',dir=HERE/'generated') as tmp:
    config=Path(tmp)/'config.yaml';config.write_text(yaml.safe_dump(cfg))
    traces=[]
    for policy in ['native','circle_contact']:
        k=GroupKernel(config,layout,'walking',43)
        contact=CircleContact(k,.25) if policy=='circle_contact' else None
        trace=[]
        try:
            for step in range(60):
                k.advance(dict(x=100.,y=100.,yaw=0.,vx=0.,vy=0.,omega=0.))
                trace.append(k.humans())
            traces.append(trace)
        finally:k.close()
    assert traces[0]==traces[1], 'Geometry change must not alter far-from-contact trajectories'
    checks.append(dict(case='60 native group steps with distant robot',identical=True,seed=43,
        trace_sha256=hashlib.sha256(json.dumps(traces[0],sort_keys=True).encode()).hexdigest()))
    k=GroupKernel(config,layout,'walking',43)
    try:
        human=k.group.humans[0];human.x=human.y=0.;k.env.robot.x=.5;k.env.robot.y=0.
        assert not human.collides(k.env.robot)
        contact=CircleContact(k,.25)
        assert human.collides(k.env.robot)
        assert k.env.ROBOT_RADIUS==k.env.robot.radius==.25
        contact.restore();assert not human.collides(k.env.robot)
        checks.append(dict(case='actual native ellipse versus circle at 0.5 m',native=False,circle=True,restored_native=False))
    finally:k.close()
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(dict(status='PASS',checks=checks),indent=2))
print('PASS:',len(checks),'closed-form/parity cases')
