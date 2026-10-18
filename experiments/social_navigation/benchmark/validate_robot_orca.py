#!/usr/bin/env python3
"""Solver parity away from robot; anticipation/velocity-frame/standing checks."""
import argparse,json,math,sys,tempfile
from pathlib import Path
import yaml
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'gazebo_adapter'))
from kernel import GroupKernel
from circle_contact import CircleContact
from robot_orca import RobotAwareGroups
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args()
layout=json.loads((HERE.parent/'warehouse/layout.json').read_text());cfg=yaml.safe_load((HERE.parent/'configs/groups.yaml').read_text())
cfg['env'].update(min_dynamic_humans=0,max_dynamic_humans=0,min_h_h_dynamic_interactions=1,max_h_h_dynamic_interactions=1,min_h_h_static_interactions=0,max_h_h_static_interactions=0,set_shape='rectangle',min_map_x=60,max_map_x=60,min_map_y=40,max_map_y=40)
(HERE/'generated').mkdir(exist_ok=True);checks=[]
with tempfile.TemporaryDirectory(dir=HERE/'generated',prefix='robot_orca_validation_') as tmp:
    config=Path(tmp)/'config.yaml';config.write_text(yaml.safe_dump(cfg));k=GroupKernel(config,layout,'walking',43)
    try:
        CircleContact(k,.25);e=k.env;r=e.robot;r.x=r.y=100.;r.orientation=0.;r.vel_x=r.vel_y=0.
        baseline=e.compute_orca_interaction_velocities();policy=RobotAwareGroups(k)
        far=e.compute_orca_interaction_velocities();assert baseline==far,(baseline,far)
        checks.append(dict(case='far robot solver parity',baseline=baseline,variant=far))
        r.x,r.y=-22.,0.;r.vel_x=-.5;r.vel_y=0.
        assert all(not h.collides(r) for h in k.group.humans)
        approaching=e.compute_orca_interaction_velocities();telemetry=policy.last.copy()
        assert telemetry['group_robot_neighbor_seen']
        assert approaching[0][0]<baseline[0][0]-.01
        checks.append(dict(case='respond before individual contact',baseline=baseline,approaching=approaching,telemetry=telemetry))
        r.vel_x=.5;away=e.compute_orca_interaction_velocities();assert abs(away[0][0]-approaching[0][0])>.01
        checks.append(dict(case='measured velocity affects result',approaching=approaching,receding=away))
        r.orientation=math.pi/2;r.vel_x=0.;r.vel_y=.5;e.compute_orca_interaction_velocities()
        vx,vy=policy.last['measured_robot_world_velocity'];assert abs(vx+.5)<1e-12 and abs(vy)<1e-12
        checks.append(dict(case='body-to-world velocity conversion',world=[vx,vy]))
        policy.restore();assert e.compute_orca_interaction_velocities()==baseline
        checks.append(dict(case='restore original group solver',unchanged=True))
    finally:k.close()
    traces=[]
    for enabled in [False,True]:
        k=GroupKernel(config,layout,'walking',43);CircleContact(k,.25)
        if enabled:RobotAwareGroups(k)
        trace=[]
        try:
            for step in range(30):
                k.advance(dict(x=100.,y=100.,yaw=0.,vx=0.,vy=0.,omega=0.));trace.append(k.humans())
            traces.append(trace)
        finally:k.close()
    assert traces[0]==traces[1];checks.append(dict(case='30-step distant robot kernel parity',identical=True))
    cfg['env'].update(min_h_h_dynamic_interactions=0,max_h_h_dynamic_interactions=0,min_h_h_static_interactions=1,max_h_h_static_interactions=1)
    config.write_text(yaml.safe_dump(cfg));traces=[]
    for enabled in [False,True]:
        k=GroupKernel(config,layout,'conversation',43);CircleContact(k,.25)
        if enabled:RobotAwareGroups(k)
        trace=[]
        try:
            for step in range(20):
                k.advance(dict(x=-23.,y=0.,yaw=0.,vx=-.5,vy=0.,omega=0.));trace.append(k.humans())
            traces.append(trace)
        finally:k.close()
    assert traces[0]==traces[1];checks.append(dict(case='standing group unchanged for 20 steps',identical=True))
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(dict(status='PASS',checks=checks),indent=2));print(json.dumps(checks,indent=2))
