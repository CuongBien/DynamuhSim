#!/usr/bin/env python3
"""Replay the native collision predicate at logged pre-update states (venv Python)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'gazebo_adapter'))
from kernel import GroupKernel
P=argparse.ArgumentParser()
P.add_argument('report_dir',type=Path)
a=P.parse_args()
report=a.report_dir
trial=json.loads((report/'socnavgym_counterflow_s43.json').read_text())
runtime=HERE/trial['runtime']
layout=json.loads((HERE.parent/'warehouse/layout.json').read_text())
k=GroupKernel(runtime/'config.yaml',layout,'walking',trial['seed'])
rows=[json.loads(l) for l in (runtime/'native_trajectory.jsonl').open()]
gate=float((runtime/'start_gate').read_text())
def clear(ro,h):
    dx,dy=h['x']-ro['x'],h['y']-ro['y'];c,s=math.cos(ro['yaw']),math.sin(ro['yaw'])
    qx,qy=abs(c*dx+s*dy)-.12,abs(-s*dx+c*dy)-.10
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
out=[]
try:
    for prev,x in zip(rows,rows[1:]):
        if not x['active']:continue
        ro=x['robot'];k.env.robot.x,k.env.robot.y,k.env.robot.orientation=ro['x'],ro['y'],ro['yaw']
        humans=[]
        for h,old,new in zip(k.group.humans,prev['humans'],x['humans']):
            assert h.id==old['id']==new['id']
            h.x,h.y,h.orientation=old['x'],old['y'],old['yaw']
            humans.append(dict(id=h.id,native_collides_before_update=bool(h.collides(k.env.robot)),
                proxy_clearance_before=clear(ro,old),proxy_clearance_after=clear(ro,new),
                displacement_m=math.hypot(new['x']-old['x'],new['y']-old['y']),speed_after=new['speed']))
        out.append(dict(time=x['time'],elapsed=x['time']-gate,robot=ro,humans=humans))
    crossing=next((dict(time=x['time'],elapsed=x['elapsed'],human=h) for x in out for h in x['humans'] if h['proxy_clearance_before']>=0 and h['proxy_clearance_after']<-.01),None)
    contact=next((dict(time=x['time'],elapsed=x['elapsed'],human=h) for x in out for h in x['humans'] if h['native_collides_before_update']),None)
    vendor=HERE.parent/'vendor/SocNavGym/socnavgym/envs'
    sources={str(p.relative_to(HERE.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [vendor/'utils/object.py',vendor/'socnavenv_v2.py']}
    summary=dict(seed=trial['seed'],nominal_robot_radius=trial['scene']['robot_radius'],
        actual_native_robot_radius=k.env.robot.radius,default_robot_radius_margin=k.env.ROBOT_RADIUS_MARGIN,
        native_human_width=k.group.humans[0].width,native_human_ellipse_semiaxes=[k.group.humans[0].width/6,k.group.humans[0].width/2],
        orientation_applied_in_native_collides=False,first_proxy_crossing_at_native_step=crossing,
        first_native_contact=contact,sources_sha256=sources,
        notes=['Probe reconstructs the seeded native geometry and calls the unmodified collides method on logged pre-update poses.',
        'This tests the native predicate, not Gazebo contact force. Group ORCA omits the robot; collision handling checks before human.update, with no swept collision check.'])
    (report/'native_collision_probe.json').write_text(json.dumps(summary,indent=2))
    (report/'native_collision_steps.json').write_text(json.dumps(out))
    print(json.dumps(summary,indent=2))
finally:k.close()
