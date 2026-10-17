#!/usr/bin/env python3
"""Paired native-kernel probe: feedback transport does not imply group anticipation."""
import json
import math
from pathlib import Path
from kernel import GroupKernel

here=Path(__file__).resolve().parent
layout=json.loads((here.parent/'warehouse/layout.json').read_text())
config=sorted((here/'generated').glob('walking_*/config.yaml'))[-1]

def trial(robot,steps=20):
    kernel=GroupKernel(config,layout,'walking',42)
    rows=[]
    try:
        for _ in range(steps):
            kernel.advance(robot)
            rows.append(kernel.humans())
        return rows
    finally:
        kernel.close()

far=dict(x=-27.,y=-5.,yaw=0.,vx=0.,vy=0.,omega=0.)
near=dict(x=-24.,y=-1.6,yaw=0.,vx=0.,vy=0.,omega=0.)
a=trial(far)
b=trial(near)
error=max(math.hypot(h['x']-j['x'],h['y']-j['y']) for row,other in zip(a,b) for h,j in zip(row,other))
assert error<1e-10
kernel=GroupKernel(config,layout,'walking',42)
contact=kernel.initial[0]
kernel.close()
c=trial(dict(x=contact['x'],y=contact['y'],yaw=0.,vx=0.,vy=0.,omega=0.),1)
assert all(h['speed']==0. for h in c[0])
result=dict(status='PASS',seed=42,paired_steps=20,
    near_vs_far_max_trajectory_difference_m=error,
    contact_group_speeds=[h['speed'] for h in c[0]],
    conclusion='Native group ORCA does not anticipate this nearby robot; native overlap handling sets group speeds to zero. No new avoidance policy added.')
(here/'reports/native_robot_response.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
