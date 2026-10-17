#!/usr/bin/env python3
"""Create common initial scenes, or host the unchanged native group kernel.
The gate only holds time before Nav2 is ready; it adds no avoidance policy.
"""
import argparse
import json
from pathlib import Path
import socket
import sys
import time
import yaml

HERE=Path(__file__).resolve().parent
ADAPTER=HERE.parent/'gazebo_adapter'
sys.path.insert(0,str(ADAPTER))
from kernel import GroupKernel

p=argparse.ArgumentParser()
p.add_argument('--runtime',type=Path,required=True)
p.add_argument('--condition',choices=['standing','counterflow'],required=True)
p.add_argument('--seed',type=int,default=42)
p.add_argument('--prepare',action='store_true')
p.add_argument('--socket')
a=p.parse_args()
mode='conversation' if a.condition=='standing' else 'walking'
layout=json.loads((HERE.parent/'warehouse/layout.json').read_text())
config=a.runtime/'config.yaml'
if a.prepare:
    cfg=yaml.safe_load((HERE.parent/'configs/groups.yaml').read_text())
    cfg['env'].update(min_dynamic_humans=0,max_dynamic_humans=0,
        min_h_h_dynamic_interactions=int(mode=='walking'),max_h_h_dynamic_interactions=int(mode=='walking'),
        min_h_h_static_interactions=int(mode=='conversation'),max_h_h_static_interactions=int(mode=='conversation'),
        set_shape='rectangle',min_map_x=60,max_map_x=60,min_map_y=40,max_map_y=40)
    config.write_text(yaml.safe_dump(cfg))
k=GroupKernel(config,layout,mode,a.seed)
if a.prepare:
    center=-23.5 if a.condition=='standing' else -21.
    initial=k.humans()
    for h in initial:
        h['x']+=center+24.
        if mode=='walking':
            h['yaw']=3.141592653589793
    scene=dict(condition=a.condition,seed=a.seed,initial=initial,
        robot_spawn=[-27.,0.,0.],robot_goal=[-14.,0.,0.],robot_radius=.25,human_radius=.36,
        group_route=[[-28.,0.],[-21.,0.]],preferred_human_speed=.8)
    (a.runtime/'scene.json').write_text(json.dumps(scene,indent=2))
    k.close()
    raise SystemExit(0)
scene=json.loads((a.runtime/'scene.json').read_text())
for h,state in zip(k.group.humans,scene['initial']):
    h.x=h.initial_x=state['x']
    h.y=h.initial_y=state['y']
    h.orientation=h.initial_orientation=state['yaw']
    h.position_history.clear()
k.group.x=sum(h.x for h in k.group.humans)/3
k.group.y=sum(h.y for h in k.group.humans)/3
k.route=scene['group_route']
k.group.set_goal(*k.route[0])
k.env.robot.goal_x,k.env.robot.goal_y=scene['robot_goal'][:2]
server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
server.bind(a.socket)
server.listen(1)
(a.runtime/'worker_ready').write_text('ready\n')
try:
    connection,_=server.accept()
    with connection,connection.makefile('rb') as stream,(a.runtime/'native_trajectory.jsonl').open('w',buffering=1) as log:
        previous=None
        for line in stream:
            req=json.loads(line)
            try:
                if previous is not None and abs(req['time']-previous-.1)>1e-6:
                    raise RuntimeError('Simulation clock discontinuity')
                active=(a.runtime/'start_gate').exists()
                begin=time.perf_counter()
                if req['advance'] and active:
                    k.advance(req['robot'])
                previous=req['time']
                reply=dict(time=req['time'],step=k.steps,humans=k.humans(),active=active,
                    groups=[dict(id=1,members=[1,2,3],type=mode)],compute_ms=1000*(time.perf_counter()-begin))
                log.write(json.dumps(dict(**reply,robot=req['robot']))+'\n')
            except Exception as ex:
                reply=dict(error=str(ex))
            connection.sendall((json.dumps(reply,allow_nan=False)+'\n').encode())
            if 'error' in reply:
                break
finally:
    server.close()
    k.close()
