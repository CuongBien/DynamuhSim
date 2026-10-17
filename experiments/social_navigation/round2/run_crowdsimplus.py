#!/usr/bin/env python3
"""Isolated CrowdSimPlus observation runner, without editing upstream source."""
import argparse
import configparser
import json
import math
from pathlib import Path
import time
import numpy as np
import gym
import torch
import crowd_sim_plus
from sicnav.policy.policy_factory import policy_factory
from crowd_sim_plus.envs.utils.robot_plus import Robot

ROOT=Path('/opt/CrowdSimPlus')
HERE=Path(__file__).resolve().parent
REPORT=HERE/'reports'
parser=argparse.ArgumentParser()
parser.add_argument('--scenario',choices=['circle','hallway','bottleneck'],default='bottleneck')
parser.add_argument('--humans',type=int,default=3)
parser.add_argument('--human-policy',choices=['orca_plus','sfm'],default='orca_plus')
parser.add_argument('--robot-policy',choices=['dwa','orca_plus'],default='dwa')
parser.add_argument('--seed',type=int,default=0,help='Upstream test_case; reset applies its own phase seed offset')
parser.add_argument('--seconds',type=float,default=30.)
parser.add_argument('--invisible-robot',action='store_true')
parser.add_argument('--video',action='store_true')
args=parser.parse_args()
rule={'circle':'circle_crossing','hallway':'hallway','bottleneck':'hallway_bottleneck'}[args.scenario]
name=f'{args.scenario}_{args.human_policy}_{args.robot_policy}_n{args.humans}_s{args.seed}_{"blind" if args.invisible_robot else "visible"}'
REPORT.mkdir(exist_ok=True)
(HERE/'configs').mkdir(exist_ok=True)
config=configparser.RawConfigParser()
config.read(ROOT/'sicnav/configs/env.config')
config.set('env','time_limit',str(args.seconds))
config.set('env','randomize_attributes','false')
config.set('sim','test_sim',rule)
config.set('sim','train_val_sim',rule)
config.set('sim','human_num',str(args.humans))
config.set('sim','starts_moving','0')
config.set('sim','circle_radius','3.5' if args.scenario=='circle' else '2.5')
config.set('sim','rect_width','3')
config.set('sim','rect_height','5')
config.set('humans','policy',args.human_policy)
config.set('humans','radius','.30')
config.set('humans','v_pref','1.0')
config.set('robot','policy',args.robot_policy)
config.set('robot','visible',str(not args.invisible_robot).lower())
with (HERE/'configs'/f'{name}.config').open('w') as stream:
    config.write(stream)
policy_cfg=configparser.RawConfigParser()
policy_cfg.read(ROOT/'sicnav/configs/policy.config')
env=gym.make('CrowdSimPlus-v0').unwrapped
env.configure(config)
# Upstream invisible-robot warning branch reads mode, but reset sets phase.
if args.invisible_robot:
    env.mode='test'  # external alias; leaves upstream movement policies unchanged
policy=policy_factory[args.robot_policy]()
if args.robot_policy=='dwa':
    policy.configure(policy_cfg)
else:
    policy.configure(config,'humans')
robot=Robot(config,'robot')
robot.set_policy(policy)
env.set_robot(robot)
policy.set_phase('test')
policy.set_device(torch.device('cpu'))
if args.robot_policy=='dwa':
    policy.time_step=env.time_step
    policy.configure_dwa(policy_cfg,config)
else:
    policy.configure(config,'humans')
    # Match the simulated robot footprint rather than the human radius.
    policy.radius=robot.radius
policy.set_env(env)
ob,static=env.reset('test',args.seed,return_stat=True)

def agent(a,index):
    return dict(id=index,x=float(a.px),y=float(a.py),vx=float(a.vx),vy=float(a.vy),
                radius=float(a.radius),goal=[float(a.gx),float(a.gy)])

def state():
    rob=agent(robot,-1)
    hum=[agent(h,i) for i,h in enumerate(env.humans)]
    rh=[math.hypot(rob['x']-h['x'],rob['y']-h['y'])-rob['radius']-h['radius'] for h in hum]
    hh=[math.hypot(a['x']-b['x'],a['y']-b['y'])-a['radius']-b['radius']
        for i,a in enumerate(hum) for b in hum[i+1:]]
    return dict(t=float(env.global_time),robot=rob,humans=hum,
        min_robot_human_clearance_m=min(rh) if rh else None,
        min_human_human_clearance_m=min(hh) if hh else None)

initial=state()
rows=[initial]
counts={k:0 for k in ['Collision','WallCollision','Frozen','Danger']}
policy_times=[]
status='running'
start=time.perf_counter()
try:
    for step in range(int(args.seconds/env.time_step)+3):
        before=time.perf_counter()
        action=robot.act(ob,static)
        policy_times.append(time.perf_counter()-before)
        ob,reward,done,info=env.step(action)
        s=state()
        s['info']={k:float(v.val) for k,v in info.items()}
        rows.append(s)
        for k in counts:
            counts[k]+=int(info[k].val!=0)
        if done:
            status='success' if info['ReachGoal'].val!=0 else 'timeout' if info['Timeout'].val!=0 else 'other_terminal'
            break
    else:
        status='step_limit'
    if args.video:
        env.render('video',str(REPORT/f'{name}.mp4'))

except Exception as exc:
    status='error'
    error=repr(exc)
    raise
finally:
    path_length=sum(math.hypot(a['robot']['x']-b['robot']['x'],a['robot']['y']-b['robot']['y']) for a,b in zip(rows,rows[1:]))
    summary=dict(scenario=args.scenario,human_policy=args.human_policy,robot_policy=args.robot_policy,
        robot_kinematics=robot.kinematics,humans=args.humans,test_case=args.seed,robot_visible=not args.invisible_robot,
        status=status,compatibility_aliases=['env.mode=test'] if args.invisible_robot else [],steps=len(rows)-1,simulation_seconds=float(env.global_time),wall_seconds=time.perf_counter()-start,
        radius_robot_m=robot.radius,radius_human_m=config.getfloat('humans','radius'),
        mean_robot_policy_ms=1000*float(np.mean(policy_times)) if policy_times else None,
        minimum_robot_human_clearance_m=min((r['min_robot_human_clearance_m'] for r in rows if r['min_robot_human_clearance_m'] is not None),default=None),
        minimum_human_human_clearance_m=min((r['min_human_human_clearance_m'] for r in rows if r['min_human_human_clearance_m'] is not None),default=None),
        robot_path_m=path_length,step_counts=counts,initial_state=initial,
        static_obstacles=[[[float(x),float(y)] for x,y in line] for line in static],
        source_commit='d526a07c013c621fa1ad50894b6ba639f61063de',
        note='Native 2D CrowdSimPlus, no ROS/Nav2 robot, no group membership model. Counts are flagged steps, not independent incidents.')
    if status=='error': summary['error']=error
    (REPORT/f'{name}.json').write_text(json.dumps(summary,indent=2))
    (REPORT/f'{name}_trajectory.json').write_text(json.dumps(rows))
    print(json.dumps({k:v for k,v in summary.items() if k not in ['initial_state','static_obstacles']},indent=2))
    env.close()
