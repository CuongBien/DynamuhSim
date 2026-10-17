#!/usr/bin/env python3
"""Run upstream SocNavGym v2 unchanged; optional observation configurations are separate."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import random
import subprocess
import time
import cv2
import gymnasium as gym
import numpy as np
import socnavgym
import yaml

HERE=Path(__file__).resolve().parent
VENDOR=HERE/'vendor/SocNavGym'
parser=argparse.ArgumentParser()
parser.add_argument('--scenario',choices=['original','groups','formation','reactive','blind'],default='original')
parser.add_argument('--seed',type=int,default=42)
parser.add_argument('--steps',type=int,default=500)
parser.add_argument('--show',action='store_true')
parser.add_argument('--no-video',action='store_true')
args=parser.parse_args()
os.environ.setdefault('SDL_AUDIODRIVER','dummy')
random.seed(args.seed)
np.random.seed(args.seed)
config_path=VENDOR/'tests/test_env.yaml'
if args.scenario!='original':
    config=yaml.safe_load(config_path.read_text())
    config['rendering'].update(resolution_x=900,resolution_y=900)
    config['episode'].update(episode_length=args.steps,end_with_collision=False)
    config['human'].update(human_policy='orca',prob_to_avoid_robot=1. if args.scenario!='blind' else 0.,max_advance_human=.8)
    ec=config['env']
    ec.update(min_map_x=14,max_map_x=14,min_map_y=14,max_map_y=14,set_shape='square',
              min_static_humans=0,max_static_humans=0,min_dynamic_humans=6,max_dynamic_humans=6,
              min_h_h_static_interactions=1,max_h_h_static_interactions=1,
              min_h_h_dynamic_interactions=1,max_h_h_dynamic_interactions=1,
              min_human_in_h_h_interactions=3,max_human_in_h_h_interactions=3)
    if args.scenario=='formation':
        ec.update(min_dynamic_humans=12,max_dynamic_humans=12,
                  min_h_h_static_interactions=0,max_h_h_static_interactions=0,
                  min_h_h_dynamic_interactions=0,max_h_h_dynamic_interactions=0,
                  crowd_formation_probability=.03,crowd_dispersal_probability=.01)
    config_path=HERE/'configs'/f'{args.scenario}.yaml'
    config_path.parent.mkdir(exist_ok=True)
    config_path.write_text(yaml.safe_dump(config,sort_keys=False))
env=gym.make('SocNavGym-v2',config=str(config_path))
env.action_space.seed(args.seed)
raw=env.unwrapped
frames=[]
rows=[]
events=[]
previous=None
episodes=0
writer=None
collisions=0
invalid_observations=0
step_times=[]
start=time.perf_counter()
name=f'socnavgym_{args.scenario}_s{args.seed}'
report=HERE/'reports'
report.mkdir(exist_ok=True)
obs,info=env.reset(seed=args.seed)
initial_invalid=not env.observation_space.contains(obs)

def state():
    groups=[{'type':kind,'members':[int(h.id) for h in g.humans],
             'center':[float(g.x),float(g.y)]}
            for kind,gl in [('moving',raw.moving_interactions),('static',raw.static_interactions)] for g in gl]
    people=list(raw.static_humans)+list(raw.dynamic_humans)
    for gl in [raw.moving_interactions,raw.static_interactions]:
        for g in gl:
            people.extend(g.humans)
    humans=[dict(id=int(h.id),x=float(h.x),y=float(h.y),heading=float(h.orientation),speed=float(h.speed)) for h in people]
    robot=dict(x=float(raw.robot.x),y=float(raw.robot.y),heading=float(raw.robot.orientation))
    clearance=min((math.hypot(h['x']-robot['x'],h['y']-robot['y'])-raw.ROBOT_RADIUS-raw.HUMAN_DIAMETER/2 for h in humans),default=None)
    return dict(robot=robot,humans=humans,groups=groups,clearance_m=clearance)

try:
    for step in range(args.steps):
        before=state()
        if args.scenario=='original':
            action=env.action_space.sample()
        elif args.scenario in ['reactive','blind']:
            # Identical robot controller approaches the nearest independent walker.
            humans=raw.dynamic_humans
            if humans:
                target=min(humans,key=lambda h:math.hypot(h.x-raw.robot.x,h.y-raw.robot.y))
                dx,dy=target.x-raw.robot.x,target.y-raw.robot.y
                theta=math.atan2(dy,dx)
                error=(theta-raw.robot.orientation+math.pi)%(2*math.pi)-math.pi
                action=np.array([.5*max(0.,math.cos(error)),0.,np.clip(error,-1,1)],dtype=np.float32)
            else:
                action=np.zeros(3,dtype=np.float32)
        else:
            action=np.zeros(3,dtype=np.float32)
        t=time.perf_counter()
        obs,reward,terminated,truncated,info=env.step(action)
        step_times.append(time.perf_counter()-t)
        invalid_observations+=int(not env.observation_space.contains(obs))
        s=state()
        s.update(step=step,episode=episodes,action=action.tolist(),reward=float(reward),
                 terminated=bool(terminated),truncated=bool(truncated))
        rows.append(s)
        signature=tuple((g['type'],tuple(sorted(g['members']))) for g in s['groups'])
        if previous is not None and signature!=previous:
            events.append(dict(step=step,before=previous,after=signature))
        previous=signature
        if s['clearance_m'] is not None and s['clearance_m']<0:
            collisions+=1
        frame=raw.render_without_showing('human',draw_human_gaze=True,draw_human_goal=False)
        frame=frame.copy()
        cv2.putText(frame,f'{args.scenario} | seed {args.seed} | t={step*raw.TIMESTEP:.1f}s',
                    (15,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),2)
        cv2.putText(frame,f'moving groups {len(raw.moving_interactions)} | static groups {len(raw.static_interactions)}',
                    (15,55),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),1)
        if not args.no_video:
            if writer is None:
                writer=cv2.VideoWriter(str(report/f'{name}.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),1/raw.TIMESTEP,
                                       (frame.shape[1],frame.shape[0]))
                if not writer.isOpened():
                    raise RuntimeError('Video writer could not open')
            writer.write(frame)
        if step in [0,args.steps//3,2*args.steps//3,args.steps-1]:
            cv2.imwrite(str(report/f'{name}_{step:04d}.png'),frame)
        if args.show:
            raw.render(draw_human_gaze=True,draw_human_goal=False)
            import pygame
            pygame.time.wait(30)
        if terminated or truncated:
            episodes+=1
            if step<args.steps-1:
                obs,info=env.reset()
                previous=None
finally:
    if writer:
        writer.release()
    env.close()
    (report/f'{name}_trajectory.json').write_text(json.dumps(rows))
    summary=dict(scenario=args.scenario,seed=args.seed,steps=len(rows),episode_resets=episodes,
        config=str(config_path.relative_to(HERE)),upstream_commit=subprocess.check_output(['git','-C',str(VENDOR),'rev-parse','HEAD'],text=True).strip(),
        initial_observation_outside_space=initial_invalid,steps_observation_outside_space=invalid_observations,
        group_membership_changes=events,steps_with_robot_human_overlap=collisions,
        minimum_clearance_m=min((r['clearance_m'] for r in rows if r['clearance_m'] is not None),default=None),
        wall_seconds=time.perf_counter()-start,mean_step_ms=1000*np.mean(step_times) if step_times else None,
        note='Exploratory 2D demo; no ROS/Gazebo adapter and no learned robot navigation policy.')
    (report/f'{name}.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k!='group_membership_changes'},indent=2))
    print('Group membership changes:',len(events))
