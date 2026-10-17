#!/usr/bin/env python3
"""Native ComplexityNav dynamics with a fixed ORCA robot and external observation code."""
import argparse
import copy
import json
import math
from pathlib import Path
import random
import time
from types import SimpleNamespace
import gym
import numpy as np
import crowd_sim
from crowd_nav.configs.icra_benchmark.config import BaseEnvConfig
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA
from crowd_sim.envs.utils import utils

HERE=Path(__file__).resolve().parent
REPORT=HERE/'reports'
parser=argparse.ArgumentParser()
parser.add_argument('--name',default='baseline')
parser.add_argument('--counts',type=int,nargs=4,default=[7,8,0,0],metavar=('ORCA','SFM','LINEAR','STATIC'))
parser.add_argument('--width',type=float,default=10.)
parser.add_argument('--scenario',choices=['passing','crossing','passing_crossing','random'],default='passing_crossing')
parser.add_argument('--seed',type=int,default=0)
parser.add_argument('--seconds',type=float,default=30.)
parser.add_argument('--invisible-robot',action='store_true')
parser.add_argument('--video',action='store_true')
args=parser.parse_args()
np.random.seed(1000+args.seed)
random.seed(1000+args.seed)
config=SimpleNamespace(**{s:copy.deepcopy(getattr(BaseEnvConfig,s)) for s in ['env','reward','sim','humans','robot']})
config.env.time_limit=args.seconds
config.env.dx_range=[-args.width/2,args.width/2]
config.env.dy_range=[-5.,5.]
config.sim.test_scenario=args.scenario
config.sim.train_val_scenario=args.scenario
config.sim.human_num=sum(args.counts)
config.robot.visible=not args.invisible_robot
for key,value in zip(['num_orca','num_sf','num_linear','num_static'],args.counts):
    setattr(config.humans,key,[value])
name=f'{args.name}_s{args.seed}_{"blind" if args.invisible_robot else "visible"}'
REPORT.mkdir(exist_ok=True)
(HERE/'configs').mkdir(exist_ok=True)
settings=dict(name=args.name,counts=args.counts,width=args.width,length=10.,scenario=args.scenario,seed=1000+args.seed,
              seconds=args.seconds,robot_visible=config.robot.visible,robot_policy='orca',timestep=config.env.time_step)
(HERE/'configs'/f'{name}.json').write_text(json.dumps(settings,indent=2))
env=gym.make('CrowdSim-v0').unwrapped
env.configure(config)
robot=Robot(config,'robot')
robot.time_step=env.time_step
robot.set_policy(ORCA())
env.set_robot(robot)
robot.policy.set_phase('test')
# Generator is native; precomputed goal sequences match the nonstop-human path.
# Keep upstream generator unchanged, but reject initially overlapping circles.
for sampling_attempt in range(100):
    sampling_seed=1000+args.seed+10000*sampling_attempt
    np.random.seed(sampling_seed)
    random.seed(sampling_seed)
    scenes,goals=utils.generate_scenarios_fixed(1,.3,env.x_width,env.y_width,env.discomfort_dist,
        *args.counts,args.scenario)
    positions=[(0.,-4.)]+[(float(h[0]),float(h[1])) for hs in scenes[0].values() for h in hs]
    if all(math.dist(a,b)>=.6-1e-9 for i,a in enumerate(positions) for b in positions[i+1:]):
        break
else:
    raise RuntimeError('No valid initial scene in 100 native samples')
settings.update(sampling_seed=sampling_seed,initial_sampling_attempts=sampling_attempt+1)
(HERE/'configs'/f'{name}.json').write_text(json.dumps(settings,indent=2))
ob=env.reset('test',scenes[0],goals[0],test_case=args.seed)

def state():
    r=env.robot
    rob=dict(x=float(r.px),y=float(r.py),vx=float(r.vx),vy=float(r.vy),radius=float(r.radius))
    humans=[]
    for i,h in enumerate(env.humans):
        kind='static' if h.v_pref<.001 else h.policy.name
        humans.append(dict(id=i,policy=kind,x=float(h.px),y=float(h.py),vx=float(h.vx),vy=float(h.vy),
            radius=float(h.radius),goal=[float(h.gx),float(h.gy)]))
    rh=[math.hypot(h['x']-rob['x'],h['y']-rob['y'])-h['radius']-rob['radius'] for h in humans]
    hh=[math.hypot(a['x']-b['x'],a['y']-b['y'])-a['radius']-b['radius'] for i,a in enumerate(humans) for b in humans[i+1:]]
    return dict(t=float(env.global_time),robot=rob,humans=humans,
        min_clearance_m=min(rh) if rh else None,min_human_human_clearance_m=min(hh) if hh else None)

rows=[state()]
status='step_limit'
info_counts={}
clustering=[]
render_error=None
render_adjustments=[]
wall_block_steps=0
frozen_steps=0
start=time.perf_counter()
try:
    for step in range(int(args.seconds/env.time_step)+3):
        action=robot.act(ob,border=env.orca_border)
        actual=env.outside_check(robot,env.orca_border,action)
        wall_block_steps+=int(abs(action.vx-actual.vx)>1e-10 or abs(action.vy-actual.vy)>1e-10)
        ob,reward,done,info=env.step(action)
        current=state()
        current['info']=str(info)
        rows.append(current)
        info_counts[str(info)]=info_counts.get(str(info),0)+1
        frozen_steps+=int(math.hypot(robot.vx,robot.vy)*env.time_step<.01)
        if step%4==0:
            env.grouper.humans=env.humans
            env.grouper.states=env.states
            labels=env.grouper.dbscan_group(len(env.states)-1)
            groups=[sum(l==x for l in labels) for x in sorted(set(labels))]
            clustering.append(dict(t=float(env.global_time),labels=[int(x) for x in labels],
                multi_person_clusters=int(sum(n>=2 for n in groups))))
        if done:
            status=str(info)
            break
    if args.video:
        if len(env.humans)<4:
            # Native renderer calls CoherentFilter(K=3), which indexes past N<4.
            # The returned colors are unused; use the documented DBSCAN method here.
            def small_population_colors(frame):
                env.grouper.group_labels=env.grouper.dbscan_group(frame)
                return ['b']*len(env.humans)
            env.grouper.group_color=small_population_colors
            render_adjustments.append('DBSCAN grouping for native renderer with N<4')
        try:
            env.render('video',str(REPORT/f'{name}.mp4'))
        except Exception as exc:
            render_error=repr(exc)
except Exception as exc:
    status='error'
    error=repr(exc)
    raise
finally:
    path_m=sum(math.hypot(a['robot']['x']-b['robot']['x'],a['robot']['y']-b['robot']['y']) for a,b in zip(rows,rows[1:]))
    summary=dict(**settings,status=status,steps=len(rows)-1,simulation_seconds=float(env.global_time),
        wall_seconds=time.perf_counter()-start,path_m=path_m,render_error=render_error,render_adjustments=render_adjustments,
        minimum_clearance_m=min(r['min_clearance_m'] for r in rows if r['min_clearance_m'] is not None),
        min_human_human_clearance_m=min(r['min_human_human_clearance_m'] for r in rows if r['min_human_human_clearance_m'] is not None),
        info_counts=info_counts,wall_block_steps=wall_block_steps,frozen_steps=frozen_steps,
        initial_state=rows[0],initial_geometry_valid=bool(rows[0]['min_clearance_m']>=0 and rows[0]['min_human_human_clearance_m']>=0),clustering_samples=clustering,
        source_commit='36acac12a940a20dceb7b4e1d36acc4c7a4e344a',
        note='2D native dynamics, ORCA holonomic robot; clustering is detection only. No Gazebo or learned robot policy.')
    if status=='error': summary['error']=error
    (REPORT/f'{name}.json').write_text(json.dumps(summary,indent=2))
    (REPORT/f'{name}_trajectory.json').write_text(json.dumps(rows))
    print(json.dumps({k:v for k,v in summary.items() if k not in ['initial_state','clustering_samples']},indent=2))
    env.close()
