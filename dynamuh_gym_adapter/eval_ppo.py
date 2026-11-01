"""Run sequential isolated evaluation; never concurrently with training Gazebo."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np
from stable_baselines3 import PPO
from phase2_common import make_env


def parse_args():
    p=argparse.ArgumentParser()
    p.add_argument('--model', required=True)
    p.add_argument('--scenario', choices=['empty','human'], default='empty')
    p.add_argument('--episodes', type=int, default=30)
    p.add_argument('--seed-base', type=int, default=20000)
    p.add_argument('--domain', type=int, default=None, help='Exclusive ROS domain; omitted = automatic')
    p.add_argument('--start', nargs=3, type=float, default=(-13.,0.,0.))
    p.add_argument('--goal', nargs=2, type=float, default=(-7.,0.))
    p.add_argument('--max-sim-seconds', type=float, default=90.)
    p.add_argument('--output', default='eval.jsonl')
    p.add_argument('--no-shield', action='store_true')
    return p.parse_args()


def main():
    a=parse_args()
    if a.episodes < 1: raise ValueError('episodes must be positive')
    output=Path(a.output);output.parent.mkdir(parents=True, exist_ok=True)
    env=make_env(scenario=a.scenario, domain=a.domain, start=a.start,
                 goal=a.goal, max_sim_seconds=a.max_sim_seconds,
                 shield=not a.no_shield)
    model=PPO.load(a.model, device='cpu')
    if model.observation_space != env.observation_space or model.action_space != env.action_space:
        raise RuntimeError('Eval observation/action spaces differ from training')
    all_rows=[]
    try:
        with output.open('w', encoding='utf8') as f:
            for e in range(a.episodes):
                obs, first_info=env.reset(seed=a.seed_base+e)
                prev_t=first_info['sim_time']
                ret=0.; steps=0; interventions=0; min_clearance=math.inf
                while True:
                    action,_=model.predict(obs, deterministic=True)
                    obs,r,term,trunc,info=env.step(action)
                    if abs(float(info['sim_time'])-float(prev_t)-.2)> .0021:
                        raise RuntimeError('Simulation step mismatch')
                    if not math.isfinite(float(r)):
                        raise RuntimeError('Non-finite eval reward')
                    ret+=float(r); steps+=1
                    interventions+=int(info.get('shield_intervened',False))
                    c=info.get('min_sector_range_m',float('nan'))
                    if math.isfinite(c): min_clearance=min(min_clearance,c)
                    prev_t=info['sim_time']
                    if term or trunc:break
                row={'episode':e, 'seed':a.seed_base+e,
                     'scenario':a.scenario, 'success':info['termination_reason']=='goal_reached',
                     'return':round(ret,6), 'steps':steps,
                     'final_distance_m':round(float(info['distance_to_goal_m']),5),
                     'end_reason':info['termination_reason'],
                     'shield_steps':interventions,
                     'min_directional_lidar_clearance_m':(round(min_clearance,4) if math.isfinite(min_clearance) else None),
                     'physical_collision_measured':False}
                f.write(json.dumps(row,allow_nan=False)+'\n');f.flush()
                all_rows.append(row)
                print(f"{e+1}/{a.episodes} success={row['success']} return={row['return']} steps={steps}",flush=True)
    finally:
        env.close()
    successes=sum(r['success'] for r in all_rows)
    print(json.dumps({'episodes':len(all_rows), 'successes':successes,
                      'success_rate':successes/len(all_rows),
                      'mean_return':sum(r['return'] for r in all_rows)/len(all_rows),
                      'output':str(output),
                      'physical_collision_rate':'NOT_MEASURED'},indent=2))


if __name__=='__main__':main()
