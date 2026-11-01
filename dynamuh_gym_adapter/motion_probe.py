"""Additional acceptance: verify robot EXECUTES nonzero cmd_vel and is reset.

Not a collision GT test, nor human reset proof. Runs only empty corridor.
"""
from __future__ import annotations
import argparse
import json
import math
import numpy as np
from env import DynamuhGazeboEnv
from real_backend import GazeboROSBackend


def odom_of(env):
    backend=env.backend
    with backend.lock:
        t, raw=backend.samples['odom']
        return float(t), np.asarray(raw).copy()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--episodes',type=int,default=10)
    parser.add_argument('--domain',type=int,default=None,help='Exclusive ROS domain; omitted = automatic')
    parser.add_argument('--steps',type=int,default=20)
    parser.add_argument('--output',default='motion_probe.jsonl')
    args=parser.parse_args()
    env=DynamuhGazeboEnv(backend=GazeboROSBackend(ros_domain_id=args.domain))
    first_pose=None
    try:
        with open(args.output,'w',encoding='utf8') as fd:
            for i in range(args.episodes):
                obs,startinfo=env.reset(seed=2000+i,options={'scenario':'empty'})
                _,initial=odom_of(env)
                # Use local-odom initial pose to detect accidental carryover.
                if first_pose is None: first_pose=initial[:3].copy()
                pose_error=float(np.linalg.norm(initial[:2]-first_pose[:2]))
                if pose_error>.07: raise AssertionError(f'reset leaked robot pose: {pose_error}')
                if np.linalg.norm(initial[3:5])>.12:
                    raise AssertionError('robot linear velocity not reset')
                if env.backend.node.count_publishers('/cmd_vel')!=1:
                    raise AssertionError('Conflicting /cmd_vel publisher found!')
                reward_sum=0.
                d0=env.previous_distance
                v_samples=[]
                for k in range(args.steps):
                    obs,reward,term,trunc,info=env.step(np.array([.30,0.,0.],dtype=np.float32))
                    reward_sum+=reward
                    _,pose=odom_of(env)
                    v_samples.append(float(pose[3]))
                    if term or trunc:break
                d1=info['distance_to_goal_m']
                moved=d0-d1
                if moved<.07 or max(v_samples)<.05:
                    raise AssertionError(f'cmd_vel did not move robot: progress={moved:.4f}, velocity={max(v_samples):.3f}')
                row={'episode':i,'status':'PASS','initial_sim_time':startinfo['sim_time'],
                     'epoch':startinfo['epoch'],'reset_odom_pose_error_m':pose_error,
                     'reset_linear_speed':float(np.linalg.norm(initial[3:5])),
                     'distance_progress_m':moved,'max_forward_speed_mps':max(v_samples),
                     'reward_sum':reward_sum}
                fd.write(json.dumps(row)+'\n');fd.flush();print(row,flush=True)
    finally:
        env.close()


if __name__=='__main__':main()
