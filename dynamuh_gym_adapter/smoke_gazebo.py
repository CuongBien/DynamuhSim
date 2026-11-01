"""Real ROS/Gazebo smoke test. MUST run on a machine with ROS2 Jazzy and Gazebo Harmonic.

Example: python3 smoke_gazebo.py --scenario empty --episodes 100 --steps 3
"""
import argparse
import json
import math
import time
import numpy as np
from env import DynamuhGazeboEnv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=100)
    ap.add_argument('--steps', type=int, default=3)
    ap.add_argument('--scenario', choices=['empty', 'human'], default='empty')
    ap.add_argument('--domain', type=int, default=None, help='Exclusive ROS domain; omitted = automatic')
    ap.add_argument('--output', default='gazebo_acceptance.jsonl')
    args = ap.parse_args()
    from real_backend import GazeboROSBackend
    env = DynamuhGazeboEnv(backend=GazeboROSBackend(ros_domain_id=args.domain))
    passed = 0
    try:
        with open(args.output, 'w', encoding='utf-8') as log:
            for ep in range(args.episodes):
                row = {'episode_index': ep, 'seed': ep+1000, 'scenario': args.scenario}
                try:
                    obs, info = env.reset(seed=ep+1000, options={'scenario': args.scenario})
                    assert env.observation_space.contains(obs)
                    last_time = info['sim_time']
                    epoch = info['epoch']
                    row['initial_sim_time'] = last_time
                    for k in range(args.steps):
                        obs, reward, terminated, truncated, info = env.step(np.zeros(3, dtype=np.float32))
                        assert env.observation_space.contains(obs)
                        assert math.isfinite(reward)
                        assert info['epoch'] == epoch
                        assert info['sim_time'] > last_time
                        assert abs(info['sim_time'] - last_time - .2) < .00801
                        last_time = info['sim_time']
                        if terminated or truncated: break
                    row.update(status='PASS', final_sim_time=last_time, epoch=epoch)
                    passed += 1
                except Exception as exc:
                    row.update(status='FAIL', error=f'{type(exc).__name__}: {exc}',
                               log_file=getattr(env.backend, 'last_log', None))
                    log.write(json.dumps(row) + '\n'); log.flush()
                    raise
                log.write(json.dumps(row) + '\n'); log.flush()
                print(f'{passed}/{args.episodes}: {row}')
    finally:
        env.close()
    print(json.dumps({'passed': passed, 'requested': args.episodes,
                      'scenario': args.scenario, 'output': args.output}))

if __name__ == '__main__':
    main()
