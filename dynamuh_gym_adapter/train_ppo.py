"""One Gazebo worker, one SB3 PPO trainer. No Nav2 and no parallel EvalCallback."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, CallbackList
from stable_baselines3.common.monitor import Monitor
from phase2_common import make_env


class QualityGateCallback(BaseCallback):
    """Stop on invalid training telemetry. Log episode outcomes reported by env."""
    def __init__(self, out_path):
        super().__init__()
        self.path = Path(out_path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.episode_index = 0

    def _on_step(self):
        rewards = np.asarray(self.locals.get('rewards', []), dtype=np.float64)
        if not np.isfinite(rewards).all():
            raise RuntimeError('Non-finite PPO reward: abort and discard rollout')
        for info in self.locals.get('infos', []):
            if info.get('reward_status') != 'PROVISIONAL_PROGRESS_ONLY':
                raise RuntimeError('Unexpected reward definition: review env.py before training')
            dt = info.get('sim_dt')
            if dt is None or abs(float(dt) - .2) > .0021:
                raise RuntimeError(f'Bad simulation step: sim_dt={dt}')
            if 'episode' in info:  # populated by Monitor when terminated/truncated
                ep = info['episode']
                record = dict(episode=self.episode_index, timesteps=int(self.num_timesteps),
                              scenario=info.get('scenario'),
                              reward=float(ep['r']), steps=int(ep['l']),
                              success=info.get('termination_reason') == 'goal_reached',
                              final_distance_m=float(info['distance_to_goal_m']),
                              reason=info.get('termination_reason'),
                              collision_truth_valid=info.get('collision_truth_valid', False))
                with self.path.open('a', encoding='utf8') as fh:
                    fh.write(json.dumps(record, allow_nan=False) + '\n')
                self.episode_index += 1
                print('Episode:', record, flush=True)
        return True


def args_parse():
    p = argparse.ArgumentParser()
    p.add_argument('--scenario', choices=['empty', 'human'], default='empty')
    p.add_argument('--timesteps', type=int, default=2048)
    p.add_argument('--domain', type=int, default=None, help='Exclusive ROS domain; omitted = automatic')
    p.add_argument('--seed', type=int, default=11)
    p.add_argument('--start', nargs=3, type=float, default=(-13., 0., 0.))
    p.add_argument('--goal', nargs=2, type=float, default=(-7., 0.))
    p.add_argument('--max-sim-seconds', type=float, default=90.)
    p.add_argument('--run-dir', default='runs/ppo_empty')
    p.add_argument('--resume', default=None, help='Previous .zip SB3 checkpoint')
    p.add_argument('--no-shield', action='store_true', help='Only for controlled experiments')
    return p.parse_args()


def main():
    args = args_parse()
    if args.timesteps <= 0: raise ValueError('timesteps must be >0')
    out = Path(args.run_dir)
    out.mkdir(parents=True, exist_ok=True)
    env = make_env(scenario=args.scenario, domain=args.domain, start=args.start,
                   goal=args.goal, max_sim_seconds=args.max_sim_seconds,
                   shield=not args.no_shield)
    env = Monitor(env, filename=str(out / 'monitor.csv'))
    manifest = vars(args).copy()
    manifest['domain'] = env.unwrapped.backend.domain_id
    manifest['collision_truth_valid'] = False
    manifest['social_reward_valid'] = False
    manifest['model'] = 'SB3 PPO MultiInputPolicy, direct Box(vx,vy,wz)'
    (out/'config.json').write_text(json.dumps(manifest, indent=2), encoding='utf8')
    try:
        if args.resume:
            model = PPO.load(args.resume, env=env, device='cpu')
            print('Resume PPO from:', args.resume)
        else:
            model = PPO(
                'MultiInputPolicy', env,
                learning_rate=3e-4,
                n_steps=256, batch_size=64, n_epochs=10,
                gamma=.99, gae_lambda=.95, clip_range=.2,
                ent_coef=.005, vf_coef=.5, max_grad_norm=.5,
                policy_kwargs={'net_arch': dict(pi=[128, 128], vf=[128, 128])},
                seed=args.seed, device='cpu', verbose=1,
                tensorboard_log=None,
            )
            model.save(str(out/'untrained'))
        callback = CallbackList([
            QualityGateCallback(out/'episode_metrics.jsonl'),
            CheckpointCallback(save_freq=2048, save_path=str(out/'checkpoints'),
                               name_prefix='ppo'),
        ])
        model.learn(total_timesteps=args.timesteps,
                    reset_num_timesteps=not bool(args.resume), callback=callback,
                    progress_bar=False)
        model.save(str(out/'final'))
        print('Saved:', out/'final.zip')
    finally:
        env.close()


if __name__ == '__main__':
    main()
