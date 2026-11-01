"""Gymnasium v0.26+ interface for *direct-velocity* PPO proof of loop.

Only one process is owned by this Env at a time; do not launch Nav2 MPPI
alongside it. World-clock stepping uses Gazebo WorldControl, not wall sleep.
"""
from __future__ import annotations
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from core import (
    ACTION_HIGH, ACTION_LOW, PHYSICS_DT, POLICY_DT, STEPS_PER_ACTION,
    Scenario, SimulatorFault, require_fresh, goal_in_odom,
    encode_observation, clamp_action,
)


class DynamuhGazeboEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, backend=None, *, max_sim_seconds=90., goal_tolerance=.20):
        super().__init__()
        if backend is None:
            from real_backend import GazeboROSBackend
            backend = GazeboROSBackend()
        self.backend = backend
        self.max_sim_seconds = float(max_sim_seconds)
        self.goal_tolerance = float(goal_tolerance)
        self.action_space = spaces.Box(ACTION_LOW, ACTION_HIGH, dtype=np.float32)
        self.observation_space = spaces.Dict({
            'lidar': spaces.Box(0, 1, shape=(72,), dtype=np.float32),
            'lidar_mask': spaces.Box(0, 1, shape=(72,), dtype=np.float32),
            'goal': spaces.Box(-1, 1, shape=(3,), dtype=np.float32),
            'ego': spaces.Box(-10, 10, shape=(3,), dtype=np.float32),
            'prev_action': spaces.Box(ACTION_LOW, ACTION_HIGH, dtype=np.float32),
            'sensor_age': spaces.Box(0, 1, shape=(3,), dtype=np.float32),
        })
        self.epoch = 0
        self.active = False
        self.finished = False
        self.previous_action = np.zeros(3, dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = dict(options or {})
        scenario = Scenario(
            kind=options.get('scenario', 'empty'),
            seed=int(seed if seed is not None else options.get('seed', 42)),
            start=tuple(options.get('start', (-13., 0., 0.))),
            goal=tuple(options.get('goal', (-7., 0.))),
        )
        self.epoch += 1
        self.active = False
        self.finished = False
        self.previous_action[:] = 0
        s = self.backend.reset(scenario, self.epoch)
        require_fresh(s, self.epoch)
        self.scenario = scenario
        self.start_time = s.sim_time
        self.previous_time = s.sim_time
        self.goal_odom = goal_in_odom(scenario, s.odom)
        obs, self.previous_distance = encode_observation(s, self.goal_odom, self.previous_action)
        if not self.observation_space.contains(obs):
            raise SimulatorFault('reset observation outside declared observation_space')
        self.active = True
        return obs, {'epoch': self.epoch, 'scenario': scenario.kind,
                     'sim_time': s.sim_time, 'seed': scenario.seed,
                     'seed_determinism_verified': False}

    def step(self, action):
        if not self.active or self.finished:
            raise RuntimeError('Call reset() before step(), and after terminal/truncated step')
        proposed = np.asarray(action, dtype=np.float32)
        applied = clamp_action(proposed, self.previous_action)
        old_time = self.previous_time
        s = self.backend.advance(applied, STEPS_PER_ACTION, self.epoch)
        require_fresh(s, self.epoch)
        delta = s.sim_time - old_time
        if abs(delta - POLICY_DT) > 2 * PHYSICS_DT:
            raise SimulatorFault(f'Clock mismatch: {delta} rather than {POLICY_DT}')
        obs, distance = encode_observation(s, self.goal_odom, applied)
        if not self.observation_space.contains(obs):
            raise SimulatorFault('step observation outside declared observation_space')
        progress = self.previous_distance - distance
        jerk_cost = float(np.linalg.norm(applied - self.previous_action) ** 2)
        # Provisional reward: no validated contact or human-instance tracker here.
        # It must NOT be reported as physical collision or social-compliance reward.
        reward = 3.0 * progress - 0.1 * delta - 0.1 * jerk_cost
        terminated = distance <= self.goal_tolerance
        truncated = (not terminated and s.sim_time - self.start_time >= self.max_sim_seconds - 1e-6)
        if terminated:
            reward += 20.0
        self.previous_action = applied.copy()
        self.previous_distance = distance
        self.previous_time = s.sim_time
        self.finished = terminated or truncated
        info = {
            'epoch': self.epoch, 'scenario': self.scenario.kind,
            'sim_time': s.sim_time, 'sim_dt': delta,
            'distance_to_goal_m': distance,
            'proposed_action': proposed.tolist(), 'executed_action': applied.tolist(),
            'termination_reason': 'goal_reached' if terminated else 'time_limit' if truncated else None,
            'collision_truth_valid': False, 'social_reward_valid': False,
            'reward_status': 'PROVISIONAL_PROGRESS_ONLY',
        }
        return obs, float(reward), bool(terminated), bool(truncated), info

    def close(self):
        self.active = False
        self.finished = True
        self.backend.close()
