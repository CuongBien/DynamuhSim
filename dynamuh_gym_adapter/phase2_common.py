"""Stage-2 DynamuhSim: fixed tasks + optional conservative LiDAR shield.

Copy beside env.py and core.py. Does NOT change or override real_backend.py.
The shield is NOT a validated collision detector; keep collision_truth_valid=False.
"""
from __future__ import annotations
import math
import numpy as np
import gymnasium as gym


class FixedTask(gym.Wrapper):
    """Keep scenario and target unchanged across SB3's automatic resets."""
    def __init__(self, env, *, scenario='empty', start=(-13., 0., 0.), goal=(-7., 0.)):
        super().__init__(env)
        if scenario not in ('empty', 'human'):
            raise ValueError('scenario must be empty or human (RVO2)')
        self.fixed_options = {'scenario': scenario, 'start': tuple(start), 'goal': tuple(goal)}

    def reset(self, *, seed=None, options=None):
        values = dict(self.fixed_options)
        if options:
            # No scenario/start/goal override across eval/train episodes.
            for k in ('scenario', 'start', 'goal'):
                if k in options and options[k] != values[k]:
                    raise ValueError(f'FixedTask cannot change {k} inside this run')
        return self.env.reset(seed=seed, options=values)


class LidarSafetyShield(gym.Wrapper):
    """Conservative last-scan gate for exploratory velocities (not physical proof of safety).

    With 360° lidar, beam angle zero is assumed robot-forward as in SDF.
    No valid beam in the intended translational direction -> translation disabled.
    The original backend's per-axis acceleration limiter still applies.
    """
    def __init__(self, env, *, clearance_m=.38, range_max_m=3.5, cone_degrees=40.):
        super().__init__(env)
        self.clearance_m = float(clearance_m)
        self.range_max_m = float(range_max_m)
        self.cone = math.radians(cone_degrees)
        self.last_obs = None
        self.last_epoch = None

    def reset(self, *, seed=None, options=None):
        obs, info = self.env.reset(seed=seed, options=options)
        self.last_obs = obs
        self.last_epoch = info['epoch']
        return obs, info

    def _shield(self, action):
        raw = np.asarray(action, dtype=np.float32)
        if raw.shape != (3,) or not np.isfinite(raw).all():
            raise ValueError('Policy must send finite [vx, vy, wz]')
        safe = raw.copy()
        reason = 'clear'
        min_sector = float('nan')
        speed = float(np.linalg.norm(safe[:2]))
        if speed > 1e-6:
            obs = self.last_obs
            if obs is None:
                raise RuntimeError('reset() is required before step()')
            # Raw LaserScan is needed: core.py correctly masks +inf for the
            # actor, but +inf may mean "no hit within range_max" in Gazebo.
            # NaN / -inf are ALWAYS unknown and block motion. Verify this
            # no-return convention on the target ROS-GZ bridge before use.
            backend = self.env.unwrapped.backend
            with backend.lock:
                sample = backend.samples.get('scan')
                if sample is None:
                    raise RuntimeError('No raw laser scan available for safety gate')
                _, raw, rmin, rmax = sample
                raw = np.asarray(raw, np.float64).copy()
            if raw.size < 72 or not np.isfinite(rmax) or rmax <= 0:
                raise RuntimeError('Invalid raw laser scan for safety gate')
            # SDF model min_angle=0, max_angle=2pi (360 beams).
            # Explicit hits are trusted; positive infinity is interpreted
            # as a no-return ray with clearance at least rmax.
            hit = np.isfinite(raw) & (raw >= rmin) & (raw <= rmax)
            far = np.isposinf(raw) | (np.isfinite(raw) & (raw > rmax))
            known = hit | far
            ranges = np.where(far, rmax, raw)
            angles = (np.arange(raw.size) + .5) * (2 * math.pi / raw.size)
            heading = math.atan2(float(safe[1]), float(safe[0])) % (2 * math.pi)
            circular_diff = np.abs((angles - heading + math.pi) % (2*math.pi) - math.pi)
            sector = circular_diff <= self.cone
            # Block if any genuinely UNKNOWN sample in travel direction.
            if not np.any(sector) or not np.all(known[sector]):
                safe[:2] = 0
                reason = 'unknown_sector'
            else:
                min_sector = float(np.min(ranges[sector]))
                if min_sector < self.clearance_m:
                    safe[:2] = 0
                    reason = 'low_clearance'
        return safe, reason, min_sector

    def step(self, action):
        policy_action = np.asarray(action, np.float32).copy()
        shielded, reason, distance = self._shield(policy_action)
        obs, reward, term, trunc, info = self.env.step(shielded)
        if info['epoch'] != self.last_epoch:
            raise RuntimeError('Episode epoch changed during step')
        self.last_obs = obs
        info = dict(info)
        info.update(policy_action=policy_action.tolist(),
                    shield_action=shielded.tolist(),
                    shield_intervened=bool(not np.allclose(policy_action, shielded)),
                    shield_reason=reason, min_sector_range_m=distance)
        return obs, reward, term, trunc, info


def make_env(*, scenario='empty', domain=None, start=(-13., 0., 0.), goal=(-7., 0.),
             max_sim_seconds=90., shield=True):
    from env import DynamuhGazeboEnv
    from real_backend import GazeboROSBackend
    env = DynamuhGazeboEnv(backend=GazeboROSBackend(ros_domain_id=domain),
                           max_sim_seconds=max_sim_seconds)
    env = FixedTask(env, scenario=scenario, start=start, goal=goal)
    if shield:
        env = LidarSafetyShield(env)
    return env
