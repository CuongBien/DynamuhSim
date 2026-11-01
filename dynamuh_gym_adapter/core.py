"""Simulator-independent contracts and observation/reward logic for DynamuhSim.

No ROS/Gazebo/Gym import: this module is deliberately unit-testable anywhere.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Protocol
import numpy as np

PHYSICS_DT = 0.004  # src/custom_corridor/worlds/corridor_090.sdf
POLICY_DT = 0.2
STEPS_PER_ACTION = 50
ACTION_LOW = np.array([-0.20, -0.40, -1.80], dtype=np.float32)
ACTION_HIGH = np.array([0.60, 0.40, 1.80], dtype=np.float32)


class SimulatorFault(RuntimeError):
    """Infrastructure/sensor failure: do NOT reinterpret as RL task failure."""


@dataclass(frozen=True)
class Scenario:
    kind: str = 'empty'      # empty | human (the existing RVO2 corridor plugin)
    seed: int = 42
    start: tuple[float, float, float] = (-13.0, 0.0, 0.0)
    goal: tuple[float, float] = (-7.0, 0.0)

    def __post_init__(self):
        if self.kind not in ('empty', 'human'):
            raise ValueError('Supported proof-of-loop scenarios: empty, human (RVO2)')
        if len(self.start) != 3 or len(self.goal) != 2:
            raise ValueError('start=(x,y,yaw), goal=(x,y) required')
        if not all(math.isfinite(float(v)) for v in (*self.start, *self.goal)):
            raise ValueError('start/goal must be finite')


@dataclass
class SensorSnapshot:
    epoch: int
    sim_time: float
    scan_time: float
    scan: np.ndarray           # raw 360-ish ranges, meters
    range_min: float
    range_max: float
    odom_time: float
    odom: np.ndarray           # x,y,yaw,vx,vy,wz (odom frame pose, body-frame twist)
    imu_time: float


class Backend(Protocol):
    def reset(self, scenario: Scenario, epoch: int) -> SensorSnapshot: ...
    def advance(self, velocity: np.ndarray, steps: int, epoch: int) -> SensorSnapshot: ...
    def close(self) -> None: ...


def encode_scan(ranges, range_min, range_max, bins=72):
    """Conservative min-pool. Invalid beams are masked, never free-space."""
    rays = np.asarray(ranges, dtype=np.float64).reshape(-1)
    valid = np.isfinite(rays) & (rays >= range_min) & (rays <= range_max)
    chunks = np.array_split(np.arange(len(rays)), bins)
    dist = np.ones(bins, dtype=np.float32)
    mask = np.zeros(bins, dtype=np.float32)
    for i, ix in enumerate(chunks):
        if ix.size and valid[ix].any():
            dist[i] = float(np.min(rays[ix][valid[ix]]) / range_max)
            mask[i] = 1.0
    return dist, mask


def goal_in_odom(scenario: Scenario, initial_odom):
    """Map configured world start/goal displacement to the robot's initial odom frame.

    Does not assume that initial /odom x,y equals Gazebo world x,y.
    Assumes odom coordinate frame is locally stationary within one episode.
    """
    sx, sy, syaw = scenario.start
    dx, dy = scenario.goal[0] - sx, scenario.goal[1] - sy
    c, s = math.cos(-syaw), math.sin(-syaw)
    local_x, local_y = c * dx - s * dy, s * dx + c * dy
    ox, oy, oyaw = map(float, initial_odom[:3])
    c, s = math.cos(oyaw), math.sin(oyaw)
    return np.array([ox + c * local_x - s * local_y,
                     oy + s * local_x + c * local_y], dtype=np.float64)


def goal_features(odom, target_xy):
    dx, dy = target_xy - np.asarray(odom[:2], dtype=np.float64)
    distance = float(math.hypot(dx, dy))
    theta = math.atan2(dy, dx) - float(odom[2])
    return np.array([min(distance / 10., 1.), math.sin(theta), math.cos(theta)],
                    dtype=np.float32), distance


def require_fresh(s: SensorSnapshot, epoch: int, max_scan_age=.26,
                  max_odom_age=.16, max_imu_age=.26):
    if s.epoch != epoch:
        raise SimulatorFault('cross-episode sensor snapshot / epoch mismatch')
    if not math.isfinite(s.sim_time) or s.sim_time <= 0:
        raise SimulatorFault('nonpositive/invalid simulation time')
    if not np.all(np.isfinite(s.odom)):
        raise SimulatorFault('nonfinite odometry')
    for key, stamp, threshold in (
        ('scan', s.scan_time, max_scan_age),
        ('odom', s.odom_time, max_odom_age),
        ('imu', s.imu_time, max_imu_age),
    ):
        age = s.sim_time - stamp
        if not math.isfinite(age) or age < -.02 or age > threshold:
            raise SimulatorFault(f'stale/future {key}: age={age:.3f}s')


def encode_observation(s: SensorSnapshot, target_xy, prev_action):
    lidar, mask = encode_scan(s.scan, s.range_min, s.range_max)
    goal, distance = goal_features(s.odom, target_xy)
    # Bounded sensor ages: no ground truth/future pose enters the actor.
    age = np.array([s.sim_time - s.scan_time, s.sim_time - s.odom_time,
                    s.sim_time - s.imu_time], dtype=np.float32)
    observation = {
        'lidar': lidar,
        'lidar_mask': mask,
        'goal': goal,
        'ego': np.asarray(s.odom[3:6], dtype=np.float32).copy(),
        'prev_action': np.asarray(prev_action, dtype=np.float32).copy(),
        'sensor_age': age,
    }
    return observation, distance


def clamp_action(command, previous, max_accel=1.5, dt=POLICY_DT):
    a = np.asarray(command, dtype=np.float32)
    if a.shape != (3,) or not np.all(np.isfinite(a)):
        raise ValueError('action must be a finite Box(3,) command [vx,vy,wz]')
    a = np.clip(a, ACTION_LOW, ACTION_HIGH)
    # Same simple bound on each axis is a conservative first-stage limiter;
    # angular acceleration needs a dedicated robot-model-specific limit later.
    return np.clip(a, np.asarray(previous) - max_accel * dt,
                   np.asarray(previous) + max_accel * dt).astype(np.float32)
