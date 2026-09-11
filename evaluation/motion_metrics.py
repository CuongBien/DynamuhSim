"""Motion quality metrics: Path efficiency, velocity profiles, and Jerk smoothness."""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from evaluation.types import MotionMetrics, compute_stats


def compute_path_length_from_points(points: Sequence[Tuple[float, float]]) -> float:
    """Compute Euclidean sum of path segments."""
    if len(points) < 2:
        return 0.0
    return sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(points, points[1:]))


def compute_speed_series(samples: Sequence[Tuple[float, float, float]]) -> List[Tuple[float, float]]:
    """Compute speeds from sequence of (t, x, y) positions."""
    res: List[Tuple[float, float]] = []
    for (t0, x0, y0), (t1, x1, y1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if 1e-6 < dt <= 2.0:
            res.append((t1, math.hypot(x1 - x0, y1 - y0) / dt))
    return res


def compute_jerk_metrics(
    cmd_samples: Sequence[Tuple[float, float, float]],
    total_time_s: float,
    path_length_m: float,
) -> Tuple[Optional[float], Optional[float], Optional[float], Dict[str, Optional[float]], Dict[str, Optional[float]]]:
    """Calculate RMS Jerk and normalized smoothness metrics from command velocity stream.

    jerk = da/dt = d^2 v / dt^2.
    """
    if len(cmd_samples) < 3 or total_time_s <= 0.0:
        return None, None, None, {}, {}

    # Compute accelerations
    linear_accels: List[Tuple[float, float]] = []
    angular_accels: List[Tuple[float, float]] = []

    for (t0, v0, w0), (t1, v1, w1) in zip(cmd_samples, cmd_samples[1:]):
        dt = t1 - t0
        if 1e-4 < dt <= 1.0:
            linear_accels.append((t1, (v1 - v0) / dt))
            angular_accels.append((t1, (w1 - w0) / dt))

    if len(linear_accels) < 2:
        return None, None, None, compute_stats([]), compute_stats([])

    # Compute jerks
    linear_jerks: List[float] = []
    angular_jerks: List[float] = []
    weighted_linear_jerk_sq = 0.0

    for (t0, a0), (t1, a1) in zip(linear_accels, linear_accels[1:]):
        dt = t1 - t0
        if 1e-4 < dt <= 1.0:
            j = (a1 - a0) / dt
            linear_jerks.append(j)
            weighted_linear_jerk_sq += (j ** 2) * dt

    for (t0, alpha0), (t1, alpha1) in zip(angular_accels, angular_accels[1:]):
        dt = t1 - t0
        if 1e-4 < dt <= 1.0:
            j_ang = (alpha1 - alpha0) / dt
            angular_jerks.append(j_ang)

    linear_jerk_rms = (
        math.sqrt(sum(j ** 2 for j in linear_jerks) / len(linear_jerks))
        if linear_jerks else None
    )
    angular_jerk_rms = (
        math.sqrt(sum(j ** 2 for j in angular_jerks) / len(angular_jerks))
        if angular_jerks else None
    )

    # Dimensionless jerk cost: sqrt( (T^5 / (2 * L^2)) * int(j^2 dt) )
    normalized_jerk: Optional[float] = None
    if path_length_m > 0.1 and total_time_s > 0.5 and weighted_linear_jerk_sq > 0:
        val = (total_time_s ** 5) / (2.0 * (path_length_m ** 2)) * weighted_linear_jerk_sq
        normalized_jerk = math.sqrt(val) if val > 0 else None

    return (
        linear_jerk_rms,
        angular_jerk_rms,
        normalized_jerk,
        compute_stats([a for _, a in linear_accels]),
        compute_stats([alpha for _, alpha in angular_accels]),
    )


def compute_motion_metrics(
    robot_trajectory: Sequence[Tuple[float, float, float]],
    cmd_samples: Sequence[Tuple[float, float, float]],
    goal_xy: Optional[Tuple[float, float]] = None,
) -> MotionMetrics:
    """Compute aggregated motion metrics."""
    if not robot_trajectory:
        return MotionMetrics()

    points = [(x, y) for _, x, y in robot_trajectory]
    path_len = compute_path_length_from_points(points)
    total_time = robot_trajectory[-1][0] - robot_trajectory[0][0]

    path_eff = None
    if goal_xy is not None and len(points) > 0:
        straight_dist = math.hypot(goal_xy[0] - points[0][0], goal_xy[1] - points[0][1])
        if path_len > 0:
            path_eff = min(1.0, straight_dist / path_len)

    (
        lin_jerk_rms,
        ang_jerk_rms,
        norm_jerk,
        lin_acc_stats,
        ang_acc_stats,
    ) = compute_jerk_metrics(cmd_samples, total_time, path_len)

    return MotionMetrics(
        total_path_length_m=path_len,
        path_efficiency_ratio=path_eff,
        linear_jerk_rms_mps3=lin_jerk_rms,
        angular_jerk_rms_radps3=ang_jerk_rms,
        normalized_jerk_cost=norm_jerk,
        linear_acceleration_stats=lin_acc_stats,
        angular_acceleration_stats=ang_acc_stats,
    )
