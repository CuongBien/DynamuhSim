"""Failure taxonomy and outcome classification based on NavigationAnalyzer."""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from evaluation.types import FailureType


def classify_failure(
    odom_samples: Sequence[Tuple[float, float, float, float]],
    proximity_samples: Sequence[Tuple[float, float, float]],
    cmd_samples: Sequence[Tuple[float, float, float]],
    goal_xy: Optional[Tuple[float, float]],
    goal_tolerance: float = 0.20,
    trial_timeout_s: float = 180.0,
    deadlock_speed_threshold: float = 0.02,
    deadlock_duration_s: float = 5.0,
    collision_clearance_threshold: float = 0.0,
) -> Tuple[FailureType, str, Dict[str, Any]]:
    """Classify the final outcome of a trial according to rigorous robotic failure taxonomy.

    Returns (FailureType, explanation_string, details_dict).
    """
    if not odom_samples:
        return FailureType.GOAL_NOT_REACHED, "No odometry data available", {}

    start_pos = (odom_samples[0][1], odom_samples[0][2])
    final_pos = (odom_samples[-1][1], odom_samples[-1][2])
    trial_duration = odom_samples[-1][0] - odom_samples[0][0]

    details: Dict[str, Any] = {
        "start_xy": list(start_pos),
        "final_xy": list(final_pos),
        "trial_duration_s": trial_duration,
        "goal_configured": goal_xy is not None,
    }

    # Populate goal metrics if configured
    reached_goal = False
    if goal_xy is not None:
        final_dist_to_goal = math.hypot(final_pos[0] - goal_xy[0], final_pos[1] - goal_xy[1])
        initial_dist_to_goal = math.hypot(start_pos[0] - goal_xy[0], start_pos[1] - goal_xy[1])
        progress = initial_dist_to_goal - final_dist_to_goal
        fraction = progress / initial_dist_to_goal if initial_dist_to_goal > 0 else None
        details["final_distance_m"] = final_dist_to_goal
        details["final_distance_to_goal_m"] = final_dist_to_goal
        details["initial_distance_m"] = initial_dist_to_goal
        details["initial_distance_to_goal_m"] = initial_dist_to_goal
        details["goal_tolerance_m"] = goal_tolerance
        details["progress_m"] = progress
        details["progress_fraction"] = fraction
        details["goal_xy_m"] = list(goal_xy)
        reached_goal = final_dist_to_goal <= goal_tolerance
    else:
        details["progress_m"] = None
        details["progress_fraction"] = None
        details["goal_xy_m"] = None

    # 1. Collision detection
    min_clearance = min((c for _, _, c in proximity_samples), default=float("inf"))
    min_center_d = min((d for _, d, _ in proximity_samples), default=float("inf"))
    details["min_clearance_m"] = min_clearance if math.isfinite(min_clearance) else None
    details["min_center_distance_m"] = min_center_d if math.isfinite(min_center_d) else None

    if min_clearance <= collision_clearance_threshold:
        return (
            FailureType.COLLISION_OBSTACLE,
            f"Physical collision detected: estimated clearance {min_clearance:.3f}m <= 0.0m",
            details,
        )

    # 2. Goal reach check
    if reached_goal:
        return FailureType.SUCCESS, f"Goal reached successfully (distance {final_dist_to_goal:.3f}m <= {goal_tolerance:.3f}m)", details

    # 3. Deadlock in narrow corridor check
    # Occurs when robot is stationary for >= deadlock_duration_s while obstacle is near (< 2.0m)
    stagnant_start: Optional[float] = None
    longest_deadlock = 0.0
    for i, (t, x, y, speed) in enumerate(odom_samples):
        # Look up nearest proximity
        obs_dist = proximity_samples[i][1] if i < len(proximity_samples) else float("inf")
        # In narrow corridor (abs(y) < 0.45m) and facing obstacle close-up
        is_stuck = (speed < deadlock_speed_threshold) and (obs_dist < 2.20) and (abs(y) < 0.50)

        if is_stuck:
            if stagnant_start is None:
                stagnant_start = t
            else:
                dur = t - stagnant_start
                longest_deadlock = max(longest_deadlock, dur)
        else:
            stagnant_start = None

    details["longest_narrow_stagnation_s"] = longest_deadlock
    if longest_deadlock >= deadlock_duration_s:
        return (
            FailureType.DEADLOCK_NARROW,
            f"Deadlock in narrow corridor: stationary for {longest_deadlock:.1f}s facing oncoming obstacle",
            details,
        )

    # 4. Local Minima Oscillation check
    # Check for frequent sign reversals of angular velocity with low linear progression
    if len(cmd_samples) > 20:
        window_s = 4.0
        max_reversals = 0
        for i, (t0, _, _) in enumerate(cmd_samples):
            w_in_window = [w for t, _, w in cmd_samples[i:] if t - t0 <= window_s]
            if len(w_in_window) < 8:
                continue
            # Count sign flips where |w| > 0.08 rad/s
            filtered_w = [w for w in w_in_window if abs(w) > 0.08]
            flips = sum(1 for a, b in zip(filtered_w, filtered_w[1:]) if (a * b) < 0)
            max_reversals = max(max_reversals, flips)

        details["max_angular_oscillation_flips"] = max_reversals
        if max_reversals >= 5 and odom_samples[-1][3] < 0.05:
            return (
                FailureType.LOCAL_MINIMA_OSCILLATION,
                f"Local minima oscillation detected: {max_reversals} angular sign reversals in {window_s}s window",
                details,
            )

    # 5. Timeout check
    if trial_duration >= trial_timeout_s - 1.0:
        return (
            FailureType.TIMEOUT,
            f"Trial exceeded maximum timeout ({trial_duration:.1f}s >= {trial_timeout_s:.1f}s)",
            details,
        )

    # 6. Fallback
    if goal_xy is None:
        return FailureType.UNAVAILABLE_NO_GOAL, "No target goal specified", details
    return FailureType.GOAL_NOT_REACHED, "Navigation finished without reaching goal tolerance", details
