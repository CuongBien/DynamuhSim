"""Social Navigation Metrics: Personal Space Intrusion (PSI) and Proactive Yielding."""
from __future__ import annotations

import math
import statistics
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

from evaluation.spatial_alignment import is_in_alcove
from evaluation.types import Encounter, PSIMetrics, SocialSpace, YieldingMetrics, compute_stats, finite


def median_mad(values: Sequence[float]) -> Tuple[Optional[float], Optional[float]]:
    """Compute median and median absolute deviation (MAD)."""
    good = finite(values)
    if not good:
        return None, None
    center = statistics.median(good)
    mad = statistics.median([abs(v - center) for v in good])
    return center, mad


def duration_where(
    samples: Sequence[Tuple[float, float]],
    threshold: float,
    after: float,
    until: Optional[float] = None,
) -> Tuple[float, float]:
    """Compute total and longest continuous duration where value <= threshold."""
    run_start: Optional[float] = None
    total = 0.0
    longest = 0.0
    prev_t: Optional[float] = None

    for t, val in samples:
        if t < after or (until is not None and t > until):
            continue
        is_below = val <= threshold
        contiguous = prev_t is not None and (t - prev_t <= 1.5)

        if is_below and (run_start is None or not contiguous):
            if run_start is not None and prev_t is not None:
                d = prev_t - run_start
                total += d
                longest = max(longest, d)
            run_start = t
        elif not is_below and run_start is not None:
            if prev_t is not None:
                d = prev_t - run_start
                total += d
                longest = max(longest, d)
            run_start = None
        prev_t = t

    if run_start is not None and prev_t is not None:
        d = prev_t - run_start
        total += d
        longest = max(longest, d)

    return total, longest


def detect_encounters(
    proximity: Sequence[Tuple[float, float, float]],
    enter: float = 1.50,
    exit_: float = 1.80,
    min_duration: float = 0.50,
) -> List[Dict[str, Any]]:
    """Segment close interaction episodes with distance hysteresis."""
    encounters: List[Dict[str, Any]] = []
    active: Optional[List[Tuple[float, float, float]]] = None
    previous: Optional[Tuple[float, float, float]] = None

    for row in proximity:
        t, center, _ = row
        if active is not None and previous is not None and t - previous[0] > 1.5:
            if previous[0] - active[0][0] >= min_duration:
                encounters.append({"start_s": active[0][0], "end_s": previous[0], "samples": active})
            active = None

        if active is None:
            if center <= enter:
                active = [row]
        else:
            active.append(row)
            if center >= exit_:
                if t - active[0][0] >= min_duration:
                    encounters.append({"start_s": active[0][0], "end_s": t, "samples": active})
                active = None
        previous = row

    if active is not None and previous is not None and (previous[0] - active[0][0] >= min_duration):
        encounters.append({"start_s": active[0][0], "end_s": previous[0], "samples": active})

    return encounters


def command_change_reaction(
    samples: Sequence[Tuple[float, float, float]],
    start: float,
    end: float,
    linear_center: float,
    linear_mad: float,
    angular_abs_center: float,
    angular_abs_mad: float,
    linear_min_decrease: float = 0.05,
    angular_min_increase: float = 0.08,
    hold_s: float = 0.25,
) -> Tuple[Optional[float], List[str], Optional[float], Optional[float]]:
    """Detect sustained command change persisting for >= hold_s."""
    linear_delta = max(linear_min_decrease, 3.0 * linear_mad)
    angular_delta = max(angular_min_increase, 3.0 * angular_abs_mad)
    candidate: Optional[float] = None
    modes: Set[str] = set()
    prev_t: Optional[float] = None
    candidate_linear: Optional[float] = None
    candidate_angular: Optional[float] = None

    for t, linear_x, angular_z in samples:
        if t < start:
            continue
        if t > end:
            break

        changed_modes = []
        if linear_x < linear_center - linear_delta:
            changed_modes.append("linear_x_decrease")
        if abs(angular_z) > angular_abs_center + angular_delta:
            changed_modes.append("angular_z_increase")

        if changed_modes:
            if candidate is None or (prev_t is not None and t - prev_t > 1.5):
                candidate = t
                modes = set()
                candidate_linear = linear_x
                candidate_angular = angular_z
            modes.update(changed_modes)
            if candidate is not None and t - candidate >= hold_s:
                return candidate, sorted(modes), candidate_linear, candidate_angular
        else:
            candidate = None
            modes = set()
            candidate_linear = None
            candidate_angular = None
        prev_t = t

    return None, [], None, None


def compute_psi_metrics(
    proximity_samples: Sequence[Tuple[float, float, float]],
    total_trial_time_s: float,
    intimate_threshold_m: float = 0.45,
    personal_threshold_m: float = 1.20,
) -> PSIMetrics:
    """Compute Personal Space Intrusion metrics according to Hall's proxemic zones."""
    if not proximity_samples:
        return PSIMetrics(intimate_threshold_m=intimate_threshold_m, personal_threshold_m=personal_threshold_m)

    min_dist = float("inf")
    min_clearance = float("inf")
    time_at_min = None
    intimate_time = 0.0
    personal_time = 0.0
    intimate_events = 0
    in_intimate = False

    for i in range(len(proximity_samples)):
        t, center_d, clearance = proximity_samples[i]
        dt = 0.0
        if i > 0:
            dt = t - proximity_samples[i - 1][0]
            if dt > 1.5 or dt < 0:
                dt = 0.0

        if center_d < min_dist:
            min_dist = center_d
            min_clearance = clearance
            time_at_min = t

        if center_d < intimate_threshold_m:
            intimate_time += dt
            if not in_intimate:
                intimate_events += 1
                in_intimate = True
        else:
            in_intimate = False

        if center_d < personal_threshold_m:
            personal_time += dt

    intimate_ratio = (intimate_time / total_trial_time_s) if total_trial_time_s > 0 else 0.0
    personal_ratio = (personal_time / total_trial_time_s) if total_trial_time_s > 0 else 0.0

    return PSIMetrics(
        intimate_threshold_m=intimate_threshold_m,
        personal_threshold_m=personal_threshold_m,
        total_intimate_time_s=intimate_time,
        intimate_time_ratio=intimate_ratio,
        total_personal_time_s=personal_time,
        personal_time_ratio=personal_ratio,
        minimum_distance_m=min_dist if math.isfinite(min_dist) else None,
        minimum_clearance_m=min_clearance if math.isfinite(min_clearance) else None,
        time_at_min_distance_s=time_at_min,
        intimate_intrusions_count=intimate_events,
    )


def compute_yielding_metrics(
    robot_trajectory: Sequence[Tuple[float, float, float]],
    obstacle_trajectory_interpolated: Sequence[Optional[Tuple[float, float]]],
    spawn_x: float = -13.0,
) -> YieldingMetrics:
    """Analyze whether the robot performed proactive yielding into the alcove recess."""
    entered_alcove = False
    alcove_entry_time: Optional[float] = None
    alcove_exit_time: Optional[float] = None
    alcove_duration = 0.0
    max_depth = 0.0

    yielding_initiation_time: Optional[float] = None
    distance_at_yielding: Optional[float] = None
    prev_t: Optional[float] = None

    for (t, rx, ry), obs_xy in zip(robot_trajectory, obstacle_trajectory_interpolated):
        in_alcove_now = is_in_alcove(rx, ry, frame="odom", spawn_x=spawn_x)
        if in_alcove_now:
            max_depth = max(max_depth, ry)
            if not entered_alcove:
                entered_alcove = True
                alcove_entry_time = t

            if prev_t is not None and t - prev_t <= 1.5:
                alcove_duration += (t - prev_t)
            alcove_exit_time = t
        else:
            # Check for initiation of lateral steering towards alcove (y > 0.08m heading into recess)
            if (
                not entered_alcove
                and yielding_initiation_time is None
                and ry > 0.08
                and rx >= 12.0  # Approaching alcove region in odom frame
            ):
                yielding_initiation_time = t
                if obs_xy is not None:
                    distance_at_yielding = math.hypot(rx - obs_xy[0], ry - obs_xy[1])

        prev_t = t

    if entered_alcove and yielding_initiation_time is None:
        yielding_initiation_time = alcove_entry_time

    return YieldingMetrics(
        yielding_detected=entered_alcove,
        entered_alcove=entered_alcove,
        alcove_entry_time_s=alcove_entry_time,
        alcove_exit_time_s=alcove_exit_time,
        alcove_duration_s=alcove_duration,
        yielding_initiation_time_s=yielding_initiation_time,
        distance_to_obstacle_at_yielding_m=distance_at_yielding,
        max_alcove_depth_reached_m=max_depth,
    )
