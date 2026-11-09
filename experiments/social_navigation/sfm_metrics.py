"""Research-fixture metrics over the existing robot-tick and solver records."""

from collections import Counter
from math import hypot, sqrt

from narrow_nav.core.types import NS_PER_SECOND

PSC_RADIUS_M = 0.5
MAX_PROGRESS_FREEZE_S = 5.0
# Technical measurement parameters absent from the source card.
STALL_SPEED_MPS = 0.02
PROGRESS_EPS_M = 1e-6


def footprint_ttc(relative_xy, relative_velocity, radius):
    x, y = relative_xy
    vx, vy = relative_velocity
    c = x * x + y * y - radius * radius
    if c <= 0:
        return 0.0
    a = vx * vx + vy * vy
    b = x * vx + y * vy
    discriminant = b * b - a * c
    if a == 0 or b >= 0 or discriminant < 0:
        return None  # JSON representation of no finite TTC.
    return c / (-b + sqrt(discriminant))


def contact_counts(contacts, physics_dt_ns):
    samples, episodes, last = Counter(), Counter(), {}
    for contact in contacts:
        kind = contact["kind"]
        pair = (kind, contact["first_id"], contact["second_id"])
        time_ns = contact["time_ns"]
        samples[kind] += 1
        if pair not in last or time_ns - last[pair] > physics_dt_ns:
            episodes[kind] += 1
        last[pair] = time_ns
    return dict(samples), dict(episodes)


def progress(rows, dt):
    best = rows[0]["robot_goal_distance_m"]
    freeze = maximum = episodes = 0
    for row in rows[1:]:
        distance = row["robot_goal_distance_m"]
        if best - distance > PROGRESS_EPS_M:
            best, freeze = distance, 0
        else:
            freeze += 1
            maximum = max(maximum, freeze)
            episodes += (freeze - 1) * dt <= MAX_PROGRESS_FREEZE_S < freeze * dt
    return maximum * dt, episodes


def _motion_summary(rows, actor):
    return {
        name: max(row.get(f"{actor}_{name}") or 0 for row in rows)
        for name in (
            "speed_mps",
            "acceleration_mps2",
            "jerk_mps3",
            "continuity_residual_m",
        )
    }


def summarize(rows, contacts, timing, status, reason, *, physically_feasible):
    dt = timing.robot_control_dt_ns / NS_PER_SECOND
    samples, collisions = contact_counts(contacts, timing.physics_dt_ns)
    observed = rows[1:]
    minimums = {
        key: min((row[key] for row in rows if row[key] is not None), default=None)
        for key in ("robot_human_m", "robot_wall_m", "human_wall_m", "human_human_m")
    }
    negative = any(value is not None and value < 0 for value in minimums.values())
    freeze_s, fp = progress(rows, dt)
    robot_goal = reason == "goal_reached"
    timeout = reason == "time_budget"
    failures = []
    if not robot_goal:
        failures.append("robot_goal_not_reached")
    if timeout:
        failures.append("timeout")
    if collisions:
        failures.append("solver_collision")
    if negative:
        failures.append("negative_footprint_clearance")
    if fp:
        failures.append("failure_to_progress")
    if not physically_feasible:
        failures.append("side_by_side_geometry_infeasible")
    if status == "failed":
        failures.append("runtime_failed")
    return {
        "verdict": "FAIL" if failures else "PASS",
        "failures": failures,
        "hard_constraints": {
            "S": robot_goal,
            "C": sum(collisions.values()),
            "TO": timeout,
            "FP": fp,
        },
        "human_goal_reached": any(row["human_at_goal"] for row in rows),
        "human_heading_velocity_error_max_rad": max(
            (
                row["human_heading_velocity_error_rad"]
                for row in rows
                if row["human_heading_velocity_error_rad"] is not None
            ),
            default=None,
        ),
        "human_final_goal_distance_m": rows[-1]["human_goal_distance_m"],
        "human_first_goal_s": next(
            (row["time_s"] for row in rows if row["human_at_goal"]), None
        ),
        "collision_samples_by_type": samples,
        "collision_episodes_by_type": collisions,
        "minimum_clearance_m": minimums,
        "negative_clearance_ticks": sum(
            any(row[key] is not None and row[key] < 0 for key in minimums)
            for row in observed
        ),
        "robot_stalled_time_s": sum(
            row["robot_speed_mps"] < STALL_SPEED_MPS for row in observed
        )
        * dt,
        "human_stalled_before_goal_s": sum(
            not row["human_at_goal"] and row["human_speed_mps"] < STALL_SPEED_MPS
            for row in observed
        )
        * dt,
        "max_robot_progress_freeze_s": freeze_s,
        "min_robot_human_center_distance_m": min(
            row["center_distance_m"] for row in rows
        ),
        "min_robot_human_ttc_s": min(
            (row["ttc_s"] for row in rows if row["ttc_s"] is not None), default=None
        ),
        "personal_space_violation_fraction": (
            sum(row["center_distance_m"] < PSC_RADIUS_M for row in observed)
            / len(observed)
            if observed
            else None
        ),
        "guard_blocked_substeps": sum(
            row["guard_blocked_substeps"] for row in observed
        ),
        "robot_max": _motion_summary(rows, "robot"),
        "human_max": _motion_summary(rows, "human"),
    }


def append_dynamics(row, previous, dt):
    for actor in ("robot", "human"):
        velocity = row[f"{actor}_velocity"]
        row[f"{actor}_speed_mps"] = hypot(*velocity)
        if previous is None:
            row[f"{actor}_speed_change_mps2"] = None
            row[f"{actor}_acceleration_mps2"] = None
            row[f"{actor}_jerk_mps3"] = None
            row[f"{actor}_continuity_residual_m"] = None
            continue
        old_velocity = previous[f"{actor}_velocity"]
        acceleration = tuple(
            (v - v0) / dt for v, v0 in zip(velocity, old_velocity, strict=True)
        )
        row[f"{actor}_acceleration"] = acceleration
        row[f"{actor}_acceleration_mps2"] = hypot(*acceleration)
        speed_change = (row[f"{actor}_speed_mps"] - previous[f"{actor}_speed_mps"]) / dt
        row[f"{actor}_speed_change_mps2"] = speed_change
        old_speed_change = previous[f"{actor}_speed_change_mps2"]
        row[f"{actor}_jerk_mps3"] = (
            abs(speed_change - old_speed_change) / dt
            if old_speed_change is not None
            else None
        )
        xy, previous_xy = row[f"{actor}_xy"], previous[f"{actor}_xy"]
        row[f"{actor}_continuity_residual_m"] = hypot(
            *(
                end - start - executed * dt
                for end, start, executed in zip(
                    xy, previous_xy, row[f"{actor}_executed"], strict=True
                )
            )
        )
