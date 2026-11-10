"""Metrics from synchronized control-tick GT and native solver contact samples."""

from collections import Counter
from itertools import pairwise
from math import dist, hypot, sqrt
from statistics import fmean

PSC_RADIUS_M = 0.5
STALL_SPEED_MPS = 0.02
PROGRESS_EPS_M = 1e-6
FREEZE_S = 5.0
SECOND = 1_000_000_000


def ttc(relative, velocity, radius):
    """First nonnegative disc contact under constant relative velocity."""
    x, y = relative
    vx, vy = velocity
    c = x * x + y * y - radius * radius
    if c <= 0:
        return 0.0
    a = vx * vx + vy * vy
    b = x * vx + y * vy
    discriminant = b * b - a * c
    if not a or b >= 0 or discriminant < 0:
        return None
    return c / (-b + sqrt(discriminant))


def distribution(values):
    values = list(values)
    return (
        {"min": min(values), "mean": fmean(values), "max": max(values)}
        if values
        else None
    )


def motion_stats(actor, dt):
    speed = [hypot(*state["velocity"]) for state in actor]
    accelerations = [
        dist(a["velocity"], b["velocity"]) / dt for a, b in pairwise(actor)
    ]
    jerk = [
        abs(c - 2 * b + a) / (dt * dt)
        for a, b, c in zip(speed, speed[1:], speed[2:], strict=False)
    ]
    return {
        "speed_mps": distribution(speed),
        "acceleration_mps2": distribution(accelerations),
        "jerk_mps3": distribution(jerk),
        "path_length_m": sum(dist(a["xy"], b["xy"]) for a, b in pairwise(actor)),
        "stall_s": sum(s < STALL_SPEED_MPS for s in speed[1:]) * dt,
    }


def progress(actor, goal, rows, dt, *, honor_wait=True):
    best = dist(actor[0]["xy"], goal)
    duration = longest = count = 0
    for state, row in zip(actor[1:], rows[1:], strict=True):
        remaining = dist(state["xy"], goal)
        if best - remaining > PROGRESS_EPS_M:
            best, duration = remaining, 0
        elif not honor_wait or not row["intentional_wait"]:
            duration += 1
            longest = max(duration, longest)
            count += (duration - 1) * dt <= FREEZE_S < duration * dt
    return count, longest * dt


def contacts_by_kind(events, physics_dt_ns):
    samples, episodes, last = Counter(), Counter(), {}
    for contact in sorted(events, key=lambda event: event["time_ns"]):
        kind = contact["kind"]
        key = (kind, *sorted((contact["first_id"], contact["second_id"])))
        stamp = contact["time_ns"]
        samples[kind] += 1
        if key not in last or stamp - last[key] > physics_dt_ns:
            episodes[kind] += 1
        last[key] = stamp
    return samples, episodes


def _social(rows, radii, dt):
    distances, times, active = [], [], set()
    encounter_count = violations = 0
    for row in rows:
        robot = row["robot"]
        seen = set()
        for person, human in row["humans"].items():
            relative = tuple(
                p - r for p, r in zip(human["xy"], robot["xy"], strict=True)
            )
            velocity = tuple(
                p - r for p, r in zip(human["velocity"], robot["velocity"], strict=True)
            )
            distance = hypot(*relative)
            distances.append(distance)
            horizon = ttc(relative, velocity, radii["robot"] + radii[person])
            if horizon is not None:
                times.append(horizon)
            if distance < PSC_RADIUS_M:
                seen.add(person)
        if row["time_ns"] > 0:
            violations += bool(seen)
        encounter_count += len(seen - active)
        active = seen
    return {
        "psc_fraction": violations / (len(rows) - 1)
        if len(rows) > 1 and distances
        else None,
        "encounters": encounter_count if distances else None,
        "encounter_threshold_m": PSC_RADIUS_M,
        "min_center_distance_m": min(distances, default=None),
        "min_ttc_s": min(times, default=None),
        "ttc_status": "finite"
        if times
        else "no_predicted_contact"
        if distances
        else "unavailable",
        "measured_duration_s": (len(rows) - 1) * dt,
    }


def evaluate(
    rows,
    events,
    *,
    timing,
    end_reason,
    infra_valid,
    goals,
    radii,
    goal_tolerance,
    solver_unobservable,
    shortest_path_m=None,
    label_valid=None,
):
    dt = timing["robot_control_dt_ns"] / SECOND
    samples, episodes = contacts_by_kind(events, timing["physics_dt_ns"])
    known = ("robot_wall", "robot_human", "human_wall", "human_human")
    robot_contacts = (
        sum(episodes[kind] for kind in ("robot_wall", "robot_human"))
        if infra_valid and not set(solver_unobservable) & {"robot_wall", "robot_human"}
        else None
    )
    completed = end_reason == "goal_reached" if infra_valid else None
    timed_out = end_reason == "time_budget" if infra_valid else None
    clearance = {
        name: distribution(
            row["clearance"][name]
            for row in rows
            if row["clearance"] is not None and row["clearance"].get(name) is not None
        )
        for name in ("robot_wall_m", "robot_human_m", "human_wall_m", "human_human_m")
    }
    physical_overlap = any(
        values and values["min"] < 0 for values in clearance.values()
    )
    expected = ("robot_wall_m", "robot_human_m", "human_wall_m") + (
        ("human_human_m",) if len(goals) > 2 else ()
    )
    clearance_complete = bool(rows) and all(
        row["clearance"] is not None
        and all(row["clearance"].get(name) is not None for name in expected)
        for row in rows
    )
    human_overlap = any(
        clearance[name] is not None and clearance[name]["min"] < 0
        for name in ("human_wall_m", "human_human_m")
    )
    metrics = {
        "infrastructure_valid": infra_valid,
        "human_physics_valid": (
            False
            if human_overlap
            else True
            if infra_valid and clearance_complete
            else None
        ),
        "scenario_label_valid": label_valid,
        "clearance_m": clearance,
        "physical_overlap": physical_overlap
        if clearance_complete or physical_overlap
        else None,
        "solver_samples": {
            k: None if k in solver_unobservable else samples[k] if rows else None
            for k in known
        },
        "solver_contact_episodes": {
            k: None if k in solver_unobservable else episodes[k] if rows else None
            for k in known
        },
        "unclassified_contact_samples": (
            sum(value for key, value in samples.items() if key not in known)
            if rows
            else None
        ),
        "hard": {"S": completed, "C": robot_contacts, "TO": timed_out, "FP": None},
        "outcome": "INVALID_INFRA" if not infra_valid else None,
        "human_behavior": "NOT_EVALUABLE",
        "episode_task_pass": None,
        "guard_substeps": sum(row["guard_substeps"] for row in rows) if rows else None,
        "quality": None,
        "quality_scope": "complete_episode"
        if infra_valid
        else "committed_prefix"
        if rows
        else None,
        "not_measured": {
            "survey_and_learned_realism": "no independent survey or learned reference data",
            "legibility": "no human-rated perception evidence",
            "card_specific_quality": "PI, group cohesion and crossing distance need approved definitions",
            "other_actor_collision": "adapter has no separate AC category",
        },
    }
    if not rows:
        return metrics
    robot = [row["robot"] for row in rows]
    people = {key: [row["humans"][key] for row in rows] for key in rows[0]["humans"]}
    human_goals = {
        key: next(
            (
                r["time_ns"] / SECOND
                for r, state in zip(rows, states, strict=True)
                if dist(state["xy"], goals[key]) <= goal_tolerance
            ),
            None,
        )
        for key, states in people.items()
    }
    human_freeze = {
        key: progress(states, goals[key], rows, dt, honor_wait=False)[1]
        if human_goals[key] is None
        else None
        for key, states in people.items()
    }
    freeze, longest = progress(robot, goals["robot"], rows, dt)
    robots = motion_stats(robot, dt)
    path = robots["path_length_m"]
    metrics["hard"]["FP"] = freeze if infra_valid else None
    metrics["quality"] = {
        **_social(rows, radii, dt),
        "robot": robots,
        "humans": {key: motion_stats(states, dt) for key, states in people.items()},
        "human_goal_s": human_goals,
        "human_unfinished_progress_freeze_s": human_freeze,
        "robot_goal_s": rows[-1]["time_ns"] / SECOND if completed else None,
        "group_completion_s": (
            max(rows[-1]["time_ns"] / SECOND, *human_goals.values())
            if completed
            and human_goals
            and all(v is not None for v in human_goals.values())
            else None
        ),
        "spl": (
            float(completed) * shortest_path_m / max(path, shortest_path_m)
            if infra_valid and shortest_path_m is not None
            else None
        ),
        "longest_robot_progress_freeze_s": longest,
        "human_count": len(people),
        "trajectory_count": 1 + len(people),
    }
    if infra_valid:
        unsafe = bool(
            robot_contacts
            or physical_overlap
            or metrics["unclassified_contact_samples"]
        )
        if metrics["human_physics_valid"] is False or unsafe:
            metrics["human_behavior"] = "SAFETY_FAILURE"
        elif not human_goals or not clearance_complete:
            metrics["human_behavior"] = "NOT_EVALUABLE"
        elif all(v is not None for v in human_goals.values()):
            metrics["human_behavior"] = "GOALS_REACHED"
        elif any(v is not None and v > FREEZE_S for v in human_freeze.values()):
            metrics["human_behavior"] = "NO_PROGRESS"
        else:
            metrics["human_behavior"] = "GOALS_INCOMPLETE"
        metrics["episode_task_pass"] = bool(
            completed
            and robot_contacts == 0
            and not timed_out
            and freeze == 0
            and not unsafe
            and clearance_complete
        )
        metrics["outcome"] = (
            "SAFETY_FAILURE"
            if unsafe
            else "TIMEOUT"
            if timed_out
            else "INSUFFICIENT_MEASUREMENT"
            if robot_contacts is None or not clearance_complete
            else "TASK_SUCCESS"
            if metrics["episode_task_pass"]
            else "TASK_FAILURE"
        )
    return metrics
