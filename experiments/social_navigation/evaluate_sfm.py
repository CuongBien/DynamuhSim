"""Run a bounded SFM research fixture suite and export complete failure evidence."""

import argparse
import csv
import json
from dataclasses import asdict
from math import atan2, dist
from pathlib import Path

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ContractError
from narrow_nav.core.geometry import body_velocity_to_world, wrap_yaw
from narrow_nav.core.types import NS_PER_SECOND, Twist2D
from narrow_nav.humans.registry import create_human_backend
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import build_world
from narrow_nav.simulation.world_geometry import opening_clearance

from .probe_jupedsim_coupling import ACTIONS, COMMON_OVERRIDES, FIXTURES, ROOT
from .probe_jupedsim_external_actor import measure_neighbor_response
from .sfm_metrics import append_dynamics, footprint_ttc, summarize

BRANCH_START_NS = 4_000_000_000
BRANCH_END_NS = 6_000_000_000
CONFIG = (
    "humans=sfm_jupedsim",
    "robots=mecanum_robot",
    "simulation.human_visuals=proxy",
    "worlds.length_m=10",
    "experiments.duration_ns=30000000000",
    "scenarios.robot.initial_pose=[-3,-0.17,0]",
    "scenarios.robot.goal=[3,-0.17]",
    "scenarios.humans.0.initial_pose=[3,0.17,-3.141592653589793]",
    "scenarios.humans.0.goal=[-3,0.17]",
)
FREE = (
    "worlds.width_m=6",
    "scenarios.robot.initial_pose=[-3,-1.9,0]",
    "scenarios.robot.goal=[3,-1.9]",
    "scenarios.humans.0.initial_pose=[3,1.1,-3.141592653589793]",
    "scenarios.humans.0.goal=[-3,1.1]",
)
REPLAY = tuple(o for o in COMMON_OVERRIDES if not o.startswith("humans.backend="))
CASES = {
    "free": (2.1, "advance", 100, FREE),
    "free_repeat": (2.1, "advance", 100, FREE),
    "static": (2.1, "static", 100, ()),
    **{f"easy_{policy}": (2.1, policy, 100, ()) for policy in ACTIONS},
    "critical_100": (1.2, "advance", 100, ()),
    "hard_100": (0.9, "advance", 100, ()),
    "critical_50": (1.2, "advance", 50, ()),
    "wall_04s_100": (0.85, "advance", 100, (*REPLAY, *FIXTURES["narrow_wall"])),
    "wall_04s_50": (0.85, "advance", 50, (*REPLAY, *FIXTURES["narrow_wall"])),
}


def _xy(value):
    return value.x, value.y


def _v(value):
    return value.linear_x, value.linear_y


def action_at(time_ns, policy):
    if policy == "static":
        return 0.0, 0.0
    if not BRANCH_START_NS <= time_ns < BRANCH_END_NS:
        return ACTIONS["advance"]
    if policy == "yield" and time_ns >= (BRANCH_START_NS + BRANCH_END_NS) // 2:
        return 0.0, -ACTIONS["yield"][1]
    return ACTIONS[policy]


def _trace_row(truth, clearance, scene, goal_tolerance, result=None):
    robot, (human,) = truth.robot, truth.humans
    rxy, hxy = _xy(robot.pose), _xy(human.pose)
    rv, hv = _v(body_velocity_to_world(robot.velocity, robot.pose)), _v(human.velocity)
    row = {
        "time_ns": truth.stamp.timestamp.time_ns,
        "time_s": truth.stamp.timestamp.time_s,
        "robot_xy": rxy,
        "human_xy": hxy,
        "robot_velocity": rv,
        "human_velocity": hv,
        "robot_yaw_rad": robot.pose.yaw,
        "human_yaw_rad": human.pose.yaw,
        "human_heading_velocity_error_rad": (
            abs(wrap_yaw(human.pose.yaw - atan2(hv[1], hv[0])))
            if hv != (0.0, 0.0)
            else None
        ),
        "robot_goal_distance_m": dist(rxy, _xy(scene.robot.goal)),
        "human_goal_distance_m": dist(hxy, _xy(human.goal)),
        "human_at_goal": dist(hxy, _xy(human.goal)) <= goal_tolerance,
        "center_distance_m": dist(rxy, hxy),
        "ttc_s": footprint_ttc(
            (hxy[0] - rxy[0], hxy[1] - rxy[1]),
            (hv[0] - rv[0], hv[1] - rv[1]),
            scene.robot.radius_m + scene.humans[0].radius_m,
        ),
        **asdict(clearance),
        "guard_blocked_substeps": 0,
        "contact_samples": 0,
    }
    if result is not None:
        row["guard_blocked_substeps"] = result.human_motion_blocked_substeps
        row["contact_samples"] = len(result.contacts)
        (motion,) = result.human_motion_results
        row["robot_requested_body"] = _v(result.requested.robot.velocity)
        row["human_requested"] = _v(motion.requested.velocity)
        row["human_limited"] = _v(motion.limited.velocity)
        row["human_executed"] = _v(motion.executed.velocity)
    return row


def run_case(name, seed=11):
    width, policy, period_ms, overrides = CASES[name]
    config = compose_config(
        ROOT / "configs",
        (
            *CONFIG,
            f"worlds.width_m={width}",
            *overrides,
            f"simulation.human_control_dt_ns={period_ms * 1_000_000}",
        ),
    )
    world = build_world(config)
    scene = world.scene
    measured_width = inspect_world(world).clear_width(0)
    slack = opening_clearance(
        measured_width, scene.robot.radius_m, scene.humans[0].radius_m
    )
    backend = create_human_backend(config.humans.backend)
    engine = MuJoCoEngine(human_backend=backend)
    rows, contacts, sync = [], [], []
    failure = None
    try:
        engine.load(config)
        engine.reset(seed=seed)
        before = engine.get_ground_truth()
        tolerance = config.humans.goal_tolerance_m
        rows.append(_trace_row(before, engine.get_clearance(), scene, tolerance))
        dt = world.episode.control_dt_ns / NS_PER_SECOND
        append_dynamics(rows[0], None, dt)
        while True:
            stamp = before.stamp.timestamp
            try:
                result = engine.step(
                    RobotAction(
                        before.robot.entity_id,
                        Twist2D(*action_at(stamp.time_ns, policy), 0),
                        stamp,
                    )
                )
            except (ContractError, RuntimeError) as error:
                failure = str(error)
                break
            after = result.ground_truth
            row = _trace_row(after, result.clearance, scene, tolerance, result)
            row["robot_executed"] = _v(
                body_velocity_to_world(
                    result.executed.robot.velocity, before.robot.pose
                )
            )
            append_dynamics(row, rows[-1], dt)
            rows.append(row)
            contacts.extend(asdict(event) for event in result.contacts)
            sample = backend.last_sync
            if sample.time_ns == stamp.time_ns:
                sync.append(asdict(sample))
            before = after
            if result.terminated or result.truncated:
                break
        metadata = engine.get_metadata()
        metrics = summarize(
            rows,
            contacts,
            metadata.timing,
            metadata.status,
            metadata.end_reason,
            physically_feasible=slack > 1e-9,
        )
        hold = []
        for row in rows[1:]:
            start = row["time_ns"] - world.episode.control_dt_ns
            sample = sync[start // metadata.timing.human_control_dt_ns]
            row["robot_hold_lag_m"] = dist(row["robot_xy"], sample["robot_position"])
            hold.append(row["robot_hold_lag_m"])
        metrics["max_robot_hold_lag_m"] = max(hold, default=None)
        metrics["max_human_pre_rebase_error_m"] = max(
            (p["pre_rebase_error_m"] for s in sync for p in s["pedestrians"]),
            default=None,
        )
        return {
            "case": name,
            "episode_id": metadata.episode.episode_id,
            "policy": policy,
            "human_period_ms": period_ms,
            "branch_window_ns": (BRANCH_START_NS, BRANCH_END_NS),
            "geometry_hash": backend.geometry_hash,
            "side_by_side_slack_m": slack,
            "measured_width_m": measured_width,
            "native_build": asdict(backend.build),
            "native_model": "SocialForceModel",
            "coupling_profile": backend.coupling_profile,
            "metadata": asdict(metadata),
            "runtime_error": failure,
            "metrics": metrics,
            "trace": rows,
            "contacts": contacts,
            "sync": sync,
        }
    finally:
        engine.close()


def export_trace(destination, episode_id, rows):
    keys = ["episode_id", *dict.fromkeys(key for row in rows for key in row)]
    with destination.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "episode_id": episode_id,
                    **{
                        key: json.dumps(value) if isinstance(value, tuple) else value
                        for key, value in row.items()
                    },
                }
            )


def _write_json(path, value):
    path.write_text(json.dumps(value, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", choices=CASES, action="append")
    parser.add_argument("--seed", type=int, default=11)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cases = args.case or list(CASES)
    summary = []
    for name in cases:
        result = run_case(name, args.seed)
        _write_json(args.output / f"{name}.json", result)
        export_trace(args.output / f"{name}.csv", result["episode_id"], result["trace"])
        summary.append(
            {
                key: result[key]
                for key in (
                    "case",
                    "policy",
                    "human_period_ms",
                    "side_by_side_slack_m",
                    "runtime_error",
                    "metrics",
                )
            }
        )
        print(
            name,
            result["metrics"]["verdict"],
            result["metrics"]["failures"],
            flush=True,
        )
    _write_json(args.output / "summary.json", summary)
    _write_json(args.output / "velocity-sensitivity.json", measure_neighbor_response())


if __name__ == "__main__":
    main()
