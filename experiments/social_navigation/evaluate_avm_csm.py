"""Run the bounded TASK 06/07 behavior fixtures with the registered 04U backends."""

import argparse
import json
from dataclasses import asdict
from hashlib import sha256
from math import dist, isclose
from pathlib import Path
from xml.etree import ElementTree

from omegaconf import OmegaConf
from shapely.geometry import LineString

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ContractError
from narrow_nav.core.geometry import body_velocity_to_world
from narrow_nav.core.types import NS_PER_SECOND, Twist2D
from narrow_nav.humans.jupedsim_bridge.geometry import build_walkable
from narrow_nav.humans.registry import create_human_backend
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import MJCF_ROOT, build_world
from narrow_nav.simulation.world_geometry import opening_clearance

from .evaluate_sfm import (
    BRANCH_END_NS,
    BRANCH_START_NS,
    CONFIG,
    FREE,
    ROOT,
    _trace_row,
    _v,
    _write_json,
    action_at,
    export_trace,
)
from .sfm_metrics import append_dynamics, progress, summarize

ACTIONS = ("advance", "stop", "yield", "reverse")
BACKENDS = ("avm_jupedsim", "csm_jupedsim", "sfm_jupedsim", "orca")
SEED = 11
DOORS = {"door_120": 1.2, "door_090": 0.9, "door_060": 0.6, "door_060_contested": 0.6}
SCENES = {
    "head_on": (),
    "free": FREE,
    "crossing": (
        "worlds.width_m=6",
        "scenarios.robot.initial_pose=[-0.85,0,0]",
        "scenarios.robot.goal=[3,0]",
        "scenarios.humans.0.initial_pose=[0,-2,1.5707963267948966]",
        "scenarios.humans.0.goal=[0,2]",
    ),
    "near_wall": (
        "worlds.width_m=4",
        "scenarios.robot.initial_pose=[-3,-1.4,0]",
        "scenarios.robot.goal=[3,-1.4]",
        "scenarios.humans.0.initial_pose=[3,1.67,-3.141592653589793]",
        "scenarios.humans.0.goal=[-3,1.67]",
    ),
    "blind_corner": (
        "worlds=blind_corner",
        "worlds.width_m=2.1",
        "scenarios.robot.initial_pose=[1.1,-0.35,0]",
        "scenarios.robot.goal=[4.15,2.4]",
        "scenarios.humans.0.initial_pose=[4.4,2.8,-1.5707963267948966]",
        "scenarios.humans.0.goal=[-3,0.2]",
    ),
}


def cases(backend):
    result = [("head_on", action, False) for action in ACTIONS]
    if backend in BACKENDS[:2]:
        extra = "blind_corner" if backend == BACKENDS[0] else "near_wall"
        result.extend(
            (scene, "advance", False)
            for scene in ("free", "crossing", extra, "door_120", "door_090", "door_060")
        )
        result.append(("head_on", "advance", True))
    return result


def _door_config(config, name, root):
    width = DOORS[name]
    template = ROOT / "tests/fixtures/jupedsim_04u/doorway.xml"
    xml = ElementTree.parse(template)
    for sign, side in ((-1, "south"), (1, "north")):
        jamb = xml.find(f".//geom[@name='{side}_jamb']")
        jamb.set("pos", f"0 {sign * (2 + width / 2) / 2:.17g} 1")
        jamb.set("size", f"0.1 {(2 - width / 2) / 2:.17g} 1")
    directory = root / "worlds/bridge_doorway"
    directory.mkdir(parents=True)
    xml.write(directory / "main.xml", encoding="unicode")
    for asset in ("templates", "models"):
        (root / asset).symlink_to(MJCF_ROOT / asset, target_is_directory=True)
    resolved = OmegaConf.to_container(config, resolve=True)
    resolved["worlds"] = {"id": "bridge_doorway", "geometry_id": "bridge_doorway"}
    resolved["scenarios"]["world_id"] = "bridge_doorway"
    return OmegaConf.create(resolved)


def _config(backend, scene, case_dir):
    common = tuple(
        item for item in CONFIG if not item.startswith(("humans=", "worlds.length_m="))
    )
    overrides = SCENES.get(scene, ())
    if scene in DOORS:
        overrides = (
            "experiments.duration_ns=12000000000",
            "scenarios.robot.initial_pose=[-1.7,-0.05,0]",
            "scenarios.robot.goal=[0.9,-0.05]",
            "scenarios.humans.0.initial_pose=[1.5,0.05,-3.141592653589793]",
            "scenarios.humans.0.goal=[-3,0.05]",
        )
        if scene == "door_060_contested":
            overrides += (
                "scenarios.robot.initial_pose=[-0.9,-0.05,0]",
                "scenarios.humans.0.initial_pose=[2.35,0.05,-3.141592653589793]",
            )
    config = compose_config(
        ROOT / "configs",
        (f"humans={backend}", "worlds.width_m=2.1", *common, *overrides),
    )
    mjcf_root = MJCF_ROOT
    if scene in DOORS:
        mjcf_root = case_dir / "fixture"
        config = _door_config(config, scene, mjcf_root)
    return config, mjcf_root


def _command(scene, action, time_ns):
    if scene == "door_060_contested":
        return 0.35, 0.0
    if scene in DOORS:
        return (0.0, 0.0) if time_ns < 2_000_000_000 else (0.35, 0.0)
    if scene == "blind_corner":
        return (0.35, 0.0) if time_ns < 8_800_000_000 else (0.0, 0.3)
    return action_at(time_ns, action)


def _pair_key(config, scene, action, seed, model_sha):
    resolved = OmegaConf.to_container(config, resolve=True)
    del resolved["humans"]
    payload = (scene, action, seed, model_sha, resolved)
    return sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def prediction_windows(rows, sync, period_ns):
    by_time = {row["time_ns"]: row for row in rows}
    pairs = []
    for sample in sync:
        start_ns = sample["time_ns"]
        final = by_time.get(start_ns + period_ns)
        if final is None:
            continue
        (pedestrian,) = sample["pedestrians"]
        start = pedestrian["position"]
        end = final["human_xy"]
        predicted = pedestrian["prediction"]
        seconds = period_ns / NS_PER_SECOND
        pairs.append(
            {
                "start_time_ns": start_ns,
                "predicted_xy_m": predicted,
                "executed_xy_m": end,
                "predicted_velocity_mps": tuple(
                    (p - s) / seconds for p, s in zip(predicted, start, strict=True)
                ),
                "executed_velocity_mps": tuple(
                    (e - s) / seconds for e, s in zip(end, start, strict=True)
                ),
                "position_error_m": dist(predicted, end),
                "velocity_error_mps": dist(predicted, end) / seconds,
            }
        )
    return pairs


def _metrics(rows, contacts, metadata, comparisons, sync, failure):
    result = summarize(
        rows,
        contacts,
        metadata.timing,
        metadata.status,
        metadata.end_reason,
        physically_feasible=True,  # A doorway may be traversed sequentially.
    )
    goal_index = next(
        (i for i, row in enumerate(rows) if row["human_at_goal"]), len(rows) - 1
    )
    human_progress = [
        {"robot_goal_distance_m": row["human_goal_distance_m"]}
        for row in rows[: goal_index + 1]
    ]
    freeze, _ = progress(
        human_progress, metadata.timing.robot_control_dt_ns / NS_PER_SECOND
    )
    result.update(
        max_human_pre_goal_freeze_s=freeze,
        max_prediction_error_m=max(
            (w["position_error_m"] for w in comparisons), default=None
        ),
        mean_prediction_error_m=(
            sum(w["position_error_m"] for w in comparisons) / len(comparisons)
            if comparisons
            else None
        ),
        max_predicted_velocity_error_mps=max(
            (w["velocity_error_mps"] for w in comparisons), default=None
        ),
        complete_prediction_windows=len(comparisons),
        max_human_pre_rebase_error_m=max(
            (p["pre_rebase_error_m"] for sample in sync for p in sample["pedestrians"]),
            default=None,
        ),
    )
    if not result["human_goal_reached"]:
        result["failures"].append("human_goal_not_reached")
    result["verdict"] = (
        "INVALID_INFRA" if failure else "FAIL" if result["failures"] else "PASS"
    )
    return result


def run_episode(backend_id, scene_name, action, case_dir):
    case_dir.mkdir(parents=True)
    config, mjcf_root = _config(backend_id, scene_name, case_dir)
    world = build_world(config, mjcf_root=mjcf_root)
    collision = inspect_world(world)
    width = collision.clear_width(0)
    if scene_name in DOORS and not isclose(width, DOORS[scene_name], abs_tol=1e-9):
        raise ValueError(f"unexpected compiled doorway width: {width}")
    scene = world.scene
    slack = opening_clearance(width, scene.robot.radius_m, scene.humans[0].radius_m)
    walkable = (
        build_walkable(collision.planar_world()).native_polygon()
        if scene_name == "blind_corner"
        else None
    )
    backend = create_human_backend(backend_id)
    engine = MuJoCoEngine(mjcf_root=mjcf_root, human_backend=backend)
    rows, contacts, sync = [], [], []
    failure = None
    try:
        engine.load(config)
        engine.reset(seed=SEED)
        truth = engine.get_ground_truth()
        tolerance = config.humans.goal_tolerance_m
        row = _trace_row(truth, engine.get_clearance(), scene, tolerance)
        dt = world.episode.control_dt_ns / NS_PER_SECOND
        append_dynamics(row, None, dt)
        rows.append(row)
        while True:
            stamp = truth.stamp.timestamp
            try:
                result = engine.step(
                    RobotAction(
                        truth.robot.entity_id,
                        Twist2D(*_command(scene_name, action, stamp.time_ns), 0),
                        stamp,
                    )
                )
            except (ContractError, RuntimeError) as error:
                failure = {
                    "type": type(error).__name__,
                    "message": str(error),
                    "tick": stamp.tick,
                }
                break
            after = result.ground_truth
            row = _trace_row(after, result.clearance, scene, tolerance, result)
            row["robot_executed"] = _v(
                body_velocity_to_world(result.executed.robot.velocity, truth.robot.pose)
            )
            append_dynamics(row, rows[-1], dt)
            rows.append(row)
            contacts.extend(asdict(event) for event in result.contacts)
            sample = getattr(backend, "last_sync", None)
            if sample is not None and sample.time_ns == stamp.time_ns:
                sync.append(asdict(sample))
            truth = after
            if result.terminated or result.truncated:
                break
        metadata = engine.get_metadata()
    finally:
        engine.close()
    if walkable is not None:
        for row in rows:
            row["line_of_sight"] = walkable.covers(
                LineString((row["robot_xy"], row["human_xy"]))
            )
    comparisons = prediction_windows(rows, sync, metadata.timing.human_control_dt_ns)
    data = {
        "case": scene_name,
        "action": action,
        "backend": backend_id,
        "episode_id": metadata.episode.episode_id,
        "paired_key": _pair_key(
            config, scene_name, action, SEED, metadata.model_sha256
        ),
        "robot_action_window_ns": (BRANCH_START_NS, BRANCH_END_NS)
        if scene_name == "head_on"
        else None,
        "metadata": asdict(metadata),
        "measured_width_m": width,
        "side_by_side_slack_m": slack,
        "sequential_clearance_m": opening_clearance(
            width, max(scene.robot.radius_m, scene.humans[0].radius_m)
        ),
        "native_model": getattr(backend, "model_class", "rvo2.PyRVOSimulator"),
        "native_build": asdict(backend.build) if hasattr(backend, "build") else None,
        "runtime_error": failure,
        "metrics": _metrics(rows, contacts, metadata, comparisons, sync, failure),
        "trace": rows,
        "contacts": contacts,
        "sync": sync,
        "prediction_windows": comparisons,
    }
    _write_json(case_dir / "episode.json", data)
    export_trace(case_dir / "trace.csv", data["episode_id"], rows)
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--backend", choices=BACKENDS, action="append")
    parser.add_argument("--scene", choices=(*SCENES, *DOORS))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = []
    for backend in args.backend or BACKENDS:
        original = None
        selected = cases(backend)
        if args.scene == "door_060_contested" and backend in BACKENDS[:2]:
            selected = [(args.scene, "advance", False)]
        for scene, action, replay in selected:
            if args.scene and scene != args.scene:
                continue
            case_id = f"{backend}/{scene}_{action}" + ("_replay" if replay else "")
            result = run_episode(backend, scene, action, args.output / case_id)
            record = {
                key: value
                for key, value in result.items()
                if key
                not in ("trace", "contacts", "sync", "prediction_windows", "metadata")
            }
            record["path"] = case_id
            record["end_reason"] = result["metadata"]["end_reason"]
            if scene == "head_on" and action == "advance":
                if replay:
                    keys = ("trace", "contacts", "prediction_windows", "metrics")
                    record["replay_exact"] = all(
                        original[key] == result[key] for key in keys
                    )
                else:
                    original = result
            summary.append(record)
            print(
                case_id,
                record["metrics"]["verdict"],
                record["metrics"]["failures"],
                flush=True,
            )
        _write_json(
            args.output / f"summary-{backend}.json",
            [r for r in summary if r["backend"] == backend],
        )
    _write_json(args.output / "summary.json", summary)


if __name__ == "__main__":
    main()
