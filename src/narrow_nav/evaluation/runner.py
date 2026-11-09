"""Persist an append-only attempt using public simulation snapshots and actions."""

import json
from contextlib import ExitStack, contextmanager
from dataclasses import asdict
from pathlib import Path
from time import perf_counter

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.geometry import body_velocity_to_world
from narrow_nav.core.types import Twist2D

from .metrics import evaluate
from .qa import command_at, scenario_label

CHANNELS = (
    "trace",
    "gt_10hz",
    "events",
    "actions_requested",
    "actions_limited",
    "actions_executed",
)


def _json_line(data):
    return json.dumps(data, sort_keys=True, allow_nan=False) + "\n"


def write_json(path: Path, data):
    temp = path.with_name(f".{path.name}.tmp")
    temp.write_text(_json_line(data), encoding="utf-8")
    temp.replace(path)


def _actor(state, velocity):
    return {
        "xy": (state.pose.x, state.pose.y),
        "yaw": state.pose.yaw,
        "velocity": (velocity.linear_x, velocity.linear_y),
    }


def _row(truth, clearance, *, guard=0, wait=False):
    robot = truth.robot
    return {
        "time_ns": truth.stamp.timestamp.time_ns,
        "robot": _actor(robot, body_velocity_to_world(robot.velocity, robot.pose)),
        "humans": {
            person.entity_id: _actor(person, person.velocity) for person in truth.humans
        },
        "clearance": asdict(clearance) if clearance is not None else None,
        "guard_substeps": guard,
        "intentional_wait": wait,
    }


@contextmanager
def _channels(path):
    with ExitStack() as stack:
        files = {
            name: stack.enter_context(
                (path / f"{name}.jsonl").open("x", encoding="utf-8", buffering=1)
            )
            for name in CHANNELS
        }
        yield lambda name, record: files[name].write(_json_line(record))


def execute(engine, config, slot, case, directory, context):
    """Write every committed tick. A missing validity record denotes interruption."""
    started = perf_counter()
    rows, events = [], []
    error = metadata = observation = None
    write_json(directory / "intent.json", {"schema_version": 1, **context})
    (directory / "resolved_config.yaml").write_text(case["yaml"], encoding="utf-8")
    try:
        with _channels(directory) as emit:
            try:
                engine.load(config)
                observation = engine.reset(seed=slot["seed"])
                truth = engine.get_ground_truth()
                rows.append(_row(truth, engine.get_clearance()))
                emit("trace", rows[-1])
                emit("gt_10hz", asdict(truth))
                while True:
                    stamp = truth.stamp.timestamp
                    body, wait = command_at(case["schedule"], stamp.time_ns)
                    result = engine.step(
                        RobotAction(truth.robot.entity_id, Twist2D(*body), stamp)
                    )
                    truth = result.ground_truth
                    rows.append(
                        _row(
                            truth,
                            result.clearance,
                            guard=result.human_motion_blocked_substeps,
                            wait=wait,
                        )
                    )
                    emit("trace", rows[-1])
                    if truth.stamp.timestamp.time_ns % 100_000_000 == 0:
                        emit("gt_10hz", asdict(truth))
                    for stage in ("requested", "limited", "executed"):
                        emit(f"actions_{stage}", asdict(getattr(result, stage)))
                    for event in result.contacts:
                        record = asdict(event)
                        events.append(record)
                        emit("events", record)
                    if result.terminated or result.truncated:
                        break
            except Exception as exc:  # noqa: BLE001 - retain unexpected runtime failures
                error = {"type": type(exc).__name__, "message": str(exc)}
            finally:
                try:
                    metadata = asdict(engine.get_metadata())
                except Exception:  # load may fail before reset creates an episode
                    if error is None:
                        raise
    finally:
        engine.close()
    if error is None and metadata is not None:
        observed = {
            "config_sha256": metadata["episode"]["config_sha256"],
            "model_sha256": metadata["model_sha256"],
            "asset_hashes": dict(metadata["mjcf_source_sha256"]),
            "backend_version": metadata["human_backend_version"],
        }
        mismatch = [key for key, value in observed.items() if value != context[key]]
        if metadata["human_backend_id"] != slot["backend"]:
            mismatch.append("human_backend_id")
        if mismatch:
            error = {"type": "ProvenanceMismatch", "message": ", ".join(mismatch)}
    end_reason = metadata["end_reason"] if metadata else None
    valid = error is None and end_reason in (
        "time_budget",
        "goal_reached",
        "robot_contact",
    )
    label = scenario_label(case["card"], slot["binding"], rows, case.get("visible"))
    metrics = evaluate(
        rows,
        events,
        timing=case["timing"],
        end_reason=end_reason,
        infra_valid=valid,
        goals=case["goals"],
        radii=case["radii"],
        goal_tolerance=case["goal_tolerance"],
        solver_unobservable=metadata["not_observable_solver"]
        if metadata
        else ("robot_wall", "robot_human", "human_wall", "human_human"),
        shortest_path_m=slot["binding"].get("shortest_path_m"),
        label_valid=label,
    )
    manifest = {
        "schema_version": 1,
        **context,
        "episode_id": (
            f"{metadata['episode']['run_id']}:{metadata['episode']['episode_ordinal']}"
            if metadata
            else None
        ),
        "runtime_metadata": metadata,
        "error": error,
        "end_reason": end_reason,
        "wall_seconds": perf_counter() - started,
        "simulated_time_ns": rows[-1]["time_ns"] if rows else None,
        "sensor_availability": [
            {"id": item.sensor_id, "status": item.status, "reason": item.reason}
            for item in observation.sensors
        ]
        if observation is not None
        else None,
    }
    write_json(directory / "manifest.json", manifest)
    write_json(directory / "validity.json", metrics)
    return metrics
