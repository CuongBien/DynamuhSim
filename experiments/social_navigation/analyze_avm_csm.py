"""Measure native sensitivity, re-read 04U timeouts and validate paired fixtures."""

import argparse
import json
from dataclasses import asdict
from importlib.metadata import version
from math import dist, hypot, pi
from pathlib import Path

import jupedsim
from shapely.geometry import box

from narrow_nav.core.seed import derive_seed
from narrow_nav.humans.registry import create_human_backend

from .evaluate_avm_csm import ACTIONS, BACKENDS, BRANCH_START_NS, _write_json
from .sfm_metrics import MAX_PROGRESS_FREEZE_S, STALL_SPEED_MPS, progress

SEEDS = (11, 23, 47)
SPEEDS = (-0.70, -0.35, 0.0, 0.35, 0.70)
STEP_COUNTS = (1, 5, 10)


def _native_branch(backend, episode_seed, speed):
    if backend == "avm_jupedsim":
        model = jupedsim.AnticipationVelocityModel(
            rng_seed=derive_seed(episode_seed, "human_backend")
        )
        parameters = jupedsim.AnticipationVelocityModelAgentParameters
    else:
        model = jupedsim.CollisionFreeSpeedModelV2()
        parameters = jupedsim.CollisionFreeSpeedModelV2AgentParameters
    sim = jupedsim.Simulation(model=model, geometry=box(-6, -3, 6, 3), dt=0.01)
    stage = sim.add_waypoint_stage((4.0, 0.0), 0.1)
    journey = sim.add_journey(jupedsim.JourneyDescription([stage]))
    human_xy, human_vxy = (0.05, 0.0), (0.4, 0.0)
    human = sim.add_agent(
        parameters(position=human_xy, journey_id=journey, stage_id=stage, radius=0.25)
    )
    robot = sim.register_external_actor(0.18, 1.0)
    robot_xy, yaw = (1.2, 0.15), pi - 0.2
    sim.apply_coupled_state(
        0,
        10,
        external=[(robot, robot_xy, (speed, 0.0), yaw)],
        pedestrians=[(human, human_xy, human_vxy, 0.2)],
    )
    agent = sim.agent(human)
    assert type(agent.model).__name__ == f"{type(model).__name__}State"
    assert agent.position == human_xy
    assert sim.coupled_human_velocity(human) == (
        human_vxy if backend == "avm_jupedsim" else None
    )
    held = sim.external_actor(robot)
    assert held[:3] == (robot, robot_xy, (speed, 0.0))
    minimum = dist(agent.position, robot_xy) - 0.43
    samples = []
    for tick in range(1, 11):
        sim.iterate()
        assert sim.external_actor(robot) == held
        minimum = min(minimum, dist(agent.position, robot_xy) - 0.43)
        if tick in STEP_COUNTS:
            samples.append(
                {
                    "duration_ms": tick * 10,
                    "xy_m": agent.position,
                    "native_velocity_mps": sim.coupled_human_velocity(human),
                    "min_footprint_clearance_m": minimum,
                }
            )
    assert minimum > 0
    return samples


def sensitivity():
    rows = []
    for backend in BACKENDS[:2]:
        for seed in SEEDS:
            reference = _native_branch(backend, seed, 0.0)
            for speed in SPEEDS:
                branch = _native_branch(backend, seed, speed)
                assert branch == _native_branch(backend, seed, speed)
                for sample, zero in zip(branch, reference, strict=True):
                    effect = dist(sample["xy_m"], zero["xy_m"])
                    if backend == "csm_jupedsim":
                        assert effect == 0.0
                    elif speed == -0.7:
                        assert effect > 0.0
                    rows.append(
                        {
                            "backend": backend,
                            "episode_seed": seed,
                            "backend_seed": derive_seed(seed, "human_backend"),
                            "robot_vx_mps": speed,
                            "zero_velocity_xy_m": zero["xy_m"],
                            "position_delta_m": effect,
                            **sample,
                        }
                    )
    return {
        "distribution": version("jupedsim"),
        "build": asdict(create_human_backend("avm_jupedsim").build),
        "robot_xy_m": (1.2, 0.15),
        "robot_yaw_rad": pi - 0.2,
        "human_xy_m": (0.05, 0.0),
        "human_velocity_mps": (0.4, 0.0),
        "actor_anticipation_time_s": 1.0,
        "rows": rows,
    }


def _xy(record):
    return record["x"], record["y"]


def historical_timeouts(root, backend):
    results = []
    for path in sorted(root.glob("*/*.json")):
        if (
            path.parent.is_symlink()
        ):  # pytest's *current aliases point at existing fixtures.
            continue
        data = json.loads(path.read_text())
        metadata = data["metadata"]
        if (
            metadata["human_backend_id"] != backend
            or metadata["end_reason"] != "time_budget"
        ):
            continue
        rows = data["rows"]
        initial = data["gt_10hz"][0]
        dt = metadata["timing"]["robot_control_dt_ns"] / 1e9
        actor_results = []
        for i, human in enumerate(initial["humans"]):
            goal = _xy(human["goal"])
            distances = [dist(_xy(human["pose"]), goal)]
            distances.extend(dist(row["humans"][i], goal) for row in rows)
            goal_tick = next(
                (i for i, distance in enumerate(distances) if distance <= 0.1), None
            )
            freeze, episodes = progress(
                [
                    {"robot_goal_distance_m": distance}
                    for distance in distances[
                        : goal_tick + 1 if goal_tick is not None else None
                    ]
                ],
                dt,
            )
            actor_results.append(
                {
                    "id": human["entity_id"],
                    "pre_goal_freeze_s": freeze,
                    "freeze_over_5s_count": episodes,
                    "final_goal_distance_m": distances[-1],
                    "goal_first_s": goal_tick * dt if goal_tick is not None else None,
                    "final_requested_mps": rows[-1]["requested"][i],
                    "final_limited_mps": rows[-1]["limited"][i],
                    "final_executed_mps": rows[-1]["executed"][i],
                }
            )
        results.append(
            {
                "source": str(path),
                "fixture": path.stem,
                "episode": metadata["episode"],
                "scenario_status": data["scenario_status"],
                "physics_steps": metadata["clocks"]["physics_steps"],
                "time_s": rows[-1]["time_ns"] / 1e9,
                "contact_samples": len(data["contacts"]),
                "guard_substeps": sum(row["guard_substeps"] for row in rows),
                "first_guard_tick_s": next(
                    (r["time_ns"] / 1e9 for r in rows if r["guard_substeps"]), None
                ),
                "guarded_stopped_ticks": sum(
                    bool(row["guard_substeps"])
                    and all(hypot(*v) < STALL_SPEED_MPS for v in row["executed"])
                    for row in rows
                ),
                "final_clearance_m": rows[-1]["clearance"],
                "actors": actor_results,
                "final_native_prediction_separation_m": (
                    dist(*(p["prediction"] for p in data["sync"][-1]["pedestrians"]))
                    if len(initial["humans"]) == 2
                    else None
                ),
            }
        )
    assert any(row["fixture"] == "crossing" for row in results)
    return results


def _verify_trace(episode):
    rows = episode["trace"]
    metadata = episode["metadata"]
    dt = metadata["timing"]["robot_control_dt_ns"]
    period = metadata["timing"]["human_control_dt_ns"]
    assert dt == 50_000_000 and period == 100_000_000
    assert metadata["timing"]["physics_dt_ns"] == 2_000_000
    assert all(row["time_ns"] == i * dt for i, row in enumerate(rows))
    last = (0.0, 0.0)
    for i, row in enumerate(rows[1:]):
        limited = row["human_limited"]
        assert hypot(*limited) <= 1.4 + 1e-12
        assert row["human_continuity_residual_m"] <= 1e-12
        if i % 2 == 0:
            assert dist(limited, last) <= 0.15 + 1e-12
        else:
            assert (
                limited == last and row["human_requested"] == rows[i]["human_requested"]
            )
        last = limited
    for i, sample in enumerate(episode["sync"]):
        assert sample["time_ns"] == i * period
        assert sample["iteration_before"] == 10 * i
        assert sample["iteration_after"] == 10 * (i + 1)
        before, command = rows[2 * i : 2 * i + 2]
        assert sample["robot_position"] == before["robot_xy"]
        assert sample["robot_velocity"] == before["robot_velocity"]
        (person,) = sample["pedestrians"]
        assert person["position"] == before["human_xy"]
        assert person["velocity"] == before["human_velocity"]
        assert (
            dist(
                command["human_requested"],
                tuple(
                    (p - x) / 0.1
                    for p, x in zip(
                        person["prediction"], person["position"], strict=True
                    )
                ),
            )
            < 1e-12
        )


def evidence(root):
    runs = {}
    outputs = []
    for group in ("avm", "csm", "baselines", "contested"):
        summary = json.loads((root / group / "summary.json").read_text())
        for row in summary:
            path = root / group / row["path"] / "episode.json"
            episode = json.loads(path.read_text())
            _verify_trace(episode)
            if "replay_exact" in row:
                assert row["replay_exact"]
            trace = episode["trace"]
            gt_fields = (
                "time_ns",
                "robot_xy",
                "human_xy",
                "robot_velocity",
                "human_velocity",
                "robot_yaw_rad",
                "human_yaw_rad",
                "robot_wall_m",
                "human_wall_m",
                "robot_human_m",
            )
            _write_json(
                path.with_name("gt-10hz.json"),
                {
                    "episode_id": episode["episode_id"],
                    "frame": "world",
                    "sample_period_ns": 100_000_000,
                    "samples": [
                        {k: r[k] for k in gt_fields}
                        for r in trace
                        if r["time_ns"] % 100_000_000 == 0
                    ],
                },
            )
            worst_jerk = max(trace, key=lambda r: r["human_jerk_mps3"] or 0)
            detail = {
                **row,
                "source": str(path),
                "time_s": trace[-1]["time_s"],
                "limits_verified": True,
                "peak_human_jerk_time_s": worst_jerk["time_s"],
                "guard_near_peak_jerk_substeps": sum(
                    r["guard_blocked_substeps"]
                    for r in trace
                    if abs(r["time_ns"] - worst_jerk["time_ns"]) <= 100_000_000
                ),
                "first_line_of_sight_s": next(
                    (r["time_s"] for r in trace if r.get("line_of_sight")), None
                ),
                "initial_line_of_sight": trace[0].get("line_of_sight"),
                "initial_wall_clearance_m": trace[0]["human_wall_m"],
                "human_max_lateral_displacement_m": max(
                    abs(r["human_xy"][1] - trace[0]["human_xy"][1]) for r in trace
                ),
            }
            if row["case"].startswith("door_"):
                detail["door_robot_cross_s"] = next(
                    (r["time_s"] for r in trace if r["robot_xy"][0] >= 0), None
                )
                detail["door_human_cross_s"] = next(
                    (r["time_s"] for r in trace if r["human_xy"][0] <= 0), None
                )
            outputs.append(detail)
            if row["case"] == "head_on" and "replay_exact" not in row:
                runs[row["action"], row["backend"]] = (detail, trace)
    for backend in BACKENDS:
        reference = [
            r for r in runs["advance", backend][1] if r["time_ns"] <= BRANCH_START_NS
        ]
        for action in ACTIONS[1:]:
            assert [
                r for r in runs[action, backend][1] if r["time_ns"] <= BRANCH_START_NS
            ] == reference
    pairings = []
    for action in ACTIONS:
        control, control_trace = runs[action, "csm_jupedsim"]
        for backend in ("avm_jupedsim", "sfm_jupedsim", "orca"):
            other, trace = runs[action, backend]
            assert control["paired_key"] == other["paired_key"]
            assert all(
                trace[0][actor] == control_trace[0][actor]
                for actor in ("robot_xy", "human_xy")
            )
            assert all(
                a["robot_requested_body"] == b["robot_requested_body"]
                for a, b in zip(control_trace[1:], trace[1:], strict=False)
            )
            pairings.append(
                {
                    "action": action,
                    "paired_key": control["paired_key"],
                    "baseline": backend,
                    "valid": all(
                        episode["metrics"]["verdict"] != "INVALID_INFRA"
                        for episode in (control, other)
                    ),
                    "csm_source": control["source"],
                    "baseline_source": other["source"],
                    "max_human_trajectory_delta_m": max(
                        dist(a["human_xy"], b["human_xy"])
                        for a, b in zip(control_trace, trace, strict=False)
                    ),
                }
            )
    return {
        "episodes": outputs,
        "paired_csm": pairings,
        "freeze_threshold_s": MAX_PROGRESS_FREEZE_S,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--historical", required=True, type=Path)
    args = parser.parse_args()
    _write_json(args.root / "native-sensitivity.json", sensitivity())
    for model in ("avm", "csm"):
        _write_json(
            args.root / f"{model}-04u-timeouts.json",
            historical_timeouts(args.historical, f"{model}_jupedsim"),
        )
    _write_json(args.root / "acceptance-evidence.json", evidence(args.root))


if __name__ == "__main__":
    main()
