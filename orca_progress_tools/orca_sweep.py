#!/usr/bin/env python3
"""Diagnostic ORCA sweep against existing NarrowNav MuJoCo worlds.
Run from the repository root using the project's .venv Python.
Uses only public NarrowNav simulation APIs. This is a smoke/coverage audit,
NOT the research benchmark and NOT a guarantee of navigation success.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import traceback

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.types import Twist2D
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine


@dataclass(frozen=True)
class Case:
    world: str
    scenario: str
    width: str | None = None

    @property
    def key(self) -> str:
        return "-".join(v for v in (self.world, self.scenario, self.width) if v)

    def overrides(self, *, robot: str, seed: int) -> list[str]:
        result = [
            f"worlds={self.world}",
            f"scenarios={self.scenario}",
            "humans=orca",
            f"robots={robot}",
            f"seed={seed}",
            "simulation.observation_sensors=[]",
        ]
        if self.width:
            result.append(f"corridor_width={self.width}")
        return result


def build_cases() -> list[Case]:
    cases = [
        Case("corridor", "head_on"),
        *(Case(f"corridor_{w}", f"corridor_{w}") for w in ("070", "090", "120")),
        *(Case(f"hospital_{d}", f"hospital_{d}") for d in ("easy", "medium", "hard")),
        *(Case(a, a, f"w{w}") for a in ("blind_corner", "passing_bay") for w in ("070", "090", "120")),
        *(Case(a, a, f"w{w}") for a in ("school_arena", "school_arena_hunav", "school_arena_1", "school_floor") for w in ("070", "090", "120")),
        *(Case("school_floor", "school_floor_baseline", f"w{w}") for w in ("070", "090", "120")),
        *(Case(a, a) for a in ("arena_obstacle", "arena_rvo2", "arena_dataset")),
    ]
    assert len(cases) == 31
    return cases


def _pose(p) -> dict[str, float]:
    return {"x": float(p.x), "y": float(p.y), "yaw": float(p.yaw)}


def _snapshot(truth) -> dict:
    return {
        "t_s": truth.stamp.timestamp.time_s,
        "robot": _pose(truth.robot.pose),
        "humans": [
            {
                "id": h.entity_id,
                **_pose(h.pose),
                "vx": float(h.velocity.linear_x),
                "vy": float(h.velocity.linear_y),
                "goal_x": float(h.goal.x),
                "goal_y": float(h.goal.y),
            }
            for h in truth.humans
        ],
    }


def _distance_goal(h) -> float:
    return math.hypot(h.goal.x - h.pose.x, h.goal.y - h.pose.y)


def _speed(motion) -> float:
    v = motion.velocity
    return math.hypot(v.linear_x, v.linear_y)


def run_case(case: Case, seed: int, args, root: Path) -> dict:
    record = {
        "case": case.key,
        "world": case.world,
        "scenario": case.scenario,
        "width_preset": case.width or "",
        "seed": seed,
        "robot_policy": args.policy,
        "runtime_status": "ERROR",
        "episode_status": "",
        "end_reason": "",
        "human_count": 0,
        "ticks": 0,
        "sim_s": 0.0,
        "human_path_m_sum": 0.0,
        "humans_moved": 0,
        "human_goal_progress_m_sum": 0.0,
        "human_requested_ticks": 0,
        "human_limited_ticks": 0,
        "human_executed_ticks": 0,
        "native_contact_samples": 0,
        "robot_human_contacts": 0,
        "robot_wall_contacts": 0,
        "human_wall_contacts": 0,
        "human_human_contacts": 0,
        "min_robot_human_clearance_m": "",
        "min_robot_wall_clearance_m": "",
        "trace_file": "",
        "error": "",
    }
    overrides = case.overrides(robot=args.robot, seed=seed)
    trace = {"overrides": overrides, "snapshots": [], "contacts": [], "note": "Debug output; not benchmark data."}
    engine = None
    try:
        config = compose_config("configs", overrides)
        engine = MuJoCoEngine()
        engine.load(config)
        engine.reset(seed=seed)
        before = engine.get_ground_truth()
        record["human_count"] = len(before.humans)
        trace["snapshots"].append(_snapshot(before))
        if not before.humans:
            record["runtime_status"] = "SKIP_NO_HUMANS"
            record["episode_status"] = engine.get_metadata().status
            return record

        prev_xy = {h.entity_id: (h.pose.x, h.pose.y) for h in before.humans}
        path = {h.entity_id: 0.0 for h in before.humans}
        before_goal = {h.entity_id: _distance_goal(h) for h in before.humans}
        min_human = None
        min_wall = None
        counter = Counter()
        dt_ticks = min(args.ticks, engine.get_metadata().episode.config.max_ticks)
        for _ in range(dt_ticks):
            robot = engine.get_ground_truth().robot
            vx = args.vx if args.policy == "advance" else 0.0
            action = RobotAction(robot.entity_id, Twist2D(vx, 0.0, 0.0), robot.timestamp)
            result = engine.step(action)
            record["ticks"] += 1
            truth = result.ground_truth
            record["sim_s"] = truth.stamp.timestamp.time_s
            if any(_speed(m.requested) > 1e-6 for m in result.human_motion_results):
                record["human_requested_ticks"] += 1
            if any(_speed(m.limited) > 1e-6 for m in result.human_motion_results):
                record["human_limited_ticks"] += 1
            if any(_speed(m.executed) > 1e-6 for m in result.human_motion_results):
                record["human_executed_ticks"] += 1
            for h in truth.humans:
                prior = prev_xy[h.entity_id]
                path[h.entity_id] += math.hypot(h.pose.x - prior[0], h.pose.y - prior[1])
                prev_xy[h.entity_id] = (h.pose.x, h.pose.y)
            for contact in result.contacts:
                counter[contact.kind or "unspecified"] += 1
                trace["contacts"].append({
                    "time_ns": contact.time_ns,
                    "kind": contact.kind,
                    "first_id": contact.first_id,
                    "second_id": contact.second_id,
                })
            clearance = engine.get_clearance()
            min_wall = clearance.robot_wall_m if min_wall is None else min(min_wall, clearance.robot_wall_m)
            if clearance.robot_human_m is not None:
                min_human = clearance.robot_human_m if min_human is None else min(min_human, clearance.robot_human_m)
            trace["snapshots"].append(_snapshot(truth))
            if result.terminated or result.truncated:
                break

        after = engine.get_ground_truth()
        meta = engine.get_metadata()
        record.update({
            "runtime_status": "EXECUTED",
            "episode_status": meta.status,
            "end_reason": meta.end_reason or "",
            "human_path_m_sum": round(sum(path.values()), 6),
            "humans_moved": sum(p > 0.05 for p in path.values()),
            "human_goal_progress_m_sum": round(sum(before_goal[h.entity_id] - _distance_goal(h) for h in after.humans), 6),
            "native_contact_samples": sum(counter.values()),
            "robot_human_contacts": counter["robot_human"],
            "robot_wall_contacts": counter["robot_wall"],
            "human_wall_contacts": counter["human_wall"],
            "human_human_contacts": counter["human_human"],
            "min_robot_human_clearance_m": round(min_human, 6) if min_human is not None else "",
            "min_robot_wall_clearance_m": round(min_wall, 6) if min_wall is not None else "",
        })
        trace["run_metadata"] = {
            "git_commit": meta.episode.git_commit,
            "git_dirty": meta.episode.git_dirty,
            "engine_version": meta.engine_version,
            "end_reason": meta.end_reason,
            "status": meta.status,
        }
    except Exception:
        record["error"] = traceback.format_exc(limit=12)
        trace["error"] = record["error"]
    finally:
        if engine is not None:
            try:
                engine.close()
            except Exception:
                if record["runtime_status"] == "EXECUTED":
                    record["runtime_status"] = "ERROR_CLOSE"
                record["error"] += "\nCLOSE ERROR:\n" + traceback.format_exc(limit=5)
        root.mkdir(parents=True, exist_ok=True)
        trace_path = root / f"{case.key}__seed-{seed}__{args.policy}.json"
        try:
            with trace_path.open("w", encoding="utf-8") as f:
                json.dump(trace, f, ensure_ascii=False, indent=2)
            record["trace_file"] = str(trace_path)
        except Exception:
            record["error"] += "\nTRACE WRITE ERROR:\n" + traceback.format_exc(limit=5)
            record["runtime_status"] = "ERROR_WRITE"
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ticks", type=int, default=400, help="max robot control ticks (400 ~= 20 sim seconds)")
    parser.add_argument("--seed", type=int, action="append", help="repeatable; default 42")
    parser.add_argument("--policy", choices=("advance", "stop"), default="advance")
    parser.add_argument("--vx", type=float, default=0.35, help="robot advance command (m/s)")
    parser.add_argument("--robot", default="mecanum_robot")
    parser.add_argument("--case", default="", help="substring filter (e.g. hospital, school, arena)")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    if args.ticks < 1 or not math.isfinite(args.vx):
        parser.error("--ticks must be positive and --vx finite")
    if not Path("configs/config.yaml").exists():
        parser.error("run this command from the project root (configs/config.yaml missing)")
    seeds = args.seed or [42]
    known_cases = build_cases()
    known_worlds = {case.world for case in known_cases}
    actual_worlds = {path.stem for path in Path("configs/worlds").glob("*.yaml")}
    additional_worlds = sorted(actual_worlds - known_worlds)
    unmapped_worlds = []
    for world in additional_worlds:
        scenario_file = Path("configs/scenarios") / f"{world}.yaml"
        if scenario_file.exists():
            # A newly discovered world gets its same-name scenario as a first-pass
            # smoke; custom width variants still require a reviewed mapping.
            known_cases.append(Case(world, world))
            print(f"DISCOVERED NEW WORLD: {world} (same-name scenario smoke only)")
        else:
            unmapped_worlds.append(world)
    if unmapped_worlds:
        print("WARNING: WORLD(S) NOT AUTO-TESTABLE WITHOUT A SCENARIO PAIR:", ", ".join(unmapped_worlds))
    cases = [case for case in known_cases if args.case.lower() in case.key.lower()]
    if not cases:
        parser.error("--case matched no known world/scenario combination")
    output = args.out or Path("runs") / ("orca_sweep_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    print(f"ORCA DIAGNOSTIC SWEEP: {len(cases)} combos × {len(seeds)} seeds; policy={args.policy}; out={output}", flush=True)
    for case in cases:
        for seed in seeds:
            row = run_case(case, seed, args, output / "traces")
            rows.append(row)
            print(f"[{row['runtime_status']}] {case.key} seed={seed} humans={row['human_count']} moved={row['humans_moved']} contacts={row['native_contact_samples']} ticks={row['ticks']} reason={row['end_reason']}", flush=True)
    csv_file = output / "summary.csv"
    with csv_file.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (output / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)
    totals = Counter(row["runtime_status"] for row in rows)
    print(f"\nRESULTS: {dict(totals)}\nCSV: {csv_file}\nTraces: {output / 'traces'}")
    print("IMPORTANT: EXECUTED means the runtime ran, NOT that the episode passed research hard constraints.")
    print("No-human scenes are SKIP_NO_HUMANS: world compilation is checked but ORCA navigation is not testable there.")
    (output / "coverage_notes.json").write_text(json.dumps({
        "worlds_found": sorted(actual_worlds),
        "unmapped_worlds": unmapped_worlds,
        "known_test_combinations": len(build_cases()),
        "new_world_policy": "same-name scenario once; review width variants manually",
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    if unmapped_worlds:
        print("COVERAGE WARNING: unmapped worlds recorded in coverage_notes.json. Sweep is not exhaustive.")
    return 1 if unmapped_worlds or any(row["runtime_status"].startswith("ERROR") for row in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
