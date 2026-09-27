#!/usr/bin/env python3
"""
collect_dataset_scenarios.py

Batch collector for trajectory-risk dataset with automatically varied scenarios.

Requires:
    collect_episode_v2.py
in the SAME scripts/ directory.

Recommended launch for v0:
    ros2 launch custom_corridor corridor_tb3.launch.py \
        width:=arena obstacle:=object

For every candidate C0..C4 inside one episode:
    - robot is reset to the same world pose
    - obstacle is reset to the same initial world pose
    - obstacle starts the same motion at rollout t=0

Outputs:
    data/scenario_manifest.csv
    data/candidate_features_scenarios.csv
    data/rollout_outcomes_scenarios.csv
"""

import sys
import os

def _early_domain():
    domain = "42"
    for i, arg in enumerate(sys.argv):
        if arg == "--domain" and i + 1 < len(sys.argv):
            domain = sys.argv[i + 1]
        elif arg.startswith("--domain="):
            domain = arg.split("=", 1)[1]
    os.environ["ROS_DOMAIN_ID"] = str(domain)

_early_domain()

import argparse
import csv
import importlib.util
import math
from pathlib import Path
import random
import tempfile
import threading
import time

import numpy as np
import rclpy

SCRIPT_DIR = Path(__file__).resolve().parent
BASE_SCRIPT = SCRIPT_DIR / "collect_episode_v2.py"

if not BASE_SCRIPT.exists():
    raise FileNotFoundError(
        f"Missing {BASE_SCRIPT}\n"
        "Place collect_dataset_scenarios.py and collect_episode_v2.py "
        "in the same scripts/ directory."
    )

_spec = importlib.util.spec_from_file_location(
    "collect_episode_v2",
    BASE_SCRIPT,
)
base = importlib.util.module_from_spec(_spec)
sys.modules["collect_episode_v2"] = base
_spec.loader.exec_module(base)

BASE_DIR = Path.home() / "nav_ws/experiments/trajectory_risk_v0"
DATA_DIR = BASE_DIR / "data"

FEATURE_FILE = DATA_DIR / "candidate_features_scenarios.csv"
OUTCOME_FILE = DATA_DIR / "rollout_outcomes_scenarios.csv"
MANIFEST_FILE = DATA_DIR / "scenario_manifest.csv"

SCENARIO_CYCLE = [
    "safe_far",
    "safe_side",
    "static_center",
    "cross_left_to_right",
    "cross_right_to_left",
    "head_on",
    "same_direction_slow",
    "diagonal_cross",
    "static_center",
    "head_on",
]

def make_scenario(episode_id, seed):
    rng = random.Random(seed * 1_000_003 + episode_id)
    family = SCENARIO_CYCLE[(episode_id - 1) % len(SCENARIO_CYCLE)]

    x = -12.0
    y = 0.0
    speed = 0.0
    heading = 0.0
    expected_risk_hint = "unknown"

    if family == "safe_far":
        x = rng.uniform(-9.0, -8.0)
        y = rng.uniform(-1.5, 1.5)
        speed = rng.uniform(0.0, 0.20)
        heading = rng.choice([0.0, math.pi])
        expected_risk_hint = "safe"

    elif family == "safe_side":
        x = rng.uniform(-12.5, -11.5)
        y = rng.choice([-1.0, 1.0]) * rng.uniform(1.4, 2.2)
        speed = rng.uniform(0.0, 0.25)
        heading = 0.0
        expected_risk_hint = "safe"

    elif family == "static_center":
        x = rng.uniform(-12.55, -12.15)
        y = rng.uniform(-0.12, 0.12)
        speed = 0.0
        heading = 0.0
        expected_risk_hint = "risky"

    elif family == "cross_left_to_right":
        x = rng.uniform(-12.55, -12.20)
        y = rng.uniform(-1.00, -0.65)
        speed = rng.uniform(0.25, 0.55)
        heading = math.pi / 2.0
        expected_risk_hint = "risky"

    elif family == "cross_right_to_left":
        x = rng.uniform(-12.55, -12.20)
        y = rng.uniform(0.65, 1.00)
        speed = rng.uniform(0.25, 0.55)
        heading = -math.pi / 2.0
        expected_risk_hint = "risky"

    elif family == "head_on":
        x = rng.uniform(-11.90, -11.45)
        y = rng.uniform(-0.12, 0.12)
        speed = rng.uniform(0.20, 0.45)
        heading = math.pi
        expected_risk_hint = "risky"

    elif family == "same_direction_slow":
        x = rng.uniform(-12.50, -12.15)
        y = rng.uniform(-0.12, 0.12)
        speed = rng.uniform(0.05, 0.16)
        heading = 0.0
        expected_risk_hint = "risky"

    elif family == "diagonal_cross":
        x = rng.uniform(-12.45, -12.10)
        side = rng.choice([-1.0, 1.0])
        y = side * rng.uniform(0.55, 0.90)
        target_x = rng.uniform(-12.55, -12.30)
        target_y = 0.0
        heading = math.atan2(target_y - y, target_x - x)
        speed = rng.uniform(0.25, 0.50)
        expected_risk_hint = "risky"

    else:
        raise ValueError(f"Unknown scenario family: {family}")

    return {
        "episode_id": int(episode_id),
        "scenario_family": family,
        "scenario_expected_risk_hint": expected_risk_hint,
        "obstacle_start_x": float(x),
        "obstacle_start_y": float(y),
        "obstacle_speed": float(speed),
        "obstacle_heading": float(heading),
        "motion_model": "linear" if speed > 0 else "static",
        "seed": int(seed),
    }

def read_csv_rows(path):
    if not path.exists() or path.stat().st_size == 0:
        return [], None
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        return list(reader), reader.fieldnames

def atomic_replace_episode(path, new_rows, episode_id):
    old_rows, old_fields = read_csv_rows(path)
    new_fields = list(new_rows[0].keys())

    if old_fields is not None and old_fields != new_fields:
        raise RuntimeError(
            f"CSV schema mismatch in {path}\n"
            "Move/delete this scenario CSV once, then rerun."
        )

    kept = [
        row for row in old_rows
        if str(row.get("episode_id", "")) != str(episode_id)
    ]
    merged = kept + new_rows

    path.parent.mkdir(parents=True, exist_ok=True)

    fd, tmp_name = tempfile.mkstemp(
        prefix=path.name + ".",
        suffix=".tmp",
        dir=str(path.parent),
        text=True,
    )
    os.close(fd)
    tmp_path = Path(tmp_name)

    try:
        with tmp_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=new_fields)
            writer.writeheader()
            writer.writerows(merged)
        os.replace(tmp_path, path)
    finally:
        tmp_path.unlink(missing_ok=True)

def set_entity_pose(world, entity, x, y, z, yaw, quiet=False):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    request = (
        f'name: "{entity}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'
    )

    out = base.run_cmd(
        [
            "gz", "service",
            "-s", f"/world/{world}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", request,
        ],
        timeout=6,
        check=True,
    )

    low = out.lower()
    if "false" in low and "true" not in low:
        raise RuntimeError(
            f"set_pose failed for entity '{entity}':\n{out}"
        )

    if not quiet:
        print(
            f"[OK] set_pose {entity}: "
            f"x={x:.3f}, y={y:.3f}, "
            f"yaw={math.degrees(yaw):.1f} deg"
        )

class LinearObstacleMover:
    def __init__(
        self,
        world,
        entity,
        z,
        scenario,
        hz=8.0,
        max_duration=4.0,
    ):
        self.world = world
        self.entity = entity
        self.z = z
        self.scenario = scenario
        self.hz = max(2.0, float(hz))
        self.max_duration = float(max_duration)
        self._stop_event = threading.Event()
        self._thread = None
        self.error = None

    def reset_initial_pose(self):
        s = self.scenario
        set_entity_pose(
            self.world,
            self.entity,
            s["obstacle_start_x"],
            s["obstacle_start_y"],
            self.z,
            s["obstacle_heading"],
            quiet=False,
        )

    def _loop(self):
        s = self.scenario
        x0 = float(s["obstacle_start_x"])
        y0 = float(s["obstacle_start_y"])
        speed = float(s["obstacle_speed"])
        heading = float(s["obstacle_heading"])

        period = 1.0 / self.hz
        start = time.monotonic()

        try:
            while not self._stop_event.is_set():
                t = time.monotonic() - start
                if t > self.max_duration:
                    break

                x = x0 + speed * t * math.cos(heading)
                y = y0 + speed * t * math.sin(heading)

                set_entity_pose(
                    self.world,
                    self.entity,
                    x,
                    y,
                    self.z,
                    heading,
                    quiet=True,
                )

                self._stop_event.wait(period)

        except Exception as exc:
            self.error = exc
            self._stop_event.set()

    def start(self):
        self.error = None
        self._stop_event.clear()

        if float(self.scenario["obstacle_speed"]) <= 1e-9:
            return

        self._thread = threading.Thread(
            target=self._loop,
            name="obstacle_mover",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self.error is not None:
            raise RuntimeError(
                f"Obstacle motion thread failed: {self.error}"
            )

class ScenarioCollector(base.EpisodeCollector):
    def __init__(self):
        super().__init__()
        self.active_mover = None

    def execute_candidate(self, v, w, duration=base.DURATION):
        mover = self.active_mover
        if mover is not None:
            mover.start()

        try:
            super().execute_candidate(v, w, duration)
        finally:
            if mover is not None:
                mover.stop()

def add_scenario_features(
    features,
    scenario,
    robot_start_x,
    robot_start_y,
    robot_start_yaw,
):
    dx_world = (
        float(scenario["obstacle_start_x"])
        - float(robot_start_x)
    )
    dy_world = (
        float(scenario["obstacle_start_y"])
        - float(robot_start_y)
    )

    c = math.cos(-robot_start_yaw)
    s = math.sin(-robot_start_yaw)

    rel_x = c * dx_world - s * dy_world
    rel_y = s * dx_world + c * dy_world

    rel_heading = base.angle_normalize(
        float(scenario["obstacle_heading"])
        - float(robot_start_yaw)
    )

    features.update(
        {
            "scenario_family": scenario["scenario_family"],
            "obs_rel_x0": float(rel_x),
            "obs_rel_y0": float(rel_y),
            "obs_speed": float(scenario["obstacle_speed"]),
            "obs_heading_rel": float(rel_heading),
            "obs_world_x0": float(scenario["obstacle_start_x"]),
            "obs_world_y0": float(scenario["obstacle_start_y"]),
        }
    )

def add_scenario_outcome_metadata(outcome, scenario):
    outcome.update(
        {
            "scenario_family": scenario["scenario_family"],
            "scenario_expected_risk_hint":
                scenario["scenario_expected_risk_hint"],
            "scenario_obstacle_speed":
                float(scenario["obstacle_speed"]),
            "scenario_obstacle_heading":
                float(scenario["obstacle_heading"]),
        }
    )

def print_episode_summary(scenario, records):
    print("\n=== SCENARIO ===")
    print(
        f"family       : {scenario['scenario_family']}\n"
        f"risk hint    : {scenario['scenario_expected_risk_hint']}\n"
        f"obstacle p0  : "
        f"({scenario['obstacle_start_x']:.3f}, "
        f"{scenario['obstacle_start_y']:.3f})\n"
        f"speed        : {scenario['obstacle_speed']:.3f} m/s\n"
        f"heading      : "
        f"{math.degrees(scenario['obstacle_heading']):.1f} deg"
    )
    base.print_summary(records)

def collect_episode(node, episode_id, scenario, args):
    print(
        f"\n\n{'#' * 68}\n"
        f"EPISODE {episode_id} | "
        f"{scenario['scenario_family']} | "
        f"hint={scenario['scenario_expected_risk_hint']}\n"
        f"{'#' * 68}"
    )

    records = []

    mover = LinearObstacleMover(
        world=args.world,
        entity=args.obstacle_model,
        z=args.obstacle_z,
        scenario=scenario,
        hz=args.obstacle_hz,
        max_duration=base.DURATION + 0.5,
    )

    for idx, candidate_id in enumerate(
        base.CANDIDATES.keys(),
        start=1,
    ):
        print(
            f"\n{'=' * 58}\n"
            f"[{idx}/{len(base.CANDIDATES)}] {candidate_id}\n"
            f"{'=' * 58}"
        )

        node.publish_stop(duration=0.4)

        base.set_robot_pose(
            args.world,
            args.robot_model,
            args.robot_x,
            args.robot_y,
            args.robot_z,
            args.robot_yaw,
        )

        mover.reset_initial_pose()

        node.wait_for_fresh_sensors(
            timeout_sec=args.sensor_timeout
        )
        node.spin_for(args.settle)

        node.publish_stop(duration=0.3)
        stationary = node.wait_stationary()
        lidar = node.get_lidar_state()

        print(
            "[READY] "
            f"v={stationary['v']:.4f}, "
            f"w={stationary['w']:.4f}, "
            f"LiDAR(F/L/R)="
            f"{lidar['front']:.3f}/"
            f"{lidar['left']:.3f}/"
            f"{lidar['right']:.3f}"
        )

        node.active_mover = mover

        features, outcome = node.collect_candidate(
            episode_id,
            candidate_id,
        )

        node.active_mover = None

        add_scenario_features(
            features,
            scenario,
            robot_start_x=args.robot_x,
            robot_start_y=args.robot_y,
            robot_start_yaw=args.robot_yaw,
        )

        add_scenario_outcome_metadata(
            outcome,
            scenario,
        )

        base.validate_displacement(outcome)

        records.append(
            {
                "candidate_id": candidate_id,
                "features": features,
                "outcome": outcome,
            }
        )

    base.validate_initial_lidar(
        records,
        tolerance=args.lidar_tol,
    )

    print_episode_summary(
        scenario,
        records,
    )

    return records

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--start-episode", type=int, default=1)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=42)

    parser.add_argument("--world", default="arena_obstacle")
    parser.add_argument("--domain", type=int, default=42)

    parser.add_argument("--robot-model", default="burger")
    parser.add_argument("--robot-x", type=float, default=-13.0)
    parser.add_argument("--robot-y", type=float, default=0.0)
    parser.add_argument("--robot-z", type=float, default=0.01)
    parser.add_argument("--robot-yaw", type=float, default=0.0)

    parser.add_argument(
        "--obstacle-model",
        default="dynamic_obstacle",
        help=(
            "Gazebo entity name. If set_pose fails, run `gz model --list` "
            "and pass the correct entity name."
        ),
    )
    parser.add_argument("--obstacle-z", type=float, default=0.85)
    parser.add_argument("--obstacle-hz", type=float, default=8.0)

    parser.add_argument("--settle", type=float, default=0.8)
    parser.add_argument("--sensor-timeout", type=float, default=10.0)
    parser.add_argument("--lidar-tol", type=float, default=0.25)

    parser.add_argument(
        "--skip-failed",
        action="store_true",
    )

    args = parser.parse_args()

    print("=== MULTI-SCENARIO TRAJECTORY-RISK COLLECTOR ===")
    print(f"episodes       : {args.episodes}")
    print(f"start episode  : {args.start_episode}")
    print(f"seed           : {args.seed}")
    print(f"world          : {args.world}")
    print(f"robot model    : {args.robot_model}")
    print(f"obstacle model : {args.obstacle_model}")
    print(f"ROS_DOMAIN_ID  : {os.environ['ROS_DOMAIN_ID']}")

    try:
        base.preflight()
    except Exception as exc:
        print(f"\n[FAIL PRECHECK] {exc}")
        sys.exit(1)

    DATA_DIR.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ScenarioCollector()

    passed = 0
    failed = 0

    try:
        for episode_id in range(
            args.start_episode,
            args.start_episode + args.episodes,
        ):
            scenario = make_scenario(
                episode_id,
                args.seed,
            )

            try:
                records = collect_episode(
                    node,
                    episode_id,
                    scenario,
                    args,
                )

                feature_rows = [
                    r["features"]
                    for r in records
                ]
                outcome_rows = [
                    r["outcome"]
                    for r in records
                ]

                atomic_replace_episode(
                    FEATURE_FILE,
                    feature_rows,
                    episode_id,
                )
                atomic_replace_episode(
                    OUTCOME_FILE,
                    outcome_rows,
                    episode_id,
                )
                atomic_replace_episode(
                    MANIFEST_FILE,
                    [scenario],
                    episode_id,
                )

                passed += 1
                print(
                    f"\n[PASS] Episode {episode_id} committed "
                    f"({len(records)} samples)."
                )

            except Exception as exc:
                failed += 1

                print(
                    f"\n[FAIL] Episode {episode_id} NOT committed."
                )
                print(f"Reason: {exc}")

                try:
                    node.publish_stop(duration=0.5)
                except Exception:
                    pass

                if not args.skip_failed:
                    raise

        print(
            "\n" + "=" * 68 +
            "\nBATCH SUMMARY\n" +
            "=" * 68
        )
        print(f"Passed episodes : {passed}")
        print(f"Failed episodes : {failed}")
        print(
            f"Samples expected: "
            f"{passed * len(base.CANDIDATES)}"
        )
        print(f"Features        : {FEATURE_FILE}")
        print(f"Outcomes        : {OUTCOME_FILE}")
        print(f"Manifest        : {MANIFEST_FILE}")

    except Exception as exc:
        print(f"\n[BATCH STOPPED] {exc}")
        sys.exit(1)

    finally:
        try:
            node.publish_stop(duration=0.5)
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
