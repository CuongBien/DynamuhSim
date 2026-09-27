#!/usr/bin/env python3
"""
Collect one validated trajectory-risk episode (C0..C4) from Gazebo.

Prerequisite:
- Gazebo + robot + ros_gz_bridge are already running.
- collect_candidate.py is in the same directory.
- Nav2 command-producing lifecycle nodes must NOT be ACTIVE.

The script:
1) Forces ROS_DOMAIN_ID (default 42).
2) Checks dangerous Nav2 lifecycle nodes.
3) Resets the whole Gazebo world before EACH candidate.
4) Waits for fresh /odom and /scan.
5) Runs C0..C4 using Collector from collect_candidate.py.
6) Validates same initial state and physically plausible displacement.
7) Only if ALL candidates pass, commits the episode to CSV.
8) Re-running the same episode replaces the old episode rows atomically.

code:
cd ~/nav_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 experiments/trajectory_risk_v0/scripts/collect_episode.py \
  --episode 1 \
  --world arena_obstacle 

Thay --episode 1 --world arena_obstacle  bằng các giá trị khác để thu thập các tập dữ liệu khác nhau.
"""

import argparse
import csv
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


NAV2_COMMAND_NODES = (
    "/controller_server",
    "/behavior_server",
    "/bt_navigator",
    "/waypoint_follower",
)


def run_cmd(cmd, timeout=10, check=True):
    p = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    if check and p.returncode != 0:
        raise RuntimeError(
            f"Command failed ({p.returncode}): {' '.join(cmd)}\n{p.stdout}"
        )
    return p.stdout.strip()


def lifecycle_state(node_name):
    out = run_cmd(
        ["ros2", "lifecycle", "get", node_name],
        timeout=5,
        check=False,
    )
    # Typical output: "active [3]" / "inactive [2]" / "unconfigured [1]"
    if not out:
        return None
    return out.split()[0].lower()


def preflight_nav2():
    nodes_text = run_cmd(["ros2", "node", "list"], timeout=8, check=False)
    nodes = set(line.strip() for line in nodes_text.splitlines() if line.strip())

    active_bad = []
    present_inactive = []

    for node in NAV2_COMMAND_NODES:
        if node not in nodes:
            continue
        state = lifecycle_state(node)
        if state == "active":
            active_bad.append(node)
        else:
            present_inactive.append((node, state or "unknown"))

    if active_bad:
        msg = "\n".join(f"  - {n}" for n in active_bad)
        raise RuntimeError(
            "Nav2 command nodes are ACTIVE. Dataset collection is aborted to "
            "avoid /cmd_vel competition.\n"
            f"{msg}\n"
            "Stop/deactivate Nav2, then run this script again."
        )

    if present_inactive:
        print("[OK] Nav2 nodes exist but are not ACTIVE:")
        for node, state in present_inactive:
            print(f"     {node}: {state}")
    else:
        print("[OK] No active Nav2 command nodes detected.")


def reset_world(world, timeout_ms=5000):
    service = f"/world/{world}/control"
    cmd = [
        "gz", "service",
        "-s", service,
        "--reqtype", "gz.msgs.WorldControl",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", str(timeout_ms),
        "--req", "reset: {all: true}",
    ]
    out = run_cmd(cmd, timeout=max(8, timeout_ms / 1000 + 3), check=True)

    low = out.lower()
    # Gazebo versions format successful replies differently.
    if "false" in low and "true" not in low:
        raise RuntimeError(f"World reset appears to have failed:\n{out}")

    print(f"[OK] Reset world: {world}")


def load_collector_module(script_dir):
    path = script_dir / "collect_candidate.py"
    if not path.exists():
        raise FileNotFoundError(
            f"Missing {path}. Put this script in the same scripts/ directory "
            "as collect_candidate.py."
        )

    spec = importlib.util.spec_from_file_location("collect_candidate", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["collect_candidate"] = module
    spec.loader.exec_module(module)
    return module


def wait_for_fresh_inputs(node, rclpy, timeout_sec):
    # New Collector instance => odom/scan must arrive after this point.
    node.odom = None
    node.scan = None

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.10)
        if node.odom is not None and node.scan is not None:
            return

    raise TimeoutError(
        f"No fresh /odom + /scan within {timeout_sec:.1f}s. "
        "Check ROS_DOMAIN_ID and ros_gz_bridge."
    )


def spin_for(node, rclpy, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def validate_candidate(outcome, max_displacement_factor, displacement_margin):
    expected = float(outcome["expected_distance"])
    actual = float(outcome["actual_displacement"])

    upper = expected * max_displacement_factor + displacement_margin
    if actual > upper:
        raise RuntimeError(
            f"Physically implausible rollout: actual_displacement={actual:.3f} m "
            f"> allowed={upper:.3f} m for expected path={expected:.3f} m. "
            "Possible odom jump, teleport/reset during rollout, or another controller."
        )


def validate_same_start(records, pos_tol, yaw_tol_deg, lidar_tol):
    ref = records[0]
    rx = float(ref["outcome"]["start_x"])
    ry = float(ref["outcome"]["start_y"])
    ryaw = float(ref["start_yaw"])

    rf = ref["features"]
    ref_scan = (
        float(rf["lidar_front_min"]),
        float(rf["lidar_left_min"]),
        float(rf["lidar_right_min"]),
    )

    for rec in records[1:]:
        cid = rec["candidate_id"]
        o = rec["outcome"]
        x = float(o["start_x"])
        y = float(o["start_y"])
        yaw = float(rec["start_yaw"])

        pos_err = math.hypot(x - rx, y - ry)
        yaw_err_deg = abs(math.degrees(angle_diff(yaw, ryaw)))

        if pos_err > pos_tol:
            raise RuntimeError(
                f"{cid}: start pose mismatch: position error={pos_err:.3f} m "
                f"(tol={pos_tol:.3f} m)."
            )

        if yaw_err_deg > yaw_tol_deg:
            raise RuntimeError(
                f"{cid}: start yaw mismatch: error={yaw_err_deg:.2f} deg "
                f"(tol={yaw_tol_deg:.2f} deg)."
            )

        f = rec["features"]
        scan = (
            float(f["lidar_front_min"]),
            float(f["lidar_left_min"]),
            float(f["lidar_right_min"]),
        )
        scan_err = max(abs(a - b) for a, b in zip(scan, ref_scan))
        if scan_err > lidar_tol:
            raise RuntimeError(
                f"{cid}: initial LiDAR state differs too much from C0: "
                f"max sector difference={scan_err:.3f} m "
                f"(tol={lidar_tol:.3f} m). "
                "Dynamic obstacle may not have reset to the same phase."
            )


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
            f"CSV schema mismatch in {path}.\n"
            f"Existing: {old_fields}\n"
            f"New:      {new_fields}\n"
            "Move/delete the old test CSV once, then rerun."
        )

    kept = [
        r for r in old_rows
        if str(r.get("episode_id", "")) != str(episode_id)
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
        if tmp_path.exists():
            tmp_path.unlink(missing_ok=True)


def print_summary(records):
    print("\n=== EPISODE VALIDATION SUMMARY ===")
    print(
        f"{'Candidate':<10}"
        f"{'start_x':>10}"
        f"{'start_y':>10}"
        f"{'disp(m)':>10}"
        f"{'path(m)':>10}"
        f"{'min_rng':>10}"
        f"{'stuck':>8}"
    )

    for rec in records:
        o = rec["outcome"]
        print(
            f"{rec['candidate_id']:<10}"
            f"{float(o['start_x']):>10.3f}"
            f"{float(o['start_y']):>10.3f}"
            f"{float(o['actual_displacement']):>10.3f}"
            f"{float(o['expected_distance']):>10.3f}"
            f"{float(o['actual_min_range']):>10.3f}"
            f"{int(o['stuck']):>8d}"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Reset-and-collect one validated C0..C4 trajectory-risk episode."
    )
    parser.add_argument("--episode", type=int, required=True)
    parser.add_argument("--world", default="arena_obstacle")
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--reset-wait", type=float, default=1.0)
    parser.add_argument("--sensor-timeout", type=float, default=10.0)
    parser.add_argument("--start-pos-tol", type=float, default=0.05)
    parser.add_argument("--start-yaw-tol-deg", type=float, default=5.0)
    parser.add_argument("--lidar-start-tol", type=float, default=0.20)
    parser.add_argument("--max-displacement-factor", type=float, default=1.20)
    parser.add_argument("--displacement-margin", type=float, default=0.05)
    args = parser.parse_args()

    # Must be set before rclpy.init().
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)

    script_dir = Path(__file__).resolve().parent
    cc = load_collector_module(script_dir)

    import rclpy

    print("=== TRAJECTORY RISK DATASET COLLECTOR ===")
    print(f"episode          : {args.episode}")
    print(f"world            : {args.world}")
    print(f"ROS_DOMAIN_ID    : {args.domain}")
    print(f"candidates       : {', '.join(cc.CANDIDATES.keys())}")

    preflight_nav2()

    records = []

    rclpy.init()
    try:
        for i, candidate_id in enumerate(cc.CANDIDATES.keys(), start=1):
            print(f"\n--- [{i}/{len(cc.CANDIDATES)}] {candidate_id} ---")

            reset_world(args.world)

            # Allow Gazebo/bridges to settle after reset.
            time.sleep(args.reset_wait)

            node = cc.Collector()
            try:
                wait_for_fresh_inputs(node, rclpy, args.sensor_timeout)
                spin_for(node, rclpy, 0.30)

                sx, sy, syaw = node.get_pose()
                print(
                    f"[START] x={sx:.3f}, y={sy:.3f}, "
                    f"yaw={math.degrees(syaw):.2f} deg"
                )

                features, outcome = node.collect(args.episode, candidate_id)

                validate_candidate(
                    outcome,
                    args.max_displacement_factor,
                    args.displacement_margin,
                )

                records.append({
                    "candidate_id": candidate_id,
                    "start_yaw": syaw,
                    "features": features,
                    "outcome": outcome,
                })

            finally:
                try:
                    node.stop_robot()
                except Exception:
                    pass
                node.destroy_node()

        # Validate all candidates only after every rollout completed.
        validate_same_start(
            records,
            pos_tol=args.start_pos_tol,
            yaw_tol_deg=args.start_yaw_tol_deg,
            lidar_tol=args.lidar_start_tol,
        )

        print_summary(records)

        feature_rows = [rec["features"] for rec in records]
        outcome_rows = [rec["outcome"] for rec in records]

        # Commit only after ALL validations pass.
        atomic_replace_episode(
            cc.FEATURE_FILE,
            feature_rows,
            args.episode,
        )
        atomic_replace_episode(
            cc.OUTCOME_FILE,
            outcome_rows,
            args.episode,
        )

        print("\n[PASS] Episode is valid and has been committed.")
        print(f"Features : {cc.FEATURE_FILE}")
        print(f"Outcomes : {cc.OUTCOME_FILE}")
        print(
            f"Episode {args.episode} now contains "
            f"{len(records)} validated trajectory samples."
        )

    except Exception as exc:
        print("\n[FAIL] Episode was NOT written to dataset.")
        print(f"Reason: {exc}")
        sys.exit(1)

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
