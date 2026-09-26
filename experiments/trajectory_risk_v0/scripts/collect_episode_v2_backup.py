#!/usr/bin/env python3
"""
collect_episode_v2.py

One-command trajectory-risk dataset collector for ROS 2 Jazzy + Gazebo Sim.

Design:
- DOES NOT reset the Gazebo world / simulation clock.
- Before every candidate:
    STOP robot -> set_pose("burger") -> wait for fresh /odom + /scan
    -> verify robot is stationary -> execute candidate -> log rollout.
- Runs C0..C4 from the same WORLD pose.
- Aborts the whole episode if:
    * Nav2 command nodes are ACTIVE
    * /odom or /scan publisher is missing
    * /odom has multiple publishers
    * sensors do not become fresh
    * robot is not stationary before rollout
    * displacement is physically implausible
    * static-scene initial LiDAR states differ too much
- Commits the episode to CSV only after ALL five candidates pass.
- Re-running the same episode replaces only that episode in the CSV.

Recommended v0 collection:
    arena + obstacle:=object
or:
    arena + obstacle:=none

Do NOT use a moving human for "same-state candidate comparison" until the
dynamic obstacle can also be reset to the same phase before every candidate.
"""

import argparse
import csv
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np

# ---------------------------------------------------------------------
# IMPORTANT: ROS_DOMAIN_ID must be set before importing / initializing rclpy.
# We parse only the --domain flag early for this purpose.
# ---------------------------------------------------------------------
def _early_domain():
    domain = "42"
    for i, arg in enumerate(sys.argv):
        if arg == "--domain" and i + 1 < len(sys.argv):
            domain = sys.argv[i + 1]
        elif arg.startswith("--domain="):
            domain = arg.split("=", 1)[1]
    os.environ["ROS_DOMAIN_ID"] = str(domain)

_early_domain()

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
BASE_DIR = Path.home() / "nav_ws/experiments/trajectory_risk_v0"
DATA_DIR = BASE_DIR / "data"

FEATURE_FILE = DATA_DIR / "candidate_features.csv"
OUTCOME_FILE = DATA_DIR / "rollout_outcomes.csv"

ODOM_TOPIC = "/odom"
SCAN_TOPIC = "/scan"
CMD_TOPIC = "/cmd_vel"

DT = 0.1
DURATION = 3.0

CANDIDATES = {
    "C0": (0.30, +0.40),
    "C1": (0.30, +0.20),
    "C2": (0.30,  0.00),
    "C3": (0.30, -0.20),
    "C4": (0.30, -0.40),
}

NAV2_COMMAND_NODES = (
    "/controller_server",
    "/behavior_server",
    "/bt_navigator",
    "/waypoint_follower",
)


# ---------------------------------------------------------------------
# Shell helpers
# ---------------------------------------------------------------------
def run_cmd(cmd, timeout=10, check=True):
    p = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        env=os.environ.copy(),
    )
    if check and p.returncode != 0:
        raise RuntimeError(
            f"Command failed ({p.returncode}): {' '.join(cmd)}\n{p.stdout}"
        )
    return p.stdout.strip()


def topic_publisher_count(topic):
    out = run_cmd(
        ["ros2", "topic", "info", topic],
        timeout=6,
        check=False,
    )
    for line in out.splitlines():
        if line.strip().startswith("Publisher count:"):
            try:
                return int(line.split(":", 1)[1].strip())
            except ValueError:
                return 0
    return 0


def lifecycle_state(node_name):
    out = run_cmd(
        ["ros2", "lifecycle", "get", node_name],
        timeout=5,
        check=False,
    )
    if not out:
        return None
    return out.split()[0].strip().lower()


def preflight():
    print("\n=== PREFLIGHT ===")

    nodes_text = run_cmd(["ros2", "node", "list"], timeout=8, check=False)
    nodes = set(x.strip() for x in nodes_text.splitlines() if x.strip())

    active_nav2 = []
    for node in NAV2_COMMAND_NODES:
        if node in nodes:
            state = lifecycle_state(node)
            print(f"Nav2 {node}: {state or 'unknown'}")
            if state == "active":
                active_nav2.append(node)

    if active_nav2:
        raise RuntimeError(
            "Nav2 command node(s) are ACTIVE and may compete for /cmd_vel:\n  "
            + "\n  ".join(active_nav2)
            + "\nDeactivate/stop Nav2 before dataset collection."
        )

    odom_pub = topic_publisher_count(ODOM_TOPIC)
    scan_pub = topic_publisher_count(SCAN_TOPIC)

    print(f"{ODOM_TOPIC} publishers: {odom_pub}")
    print(f"{SCAN_TOPIC} publishers: {scan_pub}")

    if odom_pub == 0:
        raise RuntimeError(
            "/odom has no ROS publisher. Start the odom bridge first."
        )

    if odom_pub > 1:
        raise RuntimeError(
            f"/odom has {odom_pub} publishers. This can cause odometry jumps. "
            "Keep exactly one GZ->ROS odom bridge."
        )

    if scan_pub == 0:
        raise RuntimeError(
            "/scan has no ROS publisher. Check ros_gz_bridge."
        )

    print("[OK] Preflight passed.")


def stop_robot_cli():
    # TwistStamped matches the current turtlebot3 bridge YAML.
    msg = "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
    run_cmd(
        [
            "ros2", "topic", "pub", "--once",
            CMD_TOPIC,
            "geometry_msgs/msg/TwistStamped",
            msg,
        ],
        timeout=5,
        check=False,
    )


def set_robot_pose(world, model, x, y, z, yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    request = (
        f'name: "{model}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'
    )

    out = run_cmd(
        [
            "gz", "service",
            "-s", f"/world/{world}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000",
            "--req", request,
        ],
        timeout=8,
        check=True,
    )

    # Most Gazebo versions show "data: true" or a true-like reply.
    low = out.lower()
    if "false" in low and "true" not in low:
        raise RuntimeError(f"set_pose returned failure:\n{out}")

    print(
        f"[OK] set_pose {model}: "
        f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
        f"yaw={math.degrees(yaw):.1f} deg"
    )


# ---------------------------------------------------------------------
# Math / feature helpers
# ---------------------------------------------------------------------
def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_normalize(a):
    return math.atan2(math.sin(a), math.cos(a))


def finite_scan_values(msg):
    return [
        float(r)
        for r in msg.ranges
        if np.isfinite(r)
        and float(r) >= float(msg.range_min)
        and float(r) <= float(msg.range_max)
    ]


# ---------------------------------------------------------------------
# ROS collector
# ---------------------------------------------------------------------
class EpisodeCollector(Node):

    def __init__(self):
        super().__init__("trajectory_risk_episode_collector")

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            CMD_TOPIC,
            10,
        )

        # Sensor QoS (BEST_EFFORT subscriber) is compatible with common
        # Gazebo sensor publishers and with RELIABLE publishers.
        self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.odom = None
        self.scan = None

        self.last_odom_monotonic = None
        self.last_scan_monotonic = None

        self.reset_rollout_stats()

    def reset_rollout_stats(self):
        self.rollout_global_min = float("inf")
        self.rollout_front_min = float("inf")
        self.rollout_left_min = float("inf")
        self.rollout_right_min = float("inf")

    def odom_callback(self, msg):
        self.odom = msg
        self.last_odom_monotonic = time.monotonic()

    def scan_callback(self, msg):
        self.scan = msg
        self.last_scan_monotonic = time.monotonic()

        vals = finite_scan_values(msg)
        if vals:
            self.rollout_global_min = min(
                self.rollout_global_min,
                min(vals),
            )

        front = self.sector_min_from_msg(msg, -30, 30)
        left = self.sector_min_from_msg(msg, 30, 100)
        right = self.sector_min_from_msg(msg, -100, -30)

        self.rollout_front_min = min(self.rollout_front_min, front)
        self.rollout_left_min = min(self.rollout_left_min, left)
        self.rollout_right_min = min(self.rollout_right_min, right)

    def sector_min_from_msg(self, msg, deg_min, deg_max):
        values = []

        for i, r in enumerate(msg.ranges):
            r = float(r)

            if not np.isfinite(r):
                continue
            if r < float(msg.range_min) or r > float(msg.range_max):
                continue

            deg = math.degrees(
                float(msg.angle_min)
                + i * float(msg.angle_increment)
            )

            if deg_min <= deg <= deg_max:
                values.append(r)

        if not values:
            return float(msg.range_max)

        return min(values)

    def sector_min(self, deg_min, deg_max):
        if self.scan is None:
            return float("nan")
        return self.sector_min_from_msg(
            self.scan,
            deg_min,
            deg_max,
        )

    def wait_for_fresh_sensors(self, timeout_sec=10.0):
        self.odom = None
        self.scan = None
        self.last_odom_monotonic = None
        self.last_scan_monotonic = None

        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)

            if self.odom is not None and self.scan is not None:
                return

        raise TimeoutError(
            f"No fresh {ODOM_TOPIC} + {SCAN_TOPIC} "
            f"within {timeout_sec:.1f}s."
        )

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_odom_state(self):
        if self.odom is None:
            raise RuntimeError("No odom available.")

        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        t = self.odom.twist.twist

        return {
            "x": float(p.x),
            "y": float(p.y),
            "yaw": float(yaw_from_quaternion(q)),
            "v": float(t.linear.x),
            "w": float(t.angular.z),
        }

    def get_lidar_state(self):
        if self.scan is None:
            raise RuntimeError("No scan available.")

        return {
            "front": float(self.sector_min(-30, 30)),
            "left": float(self.sector_min(30, 100)),
            "right": float(self.sector_min(-100, -30)),
        }

    def wait_stationary(
        self,
        timeout_sec=4.0,
        max_v=0.03,
        max_w=0.05,
        stable_sec=0.5,
    ):
        deadline = time.monotonic() + timeout_sec
        stable_since = None

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.odom is None:
                continue

            state = self.get_odom_state()

            okay = (
                abs(state["v"]) <= max_v
                and abs(state["w"]) <= max_w
            )

            if okay:
                if stable_since is None:
                    stable_since = time.monotonic()

                if time.monotonic() - stable_since >= stable_sec:
                    return state
            else:
                stable_since = None

        state = self.get_odom_state()
        raise RuntimeError(
            "Robot did not become stationary before rollout: "
            f"v={state['v']:.4f} m/s, "
            f"w={state['w']:.4f} rad/s"
        )

    def publish_stop(self, duration=0.5):
        msg = TwistStamped()

        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def predict_trajectory(self, v, w, duration=DURATION, dt=DT):
        x = 0.0
        y = 0.0
        theta = 0.0
        points = []

        steps = int(round(duration / dt))

        for _ in range(steps):
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            points.append((x, y, theta))

        return points

    def execute_candidate(self, v, w, duration=DURATION):
        self.reset_rollout_stats()

        cmd = TwistStamped()
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)

        start = time.monotonic()

        while time.monotonic() - start < duration:
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)

            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.08)

        self.publish_stop(duration=0.6)

    def collect_candidate(self, episode_id, candidate_id):
        if candidate_id not in CANDIDATES:
            raise ValueError(f"Unknown candidate: {candidate_id}")

        v_cmd, w_cmd = CANDIDATES[candidate_id]

        # Fresh state immediately before rollout.
        self.spin_for(0.25)

        start = self.get_odom_state()
        lidar = self.get_lidar_state()

        trajectory = self.predict_trajectory(
            v_cmd,
            w_cmd,
            DURATION,
            DT,
        )
        end_x_rel, end_y_rel, end_yaw_rel = trajectory[-1]

        traj_length = abs(v_cmd) * DURATION

        curvature = (
            abs(w_cmd / v_cmd)
            if abs(v_cmd) > 1e-9
            else 0.0
        )

        features = {
            "episode_id": int(episode_id),
            "candidate_id": candidate_id,

            "robot_v": start["v"],
            "robot_w": start["w"],

            "lidar_front_min": lidar["front"],
            "lidar_left_min": lidar["left"],
            "lidar_right_min": lidar["right"],

            "traj_v": float(v_cmd),
            "traj_w": float(w_cmd),
            "traj_duration": float(DURATION),
            "traj_length": float(traj_length),
            "traj_max_abs_w": abs(float(w_cmd)),
            "traj_curvature": float(curvature),

            "traj_end_x_rel": float(end_x_rel),
            "traj_end_y_rel": float(end_y_rel),
            "traj_end_yaw_rel": float(end_yaw_rel),
        }

        print("\nINPUT FEATURES")
        print(features)

        self.execute_candidate(
            v_cmd,
            w_cmd,
            DURATION,
        )

        self.spin_for(0.25)

        end = self.get_odom_state()

        displacement = math.hypot(
            end["x"] - start["x"],
            end["y"] - start["y"],
        )

        progress_ratio = (
            displacement / traj_length
            if traj_length > 1e-9
            else 0.0
        )

        stuck = int(
            traj_length > 0.30
            and displacement < 0.10
        )

        outcome = {
            "episode_id": int(episode_id),
            "candidate_id": candidate_id,

            # Odom coordinates are kept only for per-rollout displacement.
            # They are NOT used to prove equal world start pose.
            "start_odom_x": start["x"],
            "start_odom_y": start["y"],
            "start_odom_yaw": start["yaw"],

            "end_odom_x": end["x"],
            "end_odom_y": end["y"],
            "end_odom_yaw": end["yaw"],

            "actual_displacement": float(displacement),
            "expected_distance": float(traj_length),
            "progress_ratio": float(progress_ratio),

            # Keep raw global minimum for audit.
            "actual_min_range_raw": float(self.rollout_global_min),

            # More interpretable directional minima during the rollout.
            "rollout_front_min": float(self.rollout_front_min),
            "rollout_left_min": float(self.rollout_left_min),
            "rollout_right_min": float(self.rollout_right_min),

            # Backward-compatible proxy. Do not treat as exact physical clearance.
            "actual_min_range": float(
                min(
                    self.rollout_front_min,
                    self.rollout_left_min,
                    self.rollout_right_min,
                )
            ),

            "stuck": int(stuck),
        }

        print("\nROLLOUT OUTCOME")
        print(outcome)

        return features, outcome


# ---------------------------------------------------------------------
# Validation / CSV commit
# ---------------------------------------------------------------------
def validate_displacement(
    outcome,
    max_factor=1.20,
    margin=0.05,
):
    actual = float(outcome["actual_displacement"])
    expected = float(outcome["expected_distance"])

    upper = expected * max_factor + margin

    if actual > upper:
        raise RuntimeError(
            "Implausible odometry displacement: "
            f"{actual:.3f} m > {upper:.3f} m "
            f"(expected path={expected:.3f} m). "
            "Possible odom jump, duplicate /odom publisher, "
            "or another controller moved the robot."
        )


def validate_initial_lidar(
    records,
    tolerance=0.20,
):
    """
    For a STATIC scene, reset-to-same-world-pose should produce similar
    front/left/right LiDAR state for all candidates.
    """
    ref = records[0]["features"]
    ref_vec = np.array(
        [
            ref["lidar_front_min"],
            ref["lidar_left_min"],
            ref["lidar_right_min"],
        ],
        dtype=float,
    )

    for rec in records[1:]:
        f = rec["features"]
        vec = np.array(
            [
                f["lidar_front_min"],
                f["lidar_left_min"],
                f["lidar_right_min"],
            ],
            dtype=float,
        )

        err = float(np.max(np.abs(vec - ref_vec)))

        if err > tolerance:
            raise RuntimeError(
                f"{rec['candidate_id']}: initial LiDAR state mismatch "
                f"(max sector difference={err:.3f} m, "
                f"tolerance={tolerance:.3f} m). "
                "For dataset v0 use a static scene "
                "(obstacle:=object or obstacle:=none), or reset the dynamic "
                "obstacle to the same phase before every candidate."
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
            f"Existing CSV schema differs from v2 schema: {path}\n"
            "Move/delete the old test CSV once, then rerun.\n"
            f"Existing fields: {old_fields}\n"
            f"New fields:      {new_fields}"
        )

    kept = [
        row
        for row in old_rows
        if str(row.get("episode_id", "")) != str(episode_id)
    ]

    merged = kept + new_rows

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

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
            writer = csv.DictWriter(
                f,
                fieldnames=new_fields,
            )
            writer.writeheader()
            writer.writerows(merged)

        os.replace(
            tmp_path,
            path,
        )
    finally:
        tmp_path.unlink(missing_ok=True)


def print_summary(records):
    print("\n=== EPISODE SUMMARY ===")
    print(
        f"{'Cand':<6}"
        f"{'disp':>9}"
        f"{'expect':>9}"
        f"{'ratio':>9}"
        f"{'front':>9}"
        f"{'left':>9}"
        f"{'right':>9}"
        f"{'stuck':>8}"
    )

    for rec in records:
        o = rec["outcome"]

        print(
            f"{rec['candidate_id']:<6}"
            f"{float(o['actual_displacement']):>9.3f}"
            f"{float(o['expected_distance']):>9.3f}"
            f"{float(o['progress_ratio']):>9.3f}"
            f"{float(o['rollout_front_min']):>9.3f}"
            f"{float(o['rollout_left_min']):>9.3f}"
            f"{float(o['rollout_right_min']):>9.3f}"
            f"{int(o['stuck']):>8d}"
        )


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--episode",
        type=int,
        required=True,
    )
    parser.add_argument(
        "--world",
        default="arena_obstacle",
    )
    parser.add_argument(
        "--model",
        default="burger",
    )
    parser.add_argument(
        "--domain",
        type=int,
        default=42,
    )

    # The launch file originally spawns burger at (-13, 0, 0.01, yaw=0).
    parser.add_argument("--x", type=float, default=-13.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.01)
    parser.add_argument(
        "--yaw",
        type=float,
        default=0.0,
        help="World yaw in radians.",
    )

    parser.add_argument(
        "--settle",
        type=float,
        default=1.0,
        help="Seconds to wait after set_pose before sensor validation.",
    )
    parser.add_argument(
        "--sensor-timeout",
        type=float,
        default=10.0,
    )
    parser.add_argument(
        "--lidar-tol",
        type=float,
        default=0.20,
        help="Max allowed initial sector LiDAR difference for static scene.",
    )

    parser.add_argument(
        "--dynamic-scene",
        action="store_true",
        help=(
            "Skip initial LiDAR consistency check. "
            "Use only for debugging; it does NOT guarantee fair same-state "
            "candidate comparison if a moving obstacle changes phase."
        ),
    )

    args = parser.parse_args()

    print("=== TRAJECTORY RISK DATASET COLLECTOR V2 ===")
    print(f"episode       : {args.episode}")
    print(f"world         : {args.world}")
    print(f"model         : {args.model}")
    print(f"ROS_DOMAIN_ID : {os.environ['ROS_DOMAIN_ID']}")
    print(
        f"world pose    : "
        f"({args.x}, {args.y}, {args.z}), "
        f"yaw={args.yaw} rad"
    )
    print(f"candidates    : {', '.join(CANDIDATES.keys())}")

    try:
        preflight()
    except Exception as exc:
        print(f"\n[FAIL PRECHECK] {exc}")
        sys.exit(1)

    DATA_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    rclpy.init()
    node = EpisodeCollector()

    records = []

    try:
        for idx, candidate_id in enumerate(
            CANDIDATES.keys(),
            start=1,
        ):
            print(
                f"\n{'=' * 58}\n"
                f"[{idx}/{len(CANDIDATES)}] {candidate_id}\n"
                f"{'=' * 58}"
            )

            # 1) Stop robot before teleport.
            node.publish_stop(duration=0.5)
            stop_robot_cli()

            # 2) Reset ONLY robot world pose. Do not reset simulation clock.
            set_robot_pose(
                args.world,
                args.model,
                args.x,
                args.y,
                args.z,
                args.yaw,
            )

            # 3) Allow physics / sensors / bridge to settle.
            node.wait_for_fresh_sensors(
                timeout_sec=args.sensor_timeout
            )
            node.spin_for(args.settle)

            # 4) Ensure zero command and robot really stopped.
            node.publish_stop(duration=0.4)
            stationary = node.wait_stationary()

            lidar = node.get_lidar_state()

            print(
                "[READY] "
                f"odom_v={stationary['v']:.4f}, "
                f"odom_w={stationary['w']:.4f}, "
                f"LiDAR(F/L/R)="
                f"{lidar['front']:.3f}/"
                f"{lidar['left']:.3f}/"
                f"{lidar['right']:.3f}"
            )

            # 5) Execute candidate.
            features, outcome = node.collect_candidate(
                args.episode,
                candidate_id,
            )

            # 6) Candidate-level validation.
            validate_displacement(outcome)

            records.append(
                {
                    "candidate_id": candidate_id,
                    "features": features,
                    "outcome": outcome,
                }
            )

        # 7) Episode-level state validation.
        if not args.dynamic_scene:
            validate_initial_lidar(
                records,
                tolerance=args.lidar_tol,
            )
        else:
            print(
                "\n[WARNING] --dynamic-scene enabled: "
                "initial LiDAR consistency was NOT enforced. "
                "This mode is for debugging, not the preferred v0 dataset."
            )

        print_summary(records)

        # Warn about suspicious sensor-floor minima, but keep raw data for audit.
        raw_minima = np.array(
            [
                float(r["outcome"]["actual_min_range_raw"])
                for r in records
            ]
        )

        if (
            np.all(np.isfinite(raw_minima))
            and np.max(raw_minima) - np.min(raw_minima) < 1e-4
            and np.mean(raw_minima) < 0.15
        ):
            print(
                "\n[WARNING] Raw global LiDAR minimum is nearly identical "
                "and < 0.15 m for all candidates. "
                "Do NOT use actual_min_range_raw directly as the final "
                "risk label until the sensor geometry is verified."
            )

        # 8) Commit only after all candidates and validations pass.
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
            args.episode,
        )

        atomic_replace_episode(
            OUTCOME_FILE,
            outcome_rows,
            args.episode,
        )

        print(
            "\n[PASS] Episode validated and committed."
        )
        print(
            f"Features : {FEATURE_FILE}"
        )
        print(
            f"Outcomes : {OUTCOME_FILE}"
        )
        print(
            f"Samples  : {len(records)}"
        )

    except Exception as exc:
        print(
            "\n[FAIL] Episode was NOT committed."
        )
        print(
            f"Reason: {exc}"
        )
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
