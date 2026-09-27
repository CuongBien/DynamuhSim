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

Recommended multi-human collection:
    ros2 launch custom_corridor corridor_tb3.launch.py \
        width:=dataset obstacle:=none

For every candidate C0..C4:
    STOP robot
    -> RESET humans to the same scenario state
    -> reset robot world pose
    -> wait for fresh odom/scan and stationary state
    -> RESET+START humans exactly at rollout start
    -> execute the candidate for 3 s while recording robot + human world poses
    -> STOP humans at rollout end

This makes the five candidates a fair same-state comparison:
the robot and all human proxies start from the same world state.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import threading
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
from rosgraph_msgs.msg import Clock


# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
BASE_DIR = Path.home() / "nav_ws/experiments/trajectory_risk_v0"
DATA_DIR = BASE_DIR / "data"

FEATURE_FILE = DATA_DIR / "candidate_features_gate_c5.csv"
OUTCOME_FILE = DATA_DIR / "rollout_outcomes_gate_c5.csv"
TRACE_FILE = DATA_DIR / "rollout_traces_gate_c5.csv"

ODOM_TOPIC = "/odom"
SCAN_TOPIC = "/scan"
CMD_TOPIC = "/cmd_vel"
CLOCK_TOPIC = "/clock"

SCENARIO_TOPIC = "/multi_human/scenario"
HUMAN_COMMAND_TOPIC = "/multi_human/command"

HUMAN_NAMES = tuple(
    f"human_{i:02d}_proxy"
    for i in range(1, 5)
)

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
    clock_pub = topic_publisher_count(CLOCK_TOPIC)

    print(f"{ODOM_TOPIC} publishers: {odom_pub}")
    print(f"{SCAN_TOPIC} publishers: {scan_pub}")
    print(f"{CLOCK_TOPIC} publishers: {clock_pub}")

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

    if clock_pub == 0:
        raise RuntimeError(
            "/clock has no ROS publisher. Gate C V2.2 requires Gazebo "
            "simulation time so every rollout lasts exactly 3.0 sim seconds."
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
# Multi-human scenario helpers
# ---------------------------------------------------------------------
def gz_topic_names():
    out = run_cmd(
        ["gz", "topic", "-l"],
        timeout=6,
        check=False,
    )
    return {
        line.strip()
        for line in out.splitlines()
        if line.strip()
    }


def preflight_multi_human(
    scenario_topic=SCENARIO_TOPIC,
    command_topic=HUMAN_COMMAND_TOPIC,
):
    print("\n=== MULTI-HUMAN PREFLIGHT ===")
    topics = gz_topic_names()

    missing = [
        topic
        for topic in (scenario_topic, command_topic)
        if topic not in topics
    ]

    print(f"scenario topic : {scenario_topic}")
    print(f"command topic  : {command_topic}")

    if missing:
        raise RuntimeError(
            "MultiHumanScenarioSystem topic(s) missing:\n  "
            + "\n  ".join(missing)
            + "\nLaunch arena_dataset with the V4 plugin first."
        )

    print("[OK] MultiHumanScenarioSystem topics are available.")


def gz_publish_string(topic, data):
    # json.dumps() gives a correctly quoted protobuf string literal.
    request = f"data: {json.dumps(str(data))}"

    run_cmd(
        [
            "gz", "topic",
            "-t", topic,
            "-m", "gz.msgs.StringMsg",
            "-p", request,
        ],
        timeout=6,
        check=True,
    )


def human_command(
    command,
    command_topic=HUMAN_COMMAND_TOPIC,
):
    if command not in {
        "reset",
        "start",
        "stop",
        "reset_start",
        "hide_all",
    }:
        raise ValueError(f"Unsupported human command: {command}")

    gz_publish_string(
        command_topic,
        command,
    )


def load_human_scenario(
    scenario_path,
    episode_id,
    scenario_topic=SCENARIO_TOPIC,
):
    path = Path(scenario_path).expanduser()

    if not path.exists():
        raise FileNotFoundError(
            f"Scenario file not found: {path}"
        )

    with path.open("r") as f:
        scenario = json.load(f)

    # The dataset episode id is authoritative. This allows the same
    # scenario template to be reused without editing the JSON file.
    scenario["episode_id"] = int(episode_id)

    if "humans" not in scenario or not isinstance(
        scenario["humans"],
        list,
    ):
        raise ValueError(
            f"{path}: expected JSON field 'humans' as an array."
        )

    payload = json.dumps(
        scenario,
        separators=(",", ":"),
    )

    gz_publish_string(
        scenario_topic,
        payload,
    )

    print(
        f"[OK] Loaded human scenario: {path} "
        f"({len(scenario['humans'])} humans, "
        f"episode={episode_id})"
    )

    return scenario


def unpause_world(world):
    out = run_cmd(
        [
            "gz", "service",
            "-s", f"/world/{world}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", "pause: false",
        ],
        timeout=6,
        check=False,
    )

    if out:
        print(f"[WORLD] unpause reply: {out}")


def quaternion_to_yaw_xyzw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GazeboPoseMonitor:
    """
    One persistent `gz topic -e` process.

    It continuously parses /world/<world>/pose/info and keeps the latest
    WORLD pose for burger + human proxy models. We do NOT spawn one
    subprocess per sample.
    """

    _NAME_RE = re.compile(r'^name:\s*"([^"]+)"$')
    _VALUE_RE = re.compile(
        r'^(x|y|z|w):\s*'
        r'([-+0-9.eE]+)$'
    )

    def __init__(
        self,
        world,
        entity_names,
    ):
        self.world = str(world)
        self.topic = f"/world/{self.world}/pose/info"
        self.entity_names = set(entity_names)

        self._latest = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self._proc = None
        self._thread = None
        self.error = None

    def start(self, wait_sec=5.0):
        if self._proc is not None:
            return

        self._proc = subprocess.Popen(
            [
                "gz", "topic",
                "-e",
                "-t", self.topic,
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            env=os.environ.copy(),
        )

        self._thread = threading.Thread(
            target=self._reader_loop,
            name="gazebo_pose_monitor",
            daemon=True,
        )
        self._thread.start()

        deadline = time.monotonic() + wait_sec

        while time.monotonic() < deadline:
            if self.error is not None:
                raise RuntimeError(
                    f"Gazebo pose monitor failed: {self.error}"
                )

            snap = self.snapshot()

            # Human proxies are the critical state for Gate C.
            if all(name in snap for name in HUMAN_NAMES):
                print(
                    "[OK] Gazebo pose monitor sees: "
                    + ", ".join(
                        name
                        for name in self.entity_names
                        if name in snap
                    )
                )
                return

            time.sleep(0.05)

        snap = self.snapshot()
        missing = [
            name
            for name in HUMAN_NAMES
            if name not in snap
        ]
        raise TimeoutError(
            f"No fresh world pose for: {missing}. "
            f"Topic: {self.topic}"
        )

    def _reader_loop(self):
        current_name = None
        section = None
        values = None

        try:
            if self._proc.stdout is None:
                raise RuntimeError("pose monitor stdout is unavailable")

            for raw in self._proc.stdout:
                if self._stop.is_set():
                    break

                line = raw.strip()

                m = self._NAME_RE.match(line)
                if m:
                    current_name = m.group(1)

                    if current_name in self.entity_names:
                        values = {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0,
                            "qx": 0.0,
                            "qy": 0.0,
                            "qz": 0.0,
                            "qw": 1.0,
                        }
                    else:
                        values = None

                    section = None
                    continue

                if values is None:
                    continue

                if line.startswith("position"):
                    section = "position"
                    continue

                if line.startswith("orientation"):
                    section = "orientation"
                    continue

                m = self._VALUE_RE.match(line)
                if m and section is not None:
                    key = m.group(1)
                    val = float(m.group(2))

                    if section == "position":
                        values[key] = val
                    elif section == "orientation":
                        values["q" + key] = val

                        # In gz Pose text, w is the last orientation scalar.
                        # Commit the pose here.
                        if key == "w":
                            pose = {
                                "x": float(values["x"]),
                                "y": float(values["y"]),
                                "z": float(values["z"]),
                                "yaw": float(
                                    quaternion_to_yaw_xyzw(
                                        values["qx"],
                                        values["qy"],
                                        values["qz"],
                                        values["qw"],
                                    )
                                ),
                                "stamp_monotonic": time.monotonic(),
                            }

                            with self._lock:
                                self._latest[current_name] = pose

                            current_name = None
                            section = None
                            values = None

        except Exception as exc:
            self.error = exc

    def snapshot(self):
        with self._lock:
            return {
                name: dict(pose)
                for name, pose in self._latest.items()
            }

    def stop(self):
        self._stop.set()

        if self._proc is not None:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=2.0)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass

        if self._thread is not None:
            self._thread.join(timeout=2.0)

        self._proc = None
        self._thread = None


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

        self.create_subscription(
            Clock,
            CLOCK_TOPIC,
            self.clock_callback,
            qos_profile_sensor_data,
        )

        self.odom = None
        self.scan = None
        self.sim_time_sec = None
        self.clock_count = 0

        self.last_odom_monotonic = None
        self.last_scan_monotonic = None

        # Diagnostics / freshness tracking.
        self.odom_count = 0
        self.scan_count = 0

        # Gate C: synchronized multi-human rollout state.
        self.pose_monitor = None
        self.robot_entity_name = None
        self.human_entity_names = HUMAN_NAMES

        self.rollout_initial_world_snapshot = None
        self.last_rollout_trace = []

        self.before_rollout_callback = None
        self.after_rollout_callback = None
        self._after_rollout_invoked = False

        self.rollout_stats_enabled = False

        self.reset_rollout_stats()

    def reset_rollout_stats(self):
        self.rollout_global_min = float("inf")
        self.rollout_front_min = float("inf")
        self.rollout_left_min = float("inf")
        self.rollout_right_min = float("inf")
        self.rollout_scan_samples = 0

    def odom_callback(self, msg):
        self.odom = msg
        self.last_odom_monotonic = time.monotonic()
        self.odom_count += 1

    def scan_callback(self, msg):
        self.scan = msg
        self.last_scan_monotonic = time.monotonic()
        self.scan_count += 1

        if not self.rollout_stats_enabled:
            return

        self.rollout_scan_samples += 1

        vals = finite_scan_values(msg)
        if vals:
            self.rollout_global_min = min(
                self.rollout_global_min,
                min(vals),
            )

        front = self.sector_min_from_msg(msg, -30, 30)
        left = self.sector_min_from_msg(msg, 30, 100)
        right = self.sector_min_from_msg(msg, -100, -30)

        self.rollout_front_min = min(
            self.rollout_front_min,
            front,
        )
        self.rollout_left_min = min(
            self.rollout_left_min,
            left,
        )
        self.rollout_right_min = min(
            self.rollout_right_min,
            right,
        )

    def clock_callback(self, msg):
        self.sim_time_sec = (
            float(msg.clock.sec)
            + float(msg.clock.nanosec) * 1e-9
        )
        self.clock_count += 1

    def wait_for_sim_clock(self, timeout_sec=10.0):
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            if self.sim_time_sec is not None:
                return float(self.sim_time_sec)
        raise TimeoutError(
            f"No {CLOCK_TOPIC} message within {timeout_sec:.1f}s."
        )

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

    def wait_for_fresh_sensors(
        self,
        timeout_sec=10.0,
        after_monotonic=None,
        label="sensor wait",
    ):
        """
        Wait until both /odom and /scan have arrived.

        If after_monotonic is provided, require callbacks newer than that
        wall-clock monotonic timestamp. This is safer than clearing self.odom
        and self.scan after every Gazebo set_pose.
        """
        deadline = time.monotonic() + float(timeout_sec)
        last_report = 0.0

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)

            odom_ok = (
                self.odom is not None
                and self.last_odom_monotonic is not None
            )
            scan_ok = (
                self.scan is not None
                and self.last_scan_monotonic is not None
            )

            if after_monotonic is not None:
                odom_ok = (
                    odom_ok
                    and self.last_odom_monotonic > after_monotonic
                )
                scan_ok = (
                    scan_ok
                    and self.last_scan_monotonic > after_monotonic
                )

            if odom_ok and scan_ok:
                return

            now = time.monotonic()
            if now - last_report >= 2.0:
                odom_age = (
                    float("inf")
                    if self.last_odom_monotonic is None
                    else now - self.last_odom_monotonic
                )
                scan_age = (
                    float("inf")
                    if self.last_scan_monotonic is None
                    else now - self.last_scan_monotonic
                )

                print(
                    f"[WAIT] {label}: "
                    f"odom={'OK' if odom_ok else 'WAIT'} "
                    f"(count={self.odom_count}, age={odom_age:.2f}s), "
                    f"scan={'OK' if scan_ok else 'WAIT'} "
                    f"(count={self.scan_count}, age={scan_age:.2f}s)"
                )
                last_report = now

        missing = []
        if (
            self.odom is None
            or self.last_odom_monotonic is None
            or (
                after_monotonic is not None
                and self.last_odom_monotonic <= after_monotonic
            )
        ):
            missing.append(ODOM_TOPIC)

        if (
            self.scan is None
            or self.last_scan_monotonic is None
            or (
                after_monotonic is not None
                and self.last_scan_monotonic <= after_monotonic
            )
        ):
            missing.append(SCAN_TOPIC)

        raise TimeoutError(
            f"No fresh sensor data within {timeout_sec:.1f}s. "
            f"Missing/stale: {', '.join(missing) if missing else 'unknown'}. "
            f"callback counts: odom={self.odom_count}, scan={self.scan_count}."
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

    def attach_pose_monitor(
        self,
        monitor,
        robot_entity_name,
        human_entity_names=HUMAN_NAMES,
    ):
        self.pose_monitor = monitor
        self.robot_entity_name = str(robot_entity_name)
        self.human_entity_names = tuple(human_entity_names)

    def set_rollout_callbacks(
        self,
        before=None,
        after=None,
    ):
        self.before_rollout_callback = before
        self.after_rollout_callback = after

    def capture_initial_world_snapshot(self):
        if self.pose_monitor is None:
            self.rollout_initial_world_snapshot = None
            return None

        self.rollout_initial_world_snapshot = (
            self.pose_monitor.snapshot()
        )
        return self.rollout_initial_world_snapshot

    def _invoke_after_rollout_once(self):
        if self._after_rollout_invoked:
            return

        self._after_rollout_invoked = True

        if self.after_rollout_callback is not None:
            self.after_rollout_callback()

    def _make_trace_sample(self, t_rel):
        if self.pose_monitor is None:
            return None

        snap = self.pose_monitor.snapshot()

        odom = (
            self.get_odom_state()
            if self.odom is not None
            else None
        )
        lidar = (
            self.get_lidar_state()
            if self.scan is not None
            else None
        )

        row = {
            "episode_id": int(
                getattr(self, "_current_episode_id", -1)
            ),
            "candidate_id": str(
                getattr(self, "_current_candidate_id", "")
            ),
            "sample_idx": int(len(self.last_rollout_trace)),
            "t_rel_s": float(t_rel),
        }

        robot = snap.get(self.robot_entity_name, {})

        row.update(
            {
                "robot_world_x": robot.get("x", float("nan")),
                "robot_world_y": robot.get("y", float("nan")),
                "robot_world_yaw": robot.get("yaw", float("nan")),
                "robot_world_z": robot.get("z", float("nan")),

                "odom_x": (
                    odom["x"] if odom is not None else float("nan")
                ),
                "odom_y": (
                    odom["y"] if odom is not None else float("nan")
                ),
                "odom_yaw": (
                    odom["yaw"] if odom is not None else float("nan")
                ),
                "odom_v": (
                    odom["v"] if odom is not None else float("nan")
                ),
                "odom_w": (
                    odom["w"] if odom is not None else float("nan")
                ),

                "lidar_front": (
                    lidar["front"]
                    if lidar is not None else float("nan")
                ),
                "lidar_left": (
                    lidar["left"]
                    if lidar is not None else float("nan")
                ),
                "lidar_right": (
                    lidar["right"]
                    if lidar is not None else float("nan")
                ),
            }
        )

        for idx, name in enumerate(
            self.human_entity_names,
            start=1,
        ):
            pose = snap.get(name, {})
            prefix = f"human_{idx:02d}"

            row[f"{prefix}_x"] = pose.get(
                "x",
                float("nan"),
            )
            row[f"{prefix}_y"] = pose.get(
                "y",
                float("nan"),
            )
            row[f"{prefix}_yaw"] = pose.get(
                "yaw",
                float("nan"),
            )
            row[f"{prefix}_z"] = pose.get(
                "z",
                float("nan"),
            )

        return row

    def _append_trace_sample(self, t_rel):
        row = self._make_trace_sample(t_rel)

        if row is not None:
            self.last_rollout_trace.append(row)

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
        # V2.2: use Gazebo simulation time, not wall-clock time.
        self.reset_rollout_stats()
        self.last_rollout_trace = []
        self.rollout_stats_enabled = True

        cmd = TwistStamped()
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)

        sim_start = self.wait_for_sim_clock(timeout_sec=10.0)
        next_trace = 0.0
        wall_guard_start = time.monotonic()

        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.01)

                if self.sim_time_sec is None:
                    continue

                elapsed = float(self.sim_time_sec) - sim_start

                if elapsed >= float(duration):
                    break

                if time.monotonic() - wall_guard_start > 30.0:
                    raise TimeoutError(
                        "Gazebo /clock did not advance enough to finish rollout."
                    )

                cmd.header.stamp = self.get_clock().now().to_msg()
                self.cmd_pub.publish(cmd)

                if elapsed + 1e-9 >= next_trace:
                    self._append_trace_sample(max(0.0, elapsed))
                    next_trace += DT

                time.sleep(0.005)

            self._append_trace_sample(float(duration))

        finally:
            self.rollout_stats_enabled = False
            self._invoke_after_rollout_once()
            self.publish_stop(duration=0.6)

    def reconstruct_world_aligned_odom_trace(
        self,
        start_odom,
        start_world_x,
        start_world_y,
        start_world_yaw,
        end_odom=None,
    ):
        """
        Reconstruct a dense robot trajectory in WORLD coordinates.

        Gazebo set_pose changes the robot WORLD pose but TurtleBot odometry
        remains integrated across candidates. Relative odometry increments
        inside one candidate are still valid. We therefore anchor the relative
        odom trajectory at the exact Gazebo WORLD start pose:

            p_world(t) = p_world(0)
                       + R(yaw_world0 - yaw_odom0)
                         [p_odom(t) - p_odom(0)]

        Translation / rotation do not change path length, so this provides
        a stable actual path-length estimate without requiring odom reset.
        """
        if not all(
            np.isfinite(v)
            for v in (
                start_odom["x"],
                start_odom["y"],
                start_odom["yaw"],
                start_world_x,
                start_world_y,
                start_world_yaw,
            )
        ):
            return [], float("nan")

        yaw_offset = angle_normalize(
            float(start_world_yaw) - float(start_odom["yaw"])
        )
        c = math.cos(yaw_offset)
        ss = math.sin(yaw_offset)

        points = [
            (
                0.0,
                float(start_world_x),
                float(start_world_y),
            )
        ]

        for row in self.last_rollout_trace:
            ox = float(row.get("odom_x", float("nan")))
            oy = float(row.get("odom_y", float("nan")))

            if not (np.isfinite(ox) and np.isfinite(oy)):
                continue

            dx = ox - float(start_odom["x"])
            dy = oy - float(start_odom["y"])

            wx = float(start_world_x) + c * dx - ss * dy
            wy = float(start_world_y) + ss * dx + c * dy

            row["robot_world_recon_x"] = wx
            row["robot_world_recon_y"] = wy

            points.append(
                (
                    float(row.get("t_rel_s", float("nan"))),
                    wx,
                    wy,
                )
            )

        if end_odom is not None:
            ox = float(end_odom.get("x", float("nan")))
            oy = float(end_odom.get("y", float("nan")))

            if np.isfinite(ox) and np.isfinite(oy):
                dx = ox - float(start_odom["x"])
                dy = oy - float(start_odom["y"])

                wx = float(start_world_x) + c * dx - ss * dy
                wy = float(start_world_y) + ss * dx + c * dy

                points.append(
                    (
                        float(DURATION),
                        wx,
                        wy,
                    )
                )

        # Deduplicate consecutive equal positions.
        clean = []
        for p in points:
            if not clean:
                clean.append(p)
                continue

            if math.hypot(
                p[1] - clean[-1][1],
                p[2] - clean[-1][2],
            ) > 1e-9:
                clean.append(p)

        path_length = 0.0
        for a, b in zip(clean[:-1], clean[1:]):
            path_length += math.hypot(
                b[1] - a[1],
                b[2] - a[2],
            )

        return clean, float(path_length)

    def raw_gazebo_trace_path_length(self):
        """
        Audit-only path from asynchronous /world/.../pose/info snapshots.
        It may undersample when Gazebo runs faster than wall time, therefore
        it is NOT the Gate C.5 primary path metric.
        """
        length = 0.0
        prev = None

        for row in self.last_rollout_trace:
            x = float(row.get("robot_world_x", float("nan")))
            y = float(row.get("robot_world_y", float("nan")))

            if not (np.isfinite(x) and np.isfinite(y)):
                continue

            if prev is not None:
                length += math.hypot(
                    x - prev[0],
                    y - prev[1],
                )

            prev = (x, y)

        return float(length)

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

        # Ground-truth WORLD state immediately before reset_start.
        initial_world = self.rollout_initial_world_snapshot or {}

        robot_world = initial_world.get(
            self.robot_entity_name,
            {},
        )

        features["robot_world_x0"] = robot_world.get(
            "x",
            float("nan"),
        )
        features["robot_world_y0"] = robot_world.get(
            "y",
            float("nan"),
        )
        features["robot_world_yaw0"] = robot_world.get(
            "yaw",
            float("nan"),
        )
        features["robot_world_z0"] = robot_world.get(
            "z",
            float("nan"),
        )

        robot_x0 = features["robot_world_x0"]
        robot_y0 = features["robot_world_y0"]
        robot_yaw0 = features["robot_world_yaw0"]

        for idx, name in enumerate(
            self.human_entity_names,
            start=1,
        ):
            pose = initial_world.get(name, {})
            prefix = f"human_{idx:02d}"

            hx = pose.get("x", float("nan"))
            hy = pose.get("y", float("nan"))
            hyaw = pose.get("yaw", float("nan"))
            hz = pose.get("z", float("nan"))

            features[f"{prefix}_world_x0"] = hx
            features[f"{prefix}_world_y0"] = hy
            features[f"{prefix}_world_yaw0"] = hyaw
            features[f"{prefix}_world_z0"] = hz

            if all(
                np.isfinite(x)
                for x in (
                    robot_x0,
                    robot_y0,
                    robot_yaw0,
                    hx,
                    hy,
                )
            ):
                dx = hx - robot_x0
                dy = hy - robot_y0

                c = math.cos(-robot_yaw0)
                ss = math.sin(-robot_yaw0)

                features[f"{prefix}_rel_x0"] = (
                    c * dx - ss * dy
                )
                features[f"{prefix}_rel_y0"] = (
                    ss * dx + c * dy
                )
            else:
                features[f"{prefix}_rel_x0"] = float("nan")
                features[f"{prefix}_rel_y0"] = float("nan")

        print("\nINPUT FEATURES")
        print(features)

        self._current_episode_id = int(episode_id)
        self._current_candidate_id = str(candidate_id)
        self._after_rollout_invoked = False

        if self.before_rollout_callback is not None:
            self.before_rollout_callback()

        try:
            self.execute_candidate(
                v_cmd,
                w_cmd,
                DURATION,
            )
        finally:
            # execute_candidate normally invokes this at exactly T=duration;
            # this is also a safety net for exceptions.
            self._invoke_after_rollout_once()

        self.spin_for(0.25)

        end = self.get_odom_state()

        # Gazebo set_pose does NOT reset integrated odometry.
        # Therefore odom is audit-only across C0..C4. WORLD pose is truth.
        final_world = (
            self.pose_monitor.snapshot()
            if self.pose_monitor is not None
            else {}
        )
        final_robot = final_world.get(self.robot_entity_name, {})

        end_world_x = final_robot.get("x", float("nan"))
        end_world_y = final_robot.get("y", float("nan"))
        end_world_yaw = final_robot.get("yaw", float("nan"))
        end_world_z = final_robot.get("z", float("nan"))

        if all(np.isfinite(v) for v in (
            robot_x0, robot_y0, end_world_x, end_world_y
        )):
            displacement = math.hypot(
                end_world_x - robot_x0,
                end_world_y - robot_y0,
            )
        else:
            displacement = float("nan")

        _, world_path_length = (
            self.reconstruct_world_aligned_odom_trace(
                start_odom=start,
                start_world_x=robot_x0,
                start_world_y=robot_y0,
                start_world_yaw=robot_yaw0,
                end_odom=end,
            )
        )

        gazebo_trace_path_length_raw = (
            self.raw_gazebo_trace_path_length()
        )

        # Gate C.5 definition:
        # progress_ratio = executed path length / commanded path length.
        # It is an execution-completion ratio, NOT goal progress.
        progress_ratio = (
            world_path_length / traj_length
            if traj_length > 1e-9
            else float("nan")
        )

        stuck = int(
            traj_length > 0.30
            and (
                not np.isfinite(world_path_length)
                or world_path_length < 0.10
            )
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

            "start_world_x": float(robot_x0),
            "start_world_y": float(robot_y0),
            "start_world_yaw": float(robot_yaw0),
            "end_world_x": float(end_world_x),
            "end_world_y": float(end_world_y),
            "end_world_yaw": float(end_world_yaw),
            "end_world_z": float(end_world_z),

            "actual_displacement": float(displacement),
            "actual_world_path_length": float(world_path_length),
            "gazebo_trace_path_length_raw": float(
                gazebo_trace_path_length_raw
            ),
            "path_length_source": "relative_odom_aligned_to_gazebo_world_start",
            "expected_distance": float(traj_length),
            "progress_ratio": float(progress_ratio),
            "progress_ratio_definition": "actual_world_path_length/expected_distance",
            "rollout_scan_samples": int(self.rollout_scan_samples),

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

        # Ground-truth robot-human center-distance audit from world poses.
        min_d = float("inf")
        min_t = float("nan")
        closest_human = ""

        for row in self.last_rollout_trace:
            rx = float(row["robot_world_x"])
            ry = float(row["robot_world_y"])

            if not (
                np.isfinite(rx)
                and np.isfinite(ry)
            ):
                continue

            for idx in range(1, len(self.human_entity_names) + 1):
                hx = float(row[f"human_{idx:02d}_x"])
                hy = float(row[f"human_{idx:02d}_y"])

                if not (
                    np.isfinite(hx)
                    and np.isfinite(hy)
                ):
                    continue

                d = math.hypot(
                    hx - rx,
                    hy - ry,
                )

                if d < min_d:
                    min_d = d
                    min_t = float(row["t_rel_s"])
                    closest_human = f"human_{idx:02d}"

        outcome["trace_samples"] = int(
            len(self.last_rollout_trace)
        )
        outcome["min_human_center_distance"] = (
            float(min_d)
            if np.isfinite(min_d)
            else float("nan")
        )
        outcome["time_to_min_human_distance"] = float(min_t)
        outcome["closest_human_id"] = closest_human

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
    actual = float(outcome["actual_world_path_length"])
    expected = float(outcome["expected_distance"])

    upper = expected * max_factor + margin

    if not np.isfinite(actual):
        raise RuntimeError("Robot WORLD path length is not finite.")

    if actual > upper:
        raise RuntimeError(
            "Implausible WORLD path length: "
            f"{actual:.3f} m > {upper:.3f} m "
            f"(commanded arc length={expected:.3f} m). "
            "Check /clock synchronization or competing controllers."
        )



def validate_gate_c5_candidate(
    outcome,
    path_ratio_min=0.85,
    path_ratio_max=1.10,
    ratio_tolerance=1e-6,
):
    candidate = str(outcome["candidate_id"])

    # A. LiDAR rollout must have actually produced samples.
    scan_samples = int(outcome.get("rollout_scan_samples", 0))
    if scan_samples <= 0:
        raise RuntimeError(
            f"{candidate}: no LaserScan callback was recorded during rollout."
        )

    lidar_keys = (
        "rollout_front_min",
        "rollout_left_min",
        "rollout_right_min",
        "actual_min_range",
    )

    for key in lidar_keys:
        value = float(outcome[key])
        if not np.isfinite(value):
            raise RuntimeError(
                f"{candidate}: {key} is not finite ({value})."
            )
        if value < 0.0:
            raise RuntimeError(
                f"{candidate}: {key} is negative ({value:.4f} m)."
            )

    # B. Executed path must be close to commanded path.
    actual = float(outcome["actual_world_path_length"])
    expected = float(outcome["expected_distance"])

    if not (
        np.isfinite(actual)
        and np.isfinite(expected)
        and expected > 1e-9
    ):
        raise RuntimeError(
            f"{candidate}: invalid path lengths: "
            f"actual={actual}, expected={expected}."
        )

    path_ratio = actual / expected

    if not (
        float(path_ratio_min)
        <= path_ratio
        <= float(path_ratio_max)
    ):
        raise RuntimeError(
            f"{candidate}: path QA failed: "
            f"actual={actual:.3f} m, expected={expected:.3f} m, "
            f"ratio={path_ratio:.3f}, allowed="
            f"[{path_ratio_min:.3f}, {path_ratio_max:.3f}]."
        )

    # C. progress_ratio has ONE explicit definition.
    saved_ratio = float(outcome["progress_ratio"])
    recomputed_ratio = actual / expected

    if (
        not np.isfinite(saved_ratio)
        or abs(saved_ratio - recomputed_ratio)
        > float(ratio_tolerance)
    ):
        raise RuntimeError(
            f"{candidate}: progress_ratio inconsistent. "
            f"saved={saved_ratio:.9f}, "
            f"actual/expected={recomputed_ratio:.9f}."
        )

    print(
        f"[C5 OK] {candidate}: "
        f"path={actual:.3f}/{expected:.3f} m "
        f"(ratio={saved_ratio:.3f}), "
        f"LiDAR F/L/R="
        f"{float(outcome['rollout_front_min']):.3f}/"
        f"{float(outcome['rollout_left_min']):.3f}/"
        f"{float(outcome['rollout_right_min']):.3f}, "
        f"scan_samples={scan_samples}"
    )

    return {
        "candidate_id": candidate,
        "actual_path": actual,
        "expected_path": expected,
        "path_ratio": saved_ratio,
        "scan_samples": scan_samples,
    }

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



def validate_initial_human_state(
    records,
    position_tolerance=0.01,
    z_tolerance=0.01,
    yaw_tolerance=0.02,
):
    """
    Gate C.5 fairness:
    C0..C4 must start from the same robot + 4-human WORLD state.

    position_tolerance applies independently to x/y,
    z_tolerance applies to z,
    yaw_tolerance is circular angular error.
    """
    if not records:
        raise RuntimeError("No records for same-state validation.")

    ref = records[0]["features"]

    max_xy_err = 0.0
    max_z_err = 0.0
    max_yaw_err = 0.0
    worst = ""

    entities = [("robot", "robot")]
    for idx in range(1, len(HUMAN_NAMES) + 1):
        entities.append(
            (
                f"human_{idx:02d}",
                f"human_{idx:02d}",
            )
        )

    for rec in records[1:]:
        cand = rec["candidate_id"]
        f = rec["features"]

        for label, prefix in entities:
            for axis in ("x", "y"):
                key = f"{prefix}_world_{axis}0"
                a = float(ref[key])
                b = float(f[key])

                if not (np.isfinite(a) and np.isfinite(b)):
                    raise RuntimeError(
                        f"{cand}: non-finite initial state {key}."
                    )

                err = abs(b - a)
                if err > max_xy_err:
                    max_xy_err = err
                    worst = f"{cand}:{key}"

                if err > position_tolerance:
                    raise RuntimeError(
                        f"{cand}: same-state FAIL at {key}: "
                        f"error={err:.4f} m > "
                        f"{position_tolerance:.4f} m."
                    )

            zkey = f"{prefix}_world_z0"
            za = float(ref[zkey])
            zb = float(f[zkey])

            if not (np.isfinite(za) and np.isfinite(zb)):
                raise RuntimeError(
                    f"{cand}: non-finite initial state {zkey}."
                )

            zerr = abs(zb - za)
            max_z_err = max(max_z_err, zerr)

            if zerr > z_tolerance:
                raise RuntimeError(
                    f"{cand}: same-state FAIL at {zkey}: "
                    f"error={zerr:.4f} m > {z_tolerance:.4f} m."
                )

            ykey = f"{prefix}_world_yaw0"
            ya = float(ref[ykey])
            yb = float(f[ykey])

            if not (np.isfinite(ya) and np.isfinite(yb)):
                raise RuntimeError(
                    f"{cand}: non-finite initial state {ykey}."
                )

            yerr = abs(angle_normalize(yb - ya))
            max_yaw_err = max(max_yaw_err, yerr)

            if yerr > yaw_tolerance:
                raise RuntimeError(
                    f"{cand}: same-state FAIL at {ykey}: "
                    f"error={yerr:.4f} rad > "
                    f"{yaw_tolerance:.4f} rad."
                )

    print(
        "[C5 OK] Same initial WORLD state: "
        f"max_xy_err={max_xy_err:.4f} m, "
        f"max_z_err={max_z_err:.4f} m, "
        f"max_yaw_err={max_yaw_err:.4f} rad"
        + (f", worst={worst}" if worst else "")
    )

    return {
        "max_xy_error_m": float(max_xy_err),
        "max_z_error_m": float(max_z_err),
        "max_yaw_error_rad": float(max_yaw_err),
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
    print("\n=== GATE C.5 EPISODE SUMMARY ===")
    print(
        f"{'Cand':<6}"
        f"{'path':>9}"
        f"{'chord':>9}"
        f"{'expect':>9}"
        f"{'ratio':>9}"
        f"{'front':>9}"
        f"{'left':>9}"
        f"{'right':>9}"
        f"{'human_d':>10}"
        f"{'scans':>8}"
    )

    for rec in records:
        o = rec["outcome"]

        print(
            f"{rec['candidate_id']:<6}"
            f"{float(o['actual_world_path_length']):>9.3f}"
            f"{float(o['actual_displacement']):>9.3f}"
            f"{float(o['expected_distance']):>9.3f}"
            f"{float(o['progress_ratio']):>9.3f}"
            f"{float(o['rollout_front_min']):>9.3f}"
            f"{float(o['rollout_left_min']):>9.3f}"
            f"{float(o['rollout_right_min']):>9.3f}"
            f"{float(o['min_human_center_distance']):>10.3f}"
            f"{int(o['rollout_scan_samples']):>8d}"
        )

    print(
        "\nprogress_ratio := actual_world_path_length / expected_distance"
    )
    print(
        "path := relative odometry trajectory aligned to exact Gazebo WORLD start"
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
        default="arena_dataset",
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
        "--scenario",
        default=str(
            BASE_DIR / "scenarios" / "scenario_001.json"
        ),
        help="MultiHumanScenarioSystem JSON scenario.",
    )
    parser.add_argument(
        "--scenario-topic",
        default=SCENARIO_TOPIC,
    )
    parser.add_argument(
        "--human-command-topic",
        default=HUMAN_COMMAND_TOPIC,
    )
    parser.add_argument(
        "--human-reset-settle",
        type=float,
        default=0.25,
        help="Wall seconds to wait after human reset before snapshot.",
    )
    parser.add_argument(
        "--state-pos-tol",
        type=float,
        default=0.01,
        help="Gate C.5 same-state x/y tolerance across C0..C4 [m].",
    )
    parser.add_argument(
        "--state-z-tol",
        type=float,
        default=0.01,
        help="Gate C.5 same-state z tolerance across C0..C4 [m].",
    )
    parser.add_argument(
        "--state-yaw-tol",
        type=float,
        default=0.02,
        help="Gate C.5 same-state yaw tolerance across C0..C4 [rad].",
    )
    parser.add_argument(
        "--path-ratio-min",
        type=float,
        default=0.85,
        help="Minimum actual/commanded path ratio for Gate C.5.",
    )
    parser.add_argument(
        "--path-ratio-max",
        type=float,
        default=1.10,
        help="Maximum actual/commanded path ratio for Gate C.5.",
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
        default=15.0,
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

    print("=== TRAJECTORY RISK DATASET COLLECTOR V2.3 + GATE C.5 QA ===")
    print(f"episode       : {args.episode}")
    print(f"world         : {args.world}")
    print(f"model         : {args.model}")
    print(f"scenario      : {Path(args.scenario).expanduser()}")
    print(f"ROS_DOMAIN_ID : {os.environ['ROS_DOMAIN_ID']}")
    print(
        f"world pose    : "
        f"({args.x}, {args.y}, {args.z}), "
        f"yaw={args.yaw} rad"
    )
    print(f"candidates    : {', '.join(CANDIDATES.keys())}")

    try:
        preflight()
        preflight_multi_human(
            scenario_topic=args.scenario_topic,
            command_topic=args.human_command_topic,
        )
        unpause_world(args.world)
    except Exception as exc:
        print(f"\n[FAIL PRECHECK] {exc}")
        sys.exit(1)

    DATA_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    rclpy.init()
    node = EpisodeCollector()

    # DDS / bridge warm-up before the first set_pose.
    # This verifies that the Python node itself can actually receive both
    # ROS topics, not merely that publishers exist in the graph.
    try:
        print("\n=== SENSOR WARM-UP ===")
        node.wait_for_fresh_sensors(
            timeout_sec=max(args.sensor_timeout, 15.0),
            label="startup",
        )
        print(
            "[OK] Python collector receives /odom + /scan "
            f"(odom_count={node.odom_count}, scan_count={node.scan_count})."
        )
    except Exception as exc:
        print(f"\n[FAIL SENSOR WARM-UP] {exc}")
        print("\nRun these two commands in another terminal:")
        print("  ros2 topic echo /odom --once")
        print("  ros2 topic echo /scan --once")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)

    pose_monitor = GazeboPoseMonitor(
        args.world,
        (args.model,) + HUMAN_NAMES,
    )

    records = []
    trace_rows = []

    try:
        pose_monitor.start(wait_sec=8.0)

        node.attach_pose_monitor(
            pose_monitor,
            robot_entity_name=args.model,
            human_entity_names=HUMAN_NAMES,
        )

        scenario = load_human_scenario(
            args.scenario,
            episode_id=args.episode,
            scenario_topic=args.scenario_topic,
        )

        # Humans are reset while input state is measured. They start only
        # immediately before the 3 s candidate rollout.
        node.set_rollout_callbacks(
            before=lambda: human_command(
                "reset_start",
                args.human_command_topic,
            ),
            after=lambda: human_command(
                "stop",
                args.human_command_topic,
            ),
        )

        human_command(
            "reset",
            args.human_command_topic,
        )
        time.sleep(args.human_reset_settle)

        for idx, candidate_id in enumerate(
            CANDIDATES.keys(),
            start=1,
        ):
            print(
                f"\n{'=' * 58}\n"
                f"[{idx}/{len(CANDIDATES)}] {candidate_id}\n"
                f"{'=' * 58}"
            )

            # 1) Stop robot and freeze/reset all humans.
            node.publish_stop(duration=0.5)
            stop_robot_cli()

            human_command(
                "reset",
                args.human_command_topic,
            )
            time.sleep(args.human_reset_settle)

            # 2) Reset robot world pose. Do not reset simulation clock.
            # Record freshness threshold BEFORE set_pose, because sensor
            # messages may arrive while the Gazebo service call is executing.
            sensor_reset_mark = time.monotonic()

            set_robot_pose(
                args.world,
                args.model,
                args.x,
                args.y,
                args.z,
                args.yaw,
            )

            # 3) Require at least one NEW odom and scan callback after reset.
            node.wait_for_fresh_sensors(
                timeout_sec=args.sensor_timeout,
                after_monotonic=sensor_reset_mark,
                label=f"{candidate_id} post-reset",
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

            # 5) Capture exact same-state WORLD snapshot while humans
            # are still RESET / frozen. collect_candidate() will issue
            # RESET+START immediately before the 3 s rollout.
            initial_world = node.capture_initial_world_snapshot()

            print("[HUMANS @ T0]")
            for human_name in HUMAN_NAMES:
                p = (initial_world or {}).get(human_name, {})
                print(
                    f"  {human_name}: "
                    f"x={p.get('x', float('nan')):.3f}, "
                    f"y={p.get('y', float('nan')):.3f}, "
                    f"yaw={p.get('yaw', float('nan')):.3f}"
                )

            # 6) Execute candidate + synchronized human rollout.
            features, outcome = node.collect_candidate(
                args.episode,
                candidate_id,
            )

            # 6) Gate C.5 candidate-level data integrity validation.
            validate_gate_c5_candidate(
                outcome,
                path_ratio_min=args.path_ratio_min,
                path_ratio_max=args.path_ratio_max,
            )

            records.append(
                {
                    "candidate_id": candidate_id,
                    "features": features,
                    "outcome": outcome,
                }
            )

            trace_rows.extend(
                node.last_rollout_trace
            )

        # 7) Episode-level same-state validation.
        same_state_qa = validate_initial_human_state(
            records,
            position_tolerance=args.state_pos_tol,
            z_tolerance=args.state_z_tol,
            yaw_tolerance=args.state_yaw_tol,
        )

        # Initial LiDAR should also be reproducible because humans are RESET
        # to the same positions before every candidate.
        if not args.dynamic_scene:
            validate_initial_lidar(
                records,
                tolerance=args.lidar_tol,
            )
        else:
            print(
                "\n[WARNING] --dynamic-scene enabled: "
                "initial LiDAR consistency was NOT enforced. "
                "Same-state robot/human WORLD validation is still enforced."
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

        print(
            "\n[GATE C.5 PASS] "
            "LiDAR finite + path execution valid + progress_ratio consistent "
            "+ C0..C4 same initial robot/human state."
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

        if not trace_rows:
            raise RuntimeError(
                "No world-pose rollout trace samples were recorded."
            )

        atomic_replace_episode(
            TRACE_FILE,
            trace_rows,
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
            f"Traces   : {TRACE_FILE}"
        )
        print(
            f"Samples  : {len(records)} candidates, "
            f"{len(trace_rows)} trace rows"
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
            human_command(
                "stop",
                args.human_command_topic,
            )
            time.sleep(0.10)
            human_command(
                "reset",
                args.human_command_topic,
            )
        except Exception:
            pass

        try:
            node.publish_stop(duration=0.5)
        except Exception:
            pass

        try:
            pose_monitor.stop()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
