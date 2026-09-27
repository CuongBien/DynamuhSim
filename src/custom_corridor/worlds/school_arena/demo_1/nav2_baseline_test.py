#!/usr/bin/env python3
"""Live Gazebo/Nav2 baseline test with topic, TF and multi-goal checks."""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath

from collision_safe_pose_applier import OccupancyGuard
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


HERE = Path(__file__).resolve().parent

GOALS = [
    ("A_to_Room05", 7.05, -10.60, -math.pi / 2),
    ("Room05_to_CorridorC", 10.0, -4.0, math.pi),
    ("CorridorC_to_JunctionCD", 7.0, -4.0, math.pi),
    ("JunctionCD_to_JunctionDE", 7.0, 0.0, math.pi / 2),
    ("JunctionDE_to_JunctionEF", 15.0, 0.0, 0.0),
    ("JunctionEF_to_Room12", 17.60, 7.30, 0.0),
]


class Baseline(Node):
    def __init__(self, timeout):
        super().__init__(
            "school_demo_nav2_baseline",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.timeout = timeout
        self.seen = set()
        self.guard = OccupancyGuard(str(HERE / "map.yaml"), 0.16)
        self.unsafe_plan_pose = None
        self.create_subscription(Odometry, "/odom", lambda _: self.seen.add("odom"), 10)
        self.create_subscription(LaserScan, "/scan", lambda _: self.seen.add("scan"), 10)
        self.create_subscription(NavPath, "/plan", self._plan_cb, 10)
        self.create_subscription(
            OccupancyGrid, "/map", lambda _: self.seen.add("map"),
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _plan_cb(self, msg):
        for pose in msg.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            if not self.guard.pose_is_free(x, y):
                self.unsafe_plan_pose = (x, y)
                return

    def wait_health(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                self.tf_buffer.lookup_transform(
                    "odom", "base_footprint", rclpy.time.Time()
                )
                self.seen.add("tf_odom")
            except Exception:
                pass
            try:
                self.tf_buffer.lookup_transform(
                    "map", "odom", rclpy.time.Time()
                )
                self.seen.add("tf_map")
            except Exception:
                pass
            if {"odom", "scan", "map", "tf_odom", "tf_map"} <= self.seen:
                break
        missing = {"odom", "scan", "map", "tf_odom", "tf_map"} - self.seen
        if missing:
            raise RuntimeError(f"Missing/unstable ROS data: {sorted(missing)}")
        if not self.nav.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Nav2 action /navigate_to_pose unavailable")
        print("PASS: TF, odom, scan, map and Nav2 action", flush=True)

    def set_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = -13.5
        msg.pose.pose.position.y = -8.15
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.01
        for _ in range(5):
            self.initial_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)

    def run_goal(self, name, x, y, yaw):
        self.unsafe_plan_pose = None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        future = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{name}: goal rejected")
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=self.timeout)
        if not result.done():
            handle.cancel_goal_async()
            raise RuntimeError(f"{name}: timeout")
        status = result.result().status
        if status != 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            raise RuntimeError(f"{name}: Nav2 status={status}")
        if self.unsafe_plan_pose is not None:
            x_bad, y_bad = self.unsafe_plan_pose
            raise RuntimeError(
                f"{name}: global plan footprint intersects map at "
                f"({x_bad:.2f}, {y_bad:.2f})"
            )
        print(f"PASS: Robot {name} (global plan footprint-clear)", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    rclpy.init()
    node = Baseline(args.timeout)
    try:
        node.wait_health()
        node.set_initial_pose()
        time.sleep(2)
        for item in GOALS:
            node.run_goal(*item)
        print("PASS: Robot A → B (live Gazebo/Nav2)", flush=True)
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

