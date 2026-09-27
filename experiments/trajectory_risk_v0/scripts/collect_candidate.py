#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path

import numpy as np
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


BASE_DIR = Path.home() / "nav_ws/experiments/trajectory_risk_v0"
DATA_DIR = BASE_DIR / "data"

FEATURE_FILE = DATA_DIR / "candidate_features.csv"
OUTCOME_FILE = DATA_DIR / "rollout_outcomes.csv"

CMD_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
SCAN_TOPIC = "/scan"

DT = 0.1
DURATION = 3.0


CANDIDATES = {
    "C0": (0.30, 0.40),
    "C1": (0.30, 0.20),
    "C2": (0.30, 0.00),
    "C3": (0.30, -0.20),
    "C4": (0.30, -0.40),
}


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)


def angle_normalize(x):
    return math.atan2(math.sin(x), math.cos(x))


class Collector(Node):

    def __init__(self):
        super().__init__("trajectory_risk_collector")

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            CMD_TOPIC,
            10
        )

        self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.odom = None
        self.scan = None

        self.rollout_min_range = float("inf")

    def odom_callback(self, msg):
        self.odom = msg

    def scan_callback(self, msg):
        self.scan = msg

        valid = [
            r for r in msg.ranges
            if np.isfinite(r)
            and msg.range_min <= r <= msg.range_max
        ]

        if valid:
            self.rollout_min_range = min(
                self.rollout_min_range,
                min(valid)
            )

    def wait_for_data(self):

        self.get_logger().info(
            "Waiting for /odom and /scan ..."
        )

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.odom is not None and self.scan is not None:
                return

    def get_pose(self):

        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation

        return (
            p.x,
            p.y,
            yaw_from_quaternion(q)
        )

    def get_velocity(self):

        twist = self.odom.twist.twist

        return (
            twist.linear.x,
            twist.angular.z
        )

    def sector_min(self, deg_min, deg_max):

        msg = self.scan

        values = []

        for i, r in enumerate(msg.ranges):

            if not np.isfinite(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            deg = math.degrees(angle)

            if deg_min <= deg <= deg_max:
                values.append(r)

        if not values:
            return msg.range_max

        return min(values)

    def get_lidar_features(self):

        front = self.sector_min(
            -30,
            30
        )

        left = self.sector_min(
            30,
            100
        )

        right = self.sector_min(
            -100,
            -30
        )

        return front, left, right

    def predict_trajectory(
        self,
        v,
        w,
        duration=DURATION,
        dt=DT
    ):

        x = 0.0
        y = 0.0
        theta = 0.0

        points = []

        steps = int(duration / dt)

        for _ in range(steps):

            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt

            points.append(
                (
                    x,
                    y,
                    theta
                )
            )

        return points

    def stop_robot(self):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        for _ in range(10):

            self.cmd_pub.publish(msg)

            rclpy.spin_once(
                self,
                timeout_sec=0.02
            )

            time.sleep(0.02)

    def execute_candidate(
        self,
        v,
        w,
        duration
    ):

        cmd = TwistStamped()

        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)

        self.rollout_min_range = float("inf")

        start = time.monotonic()

        while time.monotonic() - start < duration:

            cmd.header.stamp = self.get_clock().now().to_msg()

            self.cmd_pub.publish(cmd)

            rclpy.spin_once(
                self,
                timeout_sec=0.02
            )

            time.sleep(0.08)

        self.stop_robot()

    def collect(
        self,
        episode_id,
        candidate_id
    ):

        if candidate_id not in CANDIDATES:
            raise ValueError(
                f"Unknown candidate: {candidate_id}"
            )

        v_cmd, w_cmd = CANDIDATES[candidate_id]

        self.wait_for_data()

        time.sleep(0.5)

        for _ in range(10):
            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        x0, y0, yaw0 = self.get_pose()

        robot_v, robot_w = self.get_velocity()

        front, left, right = self.get_lidar_features()

        trajectory = self.predict_trajectory(
            v_cmd,
            w_cmd
        )

        end_x, end_y, end_yaw = trajectory[-1]

        traj_length = abs(v_cmd) * DURATION

        if abs(v_cmd) > 1e-6:
            curvature = abs(w_cmd / v_cmd)
        else:
            curvature = 0.0

        feature_row = {
            "episode_id": episode_id,
            "candidate_id": candidate_id,

            "robot_v": robot_v,
            "robot_w": robot_w,

            "lidar_front_min": front,
            "lidar_left_min": left,
            "lidar_right_min": right,

            "traj_v": v_cmd,
            "traj_w": w_cmd,
            "traj_duration": DURATION,
            "traj_length": traj_length,
            "traj_max_abs_w": abs(w_cmd),
            "traj_curvature": curvature,

            "traj_end_x_rel": end_x,
            "traj_end_y_rel": end_y,
            "traj_end_yaw_rel": end_yaw,
        }

        print("\nINPUT FEATURES")
        print(feature_row)

        self.execute_candidate(
            v_cmd,
            w_cmd,
            DURATION
        )

        time.sleep(0.5)

        for _ in range(10):
            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        x1, y1, yaw1 = self.get_pose()

        displacement = math.hypot(
            x1 - x0,
            y1 - y0
        )

        expected_distance = traj_length

        progress_ratio = (
            displacement / expected_distance
            if expected_distance > 1e-6
            else 0.0
        )

        stuck = int(
            expected_distance > 0.3
            and displacement < 0.10
        )

        outcome_row = {
            "episode_id": episode_id,
            "candidate_id": candidate_id,

            "start_x": x0,
            "start_y": y0,

            "end_x": x1,
            "end_y": y1,

            "actual_displacement": displacement,
            "expected_distance": expected_distance,
            "progress_ratio": progress_ratio,

            "actual_min_range": self.rollout_min_range,

            "stuck": stuck,
        }

        print("\nROLLOUT OUTCOME")
        print(outcome_row)

        return feature_row, outcome_row


def append_csv(path, row):

    exists = path.exists()

    with path.open(
        "a",
        newline=""
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=row.keys()
        )

        if not exists:
            writer.writeheader()

        writer.writerow(row)


def main():

    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--episode",
        type=int,
        required=True
    )

    parser.add_argument(
        "--candidate",
        type=str,
        required=True
    )

    args = parser.parse_args()

    DATA_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    rclpy.init()

    node = Collector()

    try:

        features, outcome = node.collect(
            args.episode,
            args.candidate
        )

        append_csv(
            FEATURE_FILE,
            features
        )

        append_csv(
            OUTCOME_FILE,
            outcome
        )

        print("\nSaved successfully.")

    finally:

        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()