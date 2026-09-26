#!/usr/bin/env python3
"""Initialize AMCL from episode parameters, then wait for scan-time TF."""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class InitialPoseGate(Node):
    def __init__(self, pose: dict) -> None:
        super().__init__("school_episode_initial_pose")
        self.pose = pose
        self.latest_scan = None
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
        )
        self.state_client = self.create_client(GetState, "/amcl/get_state")
        self.pending_state = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _scan_cb(self, msg: LaserScan) -> None:
        if msg.header.stamp.sec > 0:
            self.latest_scan = msg.header.stamp

    def amcl_active(self) -> bool:
        if not self.state_client.service_is_ready():
            return False
        if self.pending_state is None:
            self.pending_state = self.state_client.call_async(GetState.Request())
            return False
        if not self.pending_state.done():
            return False
        result = self.pending_state.result()
        self.pending_state = None
        return result is not None and result.current_state.label == "active"

    def publish_pose(self, odom_tf_stamp) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = odom_tf_stamp
        msg.pose.pose.position.x = float(self.pose["x"])
        msg.pose.pose.position.y = float(self.pose["y"])
        yaw = float(self.pose["yaw"])
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.01
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published /initialpose from episode: {self.pose['x']:.3f}, "
            f"{self.pose['y']:.3f}, {yaw:.3f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--params-file", type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    config = yaml.safe_load(args.params_file.read_text(encoding="utf-8"))
    pose = config["amcl"]["ros__parameters"]["initial_pose"]
    rclpy.init(args=ros_args)
    node = InitialPoseGate(pose)
    deadline = time.monotonic() + 60.0
    published = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not published and node.get_clock().now().nanoseconds > 0 and node.amcl_active():
                if node.publisher.get_subscription_count() > 0 and node.buffer.can_transform(
                    "odom", "base_footprint", rclpy.time.Time()
                ):
                    odom_tf = node.buffer.lookup_transform(
                        "odom", "base_footprint", rclpy.time.Time()
                    )
                    safe_time = rclpy.time.Time.from_msg(odom_tf.header.stamp) - Duration(seconds=0.1)
                    if safe_time.nanoseconds > 0 and node.buffer.can_transform(
                        "odom", "base_footprint", safe_time
                    ):
                        node.publish_pose(safe_time.to_msg())
                        published = True
            if published and node.latest_scan is not None:
                scan_time = rclpy.time.Time.from_msg(node.latest_scan)
                now = node.get_clock().now()
                if (node.buffer.can_transform("map", "base_footprint", scan_time)
                        and node.buffer.can_transform("map", "base_footprint", now)):
                    node.get_logger().info(
                        "AMCL map->odom is available at scan and current simulation timestamps"
                    )
                    return 0
        node.get_logger().error("Timed out waiting for AMCL/scan-time TF")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
