#!/usr/bin/env python3
"""Wait until Gazebo robot odometry, TF and simulation clock are usable."""
from __future__ import annotations

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class RobotReady(Node):
    def __init__(self) -> None:
        super().__init__("school_wait_for_robot_tf")
        self.have_odom = False
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _odom_cb(self, msg: Odometry) -> None:
        self.have_odom = (
            msg.header.stamp.sec > 0
            and msg.header.frame_id.strip("/") == "odom"
            and msg.child_frame_id.strip("/") == "base_footprint"
        )


def main() -> int:
    rclpy.init()
    node = RobotReady()
    deadline = time.monotonic() + 40.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.have_odom and node.get_clock().now().nanoseconds > 0
                    and node.buffer.can_transform("odom", "base_footprint", rclpy.time.Time())):
                node.get_logger().info("Robot odom/TF and simulation clock ready")
                return 0
        node.get_logger().error("Timed out waiting for robot odom/TF on simulation clock")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
