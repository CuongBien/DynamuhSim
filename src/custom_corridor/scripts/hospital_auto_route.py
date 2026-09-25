#!/usr/bin/env python3
"""Navigate from the hospital entrance through north/south room 5 and home."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


# Room centers: -25.2, -19.6, -14.0, -8.4, -2.8, ...
# Intermediate corridor goals keep both room exits on the doorway centerline.
ROUTE = [
    ("North room 5", -2.8, 4.0, math.pi / 2),
    ("Exit north room 5", -2.8, 0.0, -math.pi / 2),
    ("South room 5", -2.8, -4.0, -math.pi / 2),
    ("Exit south room 5", -2.8, 0.0, math.pi / 2),
    ("Return to start", -18.0, 0.0, 0.0),
]


class HospitalAutoRoute(Node):
    def __init__(self):
        super().__init__("hospital_auto_route")
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_until_ready(self):
        self.get_logger().info("Waiting for Nav2 action server and map -> base_footprint TF")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            if (self.client.server_is_ready() and
                    self.tf_buffer.can_transform("map", "base_footprint", Time())):
                self.get_logger().info("Nav2 and localization are ready")
                return True
        return False

    def wait_future(self, future):
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
        return future.done()

    def send_waypoint(self, name, x, y, yaw):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        goal.pose = pose
        self.get_logger().info(f"Navigating to {name}: ({x:.1f}, {y:.1f})")
        sent = self.client.send_goal_async(goal)
        if not self.wait_future(sent) or not sent.result().accepted:
            self.get_logger().error(f"Goal rejected: {name}")
            return False
        result = sent.result().get_result_async()
        if not self.wait_future(result):
            return False
        if result.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"Navigation failed at {name}: status {result.result().status}")
            return False
        self.get_logger().info(f"Reached {name}")
        return True


def main():
    rclpy.init()
    node = HospitalAutoRoute()
    try:
        if node.wait_until_ready():
            # Allow AMCL to receive several scans before the first goal.
            until = time.monotonic() + 3.0
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.2)
            for waypoint in ROUTE:
                if not rclpy.ok() or not node.send_waypoint(*waypoint):
                    break
            else:
                node.get_logger().info("Hospital route completed")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
