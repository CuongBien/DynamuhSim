#!/usr/bin/env python3
"""Expose Gazebo ground-truth robot pose in RViz and monitor TF alignment."""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


class PoseSyncMonitor(Node):
    def __init__(self) -> None:
        super().__init__("school_pose_sync_monitor")
        self.declare_parameter("robot_name", "robot")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("initial_x", -13.5)
        self.declare_parameter("initial_y", -8.15)
        self.robot_name = str(self.get_parameter("robot_name").value).strip("/")
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.initial_xy = (float(self.get_parameter("initial_x").value),
                           float(self.get_parameter("initial_y").value))
        self.last_gazebo_xy = None
        self.pose_pub = self.create_publisher(PoseStamped, "/demo/gazebo_robot_pose", 10)
        self.path_pub = self.create_publisher(Path, "/demo/gazebo_robot_path", 10)
        self.error_pub = self.create_publisher(Float32, "/demo/pose_sync_error", 10)
        self.create_subscription(
            TFMessage, "/world/school_arena/dynamic_pose/info", self._pose_cb, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path = Path()
        self.path.header.frame_id = self.world_frame
        self.last_path_xy = None
        self.last_log = 0.0
        self.last_missing_log = 0.0
        self.get_logger().info(
            "Gazebo/RViz sync monitor active: orange pose=/demo/gazebo_robot_pose"
        )

    def _candidate_score(self, frame: str) -> int:
        clean = frame.strip("/")
        if clean == self.robot_name:
            return 3
        if clean.endswith("/" + self.robot_name):
            return 2
        if clean in (f"{self.robot_name}/base_footprint", f"{self.robot_name}/base_link"):
            return 1
        return 0

    def _pose_cb(self, msg: TFMessage) -> None:
        named = [t for t in msg.transforms if self._candidate_score(t.child_frame_id)]
        # Gazebo Sim 8 -> TFMessage currently drops Pose_V entity names. In
        # that case, associate the model pose by nearest-neighbour tracking
        # from the known spawn pose. Link-local transforms near (0, 0) are
        # therefore never mistaken for the robot model.
        reference = self.last_gazebo_xy or self.initial_xy
        candidates = named or [
            t for t in msg.transforms
            if -0.10 <= t.transform.translation.z <= 0.50
        ]
        if not candidates:
            now = time.monotonic()
            if now - self.last_missing_log > 5.0:
                frames = ", ".join(t.child_frame_id for t in msg.transforms[:8])
                self.get_logger().warn(
                    f"Waiting for Gazebo model '{self.robot_name}'. Frames: {frames or '(none)'}"
                )
                self.last_missing_log = now
            return
        if named:
            transform = max(candidates, key=lambda item: self._candidate_score(item.child_frame_id))
        else:
            transform = min(
                candidates,
                key=lambda item: math.hypot(
                    item.transform.translation.x - reference[0],
                    item.transform.translation.y - reference[1],
                ),
            )
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.frame_id = self.world_frame
        pose.header.stamp = stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = max(0.03, transform.transform.translation.z)
        pose.pose.orientation = transform.transform.rotation
        self.pose_pub.publish(pose)

        xy = (pose.pose.position.x, pose.pose.position.y)
        self.last_gazebo_xy = xy
        if self.last_path_xy is None or math.dist(xy, self.last_path_xy) >= 0.04:
            self.path.header.stamp = stamp
            self.path.poses.append(pose)
            if len(self.path.poses) > 2500:
                self.path.poses = self.path.poses[-2500:]
            self.path_pub.publish(self.path)
            self.last_path_xy = xy
        self._compare_with_tf(xy)

    def _compare_with_tf(self, gazebo_xy: tuple[float, float]) -> None:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except TransformException:
            return
        estimated = (tf.transform.translation.x, tf.transform.translation.y)
        error = math.dist(gazebo_xy, estimated)
        msg = Float32()
        msg.data = float(error)
        self.error_pub.publish(msg)
        now = time.monotonic()
        if now - self.last_log > 5.0:
            level = self.get_logger().warn if error > 0.20 else self.get_logger().info
            level(
                f"Gazebo↔RViz pose error={error:.3f} m "
                f"(gz={gazebo_xy[0]:.2f},{gazebo_xy[1]:.2f}; "
                f"tf={estimated[0]:.2f},{estimated[1]:.2f})"
            )
            self.last_log = now


def main() -> None:
    rclpy.init()
    node = PoseSyncMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
