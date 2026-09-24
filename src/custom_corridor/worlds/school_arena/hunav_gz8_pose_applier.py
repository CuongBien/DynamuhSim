#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class SchoolPoseApplier(Node):
    """Apply HuNav targets to Gazebo from the Jazzy side of the DDS boundary."""

    def __init__(self):
        super().__init__('school_hunav_pose_applier')
        self.declare_parameter('world_name', 'school_arena')
        self.declare_parameter('target_topic', '/school_hunav/target_poses')
        self.declare_parameter('apply_hz', 20.0)
        self.declare_parameter('ground_z', 0.0)

        world_name = self.get_parameter('world_name').value
        target_topic = self.get_parameter('target_topic').value
        apply_hz = float(self.get_parameter('apply_hz').value)
        self.ground_z = float(self.get_parameter('ground_z').value)

        self.latest = {}
        self.dirty = set()
        self.pending = {}
        self.last_wait_log = 0.0
        service_name = f'/world/{world_name}/set_pose'

        self.create_subscription(TFMessage, target_topic, self._target_cb, 10)
        self.set_pose_cli = self.create_client(SetEntityPose, service_name)
        self.create_timer(1.0 / apply_hz, self._tick)

        self.get_logger().info(f'HuNav target topic: {target_topic}')
        self.get_logger().info(f'Gazebo set_pose service: {service_name}')

    def _target_cb(self, msg):
        for target in msg.transforms:
            name = target.child_frame_id.strip('/')
            tr = target.transform.translation
            rot = target.transform.rotation
            values = (tr.x, tr.y, rot.x, rot.y, rot.z, rot.w)
            if name and all(math.isfinite(value) for value in values):
                self.latest[name] = target.transform
                self.dirty.add(name)

    @staticmethod
    def _yaw(rotation):
        siny = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(siny, cosy)

    def _tick(self):
        if not self.set_pose_cli.service_is_ready():
            now = time.monotonic()
            if now - self.last_wait_log > 2.0:
                self.get_logger().warn('Waiting for Gazebo set_pose service...')
                self.last_wait_log = now
            return

        for name in list(self.dirty):
            transform = self.latest[name]
            pending = self.pending.get(name)
            if pending is not None and not pending.done():
                continue
            if pending is not None:
                try:
                    result = pending.result()
                    if result is not None and hasattr(result, 'success') and not result.success:
                        self.get_logger().error(f'Gazebo rejected set_pose for {name}')
                except Exception as exc:
                    self.get_logger().error(f'set_pose failed for {name}: {exc}')

            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = Entity.MODEL
            req.pose.position.x = transform.translation.x
            req.pose.position.y = transform.translation.y
            # Human proxies are planar and visual-only.  Never forward Z,
            # roll, or pitch from transport feedback: those values are what
            # previously made feet jump or the whole body appear airborne.
            req.pose.position.z = self.ground_z
            yaw = self._yaw(transform.rotation)
            req.pose.orientation.x = 0.0
            req.pose.orientation.y = 0.0
            req.pose.orientation.z = math.sin(0.5 * yaw)
            req.pose.orientation.w = math.cos(0.5 * yaw)
            self.pending[name] = self.set_pose_cli.call_async(req)
            self.dirty.discard(name)


def main():
    rclpy.init()
    node = SchoolPoseApplier()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
