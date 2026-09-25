#!/usr/bin/env python3
"""Apply HuNav poses to Gazebo while enforcing occupancy-map collisions."""
from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_msgs.msg import TFMessage


class OccupancyGuard:
    def __init__(self, yaml_path: str, radius: float):
        cfg_path = Path(yaml_path).resolve()
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
        self.resolution = float(cfg["resolution"])
        self.origin_x, self.origin_y = map(float, cfg["origin"][:2])
        self.radius = radius
        pgm = (cfg_path.parent / cfg["image"]).resolve()
        with pgm.open("rb") as stream:
            if stream.readline().strip() != b"P5":
                raise RuntimeError(f"Only binary P5 maps are supported: {pgm}")
            line = stream.readline()
            while line.startswith(b"#"):
                line = stream.readline()
            self.width, self.height = map(int, line.split())
            if int(stream.readline()) != 255:
                raise RuntimeError(f"Unexpected PGM max value: {pgm}")
            self.pixels = stream.read()
        if len(self.pixels) != self.width * self.height:
            raise RuntimeError(f"Incomplete PGM data: {pgm}")

    def _occupied_cell(self, col: int, map_row: int) -> bool:
        if not (0 <= col < self.width and 0 <= map_row < self.height):
            return True
        image_row = self.height - 1 - map_row
        return self.pixels[image_row * self.width + col] < 128

    def pose_is_free(self, x: float, y: float) -> bool:
        center_col = int(math.floor((x - self.origin_x) / self.resolution))
        center_row = int(math.floor((y - self.origin_y) / self.resolution))
        cell_radius = int(math.ceil(self.radius / self.resolution))
        conservative_radius = self.radius + 0.5 * self.resolution
        for dc in range(-cell_radius, cell_radius + 1):
            for dr in range(-cell_radius, cell_radius + 1):
                # Distance from the pose to the nearest point of this cell.
                dx = max(abs(dc) - 0.5, 0.0) * self.resolution
                dy = max(abs(dr) - 0.5, 0.0) * self.resolution
                if math.hypot(dx, dy) <= conservative_radius:
                    if self._occupied_cell(center_col + dc, center_row + dr):
                        return False
        return True

    def segment_is_free(self, start: tuple[float, float], end: tuple[float, float]) -> bool:
        distance = math.dist(start, end)
        count = max(1, int(math.ceil(distance / (0.5 * self.resolution))))
        for index in range(1, count + 1):
            ratio = index / count
            x = start[0] + ratio * (end[0] - start[0])
            y = start[1] + ratio * (end[1] - start[1])
            if not self.pose_is_free(x, y):
                return False
        return True


class CollisionSafePoseApplier(Node):
    def __init__(self):
        super().__init__("school_collision_safe_pose_applier")
        self.declare_parameter("world_name", "school_arena")
        self.declare_parameter("target_topic", "/school_hunav/target_poses")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("apply_hz", 20.0)
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("agent_radius", 0.22)

        world = str(self.get_parameter("world_name").value)
        topic = str(self.get_parameter("target_topic").value)
        map_yaml = str(self.get_parameter("map_yaml").value)
        hz = float(self.get_parameter("apply_hz").value)
        self.ground_z = float(self.get_parameter("ground_z").value)
        radius = float(self.get_parameter("agent_radius").value)
        if not map_yaml:
            raise RuntimeError("map_yaml parameter is required")
        self.guard = OccupancyGuard(map_yaml, radius)

        self.latest = {}
        self.current = {}
        self.dirty = set()
        self.pending = {}
        self.pending_pose = {}
        self.blocked_count = {}
        self.last_wait_log = 0.0
        service = f"/world/{world}/set_pose"
        self.create_subscription(TFMessage, topic, self._target_cb, 10)
        self.client = self.create_client(SetEntityPose, service)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f"Wall guard active: map={map_yaml}, radius={radius:.2f} m, topic={topic}"
        )

    def _target_cb(self, msg: TFMessage) -> None:
        for target in msg.transforms:
            name = target.child_frame_id.strip("/")
            tr = target.transform.translation
            rot = target.transform.rotation
            values = (tr.x, tr.y, rot.x, rot.y, rot.z, rot.w)
            if name and all(math.isfinite(value) for value in values):
                self.latest[name] = target.transform
                self.dirty.add(name)

    @staticmethod
    def _yaw(rotation) -> float:
        siny = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(siny, cosy)

    def _finish_pending(self, name: str) -> bool:
        future = self.pending.get(name)
        if future is None:
            return True
        if not future.done():
            return False
        try:
            result = future.result()
            if result is not None and result.success:
                self.current[name] = self.pending_pose[name]
            else:
                self.get_logger().error(f"Gazebo rejected set_pose for {name}")
        except Exception as exc:  # ROS service failure must not kill the guard.
            self.get_logger().error(f"set_pose failed for {name}: {exc}")
        self.pending.pop(name, None)
        self.pending_pose.pop(name, None)
        return True

    def _tick(self) -> None:
        if not self.client.service_is_ready():
            now = time.monotonic()
            if now - self.last_wait_log > 2.0:
                self.get_logger().warn("Waiting for Gazebo set_pose service...")
                self.last_wait_log = now
            return

        for name in list(self.dirty):
            if not self._finish_pending(name):
                continue
            transform = self.latest[name]
            target = (transform.translation.x, transform.translation.y)
            previous = self.current.get(name)
            safe = self.guard.pose_is_free(*target)
            if previous is not None:
                safe = safe and self.guard.segment_is_free(previous, target)
            if not safe:
                count = self.blocked_count.get(name, 0) + 1
                self.blocked_count[name] = count
                if count == 1 or count % 100 == 0:
                    self.get_logger().warn(
                        f"BLOCKED wall crossing for {name}: target=({target[0]:.3f}, {target[1]:.3f})"
                    )
                self.dirty.discard(name)
                continue

            self.blocked_count[name] = 0
            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = Entity.MODEL
            req.pose.position.x, req.pose.position.y = target
            req.pose.position.z = self.ground_z
            yaw = self._yaw(transform.rotation)
            req.pose.orientation.z = math.sin(0.5 * yaw)
            req.pose.orientation.w = math.cos(0.5 * yaw)
            self.pending[name] = self.client.call_async(req)
            self.pending_pose[name] = target
            self.dirty.discard(name)


def main() -> None:
    rclpy.init()
    node = CollisionSafePoseApplier()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
