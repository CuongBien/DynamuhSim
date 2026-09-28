"""Episode sensor, TF and physical start-pose readiness checks."""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import yaml

from .episode_types import EpisodeContext, Pose2D


class ReadinessError(RuntimeError):
    def __init__(self, code: str, detail: str):
        self.code = code
        super().__init__(f"{code}: {detail}")


@dataclass(frozen=True)
class ReadinessConfig:
    timeout_sec: float = 30.0
    poll_interval_sec: float = 0.2
    odom_topic: str = "/odom"
    scan_topic: str = "/scan"
    rgb_topic: str = "/robot/camera/image_raw"
    depth_topic: str | None = None
    gazebo_pose_topic: str = "/demo/gazebo_robot_pose"
    rgb_required: bool = True
    depth_required: bool = False
    tf_parent: str = "odom"
    tf_child: str = "base_footprint"
    pose_frame: str = "map"
    position_tolerance_m: float = 0.15
    yaw_tolerance_rad: float = 0.20

    def __post_init__(self):
        for name in ("timeout_sec", "poll_interval_sec", "position_tolerance_m",
                     "yaw_tolerance_rad"):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"readiness.{name} must be positive and finite")
        for name in ("odom_topic", "scan_topic", "gazebo_pose_topic"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.startswith("/"):
                raise ValueError(f"readiness.{name} must be an absolute topic")
        for required, topic, label in ((self.rgb_required, self.rgb_topic, "rgb"),
                                       (self.depth_required, self.depth_topic, "depth")):
            if required and (not isinstance(topic, str) or not topic.startswith("/")):
                raise ValueError(f"readiness.{label}_topic required")
        if not self.tf_parent or not self.tf_child or not self.pose_frame:
            raise ValueError("readiness TF frames must be nonempty")

    @classmethod
    def from_yaml(cls, path: Path | None = None) -> "ReadinessConfig":
        path = Path(path) if path is not None else Path(__file__).parent / "config/runner.yaml"
        try:
            doc = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f"Cannot read readiness config {path}: {exc}") from exc
        cfg = doc.get("readiness") if isinstance(doc, dict) else None
        if not isinstance(cfg, dict):
            raise ValueError(f"Missing readiness config mapping: {path}")
        topics, frames, start = (cfg.get("topics"), cfg.get("tf"), cfg.get("robot_start"))
        if not all(isinstance(item, dict) for item in (topics, frames, start)):
            raise ValueError("readiness topics/tf/robot_start must be mappings")
        return cls(
            timeout_sec=cfg.get("timeout_sec", 30.0),
            poll_interval_sec=cfg.get("poll_interval_sec", 0.2),
            odom_topic=topics["odom"], scan_topic=topics["scan"],
            rgb_topic=topics.get("rgb"), depth_topic=topics.get("depth"),
            gazebo_pose_topic=topics["gazebo_pose"],
            rgb_required=cfg.get("rgb_required", True),
            depth_required=cfg.get("depth_required", False),
            tf_parent=frames["parent"], tf_child=frames["child"],
            pose_frame=frames["pose_frame"],
            position_tolerance_m=start["position_tolerance_m"],
            yaw_tolerance_rad=start["yaw_tolerance_rad"],
        )


@dataclass(frozen=True)
class ReadinessSnapshot:
    odom: bool = False
    scan: bool = False
    rgb: bool = False
    depth: bool = False
    tf: bool = False
    robot_pose: Pose2D | None = None


def _finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def _valid_quaternion(quaternion) -> bool:
    components = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    return _finite(*components) and 0.5 < sum(value * value for value in components) < 1.5


def _yaw(quaternion) -> float:
    return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


class RosReadinessBackend:
    """Own ROS context; subscriptions only accept messages received this episode."""

    def __init__(self, config: ReadinessConfig):
        import rclpy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image, LaserScan
        from geometry_msgs.msg import PoseStamped
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener

        self._rclpy = rclpy
        self._time_type = Time
        self.config = config
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("dataset_runner_readiness", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node, spin_thread=False)
        self.received = {"odom": False, "scan": False, "rgb": False, "depth": False}
        self.robot_pose = None
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Odometry, config.odom_topic, self._odom, sensor_qos)
        self.node.create_subscription(LaserScan, config.scan_topic, self._scan, sensor_qos)
        if config.rgb_required:
            self.node.create_subscription(Image, config.rgb_topic, self._rgb, sensor_qos)
        if config.depth_required:
            self.node.create_subscription(Image, config.depth_topic, self._depth, sensor_qos)
        self.node.create_subscription(PoseStamped, config.gazebo_pose_topic,
                                      self._robot_pose, sensor_qos)

    @staticmethod
    def _stamped(message) -> bool:
        stamp = message.header.stamp
        return stamp.sec > 0 or stamp.nanosec > 0

    def _odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.received["odom"] = bool(
            self._stamped(msg) and msg.header.frame_id.strip("/") == self.config.tf_parent
            and msg.child_frame_id.strip("/") == self.config.tf_child
            and _finite(p.x, p.y, p.z) and _valid_quaternion(q)
        )

    def _scan(self, msg):
        self.received["scan"] = bool(self._stamped(msg) and msg.ranges)

    def _rgb(self, msg):
        self.received["rgb"] = bool(self._stamped(msg) and msg.width > 0
                                    and msg.height > 0 and msg.data)

    def _depth(self, msg):
        self.received["depth"] = bool(self._stamped(msg) and msg.width > 0
                                      and msg.height > 0 and msg.data)

    def _robot_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        if (not self._stamped(msg) or msg.header.frame_id.strip("/") != self.config.pose_frame
                or not _finite(p.x, p.y, p.z) or not _valid_quaternion(q)):
            return
        yaw = _yaw(q)
        if math.isfinite(yaw):
            self.robot_pose = Pose2D(p.x, p.y, yaw)

    def snapshot(self) -> ReadinessSnapshot:
        tf_valid = False
        try:
            transform = self.buffer.lookup_transform(self.config.tf_parent,
                                                     self.config.tf_child, self._time_type())
            tr, q = transform.transform.translation, transform.transform.rotation
            tf_valid = bool(transform.header.stamp.sec or transform.header.stamp.nanosec)
            tf_valid = tf_valid and _finite(tr.x, tr.y, tr.z) and _valid_quaternion(q)
        except Exception:
            pass
        return ReadinessSnapshot(**self.received, tf=tf_valid, robot_pose=self.robot_pose)

    def tick(self, seconds: float) -> None:
        self.executor.spin_once(timeout_sec=seconds)

    def close(self) -> None:
        try:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
            self.executor.shutdown()
        finally:
            if self.context.ok():
                self._rclpy.shutdown(context=self.context)


class ReadinessChecker:
    def __init__(self, *, config: ReadinessConfig | None = None,
                 backend_factory: Callable[[ReadinessConfig], RosReadinessBackend] = RosReadinessBackend,
                 clock: Callable[[], float] = time.monotonic,
                 logger: logging.Logger | None = None):
        self.config = config or ReadinessConfig.from_yaml()
        self._backend_factory = backend_factory
        self._clock = clock
        self.logger = logger or logging.getLogger(__name__)
        self.backend = None

    def _check(self, context: EpisodeContext, snapshot: ReadinessSnapshot) -> tuple[str, str] | None:
        for ready, code, label in (
            (snapshot.odom, "odom_unavailable", "ODOM"),
            (snapshot.scan, "scan_unavailable", "SCAN"),
            (not self.config.rgb_required or snapshot.rgb, "rgb_unavailable", "RGB"),
            (not self.config.depth_required or snapshot.depth, "depth_unavailable", "DEPTH"),
            (snapshot.tf, "tf_unavailable", "TF"),
        ):
            if not ready:
                return code, f"{label} did not provide a valid fresh sample"
        pose = snapshot.robot_pose
        if pose is None:
            return "robot_pose_unavailable", "Gazebo robot pose not received"
        if not _finite(pose.x, pose.y, pose.heading):
            return "robot_pose_invalid", "Gazebo robot pose is non-finite"
        position_error = math.hypot(pose.x - context.start_pose.x, pose.y - context.start_pose.y)
        yaw_error = abs(math.atan2(math.sin(pose.heading - context.start_pose.heading),
                                   math.cos(pose.heading - context.start_pose.heading)))
        if (position_error > self.config.position_tolerance_m
                or yaw_error > self.config.yaw_tolerance_rad):
            return ("robot_pose_invalid", f"start error position={position_error:.3f}m "
                    f"yaw={yaw_error:.3f}rad; expected={context.start_pose} actual={pose}")
        return None

    def wait_ready(self, context: EpisodeContext) -> ReadinessSnapshot:
        if self.backend is not None:
            self.backend.close()
        try:
            self.backend = self._backend_factory(self.config)
        except Exception as exc:
            raise ReadinessError("probe_unavailable", str(exc)) from exc
        deadline = self._clock() + self.config.timeout_sec
        announced = set()
        while True:
            try:
                snapshot = self.backend.snapshot()
            except Exception as exc:
                raise ReadinessError("probe_failed", str(exc)) from exc
            ready_flags = {
                "ODOM": snapshot.odom, "SCAN": snapshot.scan,
                "RGB": self.config.rgb_required and snapshot.rgb,
                "DEPTH": self.config.depth_required and snapshot.depth,
                "TF": snapshot.tf,
            }
            for label, ready in ready_flags.items():
                if ready and label not in announced:
                    self.logger.info("[%s READY]", label)
                    announced.add(label)
            problem = self._check(context, snapshot)
            if problem is None:
                self.logger.info("[ROBOT POSE READY] x=%.3f y=%.3f yaw=%.3f",
                                 snapshot.robot_pose.x, snapshot.robot_pose.y,
                                 snapshot.robot_pose.heading)
                return snapshot
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise ReadinessError(*problem)
            self.backend.tick(min(self.config.poll_interval_sec, remaining))

    def finalize(self, context: EpisodeContext | None = None) -> None:
        if self.backend is not None:
            try:
                self.backend.close()
            finally:
                self.backend = None
