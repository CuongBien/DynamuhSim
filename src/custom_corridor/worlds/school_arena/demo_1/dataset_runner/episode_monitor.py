"""Monitor a running Nav2 episode using action, odometry and Gazebo contacts."""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import yaml

from .episode_types import EpisodeContext, EpisodeStatus, MonitorResult
from .nav2_manager import GoalState, Nav2Error


@dataclass(frozen=True)
class MonitorConfig:
    timeout_sec: float
    poll_interval_sec: float
    stuck_enabled: bool
    stuck_window_sec: float
    stuck_min_displacement_m: float
    near_goal_distance_m: float
    odom_topic: str
    contact_topics: tuple[str, ...]

    def __post_init__(self):
        for name in ("timeout_sec", "poll_interval_sec", "stuck_window_sec",
                     "stuck_min_displacement_m", "near_goal_distance_m"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"monitor.{name} must be positive and finite")
        if not isinstance(self.stuck_enabled, bool):
            raise ValueError("stuck.enabled must be boolean")
        if not self.odom_topic or not self.contact_topics or any(not x for x in self.contact_topics):
            raise ValueError("monitor topics must be configured")

    @classmethod
    def from_yaml(cls, path: Path | None = None) -> "MonitorConfig":
        path = Path(path) if path is not None else Path(__file__).parent / "config/runner.yaml"
        try:
            doc = yaml.safe_load(path.read_text(encoding="utf-8"))
            episode, stuck, collision, readiness = (doc[k] for k in
                ("episode", "stuck", "collision", "readiness"))
            return cls(
                timeout_sec=episode["timeout_sec"],
                poll_interval_sec=episode["poll_interval_sec"],
                stuck_enabled=stuck["enabled"],
                stuck_window_sec=stuck["window_sec"],
                stuck_min_displacement_m=stuck["min_displacement_m"],
                near_goal_distance_m=stuck["near_goal_distance_m"],
                odom_topic=readiness["topics"]["odom"],
                contact_topics=tuple(collision["topics"]),
            )
        except (OSError, yaml.YAMLError, KeyError, TypeError) as exc:
            raise ValueError(f"Invalid monitor config {path}: {exc}") from exc


class RosGazeboMonitorBackend:
    """Own a ROS odom subscriber and direct Gazebo transport contact subscribers."""

    def __init__(self, config: MonitorConfig, clock: Callable[[], float] = time.monotonic):
        import rclpy
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.transport13 import Node as GazeboNode
        from nav_msgs.msg import Odometry
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data

        self._rclpy = rclpy
        self._clock = clock
        self._lock = threading.Lock()
        self._odom: deque[tuple[float, float, float]] = deque(maxlen=4096)
        self._contacts: deque[tuple[str, str]] = deque(maxlen=1024)
        self._topics = config.contact_topics
        self.ros_context = Context()
        rclpy.init(context=self.ros_context)
        try:
            self.node = Node("dataset_runner_monitor", context=self.ros_context)
            self.executor = SingleThreadedExecutor(context=self.ros_context)
            self.executor.add_node(self.node)
            self.node.create_subscription(Odometry, config.odom_topic, self._on_odom,
                                          qos_profile_sensor_data)
            self.gz_node = GazeboNode()
            for topic in self._topics:
                if not self.gz_node.subscribe(Contacts, topic, self._on_contacts):
                    raise RuntimeError(f"Gazebo contact subscription failed: {topic}")
        except Exception:
            self.close()
            raise

    def _on_odom(self, message):
        pose = message.pose.pose.position
        if math.isfinite(pose.x) and math.isfinite(pose.y):
            with self._lock:
                self._odom.append((self._clock(), float(pose.x), float(pose.y)))

    def _on_contacts(self, message):
        with self._lock:
            for contact in message.contact:
                self._contacts.append((contact.collision1.name, contact.collision2.name))

    def tick(self, seconds: float):
        self.executor.spin_once(timeout_sec=seconds)

    def observations(self):
        with self._lock:
            contacts = tuple(self._contacts)
            self._contacts.clear()
            odom = tuple(self._odom)
        return contacts, odom

    def close(self):
        if hasattr(self, "gz_node"):
            for topic in self._topics:
                self.gz_node.unsubscribe(topic)
        if hasattr(self, "executor"):
            self.executor.remove_node(self.node)
            self.node.destroy_node()
            self.executor.shutdown()
        if self.ros_context.ok():
            self._rclpy.shutdown(context=self.ros_context)


def classify_contact(pair: tuple[str, str], human_names: set[str]) -> str | None:
    """Return a collision kind only for a real robot contact, excluding ground/self."""
    first, second = pair
    one, two = set(first.split("::")), set(second.split("::"))
    if ("robot" in one) == ("robot" in two):
        return None
    other = two if "robot" in one else one
    if "ground" in other:
        return None
    if not any(other):
        return "generic"
    return "human" if other & human_names else "static_obstacle"


class EpisodeMonitor:
    """Emit exactly one terminal result. Precedence: contact, action, runtime, stuck, timeout."""

    def __init__(self, *, nav2_manager, simulator_manager, hunav_manager,
                 config: MonitorConfig | None = None,
                 backend_factory: Callable | None = None,
                 clock: Callable[[], float] = time.monotonic,
                 logger: logging.Logger | None = None):
        self.nav2 = nav2_manager
        self.simulator = simulator_manager
        self.hunav = hunav_manager
        self.config = config or MonitorConfig.from_yaml()
        self._clock = clock
        self._backend_factory = backend_factory or (lambda: RosGazeboMonitorBackend(self.config, clock))
        self.logger = logger or logging.getLogger(__name__)

    def run(self, context: EpisodeContext) -> MonitorResult:
        began = self._clock()  # EpisodeRunner has just transitioned into RUNNING.
        backend = self._backend_factory()
        human_names = set(context.humans.get("hunav_loader", {}).get("ros__parameters", {}).get("agents", []))
        last_state = None
        try:
            while True:
                now = self._clock()
                contacts, odom = backend.observations()
                # A physics contact is direct evidence, even if an action also ends this tick.
                for pair in contacts:
                    kind = classify_contact(pair, human_names)
                    if kind is not None:
                        self.logger.info("[TERMINATED: COLLISION] %s %s", kind, pair)
                        return MonitorResult(EpisodeStatus.COLLISION, "physical_collision",
                                             now - began, collision_occurred=True,
                                             collision_kind=kind, collision_pair=pair)
                try:
                    nav = self.nav2.poll(context)
                except Nav2Error as exc:
                    if exc.code == "nav2_runtime_lost":
                        return MonitorResult(EpisodeStatus.SIM_FAILURE, exc.code, now - began,
                                             navigation_status=GoalState.SERVER_LOST.value)
                    return MonitorResult(EpisodeStatus.NAV_FAILURE, exc.code, now - began)
                if nav.state != last_state:
                    self.logger.info("[NAV2 %s]", nav.state.value.upper())
                    last_state = nav.state
                nav_fields = dict(navigation_status=nav.state.value,
                                  navigation_status_code=nav.status_code,
                                  navigation_result_code=nav.error_code,
                                  navigation_error_msg=nav.error_msg)
                if nav.state == GoalState.SUCCEEDED:
                    return MonitorResult(EpisodeStatus.SUCCESS, "nav_goal_reached", now - began,
                                         **nav_fields)
                if nav.state in (GoalState.ABORTED, GoalState.REJECTED, GoalState.CANCELED):
                    reason = {GoalState.ABORTED: "nav_goal_aborted",
                              GoalState.REJECTED: "nav_goal_rejected",
                              GoalState.CANCELED: "nav_goal_canceled"}[nav.state]
                    self.logger.info("[TERMINATED: NAV_FAILURE] %s code=%s", reason, nav.error_code)
                    return MonitorResult(EpisodeStatus.NAV_FAILURE, reason, now - began,
                                         **nav_fields)
                if nav.state == GoalState.SERVER_LOST:
                    return MonitorResult(EpisodeStatus.SIM_FAILURE, "nav2_runtime_lost",
                                         now - began, **nav_fields)
                if nav.state == GoalState.IDLE:
                    return MonitorResult(EpisodeStatus.NAV_FAILURE, "nav_goal_missing",
                                         now - began, **nav_fields)
                for manager, reason in ((self.simulator, "simulator_process_exited"),
                                        (self.hunav, "hunav_runtime_exited")):
                    try:
                        manager.check_health(context)
                    except Exception as exc:
                        self.logger.error("[RUNTIME FAILURE] %s: %s", reason, exc)
                        return MonitorResult(EpisodeStatus.SIM_FAILURE, reason, now - began,
                                             **nav_fields)
                if (self.config.stuck_enabled and nav.state in (GoalState.ACCEPTED, GoalState.EXECUTING)
                        and (nav.distance_remaining is None or
                             nav.distance_remaining > self.config.near_goal_distance_m)):
                    cutoff = now - self.config.stuck_window_sec
                    baseline = next((sample for sample in reversed(odom)
                                     if sample[0] <= cutoff), None)
                    # A real odom sample must cover the entire configured window.
                    if baseline is not None and len(odom) >= 2:
                        displacement = math.hypot(odom[-1][1] - baseline[1],
                                                  odom[-1][2] - baseline[2])
                        if displacement < self.config.stuck_min_displacement_m:
                            self.logger.info("[TERMINATED: STUCK] displacement=%.3fm window=%.1fs",
                                             displacement, self.config.stuck_window_sec)
                            return MonitorResult(EpisodeStatus.STUCK, "robot_stuck", now - began,
                                                 stuck_displacement_m=displacement, **nav_fields)
                if now - began >= self.config.timeout_sec:
                    return MonitorResult(EpisodeStatus.TIMEOUT, "episode_timeout", now - began,
                                         **nav_fields)
                backend.tick(min(self.config.poll_interval_sec,
                                 self.config.timeout_sec - (now - began)))
        finally:
            backend.close()
