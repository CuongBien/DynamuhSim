"""Nav2 lifecycle readiness and NavigateToPose goal ownership for one episode."""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable

import yaml

from .episode_types import EpisodeContext, Pose2D

# The navigation launch manages these nodes. waypoint_follower serves a different
# action and is not required by NavigateToPose.
REQUIRED_NODES = (
    "map_server", "amcl", "planner_server", "controller_server",
    "bt_navigator", "behavior_server",
)


class Nav2Error(RuntimeError):
    def __init__(self, code: str, detail: str):
        self.code = code
        super().__init__(f"{code}: {detail}")


class GoalState(str, Enum):
    IDLE = "idle"
    ACCEPTED = "accepted"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"
    REJECTED = "rejected"
    SERVER_LOST = "server_lost"


TERMINAL = {GoalState.SUCCEEDED, GoalState.ABORTED, GoalState.CANCELED,
            GoalState.REJECTED, GoalState.SERVER_LOST}


@dataclass(frozen=True)
class GoalObservation:
    state: GoalState
    status_code: int | None = None
    error_code: int | None = None
    error_msg: str = ""

    @property
    def failure_code(self) -> str | None:
        return {GoalState.ABORTED: "goal_aborted",
                GoalState.REJECTED: "goal_rejected",
                GoalState.SERVER_LOST: "nav2_runtime_lost"}.get(self.state)


@dataclass(frozen=True)
class Nav2Config:
    startup_timeout_sec: float = 60.0
    service_timeout_sec: float = 2.0
    goal_response_timeout_sec: float = 10.0
    cancel_timeout_sec: float = 5.0
    poll_interval_sec: float = 0.2

    def __post_init__(self):
        for name, value in vars(self).items():
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"nav2.{name} must be positive")

    @classmethod
    def from_yaml(cls, path: Path | None = None) -> "Nav2Config":
        path = Path(path) if path is not None else Path(__file__).parent / "config/runner.yaml"
        try:
            doc = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f"Cannot read Nav2 config {path}: {exc}") from exc
        if not isinstance(doc, dict) or not isinstance(doc.get("nav2"), dict):
            raise ValueError(f"Missing nav2 config mapping: {path}")
        return cls(**doc["nav2"])


class RosNav2Backend:
    """Small rclpy adapter; owns its own ROS context and action client."""

    def __init__(self):
        import rclpy
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        self._get_state = GetState
        self._goal_type = NavigateToPose
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("dataset_runner_nav2", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.action = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.services = {}
        self.results = {}

    def tick(self, seconds: float):
        self.executor.spin_once(timeout_sec=seconds)

    def _await(self, future, timeout: float):
        deadline = time.monotonic() + timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self.tick(min(0.2, remaining))

    def lifecycle_state(self, name: str, timeout: float) -> str | None:
        client = self.services.get(name)
        if client is None:
            client = self.node.create_client(self._get_state, f"/{name}/get_state")
            self.services[name] = client
        if not client.wait_for_service(timeout_sec=0):
            return None
        future = client.call_async(self._get_state.Request())
        self._await(future, timeout)
        if not future.done():
            return None
        response = future.result()
        return response.current_state.label if response is not None else None

    def action_ready(self) -> bool:
        return self.action.server_is_ready()

    def send_goal(self, pose: Pose2D, timeout: float):
        goal = self._goal_type.Goal()
        goal.pose.header.frame_id = "map"  # human_navigation_graph.yaml frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        goal.pose.pose.orientation.z = math.sin(pose.heading / 2)
        goal.pose.pose.orientation.w = math.cos(pose.heading / 2)
        future = self.action.send_goal_async(goal)
        self._await(future, timeout)
        if not future.done():
            raise Nav2Error("goal_send_failed", "Nav2 goal response timed out")
        handle = future.result()
        if handle is None:
            raise Nav2Error("goal_send_failed", "Nav2 returned no goal handle")
        if handle.accepted:
            self.results[bytes(handle.goal_id.uuid)] = handle.get_result_async()
        return handle

    def goal_result(self, handle) -> GoalObservation:
        from action_msgs.msg import GoalStatus
        future = self.results.get(bytes(handle.goal_id.uuid))
        if future is not None and future.done():
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
            states = {
                GoalStatus.STATUS_SUCCEEDED: GoalState.SUCCEEDED,
                GoalStatus.STATUS_ABORTED: GoalState.ABORTED,
                GoalStatus.STATUS_CANCELED: GoalState.CANCELED,
            }
            return GoalObservation(states.get(status, GoalState.EXECUTING), status,
                                   result.error_code, result.error_msg)
        status = handle.status
        if status == GoalStatus.STATUS_ACCEPTED:
            state = GoalState.ACCEPTED
        elif status in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING):
            state = GoalState.EXECUTING
        elif status == GoalStatus.STATUS_ABORTED:
            state = GoalState.ABORTED
        elif status == GoalStatus.STATUS_CANCELED:
            state = GoalState.CANCELED
        elif status == GoalStatus.STATUS_SUCCEEDED:
            state = GoalState.SUCCEEDED
        else:
            state = GoalState.ACCEPTED
        return GoalObservation(state, status)

    def cancel_goal(self, handle, timeout: float) -> bool:
        future = handle.cancel_goal_async()
        self._await(future, timeout)
        if not future.done() or future.result() is None:
            return False
        response = future.result()
        return response.return_code == 0 and bool(response.goals_canceling)

    def close(self):
        try:
            self.action.destroy()
            for client in self.services.values():
                self.node.destroy_client(client)
            self.executor.remove_node(self.node)
            self.node.destroy_node()
            self.executor.shutdown()
        finally:
            if self.context.ok():
                self._rclpy.shutdown(context=self.context)


class Nav2Manager:
    def __init__(self, *, config: Nav2Config | None = None,
                 backend_factory: Callable = RosNav2Backend,
                 clock: Callable[[], float] = time.monotonic,
                 logger: logging.Logger | None = None):
        self.config = config or Nav2Config.from_yaml()
        self._backend_factory = backend_factory
        self._clock = clock
        self.logger = logger or logging.getLogger(__name__)
        self.backend = None
        self.handle = None
        self.episode_id: str | None = None
        self.observation = GoalObservation(GoalState.IDLE)

    def _backend(self):
        if self.backend is None:
            try:
                self.backend = self._backend_factory()
            except Exception as exc:
                raise Nav2Error("nav2_unavailable", str(exc)) from exc
        return self.backend

    def wait_ready(self, context: EpisodeContext) -> None:
        backend = self._backend()
        deadline = self._clock() + self.config.startup_timeout_sec
        states = {}
        while True:
            for name in REQUIRED_NODES:
                states[name] = backend.lifecycle_state(name, self.config.service_timeout_sec)
            inactive = {name: state for name, state in states.items() if state != "active"}
            action_ready = backend.action_ready()
            if not inactive and action_ready:
                self.logger.info("[NAV2 READY] required lifecycle nodes active; /navigate_to_pose ready")
                return
            remaining = deadline - self._clock()
            if remaining <= 0:
                if inactive:
                    raise Nav2Error("lifecycle_not_active", f"nodes not active: {inactive}")
                raise Nav2Error("action_server_unavailable", "/navigate_to_pose unavailable")
            backend.tick(min(self.config.poll_interval_sec, remaining))

    def send_goal(self, context: EpisodeContext) -> GoalObservation:
        self.cancel_goal()
        backend = self._backend()
        if not backend.action_ready():
            raise Nav2Error("action_server_unavailable", "/navigate_to_pose unavailable")
        try:
            handle = backend.send_goal(context.goal_pose, self.config.goal_response_timeout_sec)
        except Nav2Error:
            raise
        except Exception as exc:
            raise Nav2Error("goal_send_failed", str(exc)) from exc
        if not handle.accepted:
            self.observation = GoalObservation(GoalState.REJECTED)
            raise Nav2Error("goal_rejected", f"episode {context.episode_id} goal rejected")
        self.handle = handle
        self.episode_id = context.episode_id
        self.observation = GoalObservation(GoalState.ACCEPTED, 1)
        self.logger.info("[NAV2 GOAL ACCEPTED] episode=%s x=%.3f y=%.3f yaw=%.3f",
                         context.episode_id, context.goal_pose.x, context.goal_pose.y,
                         context.goal_pose.heading)
        return self.observation

    def poll(self, context: EpisodeContext) -> GoalObservation:
        if self.handle is None:
            return self.observation
        if context.episode_id != self.episode_id:
            raise Nav2Error("nav2_runtime_lost", "goal belongs to another episode")
        backend = self._backend()
        backend.tick(0.0)
        if not backend.action_ready():
            self.observation = GoalObservation(GoalState.SERVER_LOST)
            raise Nav2Error("nav2_runtime_lost", "/navigate_to_pose action server disappeared")
        try:
            self.observation = backend.goal_result(self.handle)
        except Exception as exc:
            raise Nav2Error("nav2_runtime_lost", str(exc)) from exc
        return self.observation

    def cancel_goal(self, context: EpisodeContext | None = None) -> None:
        if self.handle is None:
            return
        if context is not None and context.episode_id != self.episode_id:
            raise Nav2Error("goal_cancel_failed", "cannot cancel another episode's goal")
        backend = self._backend()
        observation = backend.goal_result(self.handle)
        if observation.state not in TERMINAL:
            if not backend.action_ready():
                self.observation = GoalObservation(GoalState.SERVER_LOST)
                raise Nav2Error("nav2_runtime_lost", "action server lost before cancel")
            try:
                accepted = backend.cancel_goal(self.handle, self.config.cancel_timeout_sec)
            except Exception as exc:
                raise Nav2Error("goal_cancel_failed", str(exc)) from exc
            if not accepted:
                # A goal can finish while cancellation is in flight.
                observation = backend.goal_result(self.handle)
                if observation.state not in TERMINAL:
                    raise Nav2Error("goal_cancel_failed", "cancel request not acknowledged")
            else:
                deadline = self._clock() + self.config.cancel_timeout_sec
                while self._clock() < deadline:
                    observation = backend.goal_result(self.handle)
                    if observation.state in TERMINAL:
                        break
                    backend.tick(min(self.config.poll_interval_sec, deadline - self._clock()))
                else:
                    raise Nav2Error("goal_cancel_failed", "goal did not reach terminal state")
        self.observation = observation
        self.handle = None
        self.episode_id = None

    def finalize(self, context: EpisodeContext) -> None:
        try:
            self.cancel_goal(context)
        finally:
            if self.backend is not None:
                self.backend.close()
                self.backend = None
