"""Rigid transforms between world and the planar robot base."""

from math import cos, pi, remainder, sin, tau

from .types import ROBOT_BASE_FRAME, WORLD_FRAME, Point2D, Pose2D
from .validation import finite, frame_is


def wrap_yaw(yaw: float) -> float:
    """Return a finite angle in [-pi, pi)."""
    wrapped = remainder(finite(yaw, "yaw"), tau)
    return -pi if wrapped == pi else wrapped


def _rotate(x: float, y: float, yaw: float) -> tuple[float, float]:
    cosine, sine = cos(yaw), sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def world_to_body(point: Point2D, robot_pose: Pose2D) -> Point2D:
    frame_is(point.frame_id, WORLD_FRAME, "point")
    frame_is(robot_pose.frame_id, WORLD_FRAME, "robot_pose")
    x, y = _rotate(point.x - robot_pose.x, point.y - robot_pose.y, -robot_pose.yaw)
    return Point2D(x, y, ROBOT_BASE_FRAME)


def body_to_world(point: Point2D, robot_pose: Pose2D) -> Point2D:
    frame_is(point.frame_id, ROBOT_BASE_FRAME, "point")
    frame_is(robot_pose.frame_id, WORLD_FRAME, "robot_pose")
    x, y = _rotate(point.x, point.y, robot_pose.yaw)
    return Point2D(x + robot_pose.x, y + robot_pose.y)
