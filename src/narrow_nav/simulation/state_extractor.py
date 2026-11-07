"""Compute executed motion from two immutable ground-truth snapshots."""

from narrow_nav.core.actions import HumanMotion, MotionBatch, RobotAction
from narrow_nav.core.geometry import world_to_body
from narrow_nav.core.states import GroundTruthState
from narrow_nav.core.types import NS_PER_SECOND, Point2D, Twist2D, Velocity2D


def executed_motion(
    start: GroundTruthState, end: GroundTruthState, robot_delta_yaw: float
) -> MotionBatch:
    stamp = start.stamp
    dt = (end.stamp.timestamp.time_ns - stamp.timestamp.time_ns) / NS_PER_SECOND
    pose = end.robot.pose
    delta = world_to_body(Point2D(pose.x, pose.y), start.robot.pose)
    robot = RobotAction(
        start.robot.entity_id,
        Twist2D(delta.x / dt, delta.y / dt, robot_delta_yaw / dt),
        stamp.timestamp,
    )
    humans = tuple(
        HumanMotion(
            before.entity_id,
            Velocity2D(
                (after.pose.x - before.pose.x) / dt, (after.pose.y - before.pose.y) / dt
            ),
            stamp.timestamp,
        )
        for before, after in zip(start.humans, end.humans, strict=True)
    )
    return MotionBatch(stamp, robot, humans)
