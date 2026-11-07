from math import pi

import pytest

from narrow_nav.core.geometry import body_to_world, world_to_body, wrap_yaw
from narrow_nav.core.types import Point2D, Pose2D


def test_frame_transforms_round_trip_and_yaw_wraps():
    pose = Pose2D(3, 4, pi / 2)
    point = Point2D(3, 6)
    body_point = world_to_body(point, pose)
    assert body_point.frame_id == "robot_base"
    assert (body_point.x, body_point.y) == pytest.approx((2, 0))
    assert body_to_world(body_point, pose) == point
    assert wrap_yaw(3 * pi) == -pi
