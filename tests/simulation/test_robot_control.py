from math import hypot

import mujoco
import pytest

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ContractError
from narrow_nav.core.types import Twist2D
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import build_world


def _run(engine, velocity, ticks):
    for _ in range(ticks):
        robot = engine.get_ground_truth().robot
        result = engine.step(RobotAction(robot.entity_id, velocity, robot.timestamp))
    return result


def test_differential_limits_drive_and_stop():
    engine = MuJoCoEngine()
    engine.load(compose_config(overrides=("robots.wheel_speed_limit_radps=5",)))
    engine.reset()
    initial = engine.get_ground_truth().robot
    _run(engine, Twist2D(0, 0, 0), 40)
    idle = engine.get_ground_truth().robot
    assert hypot(idle.pose.x - initial.pose.x, idle.pose.y - initial.pose.y) < 1e-4
    with pytest.raises(ContractError, match="linear_y"):
        _run(engine, Twist2D(0, 0.2, 0), 1)
    first = _run(engine, Twist2D(10, 0, 0), 1)
    assert first.requested.robot.velocity.linear_x == 10
    assert first.limited.robot.velocity.linear_x == pytest.approx(0.05)
    forward = _run(engine, Twist2D(10, 0, 0), 19)
    assert forward.limited.robot.velocity.linear_x == pytest.approx(0.35)
    assert 0.2 < forward.ground_truth.robot.pose.x - initial.pose.x < 0.4
    assert abs(forward.ground_truth.robot.pose.y - initial.pose.y) < 0.01
    assert forward.executed.robot.velocity.linear_x > 0.25
    stopped = engine.stop()
    assert stopped.requested.robot.velocity.linear_x == 0
    assert stopped.limited.robot.velocity.linear_x == pytest.approx(0.30)
    for _ in range(20):
        stopped = engine.stop()
    assert stopped.limited.robot.velocity.linear_x == 0
    assert abs(stopped.executed.robot.velocity.linear_x) < 0.01

    engine.reset()
    turn = _run(engine, Twist2D(0, 0, 0.7), 30)
    assert turn.limited.robot.velocity.angular_z == pytest.approx(0.7)
    assert turn.ground_truth.robot.pose.yaw > 0.6
    assert turn.executed.robot.velocity.angular_z > 0.5
    engine.close()


def test_mecanum_sdf_dimensions_and_physical_strafe():
    config = compose_config(overrides=("robots=mecanum_robot",))
    model = mujoco.MjSpec.from_string(build_world(config).mjcf).compile()
    assert model.nu == 4 and model.njnt == 53
    assert model.body_subtreemass[model.body("robot").id] == pytest.approx(4.7)
    assert tuple(model.geom("robot/chassis_collision").size) == pytest.approx(
        (0.12, 0.10, 0.04)
    )
    assert tuple(model.body("robot/front_left_wheel").pos) == pytest.approx(
        (0.08, 0.10, 0)
    )
    assert tuple(model.cam_resolution[model.camera("robot/depth_camera").id]) == (
        424,
        240,
    )
    assert (
        model.site("robot/base_scan_origin").id >= 0
        and model.site("robot/imu_link_origin").id >= 0
    )
    engine = MuJoCoEngine()
    engine.load(config)
    engine.reset()
    initial = engine.get_ground_truth().robot.pose
    strafe = _run(engine, Twist2D(0, 0.4, 0), 20)
    pose = strafe.ground_truth.robot.pose
    assert pose.y - initial.y > 0.25
    assert abs(pose.x - initial.x) < 0.02 and abs(pose.yaw) < 0.05
    assert strafe.executed.robot.velocity.linear_y > 0.25
    engine.reset()
    turn = _run(engine, Twist2D(0, 0, 0.8), 20)
    assert turn.ground_truth.robot.pose.yaw > 0.5
    engine.close()
