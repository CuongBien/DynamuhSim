from math import atan2, hypot

import mujoco
import pytest

from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.world_builder import MJCF_ROOT, build_world


def _model(*overrides):
    return mujoco.MjModel.from_xml_string(
        build_world(compose_config(overrides=overrides)).mjcf
    )


def test_robot_joint_actuator_and_collision_structure():
    model = mujoco.MjModel.from_xml_path(
        str(MJCF_ROOT / "models/robots/mobile_robot/robot.xml")
    )
    assert (model.njnt, model.nu) == (3, 2)
    assert model.joint("base_free").type[0] == mujoco.mjtJoint.mjJNT_FREE
    for side in ("left", "right"):
        joint = model.joint(f"{side}_axle")
        actuator = model.actuator(f"{side}_drive")
        assert joint.type[0] == mujoco.mjtJoint.mjJNT_HINGE
        assert tuple(joint.axis) == (0, 1, 0)
        assert actuator.trnid[0] == joint.id
        assert actuator.trntype[0] == mujoco.mjtTrn.mjTRN_JOINT
        assert tuple(actuator.forcerange) == pytest.approx((-1.5, 1.5))
        assert actuator.forcelimited[0] and actuator.ctrllimited[0]
    assert model.geom("chassis_collision").contype[0] != 0
    visual = model.geom("chassis_visual")
    assert (visual.contype[0], visual.conaffinity[0]) == (0, 0)


def test_robot_physical_dimensions_follow_config():
    model = _model(
        "robots.radius_m=0.27",
        "robots.mass_kg=9.0",
        "robots.wheel_radius_m=0.08",
        "robots.wheel_separation_m=0.36",
    )
    assert model.geom("robot/chassis_collision").size[0] == pytest.approx(0.27)
    assert model.geom("robot/left_tire").size[0] == pytest.approx(0.08)
    assert model.body("robot").pos[2] == pytest.approx(0.08)
    left = model.body("robot/left_wheel")
    right = model.body("robot/right_wheel")
    assert left.pos[1] - right.pos[1] == pytest.approx(0.36)
    assert model.body_subtreemass[model.body("robot").id] == pytest.approx(9.0)


def test_differential_drive_advances_and_turns_under_contact():
    model = _model()
    assert model.opt.timestep == pytest.approx(0.002)
    for command in ((0, 0), (6, 6), (-4, 4)):
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        start_x, start_y, _ = data.body("robot").xpos.copy()
        data.ctrl[:] = command
        mujoco.mj_step(model, data, nstep=500)
        mujoco.mj_forward(model, data)
        x, y, z = data.body("robot").xpos
        yaw = atan2(data.body("robot").xmat[3], data.body("robot").xmat[0])
        distance = hypot(x - start_x, y - start_y)
        assert data.time == pytest.approx(1.0)
        assert z == pytest.approx(0.07, abs=0.002)
        assert data.ncon > 0
        if command == (0, 0):
            assert distance < 1e-4
        elif command == (6, 6):
            assert 0.30 < x - start_x < 0.47
            assert abs(y - start_y) < 0.01 and abs(yaw) < 0.03
        else:
            assert 1.0 < yaw < 1.65 and distance < 0.02


def test_robot_is_physically_blocked_by_wall():
    model = _model(
        "scenarios.robot.initial_pose=[4.2,-0.2,0]", "worlds.wall_height_m=1.7"
    )
    data = mujoco.MjData(model)
    data.ctrl[:] = (6, 6)
    mujoco.mj_step(model, data, nstep=1250)
    mujoco.mj_forward(model, data)
    assert data.body("robot").xpos[0] == pytest.approx(4.75, abs=0.006)
    wall_id = model.geom("wall_1").id
    assert data.geom_xpos[wall_id, 2] + model.geom_size[wall_id, 2] == pytest.approx(
        1.7
    )
    chassis_id = model.geom("robot/chassis_collision").id
    assert any(
        {contact.geom1, contact.geom2} == {wall_id, chassis_id}
        for contact in data.contact
    )
