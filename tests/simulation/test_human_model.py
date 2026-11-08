import mujoco
import numpy as np
import pytest

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.types import Twist2D
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco.human_visuals import HumanVisuals
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import MJCF_ROOT, build_world


def test_pedestrian_template_has_distinct_visual_and_collision():
    model = mujoco.MjModel.from_xml_path(
        str(MJCF_ROOT / "models/humans/pedestrian.xml")
    )
    assert (model.njnt, model.nmocap) == (0, 1)
    assert model.body("pedestrian").mocapid[0] == 0
    collision, visual = model.geom("collision"), model.geom("visual")
    assert collision.size[0] == pytest.approx(0.25)
    assert collision.contype[0] != 0
    assert (visual.contype[0], visual.conaffinity[0]) == (0, 0)


def test_multiple_pedestrians_have_unique_mocap_ids_and_configured_radius():
    config = compose_config(
        overrides=[
            "scenarios.human_profile.radius_m=0.31",
            (
                "scenarios.humans=[{id:human_01,initial_pose:[3,0.2,0],goal:[2,0.2]},"
                "{id:human_02,initial_pose:[0,0.2,0],goal:[1,0.2]}]"
            ),
        ]
    )
    built = build_world(config)
    model = mujoco.MjModel.from_xml_string(built.mjcf, dict(built.visual_assets))
    data = mujoco.MjData(model)
    first, second = (model.body(name).mocapid[0] for name in ("human_01", "human_02"))
    assert first != second and min(first, second) >= 0
    assert model.nskin == sum(len(actor.character.parts) for actor in built.skin_actors)
    assert tuple(actor.character.identifier for actor in built.skin_actors) == (
        "casual_male",
        "casual_female",
    )
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "human_02/visual") == -1
    assert model.geom("human_02/collision").group.item() == 3
    assert model.geom("human_02/collision").size[0] == pytest.approx(0.31)
    visuals = HumanVisuals(model, built.skin_actors)
    mujoco.mj_forward(model, data)
    before = data.body("hv/human_02/bone_8").xpos.copy()
    data.mocap_pos[second] = (-2.6, -0.2, 0.0)
    mujoco.mj_forward(model, data)
    visuals.update(model, data, 0)
    np.testing.assert_allclose(
        data.body("hv/human_02/bone_8").xpos - before, [-2.6, -0.4, 0], atol=1e-8
    )
    assert tuple(data.body("human_02").xpos) == pytest.approx((-2.6, -0.2, 0))
    assert tuple(data.body("human_01").xpos) == pytest.approx((3, 0.2, 0))
    human = model.geom("human_02/collision").id
    robot = model.geom("robot/chassis_collision").id
    assert any(
        {contact.geom1, contact.geom2} == {human, robot} for contact in data.contact
    )


def test_quaternius_idle_resets_and_preserves_physics_and_contact():
    traces = []
    for visual in ("quaternius", "proxy"):
        engine = MuJoCoEngine()
        try:
            engine.load(
                compose_config(
                    overrides=[
                        f"simulation.human_visuals={visual}",
                        "scenarios.robot.initial_pose=[-1,0,0]",
                        "scenarios.humans.0.initial_pose=[0,0,0]",
                    ]
                )
            )
            engine.reset()
            model, data = engine._adapter._native()
            skinned = visual == "quaternius"
            assert bool(model.nskin) == skinned
            if skinned:
                mocap = data.mocap_quat.copy()
            trace = []
            for _ in range(65):
                robot = engine.get_ground_truth().robot
                result = engine.step(
                    RobotAction(robot.entity_id, Twist2D(0.65, 0, 0), robot.timestamp)
                )
                truth = result.ground_truth
                trace.append(
                    (
                        truth.robot.pose,
                        truth.robot.velocity,
                        truth.humans,
                        result.contacts,
                    )
                )
                if result.terminated:
                    break
            assert result.end_reason == "robot_contact"
            traces.append(trace)
            if skinned:
                assert not np.allclose(data.mocap_quat, mocap)
                engine.reset()
                np.testing.assert_array_equal(data.mocap_quat, mocap)
        finally:
            engine.close()
    assert traces[0] == traces[1]


def test_hospital_people_replace_only_the_visual_part_of_static_props():
    built = build_world(
        compose_config(overrides=["worlds=hospital_easy", "scenarios=hospital_easy"])
    )
    model = mujoco.MjModel.from_xml_string(built.mjcf, dict(built.visual_assets))
    assert len(built.scene.humans) == 6
    assert len(built.scene.human_props) == 10
    assert len(built.skin_actors) == 16
    for prop in built.scene.human_props:
        geom = model.geom(prop.entity_id)
        assert geom.bodyid.item() == 0
        assert geom.group.item() == 3
        assert geom.contype.item() != 0
        assert geom.rgba[-1] > 0
        assert model.skin(f"hv/{prop.entity_id}/skin_0").id >= 0
    assert sum(actor.seat_height_m is not None for actor in built.skin_actors) == 10
