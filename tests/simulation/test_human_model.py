import mujoco
import pytest

from narrow_nav.simulation.configuration import compose_config
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
    model = mujoco.MjModel.from_xml_string(build_world(config).mjcf)
    data = mujoco.MjData(model)
    first, second = (model.body(name).mocapid[0] for name in ("human_01", "human_02"))
    assert model.nmocap == 2 and first != second
    assert model.geom("human_02/collision").size[0] == pytest.approx(0.31)
    data.mocap_pos[second] = (-2.6, -0.2, 0.0)
    mujoco.mj_forward(model, data)
    assert tuple(data.body("human_02").xpos) == pytest.approx((-2.6, -0.2, 0))
    assert tuple(data.body("human_01").xpos) == pytest.approx((3, 0.2, 0))
    human = model.geom("human_02/collision").id
    robot = model.geom("robot/chassis_collision").id
    assert any(
        {contact.geom1, contact.geom2} == {human, robot} for contact in data.contact
    )
