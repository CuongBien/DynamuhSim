from dataclasses import replace

import pytest

from narrow_nav.core.actions import HumanMotion, MotionBatch, RobotAction
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.types import Twist2D, Velocity2D
from narrow_nav.simulation import SimulationError
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco.geometry import inspect_mjcf
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_geometry import Circle, opening_clearance


def door_fixture(width_m, *, rotation=""):
    half_span = (2 - width_m / 2) / 2
    center = 2 - half_span
    return f"""
    <mujoco>
      <worldbody>
        <geom type="box" group="0" pos="0 0 -0.1" size="3 2 0.1"/>
        <geom type="box" group="1" pos="0 {-center} 0.5" size="0.25 {half_span} 0.5"/>
        <geom type="box" group="1" pos="0 {center} 0.5" size="0.25 {half_span} 0.5" {rotation}/>
        <geom type="box" group="1" pos="0 0 0.5" size="1 1 0.5" contype="0" conaffinity="0"/>
      </worldbody>
    </mujoco>
    """


@pytest.mark.parametrize(
    ("width_m", "radius_m", "pair_slack_m"),
    [
        (0.6, 0.25, -0.40),
        (0.9, 0.25, -0.10),
        (0.9, 0.18, 0.04),
        (1.2, 0.25, 0.20),
    ],
)
def test_compiled_opening_clearance_and_feasibility(width_m, radius_m, pair_slack_m):
    collision = inspect_mjcf(door_fixture(width_m))
    width = collision.clear_width(0)
    assert width == pytest.approx(width_m)
    assert len(collision.walls) == 2
    assert collision.contains(0, 0, radius_m)
    assert opening_clearance(width, radius_m, 0.25) == pytest.approx(pair_slack_m)


def test_narrow_throat_can_block_a_path_between_valid_endpoints():
    collision = inspect_mjcf(door_fixture(0.6))
    radius = 0.32
    assert collision.contains(-1, 0, radius)
    assert collision.contains(1, 0, radius)
    assert not collision.contains(0, 0, radius)
    assert opening_clearance(collision.clear_width(0), radius) == pytest.approx(-0.04)
    assert opening_clearance(0.5, 0.25) == 0
    with pytest.raises(ContractError, match="positive"):
        Circle(0, 0, -0.1)
    with pytest.raises(ContractError, match="positive"):
        opening_clearance(0.6, 0)


def test_rotated_collision_wall_rejected_in_planar_profile():
    with pytest.raises(ConfigurationError, match="axis-aligned"):
        inspect_mjcf(door_fixture(0.9, rotation='euler="0 0 15"'))


@pytest.fixture
def engine():
    runtime = MuJoCoEngine()
    runtime.load(compose_config())
    runtime.reset()
    yield runtime
    runtime.close()


@pytest.mark.parametrize(
    ("target", "value", "message"),
    [
        ("mocap_pos", (float("nan"), 0.0, 0.0), "invalid MuJoCo"),
        ("mocap_quat", (1.0, 0.0, 0.0, float("inf")), "invalid MuJoCo"),
        ("mocap_pos", (2.0, 0.2, 0.0), "stationary human mocap"),
        ("mocap_pos", (5.05, 0.2, 0.0), "stationary human mocap"),
        ("mocap_pos", (3.0, 0.2, 0.5), "stationary human mocap"),
    ],
    ids=("nan", "inf_quat", "teleport", "wall_penetration", "lift"),
)
def test_stationary_mocap_corruption_fails_without_publishing_tick(
    engine, target, value, message
):
    good = engine.stop()
    model, data = engine._adapter._native()
    mocap = model.body_mocapid[model.body("human_01").id]
    getattr(data, target)[mocap] = value
    with pytest.raises(SimulationError, match=message) as error:
        engine.stop()
    metadata = engine.get_metadata()
    assert metadata.failure == f"SimulationError: {error.value}"
    assert (metadata.status, metadata.last_valid_tick, metadata.failure_tick) == (
        "failed",
        1,
        2,
    )
    assert engine.get_ground_truth() == good.ground_truth
    assert engine.get_observation() == good.observation
    assert data.time == pytest.approx(0.05)
    engine.reset()
    assert engine.stop().ground_truth.humans[0].pose == good.ground_truth.humans[0].pose


def test_stationary_adapter_rejects_unexecutable_commands_before_stepping(engine):
    truth = engine.get_ground_truth()
    robot = RobotAction(truth.robot.entity_id, Twist2D(0, 0, 0), truth.stamp.timestamp)
    human = HumanMotion(
        truth.humans[0].entity_id, Velocity2D(0, 0), truth.stamp.timestamp
    )
    for batch, error in (
        (
            MotionBatch(replace(truth.stamp, episode_id="foreign:0"), robot, (human,)),
            "episode stamp",
        ),
        (MotionBatch(truth.stamp, robot), "configured actor IDs"),
        (
            MotionBatch(
                truth.stamp, robot, (replace(human, velocity=Velocity2D(1, 0)),)
            ),
            "zero human velocity",
        ),
    ):
        with pytest.raises(ContractError, match=error):
            engine._adapter.advance(batch)
    assert engine._adapter._data.time == 0
    assert engine.stop().ground_truth.stamp.timestamp.tick == 1


def test_solver_mask_excludes_stationary_human_wall_and_human_human(engine):
    model, _ = engine._adapter._native()
    human = model.geom("human_01/collision").id
    wall = model.geom("wall_0").id
    robot = model.geom("robot/chassis_collision").id

    def mask_allows(first, second):
        return bool(
            model.geom_contype[first] & model.geom_conaffinity[second]
            or model.geom_contype[second] & model.geom_conaffinity[first]
        )

    assert not mask_allows(human, wall)
    assert not mask_allows(human, human)
    assert mask_allows(human, robot)
