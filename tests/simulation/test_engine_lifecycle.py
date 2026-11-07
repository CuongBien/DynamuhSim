import hashlib
import weakref

import pytest

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.types import Twist2D
from narrow_nav.simulation import SimulationError
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine


def test_reset_replays_and_clears_state_and_close_releases_native_objects():
    engine = MuJoCoEngine()
    with pytest.raises(SimulationError, match="load"):
        engine.reset()
    engine.load(compose_config())
    model = weakref.ref(engine._adapter._model)
    data = weakref.ref(engine._adapter._data)
    observations, traces, ids = [], [], []
    for _ in range(2):
        observation = engine.reset(17, "head_on")
        metadata = engine.get_metadata()
        assert metadata.episode.config.episode_seed == 17
        assert (
            metadata.episode.config_sha256
            == hashlib.sha256(metadata.resolved_config.encode()).hexdigest()
        )
        assert metadata.last_valid_tick == 0
        assert (
            len(metadata.model_sha256) == 64 and len(metadata.mjcf_source_sha256) == 3
        )
        ids.append(metadata.episode.episode_id)
        observations.append(observation)
        trace = []
        for _ in range(16):
            robot = engine.get_ground_truth().robot
            result = engine.step(
                RobotAction(robot.entity_id, Twist2D(0.4, 0, 0.15), robot.timestamp)
            )
            pose = result.ground_truth.robot.pose
            velocity = result.executed.robot.velocity
            trace.append(
                (
                    pose.x,
                    pose.y,
                    pose.yaw,
                    velocity.linear_x,
                    velocity.linear_y,
                    velocity.angular_z,
                )
            )
        traces.append(trace)
    assert ids[0] != ids[1]
    assert observations[0].sensors == observations[1].sensors
    assert all(
        new == pytest.approx(old, rel=0, abs=1e-9)
        for old, new in zip(*traces, strict=True)
    )
    engine.close()
    engine.close()
    assert model() is None and data() is None
    with pytest.raises(SimulationError, match="closed"):
        engine.load(compose_config())


def test_invalid_action_is_retryable_but_failed_physics_requires_reset():
    engine = MuJoCoEngine()
    engine.load(compose_config())
    engine.reset()
    with pytest.raises(ConfigurationError, match="scenario"):
        engine.reset(scenario="blind_corner")
    state = engine.get_ground_truth()
    with pytest.raises(ContractError, match="current robot"):
        engine.step(RobotAction("wrong", Twist2D(0, 0, 0), state.stamp.timestamp))
    assert engine.get_metadata().status == "running"
    result = engine.stop()
    assert result.ground_truth.stamp.timestamp.tick == 1
    # A native non-finite input must not be silently published as a valid tick.
    data = engine._adapter._data
    data.qvel[0] = float("nan")
    with pytest.raises(SimulationError, match="invalid MuJoCo"):
        engine.stop()
    failed = engine.get_metadata()
    assert (failed.status, failed.last_valid_tick, failed.failure_tick) == (
        "failed",
        1,
        2,
    )
    assert engine.get_ground_truth() == result.ground_truth
    with pytest.raises(SimulationError, match="reset"):
        engine.stop()
    engine.reset()
    assert engine.get_metadata().status == "running"
    assert engine.stop().ground_truth.stamp.timestamp.tick == 1
    engine.close()
