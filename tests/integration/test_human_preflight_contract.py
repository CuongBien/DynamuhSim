from dataclasses import replace

import pytest

from narrow_nav.core.errors import ContractError
from narrow_nav.core.observations import Observation
from narrow_nav.core.types import Velocity2D
from narrow_nav.simulation import SimulationError
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine


@pytest.fixture
def engine():
    runtime = MuJoCoEngine()
    runtime.load(compose_config(overrides=("experiments.duration_ns=5000000000",)))
    yield runtime
    runtime.close()


def test_default_clocks_stationary_hold_and_sensor_boundary_replay(engine):
    episode_ids = set()
    for _ in range(2):
        observation = engine.reset(seed=17)
        metadata = engine.get_metadata()
        episode_ids.add(metadata.episode.episode_id)
        assert (metadata.last_valid_tick, metadata.clocks.physics_steps) == (0, 0)
        assert metadata.human_motion_mode == "stationary"
        assert isinstance(observation, Observation)
        assert [
            (slot.sensor_id, slot.status, slot.reason, slot.sample)
            for slot in observation.sensors
        ] == [
            (sensor, "missing", "capability_unavailable", None)
            for sensor in ("depth", "imu", "lidar", "odometry")
        ]
        spawn = engine.get_ground_truth().humans[0].pose
        gt_export_ticks = []
        for tick in range(1, 101):
            result = engine.stop()
            clock = engine.get_metadata().clocks
            stamp = result.ground_truth.stamp.timestamp
            assert (stamp.tick, stamp.time_ns) == (tick, 50_000_000 * tick)
            assert clock.physics_steps == 25 * tick
            assert clock.last_human_control_time_ns == 100_000_000 * (tick // 2)
            assert clock.last_observation_time_ns == stamp.time_ns
            assert result.observation.sensors == observation.sensors
            assert not result.sensor_samples
            assert result.ground_truth.humans[0].pose == spawn
            (motion,) = result.human_motion_results
            assert motion.requested == motion.limited == motion.executed
            assert motion.executed.velocity == Velocity2D(0, 0)
            if stamp.time_ns % 100_000_000 == 0:
                gt_export_ticks.append(tick)
        assert gt_export_ticks == list(range(2, 101, 2))
        assert result.truncated and result.end_reason == "time_budget"
    assert len(episode_ids) == 2


@pytest.mark.parametrize("field", ["episode", "actors"])
def test_reset_rejects_foreign_adapter_snapshots(engine, monkeypatch, field):
    reset = engine._adapter.reset

    def corrupt_reset(*args):
        snapshot = reset(*args)
        truth = snapshot.ground_truth
        if field == "episode":
            truth = replace(truth, stamp=replace(truth.stamp, episode_id="foreign:0"))
        else:
            truth = replace(truth, humans=())
        return replace(snapshot, ground_truth=truth)

    with monkeypatch.context() as patch:
        patch.setattr(engine._adapter, "reset", corrupt_reset)
        with pytest.raises(ContractError, match="adapter snapshot"):
            engine.reset()
    metadata = engine.get_metadata()
    assert (metadata.status, metadata.end_reason, metadata.failure_tick) == (
        "failed",
        "reset_failed",
        0,
    )
    assert metadata.last_valid_tick is None
    with pytest.raises(SimulationError, match="no valid"):
        engine.get_observation()
    assert engine.reset().stamp.episode_id != metadata.episode.episode_id


def test_advance_rejects_foreign_snapshot_and_keeps_last_good_tick(engine, monkeypatch):
    engine.reset()
    good = engine.stop()
    advance = engine._adapter.advance

    def corrupt_step(limited):
        step = advance(limited)
        truth = step.snapshot.ground_truth
        return replace(
            step,
            snapshot=replace(step.snapshot, ground_truth=replace(truth, humans=())),
        )

    monkeypatch.setattr(engine._adapter, "advance", corrupt_step)
    with pytest.raises(ContractError, match="configured actor IDs"):
        engine.stop()
    metadata = engine.get_metadata()
    assert (
        metadata.status,
        metadata.end_reason,
        metadata.last_valid_tick,
        metadata.failure_tick,
    ) == ("failed", "step_failed", 1, 2)
    assert "ContractError" in metadata.failure
    assert engine.get_ground_truth() == good.ground_truth
    assert engine.get_observation() == good.observation
    with pytest.raises(SimulationError, match="reset"):
        engine.stop()
