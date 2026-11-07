from math import isfinite

import pytest

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine


def test_timing_grid_and_1000_headless_physics_steps():
    engine = MuJoCoEngine()
    engine.load(
        compose_config(
            overrides=(
                "simulation.physics_dt_ns=5000000",
                "simulation.human_control_dt_ns=65000000",
                "simulation.observation_dt_ns=15000000",
                "experiments.duration_ns=5000000000",
            )
        )
    )
    engine.reset()
    for tick in range(1, 101):
        result = engine.stop()
        stamp = result.ground_truth.stamp.timestamp
        metadata = engine.get_metadata()
        assert (stamp.tick, stamp.time_ns) == (tick, tick * 50_000_000)
        assert metadata.clocks.physics_steps == tick * 10
        assert (
            metadata.clocks.last_human_control_time_ns
            == stamp.time_ns // 65_000_000 * 65_000_000
        )
        assert (
            metadata.clocks.last_observation_time_ns
            == stamp.time_ns // 15_000_000 * 15_000_000
        )
        velocity = result.ground_truth.robot.velocity
        assert all(
            map(isfinite, (velocity.linear_x, velocity.linear_y, velocity.angular_z))
        )
    assert result.truncated and result.end_reason == "time_budget"
    engine.close()


@pytest.mark.parametrize("period", ["human_control_dt_ns", "observation_dt_ns"])
def test_incompatible_period_rejected(period):
    with pytest.raises(ConfigurationError, match=period):
        compose_config(overrides=(f"simulation.{period}=51000000",))
