from pathlib import Path
from shutil import copytree

import pytest
from omegaconf import OmegaConf

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.simulation.configuration import compose_config, validate_config

CONFIGS = Path(__file__).resolve().parents[2] / "configs"


def test_hydra_composes_defaults_and_resolves_references():
    config = compose_config(CONFIGS)
    episode = validate_config(config)
    assert (episode.scenario_id, episode.max_ticks) == ("head_on", 400)
    assert config.scenarios.world_id == config.worlds.id == "corridor"
    assert OmegaConf.is_readonly(config)


def test_hydra_overrides_group_seed_width_and_dt(tmp_path):
    configs = tmp_path / "configs"
    copytree(CONFIGS, configs)
    (configs / "worlds" / "alternate.yaml").write_text(
        "id: alternate\nlength_m: 12\nwidth_m: 3\nwall_height_m: 2\nwall_thickness_m: 0.1\n"
    )
    config = compose_config(
        configs,
        [
            "worlds=alternate",
            "worlds.width_m=2.8",
            "seed=123",
            "simulation.control_dt_ns=40000000",
        ],
    )
    episode = validate_config(config)
    assert config.scenarios.world_id == "alternate"
    assert config.worlds.width_m == 2.8
    assert (episode.episode_seed, episode.control_dt_ns, episode.max_ticks) == (
        123,
        40_000_000,
        500,
    )


@pytest.mark.parametrize(
    ("override", "error"),
    [
        ("~experiments.duration_ns", "experiments.duration_ns"),
        ("simulation.control_dt_ns=0", "control_dt_ns"),
        ("simulation.physics_dt_ns=3000000", "divisible"),
        ("experiments.duration_ns=0", "duration_ns"),
        ("worlds.width_m=0", "worlds.width_m"),
    ],
)
def test_invalid_config(override, error):
    with pytest.raises(ConfigurationError, match=error):
        compose_config(CONFIGS, [override])
