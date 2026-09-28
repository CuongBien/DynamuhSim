"""Materialize planned episode inputs with atomic, resume-safe writes."""
from __future__ import annotations

import os
import shutil
import tempfile
from pathlib import Path

import yaml

from dataset_runner.episode_loader import EpisodeLoader
from scenario_generator.src.scenario_generator import ScenarioGenerator

from .plan import ProductionConfig, canonical_scenario_hash


def ensure_episode(config: ProductionConfig, row: dict[str, str],
                   generator: ScenarioGenerator | None = None) -> bool:
    """Return True when an episode is generated; verify and skip existing data."""
    generator = generator or ScenarioGenerator(config.generator_config)
    episode_id = row["episode_id"]
    root = config.output_dir / "episodes"
    root.mkdir(parents=True, exist_ok=True)
    destination = root / episode_id
    if destination.exists():
        saved = yaml.safe_load((destination / "scenario.yaml").read_text(encoding="utf-8"))
        if (saved.get("scenario_family") != row["scenario_family"]
                or saved.get("seed") != int(row["seed"])
                or saved.get("density") != row["density"]
                or canonical_scenario_hash(saved) != row["scenario_sha256"]):
            raise ValueError(f"Existing episode differs from plan: {episode_id}")
        EpisodeLoader().load(destination)
        for filename in ("school_floor.world", "nav2_school.yaml"):
            if not (destination / filename).is_file():
                raise ValueError(f"Incomplete episode {episode_id}: missing {filename}")
        return False
    scenario = generator.sample(row["scenario_family"], int(row["seed"]), row["density"])
    if canonical_scenario_hash(scenario) != row["scenario_sha256"]:
        raise ValueError(f"Generator drift for {episode_id}")
    temporary = Path(tempfile.mkdtemp(prefix=f".{episode_id}.", dir=root))
    try:
        # ScenarioGenerator.write requires a directory it creates itself.
        temporary.rmdir()
        generator.write(scenario, temporary, episode_id)
        os.replace(temporary, destination)
        EpisodeLoader().load(destination)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return True
