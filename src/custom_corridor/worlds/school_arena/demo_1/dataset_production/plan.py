"""Deterministic, duplicate-free episode plan for Dataset V1."""
from __future__ import annotations

import csv
import hashlib
import json
import os
import random
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from scenario_generator.src.scenario_generator import FAMILIES, GenerationError, ScenarioGenerator

FIELDS = (
    "episode_id", "scenario_family", "seed", "density", "status",
    "attempts", "scenario_sha256", "outcome", "reason",
)
IDENTITY_FIELDS = ("episode_id", "scenario_family", "seed", "density", "scenario_sha256")


@dataclass(frozen=True)
class ProductionConfig:
    path: Path
    dataset_version: str
    master_seed: int
    split_seed: int
    generator_config: Path
    output_dir: Path
    max_seed_attempts: int
    max_infrastructure_retries: int
    target_episodes: dict[str, int]
    densities: tuple[str, ...]
    split_ratios: dict[str, float]
    balance: dict[str, int]


def load_config(path: Path) -> ProductionConfig:
    path = Path(path).resolve()
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(doc, dict) or doc.get("dataset_version") != "v1.0":
        raise ValueError("Expected Dataset V1.0 production config")
    targets = doc.get("target_episodes")
    expected = {f"S{i:02d}" for i in range(len(FAMILIES))}
    if (not isinstance(targets, dict) or set(targets) != expected
            or any(isinstance(n, bool) or not isinstance(n, int) or n < 1
                   for n in targets.values())):
        raise ValueError("target_episodes must contain positive counts for S00–S23")
    densities = tuple(doc.get("densities", ()))
    if set(densities) != {"low", "medium", "high"} or len(densities) != 3:
        raise ValueError("densities must contain low, medium and high exactly once")
    ratios = doc.get("split_ratios")
    if (not isinstance(ratios, dict) or set(ratios) != {"train", "val", "test"}
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0
                   for v in ratios.values()) or abs(sum(ratios.values()) - 1.0) > 1e-9):
        raise ValueError("split_ratios must be positive and sum to one")
    for key in ("master_seed", "split_seed", "max_seed_attempts", "max_infrastructure_retries"):
        value = doc.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value < (0 if key == "max_infrastructure_retries" else 1):
            raise ValueError(f"Invalid {key}")
    if not isinstance(doc.get("balance"), dict):
        raise ValueError("Missing balance thresholds")
    generator_config = (path.parent / doc["generator_config"]).resolve()
    output_dir = (path.parent / doc["output_dir"]).resolve()
    if not generator_config.is_file():
        raise ValueError(f"Missing generator config: {generator_config}")
    return ProductionConfig(
        path=path, dataset_version=doc["dataset_version"],
        master_seed=doc["master_seed"], split_seed=doc["split_seed"],
        generator_config=generator_config, output_dir=output_dir,
        max_seed_attempts=doc["max_seed_attempts"],
        max_infrastructure_retries=doc["max_infrastructure_retries"],
        target_episodes=targets, densities=densities,
        split_ratios=ratios, balance=doc["balance"],
    )


def canonical_scenario_hash(scenario: dict[str, Any]) -> str:
    """Hash actual sampled configuration, excluding its registry seed."""
    comparable = {key: value for key, value in scenario.items() if key != "seed"}
    canonical = json.dumps(comparable, sort_keys=True, separators=(",", ":"),
                           ensure_ascii=False, allow_nan=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def make_plan(config: ProductionConfig, generator: ScenarioGenerator | None = None) -> list[dict[str, str]]:
    generator = generator or ScenarioGenerator(config.generator_config)
    rng = random.Random(config.master_seed)
    seen_seeds: set[int] = set()
    seen_configs: set[str] = set()
    rows = []
    for index, family in enumerate(FAMILIES):
        target = config.target_episodes[f"S{index:02d}"]
        density_schedule = [config.densities[i % len(config.densities)] for i in range(target)]
        rng.shuffle(density_schedule)
        for density in density_schedule:
            for _ in range(config.max_seed_attempts):
                seed = rng.randrange(1, 2**31)
                if seed in seen_seeds:
                    continue
                try:
                    scenario = generator.sample(family, seed, density)
                except GenerationError:
                    continue
                digest = canonical_scenario_hash(scenario)
                if digest in seen_configs:
                    continue
                seen_seeds.add(seed)
                seen_configs.add(digest)
                break
            else:
                raise RuntimeError(f"Cannot find unique {family}/{density} configuration")
            rows.append({
                "episode_id": f"ep_{len(rows)+1:06d}",
                "scenario_family": family, "seed": str(seed), "density": density,
                "status": "PLANNED", "attempts": "0", "scenario_sha256": digest,
                "outcome": "", "reason": "",
            })
    validate_plan(rows)
    return rows


def validate_plan(rows: list[dict[str, str]]) -> None:
    ids = [row["episode_id"] for row in rows]
    identities = [(row["scenario_family"], row["seed"], row["density"]) for row in rows]
    configs = [row["scenario_sha256"] for row in rows]
    if len(ids) != len(set(ids)) or len(identities) != len(set(identities)) or len(configs) != len(set(configs)):
        raise ValueError("Duplicate episode ID, identity or sampled configuration")


def read_plan(path: Path) -> list[dict[str, str]]:
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != FIELDS:
            raise ValueError("Production plan columns differ from expected schema")
        rows = list(reader)
    validate_plan(rows)
    return rows


def write_plan(path: Path, rows: list[dict[str, str]]) -> None:
    validate_plan(rows)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".production_plan_", suffix=".csv",
                                             dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def check_plan_identity(stored: list[dict[str, str]], expected: list[dict[str, str]]) -> None:
    validate_plan(stored)
    if len(stored) < len(expected) or any(
        tuple(a[key] for key in IDENTITY_FIELDS) != tuple(b[key] for key in IDENTITY_FIELDS)
        for a, b in zip(stored[:len(expected)], expected)
    ):
        raise ValueError("Stored production plan differs from current config/master seed/generator")


def extend_for_deficits(config: ProductionConfig, rows: list[dict[str, str]],
                        generator: ScenarioGenerator | None = None) -> list[dict[str, str]]:
    """Add fresh seeds only after a production pass has reached terminal states."""
    if any(row["status"] not in {"PASS", "FAIL_INFRASTRUCTURE"} for row in rows):
        raise ValueError("Cannot fill coverage while episodes remain unfinished")
    generator = generator or ScenarioGenerator(config.generator_config)
    result = [dict(row) for row in rows]
    seen_seeds = {int(row["seed"]) for row in rows}
    seen_configs = {row["scenario_sha256"] for row in rows}
    for index, family in enumerate(FAMILIES):
        target = config.target_episodes[f"S{index:02d}"]
        existing = [row for row in result if row["scenario_family"] == family]
        accepted = [row for row in existing if row["status"] == "PASS"]
        deficit = max(0, target - len(accepted))
        density_counts = {name: sum(row["density"] == name for row in accepted)
                          for name in config.densities}
        for supplement in range(deficit):
            density = min(config.densities, key=lambda name: (density_counts[name],
                                                              config.densities.index(name)))
            ordinal = len(existing) + supplement
            for attempt in range(config.max_seed_attempts):
                material = f"{config.master_seed}:supplement:{family}:{ordinal}:{attempt}"
                seed = int.from_bytes(hashlib.sha256(material.encode()).digest()[:8], "big") % (2**31-1) + 1
                if seed in seen_seeds:
                    continue
                try:
                    scenario = generator.sample(family, seed, density)
                except GenerationError:
                    continue
                digest = canonical_scenario_hash(scenario)
                if digest in seen_configs:
                    continue
                seen_seeds.add(seed)
                seen_configs.add(digest)
                break
            else:
                raise RuntimeError(f"Cannot fill {family}/{density} with unique seed")
            result.append({
                "episode_id": f"ep_{len(result)+1:06d}",
                "scenario_family": family, "seed": str(seed), "density": density,
                "status": "PLANNED", "attempts": "0", "scenario_sha256": digest,
                "outcome": "", "reason": "",
            })
            density_counts[density] += 1
    validate_plan(result)
    return result
