"""Check scenario and behavior representation without cloning any episode."""
from __future__ import annotations

from collections import Counter, defaultdict
from statistics import median

from scenario_generator.src.scenario_generator import FAMILIES, ScenarioGenerator

from .plan import ProductionConfig, canonical_scenario_hash

BEHAVIORS = ("regular", "impassive", "surprised", "scared", "curious", "threatening")


def balance_preview(config: ProductionConfig, rows: list[dict[str, str]]) -> dict:
    generator = ScenarioGenerator(config.generator_config)
    densities = Counter()
    behaviors = Counter()
    human_counts: dict[str, dict[str, list[int]]] = defaultdict(lambda: defaultdict(list))
    seed_sets: dict[str, set[int]] = defaultdict(set)
    scenario_counts = Counter()
    for row in rows:
        family = row["scenario_family"]
        density = row["density"]
        seed = int(row["seed"])
        scenario = generator.sample(family, seed, density)
        if canonical_scenario_hash(scenario) != row["scenario_sha256"]:
            raise ValueError(f"Generator drift for {row['episode_id']}")
        scenario_counts[family] += 1
        densities[density] += 1
        seed_sets[family].add(seed)
        human_counts[family][density].append(len(scenario["humans"]))
        behaviors.update(h["behavior"] for h in scenario["humans"])
    warnings = []
    for index, family in enumerate(FAMILIES):
        target = config.target_episodes[f"S{index:02d}"]
        if scenario_counts[family] < target:
            warnings.append(f"{family}: {scenario_counts[family]} episodes below target {target}")
        if len(seed_sets[family]) < config.balance["min_seeds_per_scenario"]:
            warnings.append(f"{family}: too few distinct seeds")
        by_density = human_counts[family]
        for density in config.densities:
            if not by_density[density]:
                warnings.append(f"{family}: missing {density} density")
        values = [n for group in by_density.values() for n in group]
        if family != "empty" and len(set(values)) <= 1:
            warnings.append(f"{family}: human count has no variation")
        if by_density["low"] and by_density["high"] and median(by_density["high"]) < median(by_density["low"]):
            warnings.append(f"{family}: high-density human median below low-density median")
    for behavior in BEHAVIORS:
        if behaviors[behavior] < config.balance["min_behavior_count"]:
            warnings.append(f"{behavior}: rare behavior ({behaviors[behavior]})")
    return {
        "dataset_version": config.dataset_version,
        "planned_episodes": len(rows),
        "scenario_counts": dict(sorted(scenario_counts.items())),
        "density_counts": dict(sorted(densities.items())),
        "behavior_counts": {behavior: behaviors[behavior] for behavior in BEHAVIORS},
        "human_count_by_scenario_density": {
            family: {density: {"min": min(counts), "median": median(counts), "max": max(counts)}
                     for density, counts in sorted(by_density.items()) if counts}
            for family, by_density in sorted(human_counts.items())
        },
        "warnings": warnings,
    }
