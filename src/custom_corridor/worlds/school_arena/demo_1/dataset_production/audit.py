"""Production registry audit. Incomplete output is explicitly marked unfrozen."""
from __future__ import annotations

import csv
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median

import yaml

from dataset_runner.episode_loader import EpisodeLoader
from scenario_generator.src.scenario_generator import FAMILIES

from .balance import BEHAVIORS
from .integrity import check_accepted
from .plan import ProductionConfig, canonical_scenario_hash, read_plan, validate_plan


def _distribution(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "min": None, "median": None, "max": None}
    return {"count": len(values), "min": min(values), "median": median(values), "max": max(values)}


def audit(config: ProductionConfig, *, write: bool = True) -> dict:
    """Audit the registry and input traceability; never infer QA PASS from status alone."""
    root = config.output_dir
    rows = read_plan(root / "production_plan.csv")
    validate_plan(rows)
    accepted = [row for row in rows if row["status"] == "PASS"]
    failed = [row for row in rows if row["status"] == "FAIL_INFRASTRUCTURE"]
    status = Counter(row["status"] for row in rows)
    scenario = Counter(row["scenario_family"] for row in accepted)
    density = Counter(row["density"] for row in accepted)
    outcomes = Counter(row["outcome"] for row in accepted)
    humans = []
    behaviors = Counter()
    seeds = defaultdict(set)
    issues = []
    validated = {}
    for row in rows:
        episode_id = row["episode_id"]
        folder = root / "episodes" / episode_id
        if not folder.is_dir():
            if row["status"] == "PASS":
                issues.append(f"{episode_id}: missing episode directory")
            continue
        try:
            context = EpisodeLoader().load(folder)
            saved = context.scenario
            if (context.episode_id != episode_id or context.scenario_family != row["scenario_family"]
                    or context.seed != int(row["seed"])
                    or context.metadata.get("density") != row["density"]
                    or canonical_scenario_hash(saved) != row["scenario_sha256"]):
                raise ValueError("metadata/scenario differs from production plan")
            if row["status"] == "PASS":
                humans.append(context.metadata["humans"]["total"])
                behaviors.update(h["behavior"] for h in context.scenario["humans"])
                seeds[row["scenario_family"]].add(row["seed"])
                validated[episode_id] = check_accepted(folder, row)
        except Exception as exc:
            issues.append(f"{episode_id}: {exc}")
    known = {row["episode_id"] for row in rows}
    episode_root = root / "episodes"
    orphans = sorted(p.name for p in episode_root.iterdir() if p.is_dir() and
                     not p.name.startswith(".") and p.name not in known) if episode_root.exists() else []
    if orphans:
        issues.append(f"orphan episode directories: {', '.join(orphans[:20])}")
    warnings = []
    for index, family in enumerate(FAMILIES):
        target = config.target_episodes[f"S{index:02d}"]
        if scenario[family] < target:
            warnings.append(f"{family}: accepted {scenario[family]} below target {target}")
        if len(seeds[family]) < config.balance["min_seeds_per_scenario"]:
            warnings.append(f"{family}: only {len(seeds[family])} accepted seeds")
    for level in config.densities:
        if density[level] == 0:
            warnings.append(f"density {level} absent from accepted episodes")
    for behavior in BEHAVIORS:
        if behaviors[behavior] < config.balance["min_behavior_count"]:
            warnings.append(f"behavior {behavior} rare: {behaviors[behavior]}")
    if humans and len(set(humans)) <= 1:
        warnings.append("accepted human counts have no variation")
    manifest_path = root / "manifest.csv"
    manifest_rows = []
    if manifest_path.exists():
        with manifest_path.open(newline="", encoding="utf-8") as stream:
            manifest_rows = list(csv.DictReader(stream))
        manifest_ids = [r.get("episode_id", "") for r in manifest_rows]
        if len(manifest_ids) != len(set(manifest_ids)):
            issues.append("duplicate episode_id in manifest")
        if set(manifest_ids) != {r["episode_id"] for r in accepted}:
            issues.append("manifest episode IDs differ from accepted production plan")
    elif accepted:
        issues.append("manifest.csv missing for accepted episodes")
    durations = []
    sample_counts = []
    missing_ratios = []
    for record in validated.values():
        durations.append(record["duration_sec"])
        sample_counts.append(record["sample_count"])
        missing_ratios.append(record["missing_ratio"])
    summary = {
        "dataset_version": config.dataset_version,
        "state": "COMPLETE" if not issues and not warnings and len(accepted) >= sum(config.target_episodes.values())
                 and all(s in {"PASS", "FAIL_INFRASTRUCTURE"} for s in status) else "INCOMPLETE",
        "total_episodes": len(rows), "pass": len(accepted), "fail_infrastructure": len(failed),
        "status_counts": dict(sorted(status.items())),
        "episodes_per_scenario": {f: scenario[f] for f in FAMILIES},
        "episodes_per_density": {d: density[d] for d in config.densities},
        "human_count_distribution": _distribution(humans),
        "behavior_distribution": {b: behaviors[b] for b in BEHAVIORS},
        "outcome_distribution": {o: outcomes[o] for o in
                                 ("success", "collision", "timeout", "stuck", "nav_failure")},
        "duration_sec": _distribution(durations),
        "sample_count": _distribution(sample_counts),
        "missing_ratio": _distribution(missing_ratios),
        "duplicate_episode_id": 0, "duplicate_scenario_seed_density": 0,
        "duplicate_sampled_config": 0,
        "orphan_episode_directories": orphans,
        "warnings": warnings, "integrity_issues": issues,
    }
    if write:
        root.mkdir(parents=True, exist_ok=True)
        rendered = yaml.safe_dump(summary, sort_keys=False, allow_unicode=True)
        (root / "dataset_audit.yaml").write_text(rendered, encoding="utf-8")
        (root / "dataset_summary.yaml").write_text(rendered, encoding="utf-8")
        with (root / "scenario_coverage.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=("scenario_family", "target", "planned",
                                                      "pass", "fail_infrastructure", "low", "medium", "high"))
            writer.writeheader()
            for index, family in enumerate(FAMILIES):
                family_rows = [row for row in rows if row["scenario_family"] == family]
                passing = [row for row in family_rows if row["status"] == "PASS"]
                writer.writerow({
                    "scenario_family": family, "target": config.target_episodes[f"S{index:02d}"],
                    "planned": len(family_rows), "pass": len(passing),
                    "fail_infrastructure": sum(row["status"] == "FAIL_INFRASTRUCTURE" for row in family_rows),
                    **{density: sum(row["density"] == density for row in passing)
                       for density in ("low", "medium", "high")},
                })
        with (root / "dataset_audit.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=("episode_id", "scenario_family", "seed", "density",
                                                      "status", "outcome", "attempts", "reason"))
            writer.writeheader()
            writer.writerows({key: row[key] for key in writer.fieldnames} for row in rows)
    return summary
