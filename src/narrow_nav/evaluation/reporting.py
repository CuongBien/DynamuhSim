"""Generate phase coverage, attempt QA and matched reports from run artifacts."""

import csv
import json
from collections import Counter, defaultdict
from pathlib import Path
from statistics import fmean

from .aggregate import condition_summary, paired_comparisons
from .runner import write_json


def _read(path):
    return json.loads(path.read_text())


def _csv(path, records):
    fields = list(dict.fromkeys(key for record in records for key in record))
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fields)
        writer.writeheader()
        for record in records:
            writer.writerow(
                {
                    key: json.dumps(value, ensure_ascii=False)
                    if isinstance(value, (dict, tuple, list))
                    else value
                    for key, value in record.items()
                }
            )


def _collect(root, data):
    per_episode, attempts = [], []
    for slot in data["slots"]:
        ident = slot["slot_id"]
        paths = sorted((root / "attempts" / ident).glob("attempt-*"))
        effective = metric = None
        for path in paths:
            manifest = path / "manifest.json"
            validity = path / "validity.json"
            record = _read(manifest) if manifest.exists() else {}
            values = _read(validity) if validity.exists() and manifest.exists() else {}
            if values and (
                metric is None
                or values["infrastructure_valid"]
                or not metric["infrastructure_valid"]
            ):
                effective, metric = record, values
            attempts.append(
                {
                    "slot_id": ident,
                    "backend": slot["backend"],
                    "attempt": path.name,
                    "episode_id": record.get("episode_id"),
                    "outcome": values.get("outcome", "INTERRUPTED"),
                    "path": str(path),
                    "failure": record.get("error"),
                    "infra_valid": values.get("infrastructure_valid"),
                    "human_behavior": values.get("human_behavior"),
                    "label_valid": values.get("scenario_label_valid"),
                }
            )
        values = metric or {}
        quality = values.get("quality") or {}
        hard = values.get("hard", {})
        per_episode.append(
            {
                "slot_id": ident,
                "backend": slot["backend"],
                "card_id": slot["card_id"],
                "level": slot["level"],
                "action": slot["action"],
                "seed": slot["seed"],
                "density": (slot["binding"] or {}).get("density"),
                "human_policy": (slot["binding"] or {}).get("human_policy"),
                "subtype": (slot["binding"] or {}).get("subtype"),
                "introduced": slot["introduced"],
                "attempts": len(paths),
                "outcome": values.get(
                    "outcome",
                    "INTERRUPTED"
                    if paths
                    else "BLOCKED_SPEC"
                    if slot["blocked"]
                    else "NOT_RUN",
                ),
                "block_reasons": slot["blocked"],
                "infrastructure_valid": values.get("infrastructure_valid"),
                "scenario_label_valid": values.get("scenario_label_valid"),
                "human_physics_valid": values.get("human_physics_valid"),
                "human_behavior": values.get("human_behavior"),
                "episode_task_pass": values.get("episode_task_pass"),
                "S": hard.get("S"),
                "C": hard.get("C"),
                "TO": hard.get("TO"),
                "FP": hard.get("FP"),
                "path_length_m": quality.get("robot", {}).get("path_length_m"),
                "speed_mean_mps": (quality.get("robot", {}).get("speed_mps") or {}).get(
                    "mean"
                ),
                "jerk_max_mps3": (quality.get("robot", {}).get("jerk_mps3") or {}).get(
                    "max"
                ),
                "spl": quality.get("spl"),
                "psc_fraction": quality.get("psc_fraction"),
                "min_ttc_s": quality.get("min_ttc_s"),
                "min_human_center_distance_m": quality.get("min_center_distance_m"),
                "clearance_m": values.get("clearance_m"),
                "encounters": quality.get("encounters"),
                "human_count": quality.get("human_count"),
                "trajectory_count": quality.get("trajectory_count"),
                "duration_s": quality.get("measured_duration_s"),
                "paired_key": effective.get("paired_key") if effective else None,
            }
        )
    return per_episode, attempts


def _distributions(rows_by_condition):
    for (backend, card, level, action, subtype), rows in rows_by_condition.items():
        eligible = [
            row
            for row in rows
            if row["episode_task_pass"] is True and row["scenario_label_valid"] is True
        ]
        for metric in (
            "path_length_m",
            "spl",
            "psc_fraction",
            "min_ttc_s",
            "jerk_max_mps3",
            "min_human_center_distance_m",
        ):
            values = [row[metric] for row in eligible if row[metric] is not None]
            if not values:
                edges, counts = [], []
            elif (low := min(values)) == (high := max(values)):
                edges, counts = [low, high], [len(values)]
            else:
                edges = [low + (high - low) * i / 5 for i in range(6)]
                counts = [0] * 5
                for value in values:
                    counts[min(4, int(5 * (value - low) / (high - low)))] += 1
            yield {
                "backend": backend,
                "card_id": card,
                "level": level,
                "action": action,
                "subtype": subtype,
                "metric": metric,
                "available": len(values),
                "missing": len(eligible) - len(values),
                "excluded": len(rows) - len(eligible),
                "mean": fmean(values) if values else None,
                "bin_edges": edges,
                "bin_counts": counts,
            }


def report_run(root: Path, output: Path):
    lock = _read(root / "run.json")
    plans = [_read(path) for path in sorted(root.glob("phase_*.json"))]
    if not plans or any(plan["plan_id"] != lock["plan_id"] for plan in plans):
        raise ValueError("run has no compatible saved plan")
    plan = plans[-1]
    episodes, attempts = _collect(root, plan)
    by_condition = defaultdict(list)
    for row in episodes:
        by_condition[
            (
                row["backend"],
                row["card_id"],
                row["level"],
                row["action"],
                row["subtype"],
            )
        ].append(row)
    conditions = [
        {
            "backend": key[0],
            "card_id": key[1],
            "level": key[2],
            "action": key[3],
            "subtype": key[4],
            **condition_summary(rows),
        }
        for key, rows in by_condition.items()
    ]
    comparisons = paired_comparisons(episodes, sorted(plan["matrix"]["backends"]))
    coverage = [
        {
            "backend": name,
            "slots": sum(row["backend"] == name for row in episodes),
            "attempted_cards": sorted(
                {
                    r["card_id"]
                    for r in episodes
                    if r["backend"] == name and r["card_id"] and r["attempts"]
                }
            ),
            **condition_summary([r for r in episodes if r["backend"] == name]),
        }
        for name in sorted(plan["matrix"]["backends"])
    ]
    outliers = [
        row
        for row in episodes
        if row["outcome"] not in ("TASK_SUCCESS", "NOT_RUN", "BLOCKED_SPEC")
    ]
    output.mkdir(parents=True, exist_ok=True)
    for filename, records in (
        ("per_episode.csv", episodes),
        ("qa.csv", attempts),
        ("per_condition.csv", conditions),
        ("coverage.csv", coverage),
        ("compare_backends.csv", comparisons),
        ("outliers.csv", outliers),
        ("summary.csv", coverage),
        ("quality_distributions.csv", list(_distributions(by_condition))),
    ):
        _csv(output / filename, records)
    report = {
        "schema_version": 1,
        "phase": plan["phase"],
        "run_provenance": lock,
        "outcomes": dict(Counter(r["outcome"] for r in episodes)),
        "planned": len(episodes),
        "attempt_count": len(attempts),
        "conditions": conditions,
        "paired_comparisons": comparisons,
        "coverage": coverage,
        "notes": "SR/CR use research slots with valid infrastructure and available evidence. Labeled SR is separate. Paired comparisons require two valid Card labels. Every attempt is retained in qa.csv.",
    }
    write_json(output / "report.json", report)
    (output / "README.md").write_text(
        f"# Benchmark phase {plan['phase']}\n\n"
        f"Planned slots {len(episodes)}. Attempts {len(attempts)}.\n\n"
        "Empty metric cells indicate missing or inapplicable evidence. "
        "See per_episode.csv for slot outcomes and qa.csv for every attempt.\n",
        encoding="utf-8",
    )
    return report
