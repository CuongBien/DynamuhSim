#!/usr/bin/env python3
"""Summarize generator coverage from on-disk episodes; never infer runtime PASS."""
from __future__ import annotations

import argparse
import collections
import csv
import statistics
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from src.scenario_generator import FAMILIES, ScenarioGenerator
from src.route_sampler import RouteSampler
from src.validators import validate_episode
import random


def build_report(input_dir: Path, config: Path, runtime_results: Path | None = None) -> dict:
    generator = ScenarioGenerator(config)
    runtime = collections.defaultdict(list)
    if runtime_results is not None:
        with runtime_results.open(newline="", encoding="utf-8") as stream:
            for result_row in csv.DictReader(stream):
                runtime[result_row["episode_id"]].append(result_row)
    rows = {family: {
        "scenario_family": family, "episode_count": 0,
        "density_distribution": collections.Counter(),
        "behavior_distribution": collections.Counter(),
        "human_counts": [], "seeds": set(), "PASS": 0, "FAIL": 0,
        "runtime_PASS": 0, "runtime_FAIL": 0, "runtime_NOT_RUN": 0,
        "runtime_status_distribution": collections.Counter(),
        "runtime_attempts": 0, "runtime_failed_attempts": 0,
    } for family in FAMILIES}
    failures = []
    for directory in sorted(input_dir.glob("ep_[0-9][0-9][0-9][0-9][0-9][0-9]")):
        try:
            scenario = yaml.safe_load((directory / "scenario.yaml").read_text(encoding="utf-8"))
            family = scenario["scenario_family"]
            if family not in rows:
                raise ValueError(f"unknown scenario_family {family}")
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            failures.append({"episode_id": directory.name, "reason": str(exc)})
            continue
        row = rows[family]
        row["episode_count"] += 1
        attempts = runtime.get(directory.name, [])
        row["runtime_attempts"] += len(attempts)
        row["runtime_failed_attempts"] += sum(
            attempt.get("infrastructure_pass", "").lower() != "true"
            for attempt in attempts)
        runtime_row = attempts[-1] if attempts else None
        if runtime_row is None:
            row["runtime_NOT_RUN"] += 1
        else:
            row["runtime_status_distribution"][runtime_row.get("status", "unknown")] += 1
            row["runtime_PASS" if runtime_row.get("infrastructure_pass", "").lower() == "true"
                else "runtime_FAIL"] += 1
        row["density_distribution"][scenario.get("density", "unknown")] += 1
        row["seeds"].add(scenario.get("seed"))
        humans = scenario.get("humans", [])
        row["human_counts"].append(len(humans))
        row["behavior_distribution"].update(h.get("behavior", "unknown") for h in humans)
        try:
            metadata = yaml.safe_load((directory / "metadata.yaml").read_text(encoding="utf-8"))
            if (metadata["episode_id"] != directory.name
                    or metadata["scenario_family"] != family
                    or metadata["seed"] != scenario["seed"]
                    or metadata["humans"]["total"] != len(humans)):
                raise ValueError("metadata and scenario mismatch")
            from src.human_sampler import HumanSampler
            routes = RouteSampler(generator.inputs, random.Random(scenario["seed"]))
            validate_episode(generator.inputs, routes, generator.occupancy, scenario)
            expected_agents = HumanSampler(generator.inputs, routes, random.Random(0)).hunav_yaml(
                humans, "check"
            )["hunav_loader"]["ros__parameters"].get("agents", [])
            params = yaml.safe_load((directory / "humans.yaml").read_text(encoding="utf-8"))[
                "hunav_loader"]["ros__parameters"]
            if expected_agents != params.get("agents", []):
                raise ValueError("HuNav agents differ from scenario")
            for name in ("school_floor.world", "nav2_school.yaml"):
                if not (directory / name).is_file():
                    raise ValueError(f"missing {name}")
            row["PASS"] += 1
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            row["FAIL"] += 1
            failures.append({"episode_id": directory.name, "reason": str(exc)})
    result = []
    for row in rows.values():
        counts = row.pop("human_counts")
        seeds = row.pop("seeds")
        row["seed_count"] = len(seeds)
        row["human_count"] = {
            "min": min(counts) if counts else 0,
            "mean": round(statistics.mean(counts), 3) if counts else 0,
            "max": max(counts) if counts else 0,
        }
        row["density_distribution"] = dict(sorted(row["density_distribution"].items()))
        row["behavior_distribution"] = dict(sorted(row["behavior_distribution"].items()))
        row["runtime_status_distribution"] = dict(sorted(
            row["runtime_status_distribution"].items()))
        result.append(row)
    return {"validation_scope": "PASS/FAIL validate generated artifacts; runtime fields use Runner results when supplied",
            "scenarios": result, "unclassified_failures": failures}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "generated")
    parser.add_argument("--output", type=Path, default=ROOT)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/generator.yaml")
    parser.add_argument("--runtime-results", type=Path)
    args = parser.parse_args(argv)
    report = build_report(args.input, args.config, args.runtime_results)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "scenario_coverage.yaml").write_text(
        yaml.safe_dump(report, sort_keys=False), encoding="utf-8"
    )
    with (args.output / "scenario_coverage.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ("scenario_family", "episode_count", "density_distribution",
                  "behavior_distribution", "human_count", "seed_count", "PASS", "FAIL",
                  "runtime_PASS", "runtime_FAIL", "runtime_NOT_RUN",
                  "runtime_status_distribution", "runtime_attempts",
                  "runtime_failed_attempts")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in report["scenarios"]:
            writer.writerow({key: yaml.safe_dump(row[key], default_flow_style=True).strip()
                             if isinstance(row[key], dict) else row[key] for key in fields})
    print(f"Wrote {args.output / 'scenario_coverage.csv'} and scenario_coverage.yaml")
    return 0 if not report["unclassified_failures"] and all(
        row["FAIL"] == 0 for row in report["scenarios"]
    ) else 1


if __name__ == "__main__":
    raise SystemExit(main())
