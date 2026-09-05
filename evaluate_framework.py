#!/usr/bin/env python3
"""
Reusable evaluation runner for the narrow/dynamic corridor experiments.

Keeps analyze_baseline.py as the single-trial metric engine and adds:
  1) single-trial evaluation
  2) batch evaluation over many trials
  3) algorithm comparison
  4) machine-readable comparison.csv / comparison.json
  5) experiment-level aggregate metrics

Expected trial layout:
  ~/nav_ws/experiments/corridor_090/
    baseline_01/
      trial_02/
      obstacle_ground_truth_02.csv

For future algorithms:
  <algorithm_name>/
    trial_01/
      trial/
      obstacle_ground_truth.csv
    trial_02/
      trial/
      obstacle_ground_truth.csv

This runner defines evaluation framework v1 around the baseline/proxy analyzer.
It preserves analyzer output, including raw measurements, for later extensions.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import subprocess
import sys
from pathlib import Path
from typing import Any


DEFAULT_ROOT = Path("~/nav_ws/experiments/corridor_090").expanduser()
DEFAULT_ANALYZER = Path("~/nav_ws/analyze_baseline.py").expanduser()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def num(x: Any) -> float | None:
    return float(x) if isinstance(x, (int, float)) else None


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def evaluate_one(
    analyzer: Path,
    bag: Path,
    ground_truth: Path,
    output: Path,
) -> Path:
    output.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(analyzer),
        "--bag", str(bag),
        "--ground-truth", str(ground_truth),
        "--output", str(output),
    ]
    print("$", " ".join(cmd))
    subprocess.run(cmd, check=True)
    summary = output / "summary.json"
    if not summary.is_file():
        raise RuntimeError(f"analyzer did not produce {summary}")
    return summary


def extract_metrics(summary: dict[str, Any]) -> dict[str, Any]:
    odom = summary.get("odometry", {})
    cmd = summary.get("cmd_vel", {})
    scan = summary.get("scan", {})
    obs = summary.get("obstacle_ground_truth", {})
    prox = summary.get("robot_obstacle", {})
    rsp = summary.get("response_proxy", {})
    planner = summary.get("planner", {})

    encounters = rsp.get("encounters", [])
    timely = sum(e.get("reaction_classification") == "TIMELY" for e in encounters)
    late = sum(e.get("reaction_classification") == "LATE" for e in encounters)
    no_detected_response = sum(e.get("reaction_classification") == "NO_DETECTED_RESPONSE" for e in encounters)

    reaction_delays = [
        float(e["reaction_delay_s"])
        for e in encounters
        if isinstance(e.get("reaction_delay_s"), (int, float))
    ]
    entry_leads = [
        float(e["lead_time_s"])
        for e in encounters
        if isinstance(e.get("lead_time_s"), (int, float))
    ]
    min_clearances = [
        float(e["minimum_estimated_clearance_m"])
        for e in encounters
        if isinstance(e.get("minimum_estimated_clearance_m"), (int, float))
    ]

    return {
        "robot_distance_m": num(odom.get("distance_m")),
        "odom_speed_median_mps": num(odom.get("twist_linear_speed_mps", {}).get("median")),
        "odom_speed_p95_mps": num(odom.get("twist_linear_speed_mps", {}).get("p95")),
        "cmd_linear_median_mps": num(cmd.get("linear_x_mps", {}).get("median")),
        "cmd_linear_p95_mps": num(cmd.get("linear_x_mps", {}).get("p95")),
        "scan_min_m": num(scan.get("minimum_range_m", {}).get("min")),
        "scan_min_p05_m": num(scan.get("minimum_range_m", {}).get("p05")),
        "obstacle_speed_median_mps": num(obs.get("speed_mps", {}).get("median")),
        "obstacle_speed_p95_mps": num(obs.get("speed_mps", {}).get("p95")),
        "min_center_distance_m": num(prox.get("center_distance_m", {}).get("min")),
        "min_clearance_m": num(prox.get("estimated_clearance_m", {}).get("min")),
        "encounters": len(encounters),
        "timely": timely,
        "late": late,
        "no_detected_response": no_detected_response,
        "reaction_delay_median_s": statistics.median(reaction_delays) if reaction_delays else None,
        "reaction_delay_min_s": min(reaction_delays) if reaction_delays else None,
        "lead_time_median_s": statistics.median(entry_leads) if entry_leads else None,
        "encounter_min_clearance_median": statistics.median(min_clearances) if min_clearances else None,
        "goal_success": summary.get("outcome", {}).get("goal_success"),
        "progress_m": num(summary.get("outcome", {}).get("progress_m")),
        "failure": summary.get("outcome", {}).get("failure"),
        "collision_contact": summary.get("outcome", {}).get("collision_contact"),
        "travel_time_s": num(summary.get("outcome", {}).get("travel_time_s")),
        "path_length_m": num(summary.get("outcome", {}).get("path_length_m")),
        "plan_updates": planner.get("plan", {}).get("updates"),
        "local_plan_updates": planner.get("local_plan", {}).get("updates"),
    }


def discover_trials(root: Path) -> list[tuple[str, Path, Path, Path]]:
    """
    Discover:
      <algorithm>/<trial>/trial
      <algorithm>/<trial>/obstacle_ground_truth*.csv
    """
    found = []
    direct_trials = [p for p in root.iterdir() if p.is_dir() and p.name.startswith("trial")]
    algorithm_dirs = [root] if direct_trials else sorted(p for p in root.iterdir() if p.is_dir())
    for algorithm_dir in algorithm_dirs:
        for trial_dir in sorted(p for p in algorithm_dir.iterdir() if p.is_dir()):
            bag_candidates = [
                trial_dir,
                trial_dir / "trial",
                trial_dir / "trial_01",
                trial_dir / "trial_02",
            ]
            bag = next((p for p in bag_candidates
                        if p.is_dir() and (any(p.glob("*.mcap")) or (p / "metadata.yaml").is_file())), None)
            trial_number = trial_dir.name.removeprefix("trial_")
            if trial_number.isdigit():
                gt_candidates = sorted(algorithm_dir.glob(f"obstacle_ground_truth_{trial_number}.csv"))
                if not gt_candidates:
                    gt_candidates = sorted(algorithm_dir.glob(f"obstacle_ground_truth_{int(trial_number)}.csv"))
            else:
                gt_candidates = sorted(algorithm_dir.glob("obstacle_ground_truth.csv"))
            if not gt_candidates:
                gt_candidates = sorted(trial_dir.glob("obstacle_ground_truth*.csv"))
            if bag and gt_candidates:
                found.append((algorithm_dir.name, trial_dir, bag, gt_candidates[-1]))
    return found


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    numeric = [
        "robot_distance_m", "odom_speed_median_mps", "odom_speed_p95_mps",
        "cmd_linear_median_mps", "cmd_linear_p95_mps", "scan_min_m",
        "scan_min_p05_m", "obstacle_speed_median_mps", "obstacle_speed_p95_mps",
        "min_center_distance_m", "min_clearance_m", "reaction_delay_median_s",
        "reaction_delay_min_s", "lead_time_median_s", "encounter_min_clearance_median",
        "progress_m", "travel_time_s", "path_length_m",
    ]
    out: dict[str, Any] = {"trials": len(rows)}
    for key in numeric:
        values = [float(r[key]) for r in rows if isinstance(r.get(key), (int, float))]
        out[key] = {
            "mean": mean(values),
            "median": statistics.median(values) if values else None,
            "stdev": statistics.stdev(values) if len(values) > 1 else 0.0 if values else None,
            "min": min(values) if values else None,
            "max": max(values) if values else None,
        }
    for key in ("encounters", "timely", "late", "no_detected_response"):
        values = [int(r.get(key, 0)) for r in rows]
        out[key] = sum(values)
    statuses = [r.get("trial_status") for r in rows]
    out["success_rate"] = statuses.count("SUCCESS") / len(statuses) if statuses else None
    out["failure_rate"] = statuses.count("FAILURE") / len(statuses) if statuses else None
    out["timeout_rate"] = statuses.count("TIMEOUT") / len(statuses) if statuses else None
    out["invalid_rate"] = statuses.count("INVALID") / len(statuses) if statuses else None
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    keys = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    p = argparse.ArgumentParser(
        description="Evaluation framework around analyze_baseline.py",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    sub = p.add_subparsers(dest="mode", required=True)

    one = sub.add_parser("single", help="evaluate one trial")
    one.add_argument("--analyzer", type=Path, default=DEFAULT_ANALYZER)
    one.add_argument("--bag", type=Path, required=True)
    one.add_argument("--ground-truth", type=Path, required=True)
    one.add_argument("--output", type=Path, required=True)

    batch = sub.add_parser("batch", help="evaluate every discovered trial")
    batch.add_argument("--root", "--base", dest="root", type=Path, default=DEFAULT_ROOT)
    batch.add_argument("--analyzer", type=Path, default=DEFAULT_ANALYZER)
    batch.add_argument("--output", type=Path, default=None)

    compare = sub.add_parser("compare", help="evaluate all trials and aggregate by algorithm")
    compare.add_argument("--root", "--base", dest="root", type=Path, default=DEFAULT_ROOT)
    compare.add_argument("--analyzer", type=Path, default=DEFAULT_ANALYZER)
    compare.add_argument("--output", type=Path, default=None)

    args = p.parse_args()

    if args.mode == "single":
        summary = evaluate_one(
            args.analyzer.expanduser().resolve(),
            args.bag.expanduser().resolve(),
            args.ground_truth.expanduser().resolve(),
            args.output.expanduser().resolve(),
        )
        metrics = extract_metrics(load_json(summary))
        print(json.dumps(metrics, indent=2))
        return 0

    root = args.root.expanduser().resolve()
    out = (args.output or (root / "evaluation")).expanduser().resolve()
    analyzer = args.analyzer.expanduser().resolve()

    discovered = discover_trials(root)
    if not discovered:
        raise RuntimeError(f"no trials found under {root}")

    rows = []
    for algorithm, trial_dir, bag, gt in discovered:
        analysis_dir = trial_dir / "analysis"
        summary_path = analysis_dir / "summary.json"
        if not summary_path.is_file():
            summary_path = evaluate_one(analyzer, bag, gt, analysis_dir)

        metrics = extract_metrics(load_json(summary_path))
        metrics.update({
            "algorithm": algorithm,
            "trial": trial_dir.name,
            "bag": str(bag),
            "ground_truth": str(gt),
            "analysis": str(summary_path.parent),
        })
        result_path = trial_dir / "trial_result.json"
        if result_path.is_file():
            result = load_json(result_path)
            metrics["trial_status"] = result.get("status")
            metrics["trial_reason"] = result.get("reason")
        rows.append(metrics)

    out.mkdir(parents=True, exist_ok=True)
    write_csv(out / "trial_metrics.csv", rows)

    if args.mode == "batch":
        aggregate_rows = [r for r in rows if r.get("trial_status") != "INVALID"]
        report = {
            "scope": "corridor_090",
            "framework_version": "v1",
            "metric_status": "existing baseline/proxy metrics; not official K6-K12 definitions",
            "trials": rows,
            "aggregate": aggregate(aggregate_rows),
        }
    else:
        algorithms = sorted({r["algorithm"] for r in rows})
        by_algorithm = {}
        for algorithm in algorithms:
            subset = [r for r in rows if r["algorithm"] == algorithm]
            aggregate_rows = [r for r in subset if r.get("trial_status") != "INVALID"]
            by_algorithm[algorithm] = {
                "trial_metrics": subset,
                "aggregate": aggregate(aggregate_rows),
            }

        report = {
            "scope": "corridor_090",
            "framework_version": "v1",
            "metric_status": "existing baseline/proxy metrics; not official K6-K12 definitions",
            "algorithms": by_algorithm,
            "comparison_direction": {
                "higher_is_better": [
                    "lead_time_median_s",
                    "robot_distance_m",
                    "progress_m",
                ],
                "lower_is_better": [
                    "reaction_delay_median_s",
                    "reaction_delay_min_s",
                    "min_center_distance_m",
                ],
                "context_only": [
                    "scan_min_m",
                    "obstacle_speed_median_mps",
                    "obstacle_speed_p95_mps",
                    "plan_updates",
                    "local_plan_updates",
                ],
            },
        }

    (out / "evaluation.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {out / 'trial_metrics.csv'}")
    print(f"Wrote {out / 'evaluation.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())