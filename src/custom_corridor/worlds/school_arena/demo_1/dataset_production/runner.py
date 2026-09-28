"""Crash-safe production loop around the existing generator and runtime pipeline."""
from __future__ import annotations

from pathlib import Path
from typing import Callable

from scenario_generator.src.scenario_generator import ScenarioGenerator

from .classification import ProductionDecision
from .plan import ProductionConfig, write_plan
from .prepare import ensure_episode


ProductionBackend = Callable[[Path], ProductionDecision]


def run_plan(config: ProductionConfig, rows: list[dict[str, str]],
             backend: ProductionBackend, *, resume: bool,
             max_episodes: int | None = None,
             generator: ScenarioGenerator | None = None,
             episode_ids: set[str] | None = None) -> list[dict[str, str]]:
    """Persist each attempt; never rerun PASS and never lose the rest of a batch."""
    if max_episodes is not None and max_episodes < 1:
        raise ValueError("max_episodes must be positive")
    if not resume and any(row["status"] != "PLANNED" for row in rows):
        raise ValueError("Existing production state requires --resume")
    generator = generator or ScenarioGenerator(config.generator_config)
    plan_path = config.output_dir / "production_plan.csv"
    processed = 0
    for row in rows:
        if episode_ids is not None and row["episode_id"] not in episode_ids:
            continue
        if row["status"] == "PASS":
            continue
        attempts = int(row["attempts"])
        if row["status"] == "FAIL_INFRASTRUCTURE" and attempts > config.max_infrastructure_retries:
            continue
        if row["status"] not in {"PLANNED", "RUNNING", "FAIL_INFRASTRUCTURE"}:
            raise ValueError(f"Unknown plan status: {row['status']}")
        row["status"] = "RUNNING"
        row["attempts"] = str(attempts + 1)
        write_plan(plan_path, rows)
        try:
            ensure_episode(config, row, generator)
            decision = backend(config.output_dir / "episodes" / row["episode_id"])
            if not isinstance(decision, ProductionDecision):
                raise TypeError("Production backend must return ProductionDecision")
            if decision.status not in {"PASS", "FAIL_INFRASTRUCTURE"}:
                raise ValueError(f"Invalid backend status {decision.status}")
            row["status"] = decision.status
            row["outcome"] = decision.outcome if decision.status == "PASS" else ""
            row["reason"] = decision.reason.replace("\n", " ")[:500]
        except Exception as exc:
            row["status"] = "FAIL_INFRASTRUCTURE"
            row["outcome"] = ""
            row["reason"] = f"{type(exc).__name__}: {exc}".replace("\n", " ")[:500]
        write_plan(plan_path, rows)
        processed += 1
        if max_episodes is not None and processed >= max_episodes:
            break
    return rows
