#!/usr/bin/env python3
"""Dataset V1 production entry point. Runtime backend must provide recording/build/QA."""
from __future__ import annotations

import argparse
import importlib
import logging
import os
import subprocess
import sys
from pathlib import Path

from dataset_production.audit import audit
from dataset_production.balance import balance_preview
from dataset_production.freeze import freeze
from dataset_production.plan import (check_plan_identity, extend_for_deficits, load_config,
                                     make_plan, read_plan, write_plan)
from dataset_production.prepare import ensure_episode
from dataset_production.runner import run_plan
from scenario_generator.src.scenario_generator import ScenarioGenerator
import yaml


def _source_workspace() -> None:
    setup = Path(__file__).resolve().parents[5] / "install" / "setup.bash"
    if not setup.is_file():
        raise RuntimeError(f"ROS workspace setup missing: {setup}")
    result = subprocess.run(
        ["bash", "-c", 'source "$1" >/dev/null && env -0', "bash", str(setup)],
        capture_output=True, check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Cannot source ROS workspace: {result.stderr.decode(errors='replace')}")
    for entry in result.stdout.split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        os.environ[key.decode()] = value.decode(errors="surrogateescape")
    for directory in reversed(os.environ.get("PYTHONPATH", "").split(os.pathsep)):
        if directory and directory not in sys.path:
            sys.path.insert(0, directory)


def _backend(spec: str):
    try:
        module_name, function_name = spec.rsplit(":", 1)
        backend = getattr(importlib.import_module(module_name), function_name)
    except (ValueError, ImportError, AttributeError) as exc:
        raise RuntimeError(f"Cannot load production backend {spec!r}: {exc}") from exc
    if not callable(backend):
        raise TypeError(f"Production backend {spec!r} is not callable")
    return backend


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/dataset_v1.yaml"))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--prepare-only", action="store_true", help="Generate inputs without runtime")
    parser.add_argument("--audit-only", action="store_true")
    parser.add_argument("--freeze", action="store_true", help="Freeze only after full integrity PASS")
    parser.add_argument("--max-episodes", type=int, default=None)
    parser.add_argument("--episode-id", action="append", help="Run specific planned episode(s)")
    parser.add_argument("--backend", help="B3B adapter as module:function; may also be set in config")
    args = parser.parse_args()
    if sum((args.prepare_only, args.audit_only, args.freeze)) > 1:
        parser.error("Choose only one of --prepare-only, --audit-only, --freeze")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    config = load_config(args.config)
    config.output_dir.mkdir(parents=True, exist_ok=True)
    plan_file = config.output_dir / "production_plan.csv"
    expected = make_plan(config)
    if plan_file.exists():
        rows = read_plan(plan_file)
        check_plan_identity(rows, expected)
    else:
        rows = expected
        write_plan(plan_file, rows)
    if args.audit_only:
        result = audit(config)
        print(f"AUDIT {result['state']} PASS={result['pass']} "
              f"FAIL_INFRASTRUCTURE={result['fail_infrastructure']}")
        return 0 if result["state"] == "COMPLETE" else 1
    if args.freeze:
        result = audit(config)
        if result["state"] != "COMPLETE":
            raise RuntimeError("Cannot freeze: dataset audit is incomplete")
        fingerprint = freeze(config)
        print(yaml.safe_dump(fingerprint, sort_keys=False))
        return 0
    if args.prepare_only:
        generator = ScenarioGenerator(config.generator_config)
        created = sum(ensure_episode(config, row, generator) for row in rows)
        preview = balance_preview(config, rows)
        (config.output_dir / "production_balance_preview.yaml").write_text(
            yaml.safe_dump(preview, sort_keys=False), encoding="utf-8")
        print(f"PREPARED created={created} total={len(rows)} warnings={len(preview['warnings'])}")
        return 0
    config_doc = yaml.safe_load(config.path.read_text(encoding="utf-8"))
    backend_spec = args.backend or config_doc.get("production_backend")
    if not backend_spec:
        raise RuntimeError(
            "B3B recorder/builder/QA backend is absent. Supply --backend module:function "
            "or production_backend in config; no episode status was changed.")
    _source_workspace()
    # Match the middleware explicitly selected by school_hunav_demo.launch.py.
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    backend = _backend(backend_spec)
    if not args.resume and any(row["status"] != "PLANNED" for row in rows):
        raise RuntimeError("Production already started; use --resume")
    replacement_rounds = 0
    max_replacement_rounds = config_doc.get("max_replacement_rounds", 2)
    if isinstance(max_replacement_rounds, bool) or not isinstance(max_replacement_rounds, int) or max_replacement_rounds < 0:
        raise ValueError("max_replacement_rounds must be a nonnegative integer")
    while True:
        run_plan(config, rows, backend, resume=args.resume,
                 max_episodes=args.max_episodes,
                 episode_ids=set(args.episode_id) if args.episode_id else None)
        args.resume = True
        audit_result = audit(config)
        print(f"BATCH PASS={audit_result['pass']} "
              f"FAIL_INFRASTRUCTURE={audit_result['fail_infrastructure']} "
              f"state={audit_result['state']}")
        if args.max_episodes is not None or args.episode_id:
            break
        retryable = any(
            row["status"] in {"PLANNED", "RUNNING"} or
            (row["status"] == "FAIL_INFRASTRUCTURE" and
             int(row["attempts"]) <= config.max_infrastructure_retries)
            for row in rows)
        if retryable:
            continue
        if replacement_rounds >= max_replacement_rounds:
            break
        supplemented = extend_for_deficits(config, rows)
        if len(supplemented) == len(rows):
            break
        replacement_rounds += 1
        rows = supplemented
        write_plan(plan_file, rows)
    if audit_result["state"] == "COMPLETE":
        fingerprint = freeze(config)
        print(f"FROZEN episode_count={fingerprint['episode_count']}")
        return 0
    return 1


def _bootstrap_ros_process() -> None:
    """Load ROS Python and shared libraries before the interpreter starts."""
    if os.environ.get("PBL6_ROS_BOOTSTRAPPED") == "1":
        return
    setup = Path(__file__).resolve().parents[5] / "install" / "setup.bash"
    if not setup.is_file():
        raise RuntimeError(f"ROS workspace setup missing: {setup}")
    command = [
        "/bin/bash", "-c",
        'source "$1" >/dev/null && shift && export PBL6_ROS_BOOTSTRAPPED=1 '
        'RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && exec "$@"',
        "bash", str(setup), sys.executable, str(Path(__file__).resolve()), *sys.argv[1:],
    ]
    os.execvpe(command[0], command, os.environ)


if __name__ == "__main__":
    _bootstrap_ros_process()
    raise SystemExit(main())
