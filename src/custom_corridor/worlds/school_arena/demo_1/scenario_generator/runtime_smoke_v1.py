#!/usr/bin/env python3
"""Short infrastructure smoke using the existing EpisodeRunner and managers."""
from __future__ import annotations

import argparse
import csv
import dataclasses
import logging
import os
import sys
from pathlib import Path

os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dataset_runner import (EpisodeMonitor, EpisodeRunner, HuNavManager,
                            ReadinessChecker, SimulatorManager, EpisodeStatus)
from dataset_runner.episode_monitor import MonitorConfig
from dataset_runner.hunav_manager import DockerHuNavRuntime, HuNavConfig, HuNavError
from dataset_runner.nav2_manager import Nav2Manager
from dataset_runner.scripts.smoke_runtime_integration import HuNavPreflight


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episodes", nargs="+", type=Path)
    parser.add_argument("--observe-sec", type=float, default=6.0,
                        help="EpisodeMonitor timeout after goal acceptance")
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--append", action="store_true",
                        help="Append new episode rows to an existing results file")
    parser.add_argument("--reset-hunav-container", action="store_true")
    args = parser.parse_args(argv)
    if args.observe_sec <= 0:
        parser.error("--observe-sec must be positive")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    log_dir = Path("/tmp/pbl6_v1_smoke_logs")
    hunav_config = HuNavConfig.from_yaml()
    simulator = SimulatorManager(log_dir=log_dir)
    hunav = HuNavManager(config=hunav_config,
                         runtime=DockerHuNavRuntime(hunav_config, log_dir=log_dir))
    nav2 = Nav2Manager()
    readiness = ReadinessChecker()
    monitor_config = dataclasses.replace(MonitorConfig.from_yaml(),
                                         timeout_sec=args.observe_sec)
    monitor = EpisodeMonitor(config=monitor_config, nav2_manager=nav2,
                             simulator_manager=simulator, hunav_manager=hunav)
    runner = EpisodeRunner(simulator_manager=simulator, hunav_manager=hunav,
                           nav2_manager=nav2, readiness_checker=readiness,
                           monitor=monitor)
    try:
        HuNavPreflight(hunav_config, hunav.runtime).check(
            reset_container=args.reset_hunav_container)
    except HuNavError as exc:
        parser.exit(1, f"HUNAV PREFLIGHT FAILED: {exc}\n")
    args.results.parent.mkdir(parents=True, exist_ok=True)
    failed = False
    mode = "a" if args.append and args.results.exists() else "w"
    with args.results.open(mode, newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            "episode_id", "scenario_family", "status", "reason", "duration_sec",
            "infrastructure_pass", "cleanup_clean"))
        if mode == "w":
            writer.writeheader()
        for path in args.episodes:
            result = runner.run(path)
            clean = simulator.pgid is None and not hunav.owned and nav2.handle is None
            valid = (clean and result.monitor_result is not None and result.status not in
                     (EpisodeStatus.SIM_FAILURE, EpisodeStatus.INVALID_EPISODE))
            writer.writerow({"episode_id": result.episode_id,
                             "scenario_family": result.context.scenario_family if result.context else "",
                             "status": result.status.value,
                             "reason": result.termination_reason,
                             "duration_sec": round(result.duration_sec, 3),
                             "infrastructure_pass": valid,
                             "cleanup_clean": clean})
            stream.flush()
            print(f"EPISODE {result.episode_id} {result.status.value} "
                  f"infrastructure_pass={valid} clean={clean}", flush=True)
            failed |= not valid
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
