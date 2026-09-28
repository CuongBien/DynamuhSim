#!/usr/bin/env python3
"""Live EMPTY/HEAD_ON monitor smoke; terminal Nav2 failure is a valid classification."""
from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

# Match the middleware explicitly set by school_hunav_demo.launch.py before
# the first rclpy context is initialized.
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dataset_runner import (EpisodeMonitor, EpisodeRunner, HuNavManager,
                            ReadinessChecker, SimulatorManager, EpisodeStatus)
from dataset_runner.hunav_manager import DockerHuNavRuntime, HuNavConfig, HuNavError
from dataset_runner.nav2_manager import Nav2Manager
from dataset_runner.scripts.smoke_runtime_integration import HuNavPreflight


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episodes", nargs="+", type=Path)
    parser.add_argument("--reset-hunav-container", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    logs = Path("/tmp/pbl6_monitor_smoke_logs")
    hunav_config = HuNavConfig.from_yaml()
    simulator = SimulatorManager(log_dir=logs)
    hunav = HuNavManager(config=hunav_config,
                         runtime=DockerHuNavRuntime(hunav_config, log_dir=logs))
    nav2 = Nav2Manager()
    readiness = ReadinessChecker()
    monitor = EpisodeMonitor(nav2_manager=nav2, simulator_manager=simulator,
                             hunav_manager=hunav)
    runner = EpisodeRunner(simulator_manager=simulator, hunav_manager=hunav,
                           nav2_manager=nav2, readiness_checker=readiness,
                           monitor=monitor)
    try:
        HuNavPreflight(hunav_config, hunav.runtime).check(
            reset_container=args.reset_hunav_container)
    except HuNavError as exc:
        parser.exit(1, f"HUNAV PREFLIGHT FAILED: {exc}\n")
    failed = False
    for path in args.episodes:
        result = runner.run(path)
        monitor_result = result.monitor_result
        print(f"EPISODE {result.episode_id} status={result.status.value} "
              f"reason={result.termination_reason} "
              f"nav_code={monitor_result.navigation_result_code if monitor_result else None} "
              f"duration={monitor_result.duration_sec if monitor_result else None}", flush=True)
        clean = simulator.pgid is None and not hunav.owned and nav2.handle is None
        print(f"CLEANUP {result.episode_id} clean={clean} sim_pid={simulator.pgid} "
              f"hunav_owned={len(hunav.owned)} nav_goal={nav2.handle}", flush=True)
        if not clean or monitor_result is None or result.status in (
                EpisodeStatus.SIM_FAILURE, EpisodeStatus.INVALID_EPISODE):
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
