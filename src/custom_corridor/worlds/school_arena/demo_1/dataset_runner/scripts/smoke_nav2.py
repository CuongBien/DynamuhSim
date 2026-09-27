#!/usr/bin/env python3
"""Launch one EMPTY episode, send its Nav2 goal, observe briefly, and clean up."""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dataset_runner import EpisodeLoader, SimulatorManager
from dataset_runner.nav2_manager import GoalState, Nav2Manager


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_dir", type=Path)
    parser.add_argument("--observe-sec", type=float, default=10.0)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    context = EpisodeLoader().load(args.episode_dir)
    if context.scenario_family != "empty":
        parser.error("Smoke test requires an EMPTY episode")
    simulator = SimulatorManager()
    nav2 = Nav2Manager()
    try:
        simulator.reset(context)
        simulator.wait_ready(context)
        print("SIMULATOR_READY", flush=True)
        nav2.wait_ready(context)
        print("NAV2_READY", flush=True)
        nav2.send_goal(context)
        print("GOAL_ACCEPTED", flush=True)
        deadline = time.monotonic() + args.observe_sec
        last_state = None
        while time.monotonic() < deadline:
            result = nav2.poll(context)
            if result.state != last_state:
                print(f"GOAL_STATE {result.state.value} status={result.status_code} "
                      f"error={result.error_code} {result.error_msg}", flush=True)
                last_state = result.state
            if result.state in (GoalState.SUCCEEDED, GoalState.ABORTED, GoalState.CANCELED):
                break
            nav2.backend.tick(min(nav2.config.poll_interval_sec,
                                  max(0.0, deadline - time.monotonic())))
        nav2.cancel_goal(context)
        print(f"GOAL_FINAL_STATE {nav2.observation.state.value}", flush=True)
        return 0
    finally:
        try:
            nav2.finalize(context)
            print("NAV2_FINALIZED", flush=True)
        finally:
            simulator.finalize(context)
            print("SIMULATOR_FINALIZED", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
