#!/usr/bin/env python3
"""One-episode simulator/HuNav readiness smoke test; sends no Nav2 goal."""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dataset_runner import EpisodeLoader, HuNavManager, SimulatorManager


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_dir", type=Path)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    context = EpisodeLoader().load(args.episode_dir)
    simulator = SimulatorManager()
    hunav = HuNavManager()
    try:
        simulator.reset(context)
        simulator.wait_ready(context)
        print(f"SIMULATOR_READY {context.episode_id}", flush=True)
        hunav.load_scenario(context)
        print(f"HUNAV_LOADED {len(hunav.expected_names)}", flush=True)
        hunav.wait_ready(context)
        print(f"HUNAV_READY expected={len(hunav.expected_names)} actual={hunav.actual_count}",
              flush=True)
        return 0
    finally:
        try:
            hunav.finalize(context)
            print("HUNAV_FINALIZED", flush=True)
        finally:
            simulator.finalize(context)
            print("SIMULATOR_FINALIZED", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
