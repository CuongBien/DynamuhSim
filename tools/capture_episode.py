#!/usr/bin/env python3
"""Prepare/verify/run a fresh capture without source generation or historical driver imports."""
import argparse
from pathlib import Path
import sys
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.runtime.capture import CaptureContext, prepare_run, verify_run
from dynamuhsim.runtime.nav2_trial import run_trial
from dynamuhsim.runtime.profiles import PROFILES


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="mode", required=True)
    prepare = commands.add_parser("prepare", help="Create a NEW run/config/source freeze; never overwrite")
    prepare.add_argument("--run-dir", type=Path, required=True)
    prepare.add_argument("--config", type=Path, default=ROOT / "configs/capture/moderate_v3.json")
    prepare.add_argument("--scenario", choices=["standing", "walking"], required=True)
    prepare.add_argument("--seed", type=int, required=True)
    prepare.add_argument("--gui", action="store_true")
    prepare.add_argument("--planner-diagnostics", action="store_true", help="Record exact native costmaps without changing navigation")
    prepare.add_argument("--human-motion-profile", choices=["legacy", "group_orca_v1", "group_goal_v1"], default="legacy")
    prepare.add_argument("--encounter-profile", choices=["head_on", "crossing"], help="Opt-in walking initial conditions; native motion continues after start")
    prepare.add_argument("--human-goal-clock", choices=["native_wall", "sim_v1"], default="native_wall", help="Opt-in Gazebo goal expiry from the accepted navigation gate")
    for name in ["verify", "run"]:
        cmd = commands.add_parser(name)
        cmd.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.mode == "prepare":
        prepare_run(ROOT, args.run_dir, args.config, args.scenario, args.seed, args.gui, planner_diagnostics=args.planner_diagnostics, human_motion_profile=args.human_motion_profile, encounter_profile=args.encounter_profile, human_goal_clock=args.human_goal_clock)
        print("Prepared new capture freeze:", args.run_dir.resolve())
        return 0
    verify_run(ROOT, args.run_dir)
    if args.mode == "verify":
        print("PASS: new capture freeze is unchanged")
        return 0
    capture = CaptureContext(ROOT, args.run_dir)
    plan = capture.plan
    options = SimpleNamespace(scenario=plan["scenario"], seed=plan["seed"], gui=plan["gui"], condition="people",
                              report_dir=capture.run_dir / "episode")
    return run_trial(PROFILES["socnavgym_group_nav2"], options, ROOT, Path(__file__).resolve(), capture=capture)


if __name__ == "__main__":
    raise SystemExit(main())
