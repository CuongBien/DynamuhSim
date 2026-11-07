#!/usr/bin/env python3
"""Prepare/verify/run a fresh capture without source generation or historical driver imports."""
import argparse
import datetime
import json
import signal
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import uuid
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.runtime.capture import CaptureContext, prepare_run, verify_run
from dynamuhsim.runtime.nav2_trial import run_trial
from dynamuhsim.runtime.profiles import PROFILES


DEFAULT_OUTPUT = ROOT / "experiments/social_navigation/datasets/single_policy_v1/generated/pilot_orca_crossing_seed42"


def fresh_run_directory(parent):
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(parent) / ("capture_" + stamp + "_" + uuid.uuid4().hex[:8])


def execute_capture(run_dir):
    capture = CaptureContext(ROOT, run_dir)
    plan = capture.plan
    options = SimpleNamespace(scenario=plan["scenario"], seed=plan["seed"], gui=plan["gui"],
                              rviz=plan.get("rviz", False), condition="people", model=plan.get("human_policy"),
                              report_dir=capture.run_dir / "episode")
    signals = [signal.SIGINT, signal.SIGTERM]
    previous = {sig: signal.getsignal(sig) for sig in signals}
    def interrupted(signum, frame):
        # One interruption enters the runner's finally block. A second signal
        # must not interrupt MCAP flushing and leave Gazebo/recorders orphaned.
        for sig in signals:signal.signal(sig, signal.SIG_IGN)
        raise RuntimeError(f"Interrupted by {signal.Signals(signum).name}; preserving capture and closing owned processes")
    try:
        for sig in signals:signal.signal(sig, interrupted)
        return run_trial(PROFILES[plan.get("trial_profile", "socnavgym_group_nav2")], options,
                         ROOT, Path(__file__).resolve(), capture=capture)
    finally:
        for sig, handler in previous.items():signal.signal(sig, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="mode", required=True)
    pilot = commands.add_parser("pilot", help="Prepare and run ONE ORCA+/crossing/42 episode in a fresh directory")
    pilot.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    pilot.add_argument("--run-dir", type=Path, help="Optional NEW directory; omit to allocate a unique name")
    pilot.add_argument("--ros-domain-id", type=int, default=92)
    pilot.add_argument("--gui", action=argparse.BooleanOptionalAction, default=True)
    pilot.add_argument("--rviz", action=argparse.BooleanOptionalAction, default=True)
    pilot.add_argument("--prepare-only", action="store_true", help="Freeze inputs without starting an episode")
    prepare = commands.add_parser("prepare", help="Create a NEW run/config/source freeze; never overwrite")
    prepare.add_argument("--run-dir", type=Path, required=True)
    prepare.add_argument("--config", type=Path, default=ROOT / "configs/capture/moderate_v3.json")
    prepare.add_argument("--scenario", choices=["standing", "walking", "crossing"], required=True)
    prepare.add_argument("--human-policy", choices=["socnavgym_group", "crowd_orca_plus"], default="socnavgym_group")
    prepare.add_argument("--ros-domain-id", type=int, default=92)
    prepare.add_argument("--seed", type=int, required=True)
    prepare.add_argument("--gui", action="store_true")
    prepare.add_argument("--rviz", action="store_true", help="Start RViz with the same domain, map, TF and simulation clock")
    prepare.add_argument("--planner-diagnostics", action="store_true", help="Record exact native costmaps without changing navigation")
    prepare.add_argument("--human-motion-profile", choices=["legacy", "group_orca_v1", "group_goal_v1", "group_cohesion_v1", "group_cohesion_v2"], default="legacy")
    prepare.add_argument("--encounter-profile", choices=["head_on", "crossing"], help="Opt-in walking initial conditions; native motion continues after start")
    prepare.add_argument("--human-goal-clock", choices=["native_wall", "sim_v1"], default="native_wall", help="Opt-in Gazebo goal expiry from the accepted navigation gate")
    prepare.add_argument("--human-lifecycle-profile", choices=["native", "persistent_conversation_v1"], default="native", help="Opt-in designated persistent standing groups; native lifecycle remains default")
    for name in ["verify", "run"]:
        cmd = commands.add_parser(name)
        cmd.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.mode == "pilot":
        args.run_dir = args.run_dir or fresh_run_directory(args.output_root)
    if args.mode in ["prepare", "pilot"] and args.run_dir.exists():
        parser.error(f"Run directory already exists: {args.run_dir}. Preserved unchanged. Use 'pilot' without --run-dir for a fresh capture.")
    if args.mode == "run" and any((args.run_dir / name).exists() for name in ["episode", "runtime"]):
        parser.error(f"This capture was already attempted: {args.run_dir}. Logs and MCAP are preserved. Run 'pilot' to create a new capture; do not rerun prepare/run on capture_02.")
    if args.mode == "pilot":
        prepare_run(ROOT, args.run_dir, ROOT / "configs/capture/moderate_v3.json", "crossing", 42,
                    args.gui, human_policy="crowd_orca_plus", ros_domain_id=args.ros_domain_id, rviz=args.rviz)
        print("Prepared new single episode:", args.run_dir.resolve(), flush=True)
        if args.prepare_only:return 0
        code = execute_capture(args.run_dir)
        manifest = args.run_dir / "manifest.json"
        if manifest.exists():
            receipt = json.loads(manifest.read_text())
            print("Task 01:", receipt["status"], "—", manifest.resolve(), flush=True)
            return code if code else (0 if receipt["status"] == "PASS" else 3)
        return code
    if args.mode == "prepare":
        prepare_run(ROOT, args.run_dir, args.config, args.scenario, args.seed, args.gui, planner_diagnostics=args.planner_diagnostics, human_motion_profile=args.human_motion_profile, encounter_profile=args.encounter_profile, human_goal_clock=args.human_goal_clock, human_lifecycle_profile=args.human_lifecycle_profile, human_policy=args.human_policy, ros_domain_id=args.ros_domain_id, rviz=args.rviz)
        print("Prepared new capture freeze:", args.run_dir.resolve())
        return 0
    verify_run(ROOT, args.run_dir)
    if args.mode == "verify":
        print("PASS: new capture freeze is unchanged")
        return 0
    return execute_capture(args.run_dir)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print("Capture stopped:", error, file=sys.stderr)
        raise SystemExit(2)
