#!/usr/bin/env python3
"""DynamuhSim Corridor Navigation Evaluation CLI.

A modular, backward-compatible CLI facade that coordinates offline evaluation
of narrow dynamic corridor trials using the `evaluation` package.

Inputs are read-only: a rosbag2 directory and an obstacle ground-truth CSV.
Outputs written to --output include:
  - summary.json
  - report.txt
  - time_series.csv
  - cmd_vel_debug.csv
  - cmd_vel_high_angular.csv
  - trajectory_2d.png
  - clearance_time.png
  - velocity_profile.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

from evaluation import CorridorTrialAnalyzer, FailureType, SocialSpace, TrialMetrics
from evaluation.spatial_alignment import DEFAULT_SPAWN_X, DEFAULT_SPAWN_Y

DEFAULT_BASE = "~/nav_ws/experiments/corridor_090/baseline_01"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # File I/O
    p.add_argument(
        "--bag",
        type=Path,
        default=Path(DEFAULT_BASE).expanduser() / "trial_02",
        help="rosbag2 MCAP directory",
    )
    p.add_argument(
        "--ground-truth",
        type=Path,
        default=Path(DEFAULT_BASE).expanduser() / "obstacle_ground_truth_02.csv",
        help="CSV with simulation time, obstacle x, obstacle y",
    )
    p.add_argument(
        "--output",
        type=Path,
        default=Path(DEFAULT_BASE).expanduser() / "baseline_analysis",
        help="output directory for analysis reports, CSVs, and plots",
    )

    # Robot & Obstacle Geometry
    p.add_argument("--obstacle-radius", type=float, default=0.22, help="obstacle circular radius (m)")
    p.add_argument("--robot-radius", type=float, default=0.105, help="TurtleBot3 Burger approximate circular radius (m)")
    p.add_argument("--spawn-x", type=float, default=DEFAULT_SPAWN_X, help="robot spawn x in Gazebo world frame (m)")
    p.add_argument("--spawn-y", type=float, default=DEFAULT_SPAWN_Y, help="robot spawn y in Gazebo world frame (m)")
    p.add_argument("--corridor-width", type=float, default=0.90, help="corridor width (m)")

    # Encounter Hysteresis & Proxemics
    p.add_argument("--encounter-distance", type=float, default=1.50, help="center-distance threshold for response proxy (m)")
    p.add_argument("--encounter-exit-distance", type=float, default=1.80, help="exit threshold for encounter hysteresis (m)")
    p.add_argument("--encounter-min-duration", type=float, default=0.50, help="discard shorter encounter regions (s)")

    # Reaction Window & Response Detection
    p.add_argument("--reaction-lookback", type=float, default=5.0, help="search for an early command change before danger entry (s)")
    p.add_argument("--command-baseline-window", type=float, default=5.0, help="reference window before reaction lookback (s)")
    p.add_argument("--reaction-after-closest", type=float, default=3.0, help="search for a late reaction after closest approach (s)")
    p.add_argument("--linear-change-min", type=float, default=0.05, help="minimum linear.x decrease for a command change (m/s)")
    p.add_argument("--angular-change-min", type=float, default=0.08, help="minimum abs(angular.z) increase for a command change (rad/s)")
    p.add_argument("--reaction-hold", type=float, default=0.25, help="command-change persistence required to confirm reaction (s)")
    p.add_argument("--angular-debug-threshold", type=float, default=0.25, help="threshold used for cmd_vel high angular debug CSV (rad/s)")
    p.add_argument("--ground-truth-min-dt", type=float, default=0.001, help="GT segment dt below this is an artifact for speed statistics (s)")
    p.add_argument("--obstacle-max-plausible-speed", type=float, default=5.0, help="GT speed above this is excluded only from filtered speed statistics (m/s)")
    p.add_argument("--post-window", type=float, default=1.0, help="window after reaction for post-reaction command statistics (s)")
    p.add_argument("--stop-speed", type=float, default=0.05, help="speed treated as stopped (m/s)")

    # Goal & Outcome Configuration
    p.add_argument("--goal-x", type=float, default=None, help="goal x coordinate in odom frame (m); omit when trial has no goal record")
    p.add_argument("--goal-y", type=float, default=None, help="goal y coordinate in odom frame (m); omit when trial has no goal record")
    p.add_argument("--goal-tolerance", type=float, default=0.20, help="goal success tolerance (m)")
    p.add_argument("--trial-timeout", type=float, default=180.0, help="trial maximum timeout duration (s)")

    # Visualization
    p.add_argument("--no-plots", action="store_true", help="disable automatic plot generation")

    return p


def main(args_list: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(args_list)

    if (
        args.obstacle_radius < 0
        or args.robot_radius < 0
        or args.encounter_distance <= 0
        or args.encounter_exit_distance < args.encounter_distance
        or args.encounter_min_duration < 0
        or args.reaction_lookback < 0
        or args.command_baseline_window <= 0
        or args.reaction_after_closest < 0
        or args.linear_change_min < 0
        or args.angular_change_min < 0
        or args.reaction_hold < 0
        or args.post_window <= 0
        or args.ground_truth_min_dt < 0
        or args.obstacle_max_plausible_speed <= 0
        or args.goal_tolerance <= 0
        or (args.goal_x is None) != (args.goal_y is None)
    ):
        parser.error("Invalid radius, encounter hysteresis, response window, or goal arguments.")

    args.bag = args.bag.expanduser().resolve()
    args.ground_truth = args.ground_truth.expanduser().resolve()
    args.output = args.output.expanduser().resolve()

    if not args.bag.is_dir():
        parser.error(f"bag directory does not exist: {args.bag}")
    if not args.ground_truth.is_file():
        parser.error(f"ground-truth CSV does not exist: {args.ground_truth}")

    analyzer = CorridorTrialAnalyzer(
        obstacle_radius=args.obstacle_radius,
        robot_radius=args.robot_radius,
        encounter_distance=args.encounter_distance,
        encounter_exit_distance=args.encounter_exit_distance,
        encounter_min_duration=args.encounter_min_duration,
        reaction_lookback=args.reaction_lookback,
        command_baseline_window=args.command_baseline_window,
        reaction_after_closest=args.reaction_after_closest,
        linear_change_min=args.linear_change_min,
        angular_change_min=args.angular_change_min,
        reaction_hold=args.reaction_hold,
        angular_debug_threshold=args.angular_debug_threshold,
        ground_truth_min_dt=args.ground_truth_min_dt,
        obstacle_max_plausible_speed=args.obstacle_max_plausible_speed,
        post_window=args.post_window,
        stop_speed=args.stop_speed,
        spawn_x=args.spawn_x,
        spawn_y=args.spawn_y,
        corridor_width=args.corridor_width,
        trial_timeout_s=args.trial_timeout,
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        goal_tolerance=args.goal_tolerance,
        generate_plots=not args.no_plots,
    )

    try:
        report = analyzer.analyze(
            bag_path=args.bag,
            ground_truth_path=args.ground_truth,
            output_dir=args.output,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(f"Wrote {args.output / 'report.txt'}")
    print(f"Wrote {args.output / 'summary.json'}")
    print(f"Wrote {args.output / 'time_series.csv'}")
    print(f"Wrote {args.output / 'cmd_vel_debug.csv'}")
    print(f"Wrote {args.output / 'cmd_vel_high_angular.csv'}")
    if not args.no_plots and "generated_plots" in report:
        for plot_name, plot_path in report["generated_plots"].items():
            print(f"Wrote {plot_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
