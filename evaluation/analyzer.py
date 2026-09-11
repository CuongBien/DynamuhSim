"""Core evaluation orchestrator: Coordinates data loading, metric computation, failure taxonomy, plotting, and reporting."""
from __future__ import annotations

import argparse
import math
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from evaluation.bag_reader import clock_bracket, map_storage_to_clock, read_bag_data
from evaluation.failure_taxonomy import classify_failure
from evaluation.ground_truth import compute_ground_truth_speed_series, interpolate_xy, read_ground_truth
from evaluation.motion_metrics import compute_motion_metrics, compute_speed_series
from evaluation.plotting import generate_trial_plots
from evaluation.reporter import (
    generate_text_report,
    write_cmd_vel_debug_csvs,
    write_summary_json,
    write_time_series_csv,
)
from evaluation.social_metrics import (
    command_change_reaction,
    compute_psi_metrics,
    compute_yielding_metrics,
    detect_encounters,
    duration_where,
    median_mad,
)
from evaluation.spatial_alignment import (
    DEFAULT_SPAWN_X,
    DEFAULT_SPAWN_Y,
    align_obstacle_to_robot_frame,
    is_in_alcove,
)
from evaluation.types import FailureType, compute_stats


class CorridorTrialAnalyzer:
    """Orchestrates comprehensive offline evaluation of a single dynamic corridor navigation trial."""

    def __init__(
        self,
        obstacle_radius: float = 0.22,
        robot_radius: float = 0.105,
        encounter_distance: float = 1.50,
        encounter_exit_distance: float = 1.80,
        encounter_min_duration: float = 0.50,
        reaction_lookback: float = 5.0,
        command_baseline_window: float = 5.0,
        reaction_after_closest: float = 3.0,
        linear_change_min: float = 0.05,
        angular_change_min: float = 0.08,
        reaction_hold: float = 0.25,
        angular_debug_threshold: float = 0.25,
        ground_truth_min_dt: float = 0.001,
        obstacle_max_plausible_speed: float = 5.0,
        post_window: float = 1.0,
        stop_speed: float = 0.05,
        spawn_x: float = DEFAULT_SPAWN_X,
        spawn_y: float = DEFAULT_SPAWN_Y,
        corridor_width: float = 0.90,
        trial_timeout_s: float = 180.0,
        goal_x: Optional[float] = None,
        goal_y: Optional[float] = None,
        goal_tolerance: float = 0.20,
        generate_plots: bool = True,
    ) -> None:
        self.obstacle_radius = obstacle_radius
        self.robot_radius = robot_radius
        self.encounter_distance = encounter_distance
        self.encounter_exit_distance = encounter_exit_distance
        self.encounter_min_duration = encounter_min_duration
        self.reaction_lookback = reaction_lookback
        self.command_baseline_window = command_baseline_window
        self.reaction_after_closest = reaction_after_closest
        self.linear_change_min = linear_change_min
        self.angular_change_min = angular_change_min
        self.reaction_hold = reaction_hold
        self.angular_debug_threshold = angular_debug_threshold
        self.ground_truth_min_dt = ground_truth_min_dt
        self.obstacle_max_plausible_speed = obstacle_max_plausible_speed
        self.post_window = post_window
        self.stop_speed = stop_speed
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        self.corridor_width = corridor_width
        self.trial_timeout_s = trial_timeout_s
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_tolerance = goal_tolerance
        self.generate_plots_flag = generate_plots

    def analyze(
        self,
        bag_path: Path,
        ground_truth_path: Path,
        output_dir: Path,
    ) -> Dict[str, Any]:
        """Run full evaluation pipeline and write results to output_dir."""
        output_dir = output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)

        # 1. Read input data
        raw_gt = read_ground_truth(ground_truth_path)
        bag_data = read_bag_data(bag_path)

        odom = bag_data["odom"]
        if not odom:
            raise RuntimeError("/odom has no usable messages in bag")

        clock = bag_data["clock"]
        clock_storage_times = [p[0] for p in clock]
        clock_sim_times = [p[1] for p in clock]

        # 2. Map /cmd_vel timestamps
        cmd: List[Tuple[float, float, float]] = []
        mapped_cmd: List[Tuple[float, float, float, float, Optional[float]]] = []
        unmapped_cmd = 0
        clock_bracket_gaps = []

        for storage_t, linear, angular in bag_data["cmd_raw"]:
            sim_t = map_storage_to_clock(clock_storage_times, clock_sim_times, storage_t)
            bracket = clock_bracket(clock_storage_times, storage_t)
            if sim_t is not None:
                cmd.append((sim_t, linear, angular))
                gap = bracket[1] if bracket else None
                mapped_cmd.append((storage_t, sim_t, linear, angular, gap))
                if gap is not None:
                    clock_bracket_gaps.append(gap)
            else:
                unmapped_cmd += 1

        # 3. Spatial alignment
        # Align obstacle ground-truth into robot frame
        gt_aligned = align_obstacle_to_robot_frame(raw_gt, spawn_x=self.spawn_x, spawn_y=self.spawn_y)
        gt_times = [r[0] for r in gt_aligned]

        robot_xy = [(t, x, y) for t, x, y, _ in odom]
        robot_times = [r[0] for r in robot_xy]
        total_trial_time = odom[-1][0] - odom[0][0]

        # 4. Proximity calculation
        proximity: List[Tuple[float, float, float]] = []
        time_series_rows: List[Dict[str, Any]] = []

        for t, rx, ry in robot_xy:
            obs_xy = interpolate_xy(gt_aligned, t, gt_times)
            if obs_xy is not None:
                center_d = math.hypot(rx - obs_xy[0], ry - obs_xy[1])
                clearance = center_d - self.obstacle_radius - self.robot_radius
                proximity.append((t, center_d, clearance))

                in_alcove_status = is_in_alcove(rx, ry, frame="odom", spawn_x=self.spawn_x)
                time_series_rows.append({
                    "sim_time_s": t,
                    "robot_x_m": rx,
                    "robot_y_m": ry,
                    "obstacle_x_m": obs_xy[0],
                    "obstacle_y_m": obs_xy[1],
                    "center_distance_m": center_d,
                    "estimated_clearance_m": clearance,
                    "is_intimate_intrusion": center_d < 0.45,
                    "in_alcove": in_alcove_status,
                })

        # 5. Social Navigation Metrics (PSI & Proactive Yielding)
        psi_metrics = compute_psi_metrics(proximity, total_trial_time)

        obs_interpolated_for_robot = [
            interpolate_xy(gt_aligned, t, gt_times) for t, _, _ in robot_xy
        ]
        yielding_metrics = compute_yielding_metrics(
            robot_xy, obs_interpolated_for_robot, spawn_x=self.spawn_x
        )

        # 6. Motion Quality & Jerk Smoothness
        goal_tuple = (self.goal_x, self.goal_y) if (self.goal_x is not None and self.goal_y is not None) else None
        motion_metrics = compute_motion_metrics(robot_xy, cmd, goal_xy=goal_tuple)

        # 7. Encounters & Reaction Analysis
        raw_encounters = detect_encounters(
            proximity,
            enter=self.encounter_distance,
            exit_=self.encounter_exit_distance,
            min_duration=self.encounter_min_duration,
        )

        encounter_reports: List[Dict[str, Any]] = []
        last_assigned_reaction: Optional[float] = None
        closest_rows = [min(ev["samples"], key=lambda r: r[1]) for ev in raw_encounters]

        for idx, event in enumerate(raw_encounters, start=1):
            e_start, e_end, samples = event["start_s"], event["end_s"], event["samples"]
            closest_time = closest_rows[idx - 1][0]
            prev_end = raw_encounters[idx - 2]["end_s"] if idx > 1 else float("-inf")

            search_start = max(e_start - self.reaction_lookback, prev_end)
            if last_assigned_reaction is not None:
                search_start = max(search_start, last_assigned_reaction + self.reaction_hold)

            context_end = closest_time + self.reaction_after_closest
            if idx < len(raw_encounters):
                context_end = min(context_end, raw_encounters[idx]["start_s"])
            if cmd:
                search_start = max(cmd[0][0], search_start)
                context_end = min(cmd[-1][0], context_end)

            ref_start = search_start - self.command_baseline_window
            linear_ref = [x for t, x, _ in cmd if ref_start <= t < search_start]
            angular_ref = [abs(z) for t, _, z in cmd if ref_start <= t < search_start]
            lin_base, lin_mad = median_mad(linear_ref)
            ang_base, ang_mad = median_mad(angular_ref)

            reaction, reaction_modes, r_lin_x, r_ang_z = (
                command_change_reaction(
                    cmd, search_start, context_end, lin_base, lin_mad, ang_base, ang_mad,
                    self.linear_change_min, self.angular_change_min, self.reaction_hold,
                )
                if (lin_base is not None and ang_base is not None and lin_mad is not None and ang_mad is not None)
                else (None, [], None, None)
            )

            if reaction is None:
                classification = "NO_DETECTED_RESPONSE"
            elif reaction <= closest_time:
                classification = "TIMELY"
            else:
                classification = "LATE"

            if reaction is not None:
                last_assigned_reaction = reaction

            stop_tot, stop_long = (
                duration_where([(t, abs(x)) for t, x, _ in cmd], self.stop_speed, reaction, context_end)
                if reaction is not None else (None, None)
            )

            # Robot-obstacle distance at reaction
            r_pos = interpolate_xy(robot_xy, reaction, robot_times) if reaction else None
            obs_pos = interpolate_xy(gt_aligned, reaction, gt_times) if reaction else None
            dist_at_react = math.hypot(r_pos[0] - obs_pos[0], r_pos[1] - obs_pos[1]) if (r_pos and obs_pos) else None

            encounter_reports.append({
                "id": idx,
                "encounter_time_s": e_start,
                "encounter_end_s": e_end,
                "duration_s": e_end - e_start,
                "reaction_search_start_s": search_start,
                "reaction_search_end_s": context_end,
                "minimum_center_distance_m": min(r[1] for r in samples),
                "minimum_estimated_clearance_m": min(r[2] for r in samples),
                "closest_approach_time_s": closest_time,
                "reaction_onset_s": reaction,
                "lead_time_s": (e_start - reaction) if reaction is not None else None,
                "reaction_delay_s": (reaction - closest_time) if reaction is not None else None,
                "reaction_classification": classification,
                "reaction_label": classification,
                "reaction_modes": reaction_modes,
                "distance_at_reaction_m": dist_at_react,
                "reaction_linear_x_mps": r_lin_x,
                "reaction_angular_z_radps": r_ang_z,
                "stop_total_s_after_reaction_within_response_window": stop_tot,
                "longest_continuous_stop_s_after_reaction_within_response_window": stop_long,
            })

        # Tag time series rows with encounter IDs
        for row in time_series_rows:
            t = row["sim_time_s"]
            active_ids = [
                str(e["id"]) for e in encounter_reports
                if e["encounter_time_s"] <= t <= e["encounter_end_s"]
            ]
            row["encounter_id"] = ";".join(active_ids)

        # 8. Command debug logging
        cmd_debug: List[Dict[str, Any]] = []
        for storage_t, cmd_t, linear_x, angular_z, bracket_gap in mapped_cmd:
            r_at_cmd = interpolate_xy(robot_xy, cmd_t, robot_times)
            obs_at_cmd = interpolate_xy(gt_aligned, cmd_t, gt_times)
            c_dist = math.hypot(r_at_cmd[0] - obs_at_cmd[0], r_at_cmd[1] - obs_at_cmd[1]) if (r_at_cmd and obs_at_cmd) else None
            matching_encs = [
                e["id"] for e in encounter_reports
                if e["encounter_time_s"] <= cmd_t <= e["encounter_end_s"]
            ]
            cmd_debug.append({
                "cmd_storage_time_s": storage_t,
                "cmd_sim_time": cmd_t,
                "linear_x": linear_x,
                "angular_z": angular_z,
                "encounter_id": matching_encs[0] if matching_encs else None,
                "center_distance": c_dist,
                "clock_bracket_gap_s": bracket_gap,
            })

        # 9. Failure Taxonomy Classification
        failure_type, failure_reason, outcome_details = classify_failure(
            odom_samples=odom,
            proximity_samples=proximity,
            cmd_samples=cmd,
            goal_xy=goal_tuple,
            goal_tolerance=self.goal_tolerance,
            trial_timeout_s=self.trial_timeout_s,
        )

        outcome_dict = {
            "failure_type": failure_type.value,
            "failure_explanation": failure_reason,
            "failure": None if failure_type == FailureType.SUCCESS else failure_type.value,
            "goal_success": (failure_type == FailureType.SUCCESS) if goal_tuple is not None else None,
            "travel_time_s": total_trial_time,
            "path_length_m": motion_metrics.total_path_length_m,
            "collision_contact": failure_type == FailureType.COLLISION_OBSTACLE,
            "collision_contact_basis": "proxy: estimated clearance <= 0 m; confirm with contact sensor for official contact result",
            **outcome_details,
        }

        # 10. Obstacle Speed Quality Analysis
        obs_raw_speeds, obs_filt_speeds, obs_speed_quality = compute_ground_truth_speed_series(
            raw_gt, self.ground_truth_min_dt, self.obstacle_max_plausible_speed
        )

        # 11. Assemble comprehensive report dict
        clock_range = (clock[0][1], clock[-1][1]) if clock else None
        odom_range = (odom[0][0], odom[-1][0])
        sim_range = clock_range or odom_range
        gt_range = (raw_gt[0][0], raw_gt[-1][0])

        report: Dict[str, Any] = {
            "framework_version": "v2.0",
            "assumptions": {
                "obstacle_radius_m": self.obstacle_radius,
                "robot_radius_m": self.robot_radius,
                "spawn_x_m": self.spawn_x,
                "spawn_y_m": self.spawn_y,
                "corridor_width_m": self.corridor_width,
            },
            "outcome": outcome_dict,
            "social_metrics": {
                "intimate_threshold_m": psi_metrics.intimate_threshold_m,
                "personal_threshold_m": psi_metrics.personal_threshold_m,
                "total_intimate_time_s": psi_metrics.total_intimate_time_s,
                "intimate_time_ratio": psi_metrics.intimate_time_ratio,
                "total_personal_time_s": psi_metrics.total_personal_time_s,
                "personal_time_ratio": psi_metrics.personal_time_ratio,
                "minimum_distance_m": psi_metrics.minimum_distance_m,
                "minimum_clearance_m": psi_metrics.minimum_clearance_m,
                "time_at_min_distance_s": psi_metrics.time_at_min_distance_s,
                "intimate_intrusions_count": psi_metrics.intimate_intrusions_count,
            },
            "yielding_metrics": {
                "yielding_detected": yielding_metrics.yielding_detected,
                "entered_alcove": yielding_metrics.entered_alcove,
                "alcove_entry_time_s": yielding_metrics.alcove_entry_time_s,
                "alcove_exit_time_s": yielding_metrics.alcove_exit_time_s,
                "alcove_duration_s": yielding_metrics.alcove_duration_s,
                "yielding_initiation_time_s": yielding_metrics.yielding_initiation_time_s,
                "distance_to_obstacle_at_yielding_m": yielding_metrics.distance_to_obstacle_at_yielding_m,
                "max_alcove_depth_reached_m": yielding_metrics.max_alcove_depth_reached_m,
            },
            "motion_metrics": {
                "total_path_length_m": motion_metrics.total_path_length_m,
                "path_efficiency_ratio": motion_metrics.path_efficiency_ratio,
                "linear_jerk_rms_mps3": motion_metrics.linear_jerk_rms_mps3,
                "angular_jerk_rms_radps3": motion_metrics.angular_jerk_rms_radps3,
                "normalized_jerk_cost": motion_metrics.normalized_jerk_cost,
                "linear_acceleration_mps2": motion_metrics.linear_acceleration_stats,
                "angular_acceleration_radps2": motion_metrics.angular_acceleration_stats,
            },
            "time_ranges_s": {
                "bag_storage": bag_data["storage_range"],
                "bag_storage_duration": (bag_data["storage_range"][1] - bag_data["storage_range"][0]) if bag_data["storage_range"][0] else None,
                "clock_sim": clock_range,
                "odom_sim": odom_range,
                "ground_truth": gt_range,
                "ground_truth_missing_after_s": max(0.0, sim_range[1] - gt_range[1]),
            },
            "message_counts": dict(bag_data["counts"]),
            "odometry": {
                "samples": len(odom),
                "distance_m": motion_metrics.total_path_length_m,
                "twist_linear_speed_mps": compute_stats([v for _, _, _, v in odom]),
                "position_derived_speed_mps": compute_stats([v for _, v in compute_speed_series(robot_xy)]),
            },
            "cmd_vel": {
                "samples": len(cmd),
                "unmapped_samples": unmapped_cmd,
                "clock_bracket_gap_s": compute_stats(clock_bracket_gaps),
                "linear_x_mps": compute_stats([v for _, v, _ in cmd]),
                "angular_z_radps": compute_stats([w for _, _, w in cmd]),
            },
            "cmd_vel_mapping_debug": {
                "file": "cmd_vel_debug.csv",
                "high_angular_file": "cmd_vel_high_angular.csv",
                "high_angular_threshold_radps": self.angular_debug_threshold,
                "all_mapped_samples": len(cmd_debug),
                "high_angular_samples": len([r for r in cmd_debug if abs(r["angular_z"]) >= self.angular_debug_threshold]),
                "high_angular_samples_in_response_window": sum(r["encounter_id"] is not None for r in cmd_debug if abs(r["angular_z"]) >= self.angular_debug_threshold),
                "mapped_sim_time_range_s": [cmd_debug[0]["cmd_sim_time"], cmd_debug[-1]["cmd_sim_time"]] if cmd_debug else None,
            },
            "scan": {
                "samples": len(bag_data["scan"]),
                "minimum_range_m": compute_stats([r for _, r in bag_data["scan"] if r is not None]),
            },
            "obstacle_ground_truth": {
                "samples": len(raw_gt),
                "speed_mps": compute_stats([v for _, v in obs_filt_speeds]),
                "raw_speed_mps": compute_stats([v for _, v in obs_raw_speeds]),
                "speed_quality_filter": obs_speed_quality,
            },
            "robot_obstacle": {
                "matched_odom_samples": len(proximity),
                "center_distance_m": compute_stats([d for _, d, _ in proximity]),
                "estimated_clearance_m": compute_stats([c for _, _, c in proximity]),
            },
            "response_proxy": {
                "encounters_detected": len(encounter_reports),
                "classification_counts": dict(Counter(e["reaction_classification"] for e in encounter_reports)),
                "encounters": encounter_reports,
            },
            "planner": {
                "plan": {"updates": len(bag_data["plan"]), "path_length_m": compute_stats([r[1] for r in bag_data["plan"] if r[1] is not None])},
                "local_plan": {"updates": len(bag_data["local_plan"]), "path_length_m": compute_stats([r[1] for r in bag_data["local_plan"] if r[1] is not None])},
            },
            "raw_measurements": {
                "odom": [{"sim_time_s": t, "x_m": x, "y_m": y, "speed_mps": v} for t, x, y, v in odom],
                "clock": [{"storage_time_s": st, "sim_time_s": sim} for st, sim in clock],
                "cmd_vel": [{"sim_time_s": t, "linear_x_mps": x, "angular_z_radps": z} for t, x, z in cmd],
                "obstacle_ground_truth": [{"sim_time_s": t, "x_m": x, "y_m": y} for t, x, y in gt_aligned],
                "robot_obstacle": [{"sim_time_s": t, "center_distance_m": d, "estimated_clearance_m": c} for t, d, c in proximity],
            },
        }

        # 12. Write outputs
        write_summary_json(report, output_dir / "summary.json")
        generate_text_report(report, output_dir / "report.txt")
        write_time_series_csv(time_series_rows, output_dir / "time_series.csv")
        write_cmd_vel_debug_csvs(cmd_debug, output_dir, self.angular_debug_threshold)

        # 13. Generate Plots
        if self.generate_plots_flag:
            plot_files = generate_trial_plots(
                output_dir=output_dir,
                robot_trajectory=robot_xy,
                obstacle_trajectory=gt_aligned,
                proximity_samples=proximity,
                cmd_samples=cmd,
                encounters=encounter_reports,
                goal_xy=goal_tuple,
                spawn_x=self.spawn_x,
                corridor_width=self.corridor_width,
            )
            report["generated_plots"] = plot_files

        return report
