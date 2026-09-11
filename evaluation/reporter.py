"""Reporting utilities: Export JSON, human-readable text report, and CSV logs."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

from evaluation.types import format_num


def sanitize_for_json(data: Any) -> Any:
    """Recursively convert non-finite floats (inf, -inf, nan) to None for JSON compliance."""
    if isinstance(data, dict):
        return {k: sanitize_for_json(v) for k, v in data.items()}
    if isinstance(data, (list, tuple)):
        return [sanitize_for_json(v) for v in data]
    if isinstance(data, float) and not math.isfinite(data):
        return None
    return data


def write_summary_json(report: Dict[str, Any], output_path: Path) -> None:
    """Export complete trial metrics dictionary to JSON."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    clean_report = sanitize_for_json(report)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(clean_report, f, indent=2, allow_nan=False)
        f.write("\n")


def write_time_series_csv(
    time_series_data: Sequence[Dict[str, Any]],
    output_path: Path,
) -> None:
    """Export synchronized simulation time series to CSV."""
    if not time_series_data:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(time_series_data[0].keys())
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(time_series_data)


def write_cmd_vel_debug_csvs(
    cmd_debug_rows: Sequence[Dict[str, Any]],
    output_dir: Path,
    high_angular_threshold: float = 0.25,
) -> None:
    """Export cmd_vel mapping debug tables."""
    if not cmd_debug_rows:
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    cols = ["cmd_storage_time_s", "cmd_sim_time", "linear_x", "angular_z", "encounter_id", "center_distance", "clock_bracket_gap_s"]

    with (output_dir / "cmd_vel_debug.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=cols)
        writer.writeheader()
        writer.writerows(cmd_debug_rows)

    high_angular = [r for r in cmd_debug_rows if abs(r["angular_z"]) >= high_angular_threshold]
    with (output_dir / "cmd_vel_high_angular.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=cols)
        writer.writeheader()
        writer.writerows(high_angular)


def generate_text_report(report: Dict[str, Any], output_path: Path) -> None:
    """Generate comprehensive scientific ASCII report."""
    tr = report.get("time_ranges_s", {})
    od = report.get("odometry", {})
    cmd = report.get("cmd_vel", {})
    scan = report.get("scan", {})
    obs = report.get("obstacle_ground_truth", {})
    prox = report.get("robot_obstacle", {})
    rsp = report.get("response_proxy", {})
    outcome = report.get("outcome", {})
    social = report.get("social_metrics", {})
    yielding = report.get("yielding_metrics", {})
    motion = report.get("motion_metrics", {})
    planner = report.get("planner", {})
    encounters = rsp.get("encounters", [])

    lines = [
        "=" * 78,
        " DYNAMUHSIM EXPERIMENTAL EVALUATION REPORT",
        "=" * 78,
        "",
        "1. TRIAL SUMMARY & OUTCOME CLASSIFICATION",
        "-" * 78,
        f"  Outcome:               {outcome.get('failure_type', 'UNKNOWN')}",
        f"  Explanation:           {outcome.get('failure_explanation', 'n/a')}",
        f"  Goal Reached:          {'YES' if outcome.get('goal_success') else 'NO'}",
        f"  Final Distance to Goal:{format_num(outcome.get('final_distance_m'))} m (tolerance: {format_num(outcome.get('goal_tolerance_m'))} m)",
        f"  Travel Time:           {format_num(outcome.get('travel_time_s'))} s",
        f"  Total Path Length:     {format_num(motion.get('total_path_length_m', od.get('distance_m')))} m",
        f"  Path Efficiency Ratio: {format_num(motion.get('path_efficiency_ratio'))}",
        f"  Collision Contact:     {'DETECTED' if outcome.get('collision_contact') else 'NONE'}",
        "",
        "2. SOCIAL PROXEMICS & PERSONAL SPACE INTRUSION (PSI)",
        "-" * 78,
        f"  Intimate Space (< {social.get('intimate_threshold_m', 0.45):.2f}m) Intrusion Time:  {format_num(social.get('total_intimate_time_s'))} s ({format_num(social.get('intimate_time_ratio', 0.0) * 100.0, 1)}% of trial)",
        f"  Intimate Space Intrusion Events:    {social.get('intimate_intrusions_count', 0)}",
        f"  Personal Space (< {social.get('personal_threshold_m', 1.20):.2f}m) Intrusion Time:  {format_num(social.get('total_personal_time_s'))} s ({format_num(social.get('personal_time_ratio', 0.0) * 100.0, 1)}% of trial)",
        f"  Minimum Clearance to Obstacle:      {format_num(social.get('minimum_clearance_m', prox.get('estimated_clearance_m', {}).get('min')))} m",
        f"  Minimum Center-to-Center Distance:  {format_num(social.get('minimum_distance_m', prox.get('center_distance_m', {}).get('min')))} m",
        "",
        "3. PROACTIVE YIELDING & ALCOVE EVASION",
        "-" * 78,
        f"  Yielding Detected (In Alcove):      {'YES' if yielding.get('yielding_detected') else 'NO'}",
        f"  Alcove Entry / Exit Time:           {format_num(yielding.get('alcove_entry_time_s'))} s / {format_num(yielding.get('alcove_exit_time_s'))} s",
        f"  Total Time Spent Inside Alcove:     {format_num(yielding.get('alcove_duration_s'))} s",
        f"  Maximum Alcove Lateral Depth:       {format_num(yielding.get('max_alcove_depth_reached_m'))} m",
        f"  Yielding Initiation Time:           {format_num(yielding.get('yielding_initiation_time_s'))} s",
        f"  Distance to Obstacle at Yielding:   {format_num(yielding.get('distance_to_obstacle_at_yielding_m'))} m",
        "",
        "4. MOTION SMOOTHNESS & ACCELERATION (JERK METRICS)",
        "-" * 78,
        f"  Linear Jerk RMS:                    {format_num(motion.get('linear_jerk_rms_mps3'))} m/s³",
        f"  Angular Jerk RMS:                   {format_num(motion.get('angular_jerk_rms_radps3'))} rad/s³",
        f"  Dimensionless Jerk Cost:            {format_num(motion.get('normalized_jerk_cost'))}",
        f"  Linear Velocity Median / P95:       {format_num(cmd.get('linear_x_mps', {}).get('median'))} / {format_num(cmd.get('linear_x_mps', {}).get('p95'))} m/s",
        f"  Angular Velocity |w| Median / P95:  {format_num(cmd.get('angular_z_radps', {}).get('median'))} / {format_num(cmd.get('angular_z_radps', {}).get('p95'))} rad/s",
        "",
        "5. ENCOUNTERS & REACTION RESPONSE PROXY",
        "-" * 78,
        f"  Encounters Detected:                {rsp.get('encounters_detected', 0)}",
        f"  Classifications:                    TIMELY={rsp.get('classification_counts', {}).get('TIMELY', 0)}, LATE={rsp.get('classification_counts', {}).get('LATE', 0)}, NO_RESPONSE={rsp.get('classification_counts', {}).get('NO_DETECTED_RESPONSE', 0)}",
    ]

    for e in encounters:
        lines.extend([
            f"  * Encounter {e.get('id')}: Time [{format_num(e.get('encounter_time_s'))}s - {format_num(e.get('encounter_end_s'))}s] | Min Clearance: {format_num(e.get('minimum_estimated_clearance_m'))}m",
            f"    Classification: {e.get('reaction_label')} (onset: {format_num(e.get('reaction_onset_s'))}s, modes: {', '.join(e.get('reaction_modes', [])) or 'none'})",
            f"    Reaction delay: {format_num(e.get('reaction_delay_s'))}s | Lead time: {format_num(e.get('lead_time_s'))}s | Distance at reaction: {format_num(e.get('distance_at_reaction_m'))}m",
        ])

    lines.extend([
        "",
        "6. DATA INTEGRITY & AUDIT",
        "-" * 78,
        f"  Odometry Samples:                   {od.get('samples', 0)} ({format_num(tr.get('odom_sim', [0, 0])[0])}s to {format_num(tr.get('odom_sim', [0, 0])[1])}s)",
        f"  Ground-Truth Obstacle Samples:      {obs.get('samples', 0)} ({format_num(tr.get('ground_truth', [0, 0])[0])}s to {format_num(tr.get('ground_truth', [0, 0])[1])}s)",
        f"  cmd_vel Samples Mapped:             {cmd.get('samples', 0)} (unmapped: {cmd.get('unmapped_samples', 0)})",
        f"  Planner Updates (Global / Local):   {planner.get('plan', {}).get('updates', 0)} / {planner.get('local_plan', {}).get('updates', 0)}",
        "=" * 78,
    ])

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
