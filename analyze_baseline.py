#!/usr/bin/env python3
"""Analyze a ROS 2 Jazzy rosbag2 MCAP baseline run without replaying it.

This is deliberately a *baseline/proxy* analysis.  It does not implement or
claim to implement any official K6--K12 metric definitions.

Inputs are read-only: a rosbag2 directory and an obstacle ground-truth CSV.
The script writes report.txt, summary.json, and time_series.csv to --output.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import statistics
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_BASE = "~/nav_ws/experiments/corridor_090/baseline_01"
TOPICS = {"/clock", "/odom", "/cmd_vel", "/scan", "/plan", "/local_plan"}


def stamp_seconds(stamp: Any) -> Optional[float]:
    """Return ROS builtin_interfaces/Time as seconds, or None."""
    if stamp is None or not hasattr(stamp, "sec"):
        return None
    value = float(stamp.sec) + float(getattr(stamp, "nanosec", 0)) * 1e-9
    return value if math.isfinite(value) else None


def header_seconds(msg: Any) -> Optional[float]:
    return stamp_seconds(getattr(getattr(msg, "header", None), "stamp", None))


def finite(values: Iterable[Optional[float]]) -> List[float]:
    return [float(x) for x in values if x is not None and math.isfinite(float(x))]


def percentile(values: Sequence[float], p: float) -> Optional[float]:
    values = sorted(finite(values))
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * p / 100.0
    lo, hi = int(math.floor(position)), int(math.ceil(position))
    return values[lo] + (values[hi] - values[lo]) * (position - lo)


def stats(values: Sequence[float]) -> Dict[str, Optional[float]]:
    values = finite(values)
    if not values:
        return {"count": 0, "min": None, "mean": None, "median": None,
                "p05": None, "p50": None, "p95": None, "max": None}
    return {"count": len(values), "min": min(values), "mean": statistics.fmean(values),
            "median": statistics.median(values), "p05": percentile(values, 5),
            "p50": percentile(values, 50), "p95": percentile(values, 95), "max": max(values)}


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def distance2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_length(path: Any) -> Optional[float]:
    poses = getattr(path, "poses", None)
    if not poses:
        return 0.0 if poses is not None else None
    points = []
    for pose_stamped in poses:
        pos = pose_stamped.pose.position
        points.append((float(pos.x), float(pos.y), float(pos.z)))
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def read_ground_truth(path: Path) -> List[Tuple[float, float, float]]:
    """Read a headered CSV with flexible common time/x/y column names."""
    try:
        with path.open("r", newline="", encoding="utf-8-sig") as f:
            sample = f.read(4096)
            f.seek(0)
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
            reader = csv.DictReader(f, dialect=dialect)
            if not reader.fieldnames:
                raise ValueError("CSV has no header row")
            names = {n.strip().lower().replace(" ", "_"): n for n in reader.fieldnames if n}
            def choose(candidates: Sequence[str]) -> Optional[str]:
                return next((names[x] for x in candidates if x in names), None)
            t_col = choose(("time", "t", "sim_time", "timestamp", "stamp", "time_sec", "seconds"))
            x_col = choose(("x", "pose_x", "position_x", "obstacle_x", "world_x"))
            y_col = choose(("y", "pose_y", "position_y", "obstacle_y", "world_y"))
            if not all((t_col, x_col, y_col)):
                raise ValueError("need time/x/y columns; found: " + ", ".join(reader.fieldnames))
            rows = []
            for line, row in enumerate(reader, start=2):
                try:
                    t, x, y = float(row[t_col]), float(row[x_col]), float(row[y_col])
                    # A common export uses integer nanoseconds.  Convert only obvious values.
                    if abs(t) > 1e12:
                        t *= 1e-9
                    if all(math.isfinite(v) for v in (t, x, y)):
                        rows.append((t, x, y))
                except (TypeError, ValueError):
                    print(f"warning: ignored malformed ground-truth row {line}", file=sys.stderr)
    except OSError as exc:
        raise RuntimeError(f"cannot read ground-truth CSV {path}: {exc}") from exc
    rows.sort(key=lambda r: r[0])
    if len(rows) < 2:
        raise RuntimeError("ground-truth CSV needs at least two valid time/x/y rows")
    return rows


def interpolate_xy(rows: Sequence[Tuple[float, float, float]], t: float,
                   times: Optional[Sequence[float]] = None) -> Optional[Tuple[float, float]]:
    """Linear interpolation; deliberately returns None outside measured coverage."""
    if t < rows[0][0] or t > rows[-1][0]:
        return None
    times = times if times is not None else [r[0] for r in rows]
    i = bisect.bisect_left(times, t)
    if i == 0:
        return rows[0][1], rows[0][2]
    if i == len(rows):
        return rows[-1][1], rows[-1][2]
    t0, x0, y0 = rows[i - 1]
    t1, x1, y1 = rows[i]
    if t1 == t0:
        return x0, y0
    q = (t - t0) / (t1 - t0)
    return x0 + q * (x1 - x0), y0 + q * (y1 - y0)


def clock_bracket(clock_storage_times: Sequence[float], storage_t: float) -> Optional[Tuple[int, float]]:
    """Return the right /clock index and its enclosing storage-time gap."""
    if not clock_storage_times or storage_t < clock_storage_times[0] or storage_t > clock_storage_times[-1]:
        return None
    i = bisect.bisect_left(clock_storage_times, storage_t)
    if i == 0:
        return i, 0.0
    if i == len(clock_storage_times):
        return i - 1, 0.0
    return i, clock_storage_times[i] - clock_storage_times[i - 1]


def speed_series(samples: Sequence[Tuple[float, float, float]]) -> List[Tuple[float, float]]:
    result = []
    for (t0, x0, y0), (t1, x1, y1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if 1e-6 < dt <= 2.0:
            result.append((t1, math.hypot(x1 - x0, y1 - y0) / dt))
    return result


def ground_truth_speed_series(samples: Sequence[Tuple[float, float, float]], min_dt: float,
                              max_plausible_speed: float) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]], Dict[str, int]]:
    """Return raw and quality-filtered GT segment speeds without hiding artifacts."""
    raw: List[Tuple[float, float]] = []
    filtered: List[Tuple[float, float]] = []
    quality = {"nonpositive_or_tiny_dt": 0, "speed_outlier": 0, "kept": 0}
    for (t0, x0, y0), (t1, x1, y1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0:
            quality["nonpositive_or_tiny_dt"] += 1
            continue
        speed = math.hypot(x1 - x0, y1 - y0) / dt
        raw.append((t1, speed))
        if dt < min_dt:
            quality["nonpositive_or_tiny_dt"] += 1
        elif speed > max_plausible_speed:
            quality["speed_outlier"] += 1
        else:
            filtered.append((t1, speed))
            quality["kept"] += 1
    return raw, filtered, quality


def duration_where(samples: Sequence[Tuple[float, float]], threshold: float, after: float,
                   until: Optional[float] = None) -> Tuple[float, float]:
    """Total and longest continuous stopped duration, clipped to samples after a time."""
    run_start = None
    total = longest = 0.0
    previous_t = None
    for t, value in samples:
        if t < after or (until is not None and t > until):
            continue
        stopped = value <= threshold
        # A gap means it is unsafe to infer continuous stopping through it.
        contiguous = previous_t is not None and t - previous_t <= 1.5
        if stopped and (run_start is None or not contiguous):
            if run_start is not None and previous_t is not None:
                d = previous_t - run_start; total += d; longest = max(longest, d)
            run_start = t
        elif not stopped and run_start is not None:
            d = t - run_start; total += d; longest = max(longest, d); run_start = None
        previous_t = t
    if run_start is not None and previous_t is not None:
        d = previous_t - run_start; total += d; longest = max(longest, d)
    return total, longest


def detect_encounters(proximity: Sequence[Tuple[float, float, float]], enter: float, exit_: float,
                      min_duration: float) -> List[Dict[str, Any]]:
    """Segment close approaches with distance hysteresis, never extrapolating GT."""
    encounters: List[Dict[str, Any]] = []
    active: Optional[List[Tuple[float, float, float]]] = None
    previous: Optional[Tuple[float, float, float]] = None
    for row in proximity:
        t, center, _ = row
        if active is not None and previous is not None and t - previous[0] > 1.5:
            if previous[0] - active[0][0] >= min_duration:
                encounters.append({"start_s": active[0][0], "end_s": previous[0], "samples": active})
            active = None
        if active is None:
            if center <= enter:
                active = [row]
        else:
            active.append(row)
            if center >= exit_:
                if t - active[0][0] >= min_duration:
                    encounters.append({"start_s": active[0][0], "end_s": t, "samples": active})
                active = None
        previous = row
    if active is not None and previous is not None and previous[0] - active[0][0] >= min_duration:
        encounters.append({"start_s": active[0][0], "end_s": previous[0], "samples": active})
    return encounters


def median_mad(values: Sequence[float]) -> Tuple[Optional[float], Optional[float]]:
    """Robust center and natural variation of one command component."""
    good = finite(values)
    if not good:
        return None, None
    center = statistics.median(good)
    return center, statistics.median([abs(v - center) for v in good])


def command_change_reaction(samples: Sequence[Tuple[float, float, float]], start: float, end: float,
                            linear_center: float, linear_mad: float,
                            angular_abs_center: float, angular_abs_mad: float,
                            linear_min_decrease: float, angular_min_increase: float,
                            hold_s: float) -> Tuple[Optional[float], List[str], Optional[float], Optional[float]]:
    """Find a sustained command *change*, not a fixed speed/turn threshold.

    A decrease in linear.x or increase in abs(angular.z) must exceed the larger
    of a small physical floor and three median-absolute-deviations of the
    pre-reaction command baseline.  It must persist for hold_s.
    """
    linear_delta = max(linear_min_decrease, 3.0 * linear_mad)
    angular_delta = max(angular_min_increase, 3.0 * angular_abs_mad)
    candidate: Optional[float] = None
    modes: set[str] = set()
    previous_t: Optional[float] = None
    candidate_linear: Optional[float] = None
    candidate_angular: Optional[float] = None
    for t, linear_x, angular_z in samples:
        if t < start:
            continue
        if t > end:
            break
        changed_modes = []
        if linear_x < linear_center - linear_delta:
            changed_modes.append("linear_x_decrease")
        if abs(angular_z) > angular_abs_center + angular_delta:
            changed_modes.append("angular_z_increase")
        if changed_modes:
            if candidate is None or (previous_t is not None and t - previous_t > 1.5):
                candidate, modes = t, set()
                candidate_linear, candidate_angular = linear_x, angular_z
            modes.update(changed_modes)
            if candidate is not None and t - candidate >= hold_s:
                return candidate, sorted(modes), candidate_linear, candidate_angular
        else:
            candidate, modes = None, set()
            candidate_linear, candidate_angular = None, None
        previous_t = t
    return None, [], None, None


def window_stats(samples: Sequence[Tuple[float, float, float]], start: float, end: float) -> Dict[str, Any]:
    rows = [(linear, angular) for t, linear, angular in samples if start <= t <= end]
    return {"sample_count": len(rows), "linear_x_mps": stats([x for x, _ in rows]),
            "angular_z_radps": stats([z for _, z in rows]),
            "abs_angular_z_radps": stats([abs(z) for _, z in rows])}


def goal_outcome(odom: Sequence[Tuple[float, float, float, float]], args: argparse.Namespace) -> Dict[str, Any]:
    """Report goal outcome without inventing a goal when the trial did not record one."""
    configured = args.goal_x is not None and args.goal_y is not None
    if not odom or not configured:
        return {"goal_configured": configured, "goal_success": None, "progress_m": None,
                "progress_fraction": None, "failure": "UNAVAILABLE_GOAL_NOT_CONFIGURED"}
    goal = (args.goal_x, args.goal_y)
    start = (odom[0][1], odom[0][2])
    finish = (odom[-1][1], odom[-1][2])
    initial_distance = distance2(start, goal)
    final_distance = distance2(finish, goal)
    progress = initial_distance - final_distance
    fraction = progress / initial_distance if initial_distance > 0 else None
    success = final_distance <= args.goal_tolerance
    return {"goal_configured": True, "goal_xy_m": [args.goal_x, args.goal_y],
            "goal_tolerance_m": args.goal_tolerance, "initial_distance_m": initial_distance,
            "final_distance_m": final_distance, "goal_success": success,
            "progress_m": progress, "progress_fraction": fraction,
            "failure": None if success else "GOAL_TOLERANCE_NOT_REACHED"}


def read_bag(bag: Path) -> Dict[str, Any]:
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "ROS Python dependencies are unavailable. Source ROS 2 Jazzy first: "
            "source /opt/ros/jazzy/setup.bash (and source your workspace install/setup.bash if used). "
            f"Original import error: {exc}"
        ) from exc

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
    except Exception as exc:
        raise RuntimeError(f"could not open MCAP bag {bag}: {exc}") from exc
    topic_types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    present = sorted(TOPICS.intersection(topic_types))
    if not present:
        raise RuntimeError("none of the expected topics exist; found: " + ", ".join(sorted(topic_types)))
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    except Exception:
        # Older rosbag2_py bindings can read safely without topic filtering.
        pass
    classes = {name: get_message(topic_types[name]) for name in present}
    out: Dict[str, Any] = {"topic_types": topic_types, "counts": Counter(), "clock": [],
                           "odom": [], "cmd_raw": [], "scan": [], "plan": [], "local_plan": []}
    storage_range: List[int] = []
    while reader.has_next():
        topic, data, storage_ns = reader.read_next()
        if topic not in classes:
            continue
        storage_range.append(storage_ns)
        out["counts"][topic] += 1
        try:
            msg = deserialize_message(data, classes[topic])
        except Exception as exc:
            print(f"warning: could not deserialize {topic}: {exc}", file=sys.stderr)
            continue
        storage_t = storage_ns * 1e-9
        if topic == "/clock":
            value = stamp_seconds(getattr(msg, "clock", None))
            if value is not None:
                out["clock"].append((storage_t, value))
        elif topic == "/odom":
            t = header_seconds(msg)
            p = msg.pose.pose.position
            twist = msg.twist.twist.linear
            if t is not None:
                out["odom"].append((t, float(p.x), float(p.y), math.hypot(float(twist.x), float(twist.y))))
        elif topic == "/cmd_vel":
            linear, angular = msg.linear, msg.angular
            # Preserve signed x velocity.  Avoidance can be visible in yaw even
            # while forward velocity remains at its nominal value.
            out["cmd_raw"].append((storage_t, float(linear.x), float(angular.z)))
        elif topic == "/scan":
            t = header_seconds(msg)
            if t is not None:
                ranges = [float(r) for r in msg.ranges if math.isfinite(float(r)) and float(r) >= float(msg.range_min)]
                out["scan"].append((t, min(ranges) if ranges else None))
        elif topic in ("/plan", "/local_plan"):
            t = header_seconds(msg)
            if t is not None:
                out[topic[1:]].append((t, path_length(msg), len(getattr(msg, "poses", []))))
    out["storage_range"] = (min(storage_range) * 1e-9, max(storage_range) * 1e-9) if storage_range else (None, None)
    for key in ("clock", "odom", "cmd_raw", "scan", "plan", "local_plan"):
        out[key].sort(key=lambda r: r[0])
    return out


def map_storage_to_clock(clock_storage_times: Sequence[float], clock_sim_times: Sequence[float],
                         storage_t: float) -> Optional[float]:
    """Map an unheadered message timestamp through the surrounding /clock samples.

    /cmd_vel has no Header, so rosbag storage time is its only timestamp.  This
    local linear interpolation is preferred to assuming storage time is sim
    time.  Values outside recorded /clock coverage are intentionally rejected.
    """
    if not clock_storage_times or storage_t < clock_storage_times[0] or storage_t > clock_storage_times[-1]:
        return None
    xs = clock_storage_times
    i = bisect.bisect_left(xs, storage_t)
    if i == 0:
        return clock_sim_times[0]
    if i == len(xs):
        return clock_sim_times[-1]
    x0, x1 = xs[i - 1], xs[i]
    y0, y1 = clock_sim_times[i - 1], clock_sim_times[i]
    if x1 == x0:
        return y0
    return y0 + (storage_t - x0) * (y1 - y0) / (x1 - x0)


def analyze(bag_data: Dict[str, Any], gt: Sequence[Tuple[float, float, float]], args: argparse.Namespace) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    odom = bag_data["odom"]
    if not odom:
        raise RuntimeError("/odom has no usable header timestamps/positions")
    clock = bag_data["clock"]
    clock_storage_times = [p[0] for p in clock]
    clock_sim_times = [p[1] for p in clock]
    cmd = []
    mapped_cmd = []
    unmapped_cmd = 0
    clock_bracket_gaps = []
    for storage_t, linear, angular in bag_data["cmd_raw"]:
        t = map_storage_to_clock(clock_storage_times, clock_sim_times, storage_t)
        bracket = clock_bracket(clock_storage_times, storage_t)
        if t is not None:
            cmd.append((t, linear, angular))
            mapped_cmd.append((storage_t, t, linear, angular, bracket[1] if bracket else None))
            if bracket is not None:
                clock_bracket_gaps.append(bracket[1])
        else:
            unmapped_cmd += 1
    # If bag record timestamps are already simulation time but no /clock was read,
    # leave cmd absent rather than pretending wall-clock and simulation time match.
    robot_xy = [(t, x, y) for t, x, y, _ in odom]
    odom_speed = [(t, v) for t, _, _, v in odom]
    derived_speed = speed_series(robot_xy)
    distance = sum(distance2((x0, y0), (x1, y1)) for (_, x0, y0), (_, x1, y1) in zip(robot_xy, robot_xy[1:]))
    obstacle_steps_raw, obstacle_steps_filtered, obstacle_speed_quality = ground_truth_speed_series(
        gt, args.ground_truth_min_dt, args.obstacle_max_plausible_speed)
    obstacle_distance = sum(distance2((x0, y0), (x1, y1)) for (_, x0, y0), (_, x1, y1) in zip(gt, gt[1:]))
    gt_times = [r[0] for r in gt]

    proximity = []
    for t, x, y in robot_xy:
        obs = interpolate_xy(gt, t, gt_times)
        if obs is not None:
            center = distance2((x, y), obs)
            proximity.append((t, center, center - args.obstacle_radius - args.robot_radius))
    raw_encounters = detect_encounters(proximity, args.encounter_distance, args.encounter_exit_distance,
                                       args.encounter_min_duration)
    robot_times = [r[0] for r in robot_xy]
    closest_rows = [min(event["samples"], key=lambda r: r[1]) for event in raw_encounters]
    closest_times = [row[0] for row in closest_rows]
    encounter_reports = []
    last_assigned_reaction: Optional[float] = None
    for index, event in enumerate(raw_encounters, start=1):
        start, end, rows = event["start_s"], event["end_s"], event["samples"]
        closest_row = closest_rows[index - 1]
        closest_time = closest_row[0]
        # Reaction belongs to this encounter only.
        # Look back before danger entry, but never into the previous encounter.
        previous_encounter_end = (
            raw_encounters[index - 2]["end_s"]
            if index > 1
            else float("-inf")
        )

        search_start = max(
            start - args.reaction_lookback,
            previous_encounter_end,
        )
        if last_assigned_reaction is not None:
            search_start = max(search_start, last_assigned_reaction + args.reaction_hold)

        context_end = closest_time + args.reaction_after_closest
        if index < len(raw_encounters):
            context_end = min(context_end, raw_encounters[index]["start_s"])
        if cmd:
            search_start = max(cmd[0][0], search_start)
            context_end = min(cmd[-1][0], context_end)
        reference_start = search_start - args.command_baseline_window
        before = window_stats(cmd, reference_start, search_start) if cmd else None
        linear_reference = [x for t, x, _ in cmd if reference_start <= t < search_start]
        angular_reference = [abs(z) for t, _, z in cmd if reference_start <= t < search_start]
        linear_baseline, linear_mad = median_mad(linear_reference)
        angular_baseline, angular_mad = median_mad(angular_reference)
        reaction, reaction_modes, reaction_linear_x, reaction_angular_z = (command_change_reaction(cmd, search_start, context_end,
                                                             linear_baseline, linear_mad,
                                                             angular_baseline, angular_mad,
                                                             args.linear_change_min, args.angular_change_min,
                                                             args.reaction_hold)
                                    if linear_baseline is not None and angular_baseline is not None
                                    and linear_mad is not None and angular_mad is not None else (None, [], None, None))
        if reaction is None:
            reaction_classification = "NO_DETECTED_RESPONSE"
            reaction_label = "NO DETECTED RESPONSE"
        elif reaction <= closest_time:
            reaction_classification = "TIMELY"
            reaction_label = "TIMELY"
        else:
            reaction_classification = "LATE"
            reaction_label = "LATE"
        if reaction is not None:
            last_assigned_reaction = reaction
        after_end = min(context_end, reaction + args.post_window) if reaction is not None else context_end
        after = window_stats(cmd, reaction, after_end) if reaction is not None else None
        stop_total, stop_longest = duration_where([(t, abs(x)) for t, x, _ in cmd], args.stop_speed,
                                                   reaction, context_end) if reaction is not None else (None, None)
        encounter_reports.append({
            "id": index, "encounter_time_s": start, "encounter_end_s": end,
            "reaction_search_start_s": search_start, "reaction_search_end_s": context_end,
            "duration_s": end - start, "minimum_center_distance_m": min(r[1] for r in rows),
            "minimum_estimated_clearance_m": min(r[2] for r in rows),
            "closest_approach_time_s": closest_time,
            "reaction_onset_s": reaction,
            "lead_time_s": start - reaction if reaction is not None else None,
            "reaction_delay_s": reaction - closest_time if reaction is not None else None,
            "reaction_classification": reaction_classification,
            "reaction_label": reaction_label,
            "reaction_modes": reaction_modes,
            "distance_at_reaction_m": (distance2(interpolate_xy(robot_xy, reaction, robot_times), interpolate_xy(gt, reaction, gt_times))
                                       if reaction is not None and interpolate_xy(robot_xy, reaction, robot_times) is not None and interpolate_xy(gt, reaction, gt_times) is not None else None),
            "reaction_linear_x_mps": reaction_linear_x,
            "reaction_angular_z_radps": reaction_angular_z,
            "linear_x_before_reaction": before["linear_x_mps"] if before else None,
            "angular_z_before_reaction": before["angular_z_radps"] if before else None,
            "abs_angular_z_before_reaction": before["abs_angular_z_radps"] if before else None,
            "linear_x_after_reaction": after["linear_x_mps"] if after else None,
            "angular_z_after_reaction": after["angular_z_radps"] if after else None,
            "abs_angular_z_after_reaction": after["abs_angular_z_radps"] if after else None,
            "command_baseline_window_s": [reference_start, search_start],
            "baseline_linear_x_median_mps": linear_baseline,
            "baseline_linear_x_mad_mps": linear_mad,
            "baseline_abs_angular_z_median_radps": angular_baseline,
            "baseline_abs_angular_z_mad_radps": angular_mad,
            "stop_total_s_after_reaction_within_response_window": stop_total,
            "longest_continuous_stop_s_after_reaction_within_response_window": stop_longest,
        })
    # Audit every mapped cmd_vel sample against the same response windows used
    # for reaction detection.  This is intentionally independent of the
    # before/after summary statistics.
    cmd_debug: List[Dict[str, Any]] = []
    for storage_t, cmd_t, linear_x, angular_z, bracket_gap in mapped_cmd:
        robot_at_cmd = interpolate_xy(robot_xy, cmd_t, robot_times)
        obstacle_at_cmd = interpolate_xy(gt, cmd_t, gt_times)
        center = distance2(robot_at_cmd, obstacle_at_cmd) if robot_at_cmd is not None and obstacle_at_cmd is not None else None
        active_ids = [e["id"] for e in encounter_reports
                  if e["encounter_time_s"] <= cmd_t <= e["encounter_end_s"]]
        future_ids = [e for e in encounter_reports
                  if cmd_t < e["encounter_time_s"]
                  and e["reaction_search_start_s"] <= cmd_t <= e["reaction_search_end_s"]]
        encounter_id = (active_ids[0] if active_ids else
                min(future_ids, key=lambda e: e["encounter_time_s"])["id"] if future_ids else None)
        cmd_debug.append({"cmd_storage_time_s": storage_t, "cmd_sim_time": cmd_t,
                          "linear_x": linear_x, "angular_z": angular_z,
                          "encounter_id": encounter_id, "center_distance": center,
                          "clock_bracket_gap_s": bracket_gap})
    high_angular = [r for r in cmd_debug if abs(r["angular_z"]) >= args.angular_debug_threshold]
    scan_values = [v for _, v in bag_data["scan"] if v is not None]
    def plan_summary(name: str) -> Dict[str, Any]:
        rows = bag_data[name]
        return {"updates": len(rows), "path_length_m": stats([r[1] for r in rows if r[1] is not None]),
                "pose_count": stats([float(r[2]) for r in rows])}
    # /clock is the authoritative simulation-time range when present.  Header
    # timestamps are retained separately because a topic can start late.
    odom_range = (odom[0][0], odom[-1][0])
    clock_range = (clock[0][1], clock[-1][1]) if clock else None
    sim_range = clock_range or odom_range
    gt_range = (gt[0][0], gt[-1][0])
    outcome = goal_outcome(odom, args)
    outcome.update({
        "travel_time_s": odom[-1][0] - odom[0][0],
        "path_length_m": distance,
        "collision_contact": bool(proximity and min(row[2] for row in proximity) <= 0.0),
        "collision_contact_basis": "proxy: estimated clearance <= 0 m; confirm with contact sensor for official contact result",
    })
    report = {
        "framework_version": "v1",
        "metric_scope": "Baseline/proxy metrics only; not official K6-K12 definitions.",
        "assumptions": {"obstacle_radius_m": args.obstacle_radius, "robot_radius_m": args.robot_radius,
                        "clearance_formula": "center_distance - obstacle_radius - robot_radius",
                        "robot_model": "TurtleBot3 Burger approximate circular radius"},
        "time_ranges_s": {"bag_storage": bag_data["storage_range"],
                          "bag_storage_duration": (bag_data["storage_range"][1] - bag_data["storage_range"][0]) if bag_data["storage_range"][0] is not None else None,
                          "clock_sim": clock_range, "odom_sim": odom_range,
                          "ground_truth": gt_range,
                          "ground_truth_missing_after_s": max(0.0, sim_range[1] - gt_range[1]),
                          "ground_truth_overlap_s": [max(sim_range[0], gt_range[0]), min(sim_range[1], gt_range[1])]},
        "message_counts": dict(bag_data["counts"]),
        "odometry": {"samples": len(odom), "distance_m": distance, "twist_linear_speed_mps": stats([v for _, v in odom_speed]),
                      "position_derived_speed_mps": stats([v for _, v in derived_speed])},
        "cmd_vel": {"samples": len(cmd), "unmapped_samples": unmapped_cmd,
                    "timestamp_basis": "local linear interpolation between bracketing /clock messages using rosbag storage timestamps" if cmd else "unavailable",
                    "clock_bracket_gap_s": stats(clock_bracket_gaps),
                    "linear_x_mps": stats([v for _, v, _ in cmd]), "angular_z_radps": stats([w for _, _, w in cmd])},
        "cmd_vel_mapping_debug": {"file": "cmd_vel_debug.csv", "high_angular_file": "cmd_vel_high_angular.csv",
                                  "high_angular_threshold_radps": args.angular_debug_threshold,
                                  "all_mapped_samples": len(cmd_debug), "high_angular_samples": len(high_angular),
                                  "high_angular_samples_in_response_window": sum(r["encounter_id"] is not None for r in high_angular),
                                  "mapped_sim_time_range_s": [cmd_debug[0]["cmd_sim_time"], cmd_debug[-1]["cmd_sim_time"]] if cmd_debug else None},
        "scan": {"samples": len(bag_data["scan"]), "valid_min_range_samples": len(scan_values), "minimum_range_m": stats(scan_values)},
        "obstacle_ground_truth": {"samples": len(gt), "trajectory_distance_m": obstacle_distance,
                                    "x_range_m": [min(r[1] for r in gt), max(r[1] for r in gt)],
                                    "y_range_m": [min(r[2] for r in gt), max(r[2] for r in gt)],
                                    "speed_mps": stats([v for _, v in obstacle_steps_filtered]),
                                    "raw_speed_mps": stats([v for _, v in obstacle_steps_raw]),
                                    "speed_quality_filter": {"min_segment_dt_s": args.ground_truth_min_dt,
                                                             "max_plausible_speed_mps": args.obstacle_max_plausible_speed,
                                                             **obstacle_speed_quality}},
        "robot_obstacle": {"matched_odom_samples": len(proximity), "center_distance_m": stats([d for _, d, _ in proximity]),
                           "estimated_clearance_m": stats([c for _, _, c in proximity])},
        "response_proxy": {
            "source": "/cmd_vel linear.x and angular.z, mapped through /clock" if cmd else "unavailable: /cmd_vel could not be mapped through /clock",
            "encounter_detection": f"enter center distance <= {args.encounter_distance:g} m; exit >= {args.encounter_exit_distance:g} m; minimum duration {args.encounter_min_duration:g} s",
            "reaction_search_window": (
                f"from up to {args.reaction_lookback:g} s before danger entry "
                f"to {args.reaction_after_closest:g} s after closest approach; "
                f"lookback is clipped at the end of the previous encounter"
            ),
            "criterion": f"first command change persisting >= {args.reaction_hold:g}s: linear.x decreases by >= max({args.linear_change_min:g} m/s, 3 x baseline MAD) OR abs(angular.z) increases by >= max({args.angular_change_min:g} rad/s, 3 x baseline MAD)",
            "classification_rule": (
                "TIMELY if reaction onset <= closest_approach_time_s; "
                "LATE if reaction onset is after closest approach; "
                "NO_DETECTED_RESPONSE if no qualifying command change is found"
            ),
            "post_reaction_window_s": args.post_window, "stop_threshold_abs_linear_x_mps": args.stop_speed,
            "encounters_detected": len(encounter_reports),
            "classification_counts": dict(Counter(e["reaction_classification"] for e in encounter_reports)),
            "encounters": encounter_reports},
        "outcome": outcome,
        "raw_measurements": {
            "odom": [{"sim_time_s": t, "x_m": x, "y_m": y, "speed_mps": v} for t, x, y, v in odom],
            "clock": [{"storage_time_s": storage_t, "sim_time_s": sim_t} for storage_t, sim_t in clock],
            "cmd_vel": [{"sim_time_s": t, "linear_x_mps": x, "angular_z_radps": z} for t, x, z in cmd],
            "cmd_vel_raw": [{"storage_time_s": t, "linear_x_mps": x, "angular_z_radps": z}
                            for t, x, z in bag_data["cmd_raw"]],
            "scan": [{"sim_time_s": t, "minimum_range_m": v} for t, v in bag_data["scan"]],
            "obstacle_ground_truth": [{"sim_time_s": t, "x_m": x, "y_m": y} for t, x, y in gt],
            "robot_obstacle": [{"sim_time_s": t, "center_distance_m": d, "estimated_clearance_m": c} for t, d, c in proximity],
            "plan": [{"sim_time_s": t, "path_length_m": length, "pose_count": count} for t, length, count in bag_data["plan"]],
            "local_plan": [{"sim_time_s": t, "path_length_m": length, "pose_count": count} for t, length, count in bag_data["local_plan"]],
        },
        "planner": {"plan": plan_summary("plan"), "local_plan": plan_summary("local_plan")},
    }
    return report, cmd_debug


def write_outputs(report: Dict[str, Any], output: Path, proximity: Sequence[Tuple[float, float, float]],
                  cmd_debug: Sequence[Dict[str, Any]]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    encounter_rows = report["response_proxy"]["encounters"]
    with (output / "time_series.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["sim_time_s", "robot_obstacle_center_distance_m", "estimated_clearance_m", "encounter_id"])
        for t, center, clearance in proximity:
            ids = [str(e["id"]) for e in encounter_rows if e["encounter_time_s"] <= t <= e["encounter_end_s"]]
            w.writerow([t, center, clearance, ";".join(ids)])
    debug_columns = ["cmd_storage_time_s", "cmd_sim_time", "linear_x", "angular_z", "encounter_id", "center_distance", "clock_bracket_gap_s"]
    for filename, rows in (("cmd_vel_debug.csv", cmd_debug),
                           ("cmd_vel_high_angular.csv", [r for r in cmd_debug if abs(r["angular_z"]) >= report["cmd_vel_mapping_debug"]["high_angular_threshold_radps"]])):
        with (output / filename).open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=debug_columns)
            writer.writeheader()
            writer.writerows(rows)
    tr, od, cmd, scan, obs, prox, rsp, planner = (report["time_ranges_s"], report["odometry"], report["cmd_vel"], report["scan"], report["obstacle_ground_truth"], report["robot_obstacle"], report["response_proxy"], report["planner"])
    lines = ["BASELINE / PROXY ANALYSIS (not official K6-K12 metrics)", "",
             f"Bag storage duration: {fmt(tr['bag_storage_duration'])} s; /clock sim range: {fmt(tr['clock_sim'][0]) if tr['clock_sim'] else 'n/a'} to {fmt(tr['clock_sim'][1]) if tr['clock_sim'] else 'n/a'} s",
             f"Odometry simulation range: {fmt(tr['odom_sim'][0])} to {fmt(tr['odom_sim'][1])} s",
             f"Ground-truth range: {fmt(tr['ground_truth'][0])} to {fmt(tr['ground_truth'][1])} s",
             f"Ground-truth missing after coverage: {fmt(tr['ground_truth_missing_after_s'])} s",
             f"Robot distance: {fmt(od['distance_m'])} m; odom samples: {od['samples']}",
             f"Odom twist speed median/p95: {fmt(od['twist_linear_speed_mps']['median'])} / {fmt(od['twist_linear_speed_mps']['p95'])} m/s",
             f"cmd_vel samples: {cmd['samples']}; linear.x median/p95: {fmt(cmd['linear_x_mps']['median'])} / {fmt(cmd['linear_x_mps']['p95'])} m/s",
             f"cmd_vel timestamp mapping: {cmd['timestamp_basis']}; unmapped {cmd['unmapped_samples']}; /clock bracket gap median/p95 {fmt(cmd['clock_bracket_gap_s']['median'], 6)} / {fmt(cmd['clock_bracket_gap_s']['p95'], 6)} s",
             f"Scan min range min/p05/median: {fmt(scan['minimum_range_m']['min'])} / {fmt(scan['minimum_range_m']['p05'])} / {fmt(scan['minimum_range_m']['median'])} m",
             f"Obstacle trajectory: {fmt(obs['trajectory_distance_m'])} m; quality-filtered speed median/p95/max: {fmt(obs['speed_mps']['median'])} / {fmt(obs['speed_mps']['p95'])} / {fmt(obs['speed_mps']['max'])} m/s",
             f"Obstacle raw speed max: {fmt(obs['raw_speed_mps']['max'])} m/s; excluded GT segments (tiny dt/outlier): {obs['speed_quality_filter']['nonpositive_or_tiny_dt']} / {obs['speed_quality_filter']['speed_outlier']}",
             f"Robot-obstacle center distance min: {fmt(prox['center_distance_m']['min'])} m",
             f"Estimated clearance min: {fmt(prox['estimated_clearance_m']['min'])} m (assumes obstacle r={report['assumptions']['obstacle_radius_m']:.3f} m, Burger r={report['assumptions']['robot_radius_m']:.3f} m)",
             f"Response proxy source: {rsp['source']}",
             f"Reaction criterion: {rsp['criterion']}",
             f"Encounter detection: {rsp['encounter_detection']}",
             f"Reaction search window: {rsp['reaction_search_window']}",
             f"Encounters detected: {rsp['encounters_detected']}; classifications: TIMELY={rsp['classification_counts'].get('TIMELY', 0)}, LATE={rsp['classification_counts'].get('LATE', 0)}, NO DETECTED RESPONSE={rsp['classification_counts'].get('NO_DETECTED_RESPONSE', 0)}",
             f"Planner updates plan/local_plan: {planner['plan']['updates']} / {planner['local_plan']['updates']}",
             f"Plan path length median: {fmt(planner['plan']['path_length_m']['median'])} m; local plan median: {fmt(planner['local_plan']['path_length_m']['median'])} m", "",
             "", "Per-encounter response proxy results:"]
    if not encounter_rows:
        lines.append("  None (or /cmd_vel was not timestamp-mappable).")
    for e in encounter_rows:
        lines.extend([
            f"  Encounter {e['id']}: danger entry/end {fmt(e['encounter_time_s'])} / {fmt(e['encounter_end_s'])} s; closest approach {fmt(e['closest_approach_time_s'])} s; response search {fmt(e['reaction_search_start_s'])} to {fmt(e['reaction_search_end_s'])} s; min center/clearance {fmt(e['minimum_center_distance_m'])} / {fmt(e['minimum_estimated_clearance_m'])} m",
            f"    {e['reaction_label']}: onset {fmt(e['reaction_onset_s']) if e['reaction_onset_s'] is not None else 'none'}; modes {', '.join(e['reaction_modes']) if e['reaction_modes'] else 'none'}; reaction delay from closest {fmt(e['reaction_delay_s']) if e['reaction_delay_s'] is not None else 'not applicable'} s; lead time from entry {fmt(e['lead_time_s']) if e['lead_time_s'] is not None else 'not applicable'} s",
            f"    distance at reaction {fmt(e['distance_at_reaction_m']) if e['distance_at_reaction_m'] is not None else 'not applicable'} m; linear.x/angular.z at onset {fmt(e['reaction_linear_x_mps']) if e['reaction_linear_x_mps'] is not None else 'not applicable'} / {fmt(e['reaction_angular_z_radps']) if e['reaction_angular_z_radps'] is not None else 'not applicable'}",
            f"    stop total/longest {fmt(e['stop_total_s_after_reaction_within_response_window']) if e['reaction_onset_s'] is not None else 'not applicable'} / {fmt(e['longest_continuous_stop_s_after_reaction_within_response_window']) if e['reaction_onset_s'] is not None else 'not applicable'} s",
            f"    linear.x before median {fmt(e['linear_x_before_reaction']['median'] if e['linear_x_before_reaction'] else None)}; after median {fmt(e['linear_x_after_reaction']['median'] if e['linear_x_after_reaction'] else None)} m/s",
            f"    |angular.z| before median {fmt(e['abs_angular_z_before_reaction']['median'] if e['abs_angular_z_before_reaction'] else None)}; after median {fmt(e['abs_angular_z_after_reaction']['median'] if e['abs_angular_z_after_reaction'] else None)} rad/s",
        ])
    lines.append("Files: summary.json contains all fields; time_series.csv contains matched robot/obstacle clearance samples and encounter IDs.")
    debug = report["cmd_vel_mapping_debug"]
    lines.append(f"cmd_vel_debug.csv contains all {debug['all_mapped_samples']} mapped cmd_vel samples. cmd_vel_high_angular.csv contains {debug['high_angular_samples']} samples with |angular.z| >= {debug['high_angular_threshold_radps']:.3f} rad/s; {debug['high_angular_samples_in_response_window']} fall in a response window.")
    (output / "report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--bag", type=Path, default=Path(DEFAULT_BASE).expanduser() / "trial_02", help="rosbag2 MCAP directory")
    p.add_argument("--ground-truth", type=Path, default=Path(DEFAULT_BASE).expanduser() / "obstacle_ground_truth_02.csv", help="CSV with simulation time, obstacle x, obstacle y")
    p.add_argument("--output", type=Path, default=Path(DEFAULT_BASE).expanduser() / "baseline_analysis", help="output directory")
    p.add_argument("--obstacle-radius", type=float, default=0.20, help="known obstacle radius (m)")
    p.add_argument("--robot-radius", type=float, default=0.105, help="Burger approximate circular radius (m)")
    p.add_argument("--encounter-distance", type=float, default=1.50, help="center-distance threshold for response proxy (m)")
    p.add_argument("--encounter-exit-distance", type=float, default=1.80, help="exit threshold for encounter hysteresis (m)")
    p.add_argument("--encounter-min-duration", type=float, default=0.50, help="discard shorter encounter regions (s)")
    p.add_argument("--reaction-lookback", type=float, default=5.0, help="search for an early command change before danger entry (s)")
    p.add_argument("--command-baseline-window", type=float, default=5.0, help="reference window before reaction lookback (s)")
    p.add_argument("--reaction-after-closest", type=float, default=3.0, help="search for a late reaction after closest approach (s)")
    p.add_argument("--linear-change-min", type=float, default=0.05, help="minimum linear.x decrease for a command change (m/s)")
    p.add_argument("--angular-change-min", type=float, default=0.08, help="minimum abs(angular.z) increase for a command change (rad/s)")
    p.add_argument("--reaction-hold", type=float, default=0.25, help="command-change persistence required to confirm reaction (s)")
    p.add_argument("--angular-debug-threshold", type=float, default=0.25, help="threshold used only for cmd_vel mapping debug CSV (rad/s)")
    p.add_argument("--ground-truth-min-dt", type=float, default=0.001, help="GT segment dt below this is an artifact for speed statistics (s)")
    p.add_argument("--obstacle-max-plausible-speed", type=float, default=5.0, help="GT speed above this is excluded only from filtered speed statistics (m/s)")
    p.add_argument("--post-window", type=float, default=1.0, help="window after reaction for post-reaction command statistics (s)")
    p.add_argument("--stop-speed", type=float, default=0.05, help="speed treated as stopped (m/s)")
    p.add_argument("--goal-x", type=float, default=None, help="goal x coordinate (m); omit when trial has no goal record")
    p.add_argument("--goal-y", type=float, default=None, help="goal y coordinate (m); omit when trial has no goal record")
    p.add_argument("--goal-tolerance", type=float, default=0.15, help="goal success tolerance (m)")
    args = p.parse_args()
    if (args.obstacle_radius < 0 or args.robot_radius < 0 or args.encounter_distance <= 0
            or args.encounter_exit_distance < args.encounter_distance or args.encounter_min_duration < 0
            or args.reaction_lookback < 0 or args.command_baseline_window <= 0
            or args.reaction_after_closest < 0 or args.linear_change_min < 0 or args.angular_change_min < 0
            or args.reaction_hold < 0 or args.post_window <= 0
            or args.ground_truth_min_dt < 0 or args.obstacle_max_plausible_speed <= 0
            or args.goal_tolerance <= 0 or (args.goal_x is None) != (args.goal_y is None)):
        p.error("invalid radius, encounter hysteresis, or response window argument")
    args.bag, args.ground_truth, args.output = (x.expanduser().resolve() for x in (args.bag, args.ground_truth, args.output))
    if not args.bag.is_dir():
        p.error(f"bag directory does not exist: {args.bag}")
    if not args.ground_truth.is_file():
        p.error(f"ground-truth CSV does not exist: {args.ground_truth}")
    try:
        gt = read_ground_truth(args.ground_truth)
        data = read_bag(args.bag)
        report, cmd_debug = analyze(data, gt, args)
        proximity = []
        gt_times = [r[0] for r in gt]
        for t, x, y, _ in data["odom"]:
            xy = interpolate_xy(gt, t, gt_times)
            if xy is not None:
                center = distance2((x, y), xy)
                proximity.append((t, center, center - args.obstacle_radius - args.robot_radius))
        write_outputs(report, args.output, proximity, cmd_debug)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"Wrote {args.output / 'report.txt'}")
    print(f"Wrote {args.output / 'summary.json'}")
    print(f"Wrote {args.output / 'time_series.csv'}")
    print(f"Wrote {args.output / 'cmd_vel_debug.csv'}")
    print(f"Wrote {args.output / 'cmd_vel_high_angular.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
