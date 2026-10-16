#!/usr/bin/env python3
"""
Recalculate Persona Metrics on Pilot 300 Dataset (Strict Offline Evaluation).
No Gazebo execution required.

Rules:
1. Interaction Window:
   - Discard first 1.0 s of the episode (t >= 1.0 s).
   - Starts when distance(robot, human_1) < 6.0 m OR predicted CPA < 1.5 m.
   - Ends at first collision (t_col) OR closest approach + 1.0 s (min(t_col, t_min_dist + 1.0 s)).
2. Human Lateral Deviation:
   - Perpendicular distance from human (x, y) to straight line from start to destination waypoint.
3. Human Velocity Drop:
   - Steady velocity at window start minus minimum velocity in window: v_start - v_min.
4. Evasion Onset:
   - First step where human lateral deviation > 0.10 m in window.
   - Report distance to robot at that onset.
5. Robot Stopping / Yielding Flag:
   - Robot speed < 0.05 m/s continuously for >= 1.0 s, evaluated strictly after t >= 2.0 s and when human is within 3.0 m.
6. Stratification:
   - Stratified by corridor (0.9 m vs 1.2 m) and overall.
   - Also report with steps where human penetrates wall > 2 cm excluded.
7. Statistical Tests:
   - For H1-H2, H2-H3, H1-H3: Mann-Whitney U test p-value and Cliff's delta with 95% bootstrap CI (2,000 iterations).
8. Actual Persona Parameters:
   - Report responsibility weight (alpha) and politeness used by C++ RVO2 plugin.
"""

import os
import sys
import json
import math
import numpy as np
from scipy import stats
from typing import Dict, List, Any, Tuple, Optional

ROOT_DIR = "/home/cuongbien/DynamuhSim"
EPISODES_DIR = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/pilot_300/pilot_300_dataset/episodes")
OUTPUT_JSON = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/pilot_300/pilot_300_recalculated_persona_results.json")


def point_to_line_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """Perpendicular distance from point P to line passing through A and B."""
    dx = bx - ax
    dy = by - ay
    line_len = math.hypot(dx, dy)
    if line_len < 1e-6:
        return math.hypot(px - ax, py - ay)
    return abs(dx * (ay - py) - (ax - px) * dy) / line_len


def compute_cpa(rx: float, ry: float, rvx: float, rvy: float,
                hx: float, hy: float, hvx: float, hvy: float, max_t: float = 18.0) -> float:
    """Instantaneous predicted Closest Point of Approach (CPA)."""
    dx = rx - hx
    dy = ry - hy
    dvx = rvx - hvx
    dvy = rvy - hvy
    v2 = dvx**2 + dvy**2
    if v2 < 1e-6:
        return math.hypot(dx, dy)
    t_cpa = -(dx * dvx + dy * dvy) / v2
    t_clamped = min(max_t, max(0.0, t_cpa))
    cpa_x = dx + dvx * t_clamped
    cpa_y = dy + dvy * t_clamped
    return math.hypot(cpa_x, cpa_y)


def compute_median_iqr(vals: List[float]) -> Dict[str, Any]:
    valid = [v for v in vals if v is not None and not math.isnan(v)]
    if not valid:
        return {"median": None, "q25": None, "q75": None, "iqr": None, "count": 0}
    q25, med, q75 = np.percentile(valid, [25, 50, 75])
    return {
        "median": round(float(med), 4),
        "q25": round(float(q25), 4),
        "q75": round(float(q75), 4),
        "iqr": round(float(q75 - q25), 4),
        "count": len(valid)
    }


def compute_cliffs_delta(x: List[float], y: List[float]) -> float:
    """Compute Cliff's Delta effect size d between samples x and y."""
    vx = np.array([v for v in x if v is not None and not math.isnan(v)])
    vy = np.array([v for v in y if v is not None and not math.isnan(v)])
    nx = len(vx)
    ny = len(vy)
    if nx == 0 or ny == 0:
        return 0.0
    # Vectorized comparison: (nx, 1) vs (1, ny)
    diff = vx[:, None] - vy[None, :]
    gt = np.sum(diff > 0)
    lt = np.sum(diff < 0)
    delta = (gt - lt) / (nx * ny)
    return float(delta)


def bootstrap_cliffs_delta_ci(x: List[float], y: List[float], n_bootstrap: int = 2000, alpha: float = 0.05, seed: int = 42) -> Tuple[float, float]:
    """Compute percentile 95% bootstrap confidence interval for Cliff's Delta."""
    vx = np.array([v for v in x if v is not None and not math.isnan(v)])
    vy = np.array([v for v in y if v is not None and not math.isnan(v)])
    nx = len(vx)
    ny = len(vy)
    if nx == 0 or ny == 0:
        return 0.0, 0.0

    rng = np.random.RandomState(seed)
    deltas = np.empty(n_bootstrap, dtype=float)
    for b in range(n_bootstrap):
        sample_x = rng.choice(vx, size=nx, replace=True)
        sample_y = rng.choice(vy, size=ny, replace=True)
        diff = sample_x[:, None] - sample_y[None, :]
        gt = np.sum(diff > 0)
        lt = np.sum(diff < 0)
        deltas[b] = (gt - lt) / (nx * ny)

    low = float(np.percentile(deltas, 100.0 * (alpha / 2.0)))
    high = float(np.percentile(deltas, 100.0 * (1.0 - alpha / 2.0)))
    return round(low, 4), round(high, 4)


def perform_statistical_comparison(group_a: List[float], group_b: List[float], label_a: str, label_b: str) -> Dict[str, Any]:
    """Perform Mann-Whitney U test and Cliff's Delta with bootstrap CI."""
    va = [v for v in group_a if v is not None and not math.isnan(v)]
    vb = [v for v in group_b if v is not None and not math.isnan(v)]
    if len(va) < 2 or len(vb) < 2:
        return {
            "comparison": f"{label_a} vs {label_b}",
            "mann_whitney_u": None,
            "p_value": None,
            "cliffs_delta": None,
            "cliffs_delta_ci_95": None,
            "interpretation": "Insufficient data"
        }

    u_stat, p_val = stats.mannwhitneyu(va, vb, alternative='two-sided')
    delta = compute_cliffs_delta(va, vb)
    ci_low, ci_high = bootstrap_cliffs_delta_ci(va, vb)

    abs_d = abs(delta)
    if abs_d < 0.147:
        interp = "negligible"
    elif abs_d < 0.33:
        interp = "small"
    elif abs_d < 0.474:
        interp = "medium"
    else:
        interp = "large"

    return {
        "comparison": f"{label_a} vs {label_b}",
        "sample_sizes": {label_a: len(va), label_b: len(vb)},
        "mann_whitney_u": round(float(u_stat), 2),
        "p_value": float(f"{p_val:.4e}"),
        "cliffs_delta": round(delta, 4),
        "cliffs_delta_ci_95": [ci_low, ci_high],
        "effect_size": interp
    }


def analyze_episode(ep_dir: str) -> Optional[Dict[str, Any]]:
    m_file = os.path.join(ep_dir, "manifest.json")
    sc_file = os.path.join(ep_dir, "scenario.json")
    gt_file = os.path.join(ep_dir, "ground_truth.json")
    sm_file = os.path.join(ep_dir, "step_metrics.json")

    if not (os.path.exists(m_file) and os.path.exists(sc_file) and os.path.exists(gt_file)):
        return None

    with open(m_file) as f: m = json.load(f)
    with open(sc_file) as f: sc = json.load(f)
    with open(gt_file) as f: gt = json.load(f)

    sm = []
    if os.path.exists(sm_file):
        with open(sm_file) as f: sm = json.load(f)

    # Filter: Nav2 MPPI, non-frozen, interactive CPA < 1.5m, Persona in {H1, H2, H3}
    if not (m.get('use_nav2', False) and not m.get('is_frozen', False) and m.get('has_cpa_lt_15', False)):
        return None
    if m.get('persona') not in ['H1', 'H2', 'H3']:
        return None

    corr = m.get('corridor', 'corridor_090')
    half_w = 0.45 if corr == 'corridor_090' else 0.60
    # Wall penetration threshold: human cylinder r=0.28m penetrating wall by > 0.02m
    # i.e., |hy| + 0.28 > half_w + 0.02  <=>  |hy| > half_w - 0.26
    y_wall_pen_thresh = half_w - 0.26

    # Nominal line for human 1 from scenario waypoints
    h1_sc = sc['humans'][0]
    ax = h1_sc['start']['x']
    ay = h1_sc['start']['y']
    bx = h1_sc['waypoints'][-1][0]
    by = h1_sc['waypoints'][-1][1]

    # Step-by-step parsing
    total_steps = len(gt)
    t_start = None
    t_end = None
    t_col = None
    min_dist_robot_h1 = 999.0
    t_min_dist = None

    t0_stamp = gt[0].get('timestamp', 0.0) if gt else 0.0

    # Step data structures
    step_records = []
    for idx, s in enumerate(gt):
        t = s.get('sim_time_rel', 0.0)
        if t == 0.0 and len(gt) > 1 and s.get('timestamp') is not None:
            t = round(s['timestamp'] - t0_stamp, 4)

        rx = s['robot']['x']
        ry = s['robot']['y']
        rvx = s['robot'].get('vx', 0.0)
        rvy = s['robot'].get('vy', 0.0)
        rv = math.hypot(rvx, rvy)

        # Primary human is id == 1
        h1_list = [h for h in s['humans'] if h.get('id') == 1]
        if not h1_list:
            continue
        h1 = h1_list[0]
        hx = h1['x']
        hy = h1['y']
        hvx = h1.get('vx', 0.0)
        hvy = h1.get('vy', 0.0)
        hv = math.hypot(hvx, hvy)
        h_resp = h1.get('responsibility_weight', 0.0)
        h_pol = h1.get('politeness', 0.0)

        dist_rh = math.hypot(rx - hx, ry - hy)
        cpa_instant = compute_cpa(rx, ry, rvx, rvy, hx, hy, hvx, hvy)

        # Human lateral deviation from straight line A -> B
        lat_dev = point_to_line_distance(hx, hy, ax, ay, bx, by)

        # Wall penetration for human 1
        is_wall_pen_gt_2cm = (abs(hy) > y_wall_pen_thresh)

        # Clearance from step_metrics if available
        d_clear = sm[idx].get('min_d_clear', dist_rh - 0.40) if idx < len(sm) else (dist_rh - 0.40)

        step_records.append({
            'idx': idx,
            't': t,
            'rx': rx, 'ry': ry, 'rv': rv,
            'hx': hx, 'hy': hy, 'hv': hv,
            'dist_rh': dist_rh,
            'cpa_instant': cpa_instant,
            'lat_dev': lat_dev,
            'is_wall_pen': is_wall_pen_gt_2cm,
            'd_clear': d_clear,
            'h_resp': h_resp,
            'h_pol': h_pol
        })

        if t >= 1.0:
            if dist_rh < min_dist_robot_h1:
                min_dist_robot_h1 = dist_rh
                t_min_dist = t
            if t_start is None and (dist_rh < 6.0 or cpa_instant < 1.5):
                t_start = t

    collision_occurred = m.get('collision_occurred', False)

    # If episode collided before t = 1.0s (e.g. at step 1)
    if collision_occurred and (not step_records or step_records[-1]['t'] < 1.0):
        init_clear = step_records[0]['d_clear'] if step_records else -0.1
        init_lat = step_records[0]['lat_dev'] if step_records else 0.0
        return {
            "episode_id": m.get("episode_id"),
            "persona": m.get("persona"),
            "corridor": corr,
            "num_humans": m.get("num_humans"),
            "collision_occurred": True,
            "time_to_goal": None,
            "window": {
                "t_start": 0.0,
                "t_end": step_records[-1]['t'] if step_records else 0.0,
                "duration": step_records[-1]['t'] if step_records else 0.0,
                "t_min_dist": 0.0,
                "min_dist_m": step_records[0]['dist_rh'] if step_records else 0.25,
                "early_collision_pre_1s": True
            },
            "metrics_unfiltered": {
                "min_d_clear_m": round(init_clear, 4),
                "max_h_lat_dev_m": round(init_lat, 4),
                "h_vel_drop_mps": 0.0,
                "dist_at_lat_onset_m": None,
                "evasion_detected": False,
                "max_r_lat_dev_m": 0.0,
                "min_r_speed_mps": 0.0,
                "robot_stopped_or_yielded": False
            },
            "metrics_wall_filtered": {
                "valid_steps_count": 0,
                "total_window_steps": len(step_records),
                "max_h_lat_dev_m": None,
                "min_d_clear_m": None,
                "h_vel_drop_mps": None
            },
            "plugin_params": {
                "responsibility_weight": round(step_records[0]['h_resp'], 4) if step_records else 0.0,
                "politeness": round(step_records[0]['h_pol'], 4) if step_records else 0.0
            }
        }

    if t_start is None:
        t_start = 1.0

    # Determine end of window
    if collision_occurred:
        t_col = step_records[-1]['t'] if step_records else 0.0
        t_end = t_col
    else:
        if t_min_dist is not None:
            t_end = min(step_records[-1]['t'], t_min_dist + 1.0)
        else:
            t_end = step_records[-1]['t']

    # Filter steps strictly inside interaction window [t_start, t_end]
    window_steps = [s for s in step_records if t_start <= s['t'] <= t_end]
    if not window_steps:
        return None

    # Compute Metrics (All Steps)
    v_start = window_steps[0]['hv']
    min_hv = min(s['hv'] for s in window_steps)
    h_vel_drop = max(0.0, v_start - min_hv)

    max_h_lat_dev = max(s['lat_dev'] for s in window_steps)

    # Evasion onset: first step in window where lat_dev > 0.10 m
    dist_at_lat_onset = None
    t_lat_onset = None
    for s in window_steps:
        if s['lat_dev'] > 0.10:
            dist_at_lat_onset = s['dist_rh']
            t_lat_onset = s['t']
            break

    # Robot stopping / yielding:
    # v < 0.05 m/s continuously >= 1.0 s, strictly after t >= 2.0 s and when human within 3.0 m
    robot_stopped_or_yielded = False
    streak_start = None
    for s in step_records:
        if s['t'] >= 2.0 and s['dist_rh'] <= 3.0:
            if s['rv'] < 0.05:
                if streak_start is None:
                    streak_start = s['t']
                elif (s['t'] - streak_start) >= 1.0:
                    robot_stopped_or_yielded = True
                    break
            else:
                streak_start = None
        else:
            streak_start = None

    max_r_lat_dev = max(abs(s['ry']) for s in window_steps)
    min_r_speed = min(s['rv'] for s in window_steps)
    min_d_clear = min(s['d_clear'] for s in window_steps)

    # Filtered Steps (Excluding human wall penetration > 2cm)
    valid_wall_steps = [s for s in window_steps if not s['is_wall_pen']]
    if valid_wall_steps:
        max_h_lat_dev_filtered = max(s['lat_dev'] for s in valid_wall_steps)
        min_d_clear_filtered = min(s['d_clear'] for s in valid_wall_steps)
        v_start_f = valid_wall_steps[0]['hv']
        min_hv_f = min(s['hv'] for s in valid_wall_steps)
        h_vel_drop_filtered = max(0.0, v_start_f - min_hv_f)
    else:
        max_h_lat_dev_filtered = None
        min_d_clear_filtered = None
        h_vel_drop_filtered = None

    return {
        "episode_id": m.get("episode_id"),
        "persona": m.get("persona"),
        "corridor": corr,
        "num_humans": m.get("num_humans"),
        "collision_occurred": collision_occurred,
        "time_to_goal": m.get("time_to_goal"),
        "window": {
            "t_start": round(t_start, 3),
            "t_end": round(t_end, 3),
            "duration": round(t_end - t_start, 3),
            "t_min_dist": round(t_min_dist, 3) if t_min_dist is not None else None,
            "min_dist_m": round(min_dist_robot_h1, 4)
        },
        "metrics_unfiltered": {
            "min_d_clear_m": round(min_d_clear, 4),
            "max_h_lat_dev_m": round(max_h_lat_dev, 4),
            "h_vel_drop_mps": round(h_vel_drop, 4),
            "dist_at_lat_onset_m": round(dist_at_lat_onset, 4) if dist_at_lat_onset is not None else None,
            "evasion_detected": (dist_at_lat_onset is not None),
            "max_r_lat_dev_m": round(max_r_lat_dev, 4),
            "min_r_speed_mps": round(min_r_speed, 4),
            "robot_stopped_or_yielded": robot_stopped_or_yielded
        },
        "metrics_wall_filtered": {
            "valid_steps_count": len(valid_wall_steps),
            "total_window_steps": len(window_steps),
            "max_h_lat_dev_m": round(max_h_lat_dev_filtered, 4) if max_h_lat_dev_filtered is not None else None,
            "min_d_clear_m": round(min_d_clear_filtered, 4) if min_d_clear_filtered is not None else None,
            "h_vel_drop_mps": round(h_vel_drop_filtered, 4) if h_vel_drop_filtered is not None else None
        },
        "plugin_params": {
            "responsibility_weight": round(step_records[0]['h_resp'], 4),
            "politeness": round(step_records[0]['h_pol'], 4)
        }
    }


def aggregate_group(eps: List[Dict[str, Any]], wall_filtered: bool = False) -> Dict[str, Any]:
    n = len(eps)
    if n == 0:
        return {"n": 0}

    col_count = sum(1 for e in eps if e["collision_occurred"])
    col_rate = round(col_count / n * 100.0, 1)

    stop_count = sum(1 for e in eps if e["metrics_unfiltered"]["robot_stopped_or_yielded"])
    stop_rate = round(stop_count / n * 100.0, 1)

    evasion_count = sum(1 for e in eps if e["metrics_unfiltered"]["evasion_detected"])
    evasion_rate = round(evasion_count / n * 100.0, 1)

    t_goals = [e["time_to_goal"] for e in eps if e["time_to_goal"] is not None]

    if not wall_filtered:
        min_d_clears = [e["metrics_unfiltered"]["min_d_clear_m"] for e in eps]
        max_h_devs = [e["metrics_unfiltered"]["max_h_lat_dev_m"] for e in eps]
        h_drops = [e["metrics_unfiltered"]["h_vel_drop_mps"] for e in eps]
    else:
        min_d_clears = [e["metrics_wall_filtered"]["min_d_clear_m"] for e in eps if e["metrics_wall_filtered"]["min_d_clear_m"] is not None]
        max_h_devs = [e["metrics_wall_filtered"]["max_h_lat_dev_m"] for e in eps if e["metrics_wall_filtered"]["max_h_lat_dev_m"] is not None]
        h_drops = [e["metrics_wall_filtered"]["h_vel_drop_mps"] for e in eps if e["metrics_wall_filtered"]["h_vel_drop_mps"] is not None]

    dist_onsets = [e["metrics_unfiltered"]["dist_at_lat_onset_m"] for e in eps if e["metrics_unfiltered"]["dist_at_lat_onset_m"] is not None]
    max_r_devs = [e["metrics_unfiltered"]["max_r_lat_dev_m"] for e in eps]
    min_r_spds = [e["metrics_unfiltered"]["min_r_speed_mps"] for e in eps]

    return {
        "n_episodes": n,
        "collision_rate_pct": col_rate,
        "robot_stop_yield_rate_pct": stop_rate,
        "human_evasion_rate_pct": evasion_rate,
        "min_d_clear_m": compute_median_iqr(min_d_clears),
        "max_human_lat_dev_m": compute_median_iqr(max_h_devs),
        "human_vel_drop_mps": compute_median_iqr(h_drops),
        "dist_at_lat_onset_m": compute_median_iqr(dist_onsets),
        "max_robot_lat_dev_m": compute_median_iqr(max_r_devs),
        "min_robot_speed_mps": compute_median_iqr(min_r_spds),
        "time_to_goal_s": compute_median_iqr(t_goals)
    }


def main():
    ep_dirs = sorted([d for d in os.listdir(EPISODES_DIR) if d.startswith("ep_")])
    print(f"Loading and processing {len(ep_dirs)} episode runs...")

    all_ep_results = []
    for ep in ep_dirs:
        res = analyze_episode(os.path.join(EPISODES_DIR, ep))
        if res is not None:
            all_ep_results.append(res)

    print(f"Total interactive evaluation episodes: {len(all_ep_results)}")

    # Actual Persona Plugin Parameters Report
    plugin_summary = {}
    for p in ["H1", "H2", "H3"]:
        sub = [e for e in all_ep_results if e["persona"] == p]
        rw = [e["plugin_params"]["responsibility_weight"] for e in sub]
        pol = [e["plugin_params"]["politeness"] for e in sub]
        plugin_summary[p] = {
            "n": len(sub),
            "responsibility_alpha": {
                "theoretical": "0.50 (fixed)" if p == "H1" else ("Uniform [0.10, 0.35]" if p == "H2" else "0.00 (fixed)"),
                "empirical_min": min(rw) if rw else 0.0,
                "empirical_max": max(rw) if rw else 0.0,
                "empirical_mean": round(float(np.mean(rw)), 4) if rw else 0.0
            },
            "politeness": {
                "theoretical": "0.60 (fixed)" if p == "H1" else ("= responsibility_alpha" if p == "H2" else "0.00 (fixed)"),
                "empirical_mean": round(float(np.mean(pol)), 4) if pol else 0.0
            },
            "ignores_robot": (p == "H3")
        }

    # Stratified Tables:
    # 1. Overall Unfiltered
    # 2. Overall Wall-Filtered (excluding human wall penetration > 2cm)
    # 3. corridor_090 Unfiltered
    # 4. corridor_090 Wall-Filtered
    # 5. corridor_120 Unfiltered
    # 6. corridor_120 Wall-Filtered

    strata = {
        "overall_unfiltered": {},
        "overall_wall_filtered": {},
        "corridor_090_unfiltered": {},
        "corridor_090_wall_filtered": {},
        "corridor_120_unfiltered": {},
        "corridor_120_wall_filtered": {}
    }

    for p in ["H1", "H2", "H3"]:
        # Overall
        sub_all = [e for e in all_ep_results if e["persona"] == p]
        strata["overall_unfiltered"][p] = aggregate_group(sub_all, wall_filtered=False)
        strata["overall_wall_filtered"][p] = aggregate_group(sub_all, wall_filtered=True)

        # 0.90 m
        sub_090 = [e for e in sub_all if e["corridor"] == "corridor_090"]
        strata["corridor_090_unfiltered"][p] = aggregate_group(sub_090, wall_filtered=False)
        strata["corridor_090_wall_filtered"][p] = aggregate_group(sub_090, wall_filtered=True)

        # 1.20 m
        sub_120 = [e for e in sub_all if e["corridor"] == "corridor_120"]
        strata["corridor_120_unfiltered"][p] = aggregate_group(sub_120, wall_filtered=False)
        strata["corridor_120_wall_filtered"][p] = aggregate_group(sub_120, wall_filtered=True)

    # Statistical Significance Testing for Pairs: (H1, H2), (H2, H3), (H1, H3)
    # Tested metrics:
    # - min_d_clear_m
    # - max_human_lat_dev_m
    # - human_vel_drop_mps
    # - dist_at_lat_onset_m
    # - max_robot_lat_dev_m
    # - min_robot_speed_mps
    metrics_to_test = [
        ("min_d_clear_m", lambda e: e["metrics_unfiltered"]["min_d_clear_m"]),
        ("max_human_lat_dev_m", lambda e: e["metrics_unfiltered"]["max_h_lat_dev_m"]),
        ("human_vel_drop_mps", lambda e: e["metrics_unfiltered"]["h_vel_drop_mps"]),
        ("dist_at_lat_onset_m", lambda e: e["metrics_unfiltered"]["dist_at_lat_onset_m"]),
        ("max_robot_lat_dev_m", lambda e: e["metrics_unfiltered"]["max_r_lat_dev_m"]),
        ("min_robot_speed_mps", lambda e: e["metrics_unfiltered"]["min_r_speed_mps"]),
    ]

    pairs = [("H1", "H2"), ("H2", "H3"), ("H1", "H3")]
    stat_tests_overall = {}
    stat_tests_090 = {}
    stat_tests_120 = {}

    def run_tests_for_subset(subset: List[Dict[str, Any]]) -> Dict[str, Any]:
        pair_results = {}
        for p1, p2 in pairs:
            grp1 = [e for e in subset if e["persona"] == p1]
            grp2 = [e for e in subset if e["persona"] == p2]
            pair_key = f"{p1}_vs_{p2}"
            pair_results[pair_key] = {}
            for m_name, extractor in metrics_to_test:
                v1 = [extractor(e) for e in grp1]
                v2 = [extractor(e) for e in grp2]
                pair_results[pair_key][m_name] = perform_statistical_comparison(v1, v2, p1, p2)
        return pair_results

    stat_tests_overall = run_tests_for_subset(all_ep_results)
    stat_tests_090 = run_tests_for_subset([e for e in all_ep_results if e["corridor"] == "corridor_090"])
    stat_tests_120 = run_tests_for_subset([e for e in all_ep_results if e["corridor"] == "corridor_120"])

    output_data = {
        "metadata": {
            "source_dataset": "pilot_300_dataset",
            "eval_filter": "use_nav2 == True and not is_frozen and has_cpa_lt_15 == True and persona in [H1, H2, H3]",
            "total_eval_episodes": len(all_ep_results),
            "bootstrap_iterations": 2000,
            "bootstrap_ci": 0.95
        },
        "actual_persona_plugin_parameters": plugin_summary,
        "stratified_results": strata,
        "statistical_tests": {
            "overall": stat_tests_overall,
            "corridor_090": stat_tests_090,
            "corridor_120": stat_tests_120
        },
        "episodes_raw": all_ep_results
    }

    with open(OUTPUT_JSON, "w") as fp:
        json.dump(output_data, fp, indent=2)

    print(f"\n[OK] Successfully exported recalculated results to: {OUTPUT_JSON}")
    print(f"File size: {os.path.getsize(OUTPUT_JSON) / 1024:.1f} KB\n")

    # Pretty print summary tables
    def fmt_med_iqr(d):
        if d.get("median") is None: return "N/A"
        return f"{d['median']:.4f} (IQR {d['iqr']:.4f})"

    print("=" * 100)
    print("1. ACTUAL PERSONA PLUGIN PARAMETERS (rvo2_human_system.cpp)")
    print("=" * 100)
    for p in ["H1", "H2", "H3"]:
        info = plugin_summary[p]
        print(f"Persona {p} (N={info['n']}):")
        print(f"  * Responsibility alpha: {info['responsibility_alpha']['theoretical']} | Empirical Mean = {info['responsibility_alpha']['empirical_mean']} [min={info['responsibility_alpha']['empirical_min']}, max={info['responsibility_alpha']['empirical_max']}]")
        print(f"  * Politeness: {info['politeness']['theoretical']} | Empirical Mean = {info['politeness']['empirical_mean']}")
        print(f"  * Ignores Robot: {info['ignores_robot']}")

    print("\n" + "=" * 100)
    print("2. PERSONA MANIPULATION CHECK: OVERALL (UNFILTERED)")
    print("=" * 100)
    headers = ["Metric", "H1 (Cooperative)", "H2 (Unyielding)", "H3 (RobotBlind)"]
    print(f"{headers[0]:<40} | {headers[1]:<20} | {headers[2]:<20} | {headers[3]:<20}")
    print("-" * 106)
    o = strata["overall_unfiltered"]
    print(f"{'N Episodes':<40} | {o['H1']['n_episodes']:<20} | {o['H2']['n_episodes']:<20} | {o['H3']['n_episodes']:<20}")
    print(f"{'Collision Rate (%)':<40} | {o['H1']['collision_rate_pct']:<20} | {o['H2']['collision_rate_pct']:<20} | {o['H3']['collision_rate_pct']:<20}")
    print(f"{'Robot Stop/Yield Rate (%)':<40} | {o['H1']['robot_stop_yield_rate_pct']:<20} | {o['H2']['robot_stop_yield_rate_pct']:<20} | {o['H3']['robot_stop_yield_rate_pct']:<20}")
    print(f"{'Human Evasion Rate (%)':<40} | {o['H1']['human_evasion_rate_pct']:<20} | {o['H2']['human_evasion_rate_pct']:<20} | {o['H3']['human_evasion_rate_pct']:<20}")
    print(f"{'Min d_clear (m) [Med (IQR)]':<40} | {fmt_med_iqr(o['H1']['min_d_clear_m']):<20} | {fmt_med_iqr(o['H2']['min_d_clear_m']):<20} | {fmt_med_iqr(o['H3']['min_d_clear_m']):<20}")
    print(f"{'Max Human Lat Dev (m) [Med (IQR)]':<40} | {fmt_med_iqr(o['H1']['max_human_lat_dev_m']):<20} | {fmt_med_iqr(o['H2']['max_human_lat_dev_m']):<20} | {fmt_med_iqr(o['H3']['max_human_lat_dev_m']):<20}")
    print(f"{'Human Vel Drop (m/s) [Med (IQR)]':<40} | {fmt_med_iqr(o['H1']['human_vel_drop_mps']):<20} | {fmt_med_iqr(o['H2']['human_vel_drop_mps']):<20} | {fmt_med_iqr(o['H3']['human_vel_drop_mps']):<20}")
    print(f"{'Dist at Lat Onset (m) [Med (IQR)]':<40} | {fmt_med_iqr(o['H1']['dist_at_lat_onset_m']):<20} | {fmt_med_iqr(o['H2']['dist_at_lat_onset_m']):<20} | {fmt_med_iqr(o['H3']['dist_at_lat_onset_m']):<20}")
    print(f"{'Max Robot Lat Dev (m) [Med (IQR)]':<40} | {fmt_med_iqr(o['H1']['max_robot_lat_dev_m']):<20} | {fmt_med_iqr(o['H2']['max_robot_lat_dev_m']):<20} | {fmt_med_iqr(o['H3']['max_robot_lat_dev_m']):<20}")
    print(f"{'Min Robot Speed (m/s) [Med (IQR)]':<40} | {fmt_med_iqr(o['H1']['min_robot_speed_mps']):<20} | {fmt_med_iqr(o['H2']['min_robot_speed_mps']):<20} | {fmt_med_iqr(o['H3']['min_robot_speed_mps']):<20}")
    print(f"{'Time to Goal (s) [Med (IQR)]':<40} | {fmt_med_iqr(o['H1']['time_to_goal_s']):<20} | {fmt_med_iqr(o['H2']['time_to_goal_s']):<20} | {fmt_med_iqr(o['H3']['time_to_goal_s']):<20}")

    print("\n" + "=" * 100)
    print("3. STATISTICAL COMPARISONS: MANN-WHITNEY U & CLIFF'S DELTA (OVERALL)")
    print("=" * 100)
    for pair_key in ["H1_vs_H2", "H2_vs_H3", "H1_vs_H3"]:
        print(f"\n--- Comparison: {pair_key} ---")
        for m_name in [m[0] for m in metrics_to_test]:
            st = stat_tests_overall[pair_key][m_name]
            ci_str = f"[{st['cliffs_delta_ci_95'][0]}, {st['cliffs_delta_ci_95'][1]}]" if st['cliffs_delta_ci_95'] else "N/A"
            print(f"  * {m_name:<25}: p={st['p_value']:<10} | Cliff's delta={st['cliffs_delta']:<8} (95% CI: {ci_str:<18}) | Effect: {st['effect_size']}")



if __name__ == '__main__':
    main()
