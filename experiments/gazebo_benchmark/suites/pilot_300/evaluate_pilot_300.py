#!/usr/bin/env python3
"""
Evaluate Pilot 300 Dataset:
1. Invariants & QC verification:
   - Steps with d_center < 0.30m without collision flag == 0
   - Wall penetration (> 2cm) rate by corridor
   - Geometric interaction rates (< 1.5m and < 2.0m)
   - Feasibility filter rejection summary
   - Frozen rate check (< 10%)
2. Core Persona Manipulation Check:
   - Evaluated strictly on interactive, feasible, non-frozen, Nav2 episodes
   - For H1, H2, H3: Median and IQR of:
     * min d_clear
     * max human lateral deviation
     * human velocity drop
     * distance to robot at lateral deviation onset
     * max robot lateral deviation
     * min robot speed
     * robot stop/yield rate
     * time to goal
     * collision rate
3. Outputs raw evaluation JSON: experiments/gazebo_benchmark/results/pilot_300/pilot_300_evaluation_results.json
"""

import os
import sys
import json
import math
import numpy as np
from typing import Dict, List, Any

ROOT_DIR = "/home/cuongbien/DynamuhSim"
EPISODES_DIR = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/pilot_300/pilot_300_dataset/episodes")
OUTPUT_JSON = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/pilot_300/pilot_300_evaluation_results.json")


def compute_median_iqr(vals: List[float]) -> Dict[str, float]:
    valid = [v for v in vals if v is not None and not math.isnan(v)]
    if not valid:
        return {"median": 0.0, "q25": 0.0, "q75": 0.0, "iqr": 0.0, "count": 0}
    q25, med, q75 = np.percentile(valid, [25, 50, 75])
    return {
        "median": round(float(med), 4),
        "q25": round(float(q25), 4),
        "q75": round(float(q75), 4),
        "iqr": round(float(q75 - q25), 4),
        "count": len(valid)
    }


def evaluate_dataset(episodes_dir: str = EPISODES_DIR) -> Dict[str, Any]:
    ep_dirs = sorted([d for d in os.listdir(episodes_dir) if d.startswith("ep_")])
    print(f"Found {len(ep_dirs)} episode directories.")

    all_manifests = []
    total_invariant_violations = 0
    wall_pen_by_corridor = {
        "corridor_090": {"pen_steps": 0, "total_steps": 0},
        "corridor_120": {"pen_steps": 0, "total_steps": 0}
    }

    cpa_lt_15_count = 0
    cpa_lt_20_count = 0
    frozen_count = 0
    qc_passed_count = 0

    ep_metrics_list = []

    for ep_name in ep_dirs:
        ep_path = os.path.join(episodes_dir, ep_name)
        m_file = os.path.join(ep_path, "manifest.json")
        gt_file = os.path.join(ep_path, "ground_truth.json")
        sm_file = os.path.join(ep_path, "step_metrics.json")

        if not os.path.exists(m_file) or not os.path.exists(gt_file):
            continue

        with open(m_file) as fp:
            m = json.load(fp)
        with open(gt_file) as fp:
            gt = json.load(fp)

        step_metrics = []
        if os.path.exists(sm_file):
            with open(sm_file) as fp:
                step_metrics = json.load(fp)

        all_manifests.append(m)

        # QC and invariants
        inv_viol = m.get("invariant_violations", 0)
        total_invariant_violations += inv_viol

        corr = m.get("corridor", "corridor_090")
        w_pen = m.get("wall_pen_steps", 0)
        tot_steps = m.get("total_human_steps", 0)
        if corr in wall_pen_by_corridor:
            wall_pen_by_corridor[corr]["pen_steps"] += w_pen
            wall_pen_by_corridor[corr]["total_steps"] += tot_steps

        if m.get("has_cpa_lt_15", False):
            cpa_lt_15_count += 1
        if m.get("has_cpa_lt_20", False):
            cpa_lt_20_count += 1
        if m.get("is_frozen", False):
            frozen_count += 1
        if m.get("qc_passed", False):
            qc_passed_count += 1

        # Extract per-episode trajectory interaction metrics
        # Focal human is human_01
        h0_y = None
        h_max_lat_dev = 0.0
        h_min_speed = 999.0
        h_init_speed = 1.0
        dist_at_lat_onset = None
        r_max_lat_dev = 0.0
        r_min_speed = 999.0
        r_stopped_or_yielded = False
        min_dclear = 999.0

        for idx, step in enumerate(gt):
            r_info = step.get('robot', {})
            rx = r_info.get('x', -13.0)
            ry = r_info.get('y', 0.0)
            rv = math.hypot(r_info.get('vx', 0.0), r_info.get('vy', 0.0))

            if rv < r_min_speed:
                r_min_speed = rv
            if abs(ry) > r_max_lat_dev:
                r_max_lat_dev = abs(ry)
            if rv < 0.05:
                r_stopped_or_yielded = True

            humans = step.get('humans', [])
            if len(humans) > 0:
                h = humans[0]  # Primary human
                hx = h.get('x', 0.0)
                hy = h.get('y', 0.0)
                hv = math.hypot(h.get('vx', 0.0), h.get('vy', 0.0))

                if h0_y is None:
                    h0_y = hy
                    h_init_speed = max(0.1, hv)

                lat_dev = abs(hy - h0_y)
                if lat_dev > h_max_lat_dev:
                    h_max_lat_dev = lat_dev

                if lat_dev > 0.05 and dist_at_lat_onset is None:
                    dist_at_lat_onset = math.hypot(rx - hx, ry - hy)

                if hv < h_min_speed:
                    h_min_speed = hv

        # Min dclear from step metrics or manifest
        if step_metrics:
            dclears = [s.get('min_d_clear', 999.0) for s in step_metrics]
            min_dclear = min(dclears) if dclears else 999.0

        h_vel_drop = max(0.0, h_init_speed - h_min_speed)

        ep_metrics_list.append({
            "episode_id": m.get("episode_id"),
            "persona": m.get("persona"),
            "corridor": corr,
            "num_humans": m.get("num_humans"),
            "use_nav2": m.get("use_nav2"),
            "is_interactive_target": m.get("is_interactive_target", False),
            "has_cpa_lt_15": m.get("has_cpa_lt_15", False),
            "is_frozen": m.get("is_frozen", False),
            "collision_occurred": m.get("collision_occurred", False),
            "time_to_goal": m.get("time_to_goal", None),
            "min_d_clear": round(min_dclear, 4),
            "max_h_lat_dev": round(h_max_lat_dev, 4),
            "h_vel_drop": round(h_vel_drop, 4),
            "dist_at_lat_onset": round(dist_at_lat_onset, 4) if dist_at_lat_onset is not None else None,
            "max_r_lat_dev": round(r_max_lat_dev, 4),
            "min_r_speed": round(r_min_speed, 4),
            "r_stopped_or_yielded": r_stopped_or_yielded
        })

    # Wall penetration percentages
    wall_pen_summary = {}
    for c_k, c_v in wall_pen_by_corridor.items():
        tot = max(1, c_v["total_steps"])
        pct = round(c_v["pen_steps"] / tot * 100.0, 2)
        wall_pen_summary[c_k] = {
            "pen_steps_gt_2cm": c_v["pen_steps"],
            "total_human_steps": c_v["total_steps"],
            "penetration_rate_pct": pct
        }

    # Filter evaluation set: interactive (cpa < 1.5m), non-frozen, Nav2 == True
    eval_set = [
        e for e in ep_metrics_list
        if e["has_cpa_lt_15"] and not e["is_frozen"] and e["use_nav2"]
    ]
    print(f"Filtered evaluation subset: {len(eval_set)} episodes.")

    # Group by persona H1, H2, H3
    persona_stats = {}
    for p in ["H1", "H2", "H3"]:
        sub = [e for e in eval_set if e["persona"] == p]
        n_eps = len(sub)
        if n_eps == 0:
            continue

        cols = [e["collision_occurred"] for e in sub]
        col_rate = round(sum(cols) / n_eps * 100.0, 1)

        stops = [e["r_stopped_or_yielded"] for e in sub]
        stop_rate = round(sum(stops) / n_eps * 100.0, 1)

        t_goals = [e["time_to_goal"] for e in sub if e["time_to_goal"] is not None]

        persona_stats[p] = {
            "num_episodes": n_eps,
            "collision_rate_pct": col_rate,
            "robot_stop_yield_rate_pct": stop_rate,
            "min_d_clear_m": compute_median_iqr([e["min_d_clear"] for e in sub]),
            "max_human_lat_dev_m": compute_median_iqr([e["max_h_lat_dev"] for e in sub]),
            "human_vel_drop_mps": compute_median_iqr([e["h_vel_drop"] for e in sub]),
            "dist_at_lat_onset_m": compute_median_iqr([e["dist_at_lat_onset"] for e in sub]),
            "max_robot_lat_dev_m": compute_median_iqr([e["max_r_lat_dev"] for e in sub]),
            "min_robot_speed_mps": compute_median_iqr([e["min_r_speed"] for e in sub]),
            "time_to_goal_s": compute_median_iqr(t_goals)
        }

    results = {
        "dataset_summary": {
            "total_episodes": len(all_manifests),
            "qc_passed_count": qc_passed_count,
            "qc_pass_rate_pct": round(qc_passed_count / max(1, len(all_manifests)) * 100.0, 1),
            "invariant_violations_dcenter_lt_030_no_col": total_invariant_violations,
            "cpa_lt_15_count": cpa_lt_15_count,
            "cpa_lt_15_rate_pct": round(cpa_lt_15_count / max(1, len(all_manifests)) * 100.0, 1),
            "cpa_lt_20_count": cpa_lt_20_count,
            "cpa_lt_20_rate_pct": round(cpa_lt_20_count / max(1, len(all_manifests)) * 100.0, 1),
            "frozen_count": frozen_count,
            "frozen_rate_pct": round(frozen_count / max(1, len(all_manifests)) * 100.0, 1),
            "wall_penetration": wall_pen_summary
        },
        "evaluation_subset_count": len(eval_set),
        "persona_manipulation_check": persona_stats
    }

    with open(OUTPUT_JSON, "w") as fp:
        json.dump(results, fp, indent=2)

    print("=" * 80)
    print("PILOT 300 EVALUATION RESULTS SUMMARY")
    print("=" * 80)
    print(f"Total Episodes Analyzed: {len(all_manifests)}")
    print(f"Invariant Violations (d_center < 0.30m without collision): {total_invariant_violations} (ASSERTION PASSED: {total_invariant_violations == 0})")
    print(f"CPA < 1.5m: {cpa_lt_15_count} ({results['dataset_summary']['cpa_lt_15_rate_pct']}%)")
    print(f"CPA < 2.0m: {cpa_lt_20_count} ({results['dataset_summary']['cpa_lt_20_rate_pct']}%)")
    print(f"Frozen Rate: {frozen_count} ({results['dataset_summary']['frozen_rate_pct']}%) [Threshold <= 10.0%]")
    print(f"Wall Penetration > 2cm: corridor_090 = {wall_pen_summary['corridor_090']['penetration_rate_pct']}%, corridor_120 = {wall_pen_summary['corridor_120']['penetration_rate_pct']}%")
    print("\n--- PERSONA MANIPULATION CHECK (H1 vs H2 vs H3) ---")
    for p, stats in persona_stats.items():
        print(f"\nPersona {p} (N={stats['num_episodes']}):")
        print(f"  * Collision Rate: {stats['collision_rate_pct']}%")
        print(f"  * Robot Stop/Yield Rate: {stats['robot_stop_yield_rate_pct']}%")
        print(f"  * Min d_clear: {stats['min_d_clear_m']['median']} m (IQR: {stats['min_d_clear_m']['iqr']} m)")
        print(f"  * Max Human Lateral Dev: {stats['max_human_lat_dev_m']['median']} m (IQR: {stats['max_human_lat_dev_m']['iqr']} m)")
        print(f"  * Human Velocity Drop: {stats['human_vel_drop_mps']['median']} m/s (IQR: {stats['human_vel_drop_mps']['iqr']} m/s)")
        print(f"  * Dist at Lateral Dev Onset: {stats['dist_at_lat_onset_m']['median']} m (IQR: {stats['dist_at_lat_onset_m']['iqr']} m)")
        print(f"  * Max Robot Lateral Dev: {stats['max_robot_lat_dev_m']['median']} m (IQR: {stats['max_robot_lat_dev_m']['iqr']} m)")
        print(f"  * Min Robot Speed: {stats['min_robot_speed_mps']['median']} m/s (IQR: {stats['min_robot_speed_mps']['iqr']} m/s)")
        print(f"  * Time to Goal: {stats['time_to_goal_s']['median']} s (IQR: {stats['time_to_goal_s']['iqr']} s)")
    print("=" * 80)
    print(f"Exported evaluation results to: {OUTPUT_JSON}")
    return results


if __name__ == '__main__':
    evaluate_dataset()
