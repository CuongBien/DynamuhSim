#!/usr/bin/env python3
"""
Step 2 Benchmark Runner (2a and 2b):
- Step 2a: inflation_radius = 0.20, cost_scaling_factor = 10.0 in local and global costmaps.
           near_collision_cost = 140 (unchanged), footprint 0.24x0.20 (unchanged).
- Step 2b: From 2a, near_collision_cost = 253 (Nav2 default).
- Model: Lidar 8m (model_mecanum_step1.sdf), local costmap 10x10m, obstacle range 7m (from Step 1).
- 40 trials: corridor 0.9m & 1.2m x H1 & H3 x 10 seeds (101-110, 201-210, 301-310, 401-410). RTF = 1.0.
"""

import os
import sys
import time
import subprocess
import json
import argparse
import numpy as np


from pathlib import Path as FilePath

ROOT_DIR = str(FilePath(__file__).resolve().parents[4])
sys.path.insert(0, ROOT_DIR)
from experiments.gazebo_benchmark.core.mppi_monitor import Step2InteractionMonitor
from experiments.gazebo_benchmark.core.mppi_trial import run_trial


def run_single_trial(config_mode: str, corridor_type: str, persona: str, seed: int, v_human: float, domain_id: int, out_json: str):
    return run_trial(ROOT_DIR, config_mode, corridor_type, persona, seed, v_human, domain_id, out_json,
                     monitor_factory=Step2InteractionMonitor)


def aggregate_and_report(trials: list, config_mode: str, out_file: str):
    valid_trials = [t for t in trials if not t.get("error")]
    print(f"\n=======================================================", flush=True)
    print(f"STEP {config_mode.upper()} BENCHMARK AGGREGATION ({len(valid_trials)}/{len(trials)} Valid Trials)", flush=True)
    print(f"=======================================================", flush=True)

    summary = {}
    for c_type in ["corridor_090", "corridor_120"]:
        for persona in ["H1", "H3"]:
            key = f"{c_type}_{persona}"
            c_trials = [t for t in valid_trials if t["corridor"] == c_type and t["persona"] == persona]
            n = len(c_trials)
            if n == 0:
                continue

            contact_collisions = sum(1 for t in c_trials if t.get("collision_contact"))
            contact_rate = round(contact_collisions / n * 100.0, 1)

            goal_reaches = sum(1 for t in c_trials if t.get("goal_reached"))
            goal_rate = round(goal_reaches / n * 100.0, 1)

            detours = sum(1 for t in c_trials if t.get("detour_around_human"))
            detour_rate = round(detours / n * 100.0, 1)

            dmin_clears = [t["dmin_edge_to_edge"] for t in c_trials if t.get("dmin_edge_to_edge") is not None]
            dmin_clear_med = round(float(np.median(dmin_clears)), 4) if dmin_clears else None

            max_lat_devs = [t["max_lateral_dev"] for t in c_trials if t.get("max_lateral_dev") is not None]
            max_lat_med = round(float(np.median(max_lat_devs)), 4) if max_lat_devs else None

            yielded_stops = sum(1 for t in c_trials if t.get("robot_stopped_or_yielded"))
            yielded_rate = round(yielded_stops / n * 100.0, 1)

            frozen_starts = sum(1 for t in c_trials if t.get("frozen_at_start"))

            reach_times = [t["time_to_goal"] for t in c_trials if t.get("goal_reached") and t.get("time_to_goal") is not None]
            t_goal_med = round(float(np.median(reach_times)), 2) if reach_times else None

            hz_means = [t["cmd_vel_sim_hz_mean"] for t in c_trials if t.get("cmd_vel_sim_hz_mean") and t["cmd_vel_sim_hz_mean"] > 0]
            hz_mean_val = round(float(np.mean(hz_means)), 2) if hz_means else None

            first_costs = [t["first_costmap_dist_m"] for t in c_trials if t.get("first_costmap_dist_m") is not None]
            first_cost_med = round(float(np.median(first_costs)), 2) if first_costs else None

            first_ts = [t["first_costmap_t_s"] for t in c_trials if t.get("first_costmap_t_s") is not None]
            first_t_med = round(float(np.median(first_ts)), 2) if first_ts else None

            costs_left_2m = [t["cost_left_lane_at_2m"] for t in c_trials if t.get("cost_left_lane_at_2m") is not None]
            cost_left_2m_med = round(float(np.median(costs_left_2m)), 1) if costs_left_2m else None

            costs_right_2m = [t["cost_right_lane_at_2m"] for t in c_trials if t.get("cost_right_lane_at_2m") is not None]
            cost_right_2m_med = round(float(np.median(costs_right_2m)), 1) if costs_right_2m else None

            min_left_profs = [t["min_cost_left_lane_profile_at_2m"] for t in c_trials if t.get("min_cost_left_lane_profile_at_2m") is not None]
            min_left_prof_med = round(float(np.median(min_left_profs)), 1) if min_left_profs else None

            min_right_profs = [t["min_cost_right_lane_profile_at_2m"] for t in c_trials if t.get("min_cost_right_lane_profile_at_2m") is not None]
            min_right_prof_med = round(float(np.median(min_right_profs)), 1) if min_right_profs else None

            summary[key] = {
                "n_trials": n,
                "contact_collision_rate_pct": contact_rate,
                "goal_reach_rate_pct": goal_rate,
                "detour_rate_pct": detour_rate,
                "dmin_clear_median_m": dmin_clear_med,
                "max_lateral_dev_median_m": max_lat_med,
                "yielded_stop_rate_pct": yielded_rate,
                "frozen_at_start_count": frozen_starts,
                "time_to_goal_median_s": t_goal_med,
                "cmd_vel_sim_hz_mean": hz_mean_val,
                "first_costmap_dist_median_m": first_cost_med,
                "first_costmap_t_median_s": first_t_med,
                "cost_left_lane_at_2m_median": cost_left_2m_med,
                "cost_right_lane_at_2m_median": cost_right_2m_med,
                "min_cost_left_lane_profile_at_2m_median": min_left_prof_med,
                "min_cost_right_lane_profile_at_2m_median": min_right_prof_med,
            }

    final_payload = {
        "step": f"Step {config_mode}",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "hardware": "AMD Ryzen 7 7735H, RTX 4060 Laptop GPU",
        "trials": trials,
        "summary": summary,
    }

    with open(out_file, "w") as fp:
        json.dump(final_payload, fp, indent=2)
    print(f"\n[OK] Raw results saved to: {out_file}", flush=True)

    print("\n" + "=" * 120, flush=True)
    print(f"{f'BẢNG TỔNG HỢP A3.3 BƯỚC {config_mode.upper()}':^120}", flush=True)
    print("=" * 120, flush=True)
    print(f"{'Chỉ số':<42} | {'0.90m H1':<16} | {'0.90m H3':<16} | {'1.20m H1':<16} | {'1.20m H3':<16}")
    print("-" * 120, flush=True)

    rows = [
        ("Tỉ lệ va chạm footprint (%)", "contact_collision_rate_pct", "{:.1f}%"),
        ("Tỉ lệ tới đích (%)", "goal_reach_rate_pct", "{:.1f}%"),
        ("d_clear nhỏ nhất mép-mép (m)", "dmin_clear_median_m", "{:.4f}m"),
        ("Độ lệch ngang lớn nhất của robot (m)", "max_lateral_dev_median_m", "{:.3f}m"),
        ("Tỉ lệ robot đi vòng qua người (|y|>=0.30m) (%)", "detour_rate_pct", "{:.1f}%"),
        ("Tỉ lệ robot dừng/nhường (%)", "yielded_stop_rate_pct", "{:.1f}%"),
        ("Thời gian tới đích trung vị (s)", "time_to_goal_median_s", "{:.2f}s"),
        ("Tần số cmd_vel sim (Mean Hz)", "cmd_vel_sim_hz_mean", "{:.1f} Hz"),
        ("Cự ly người vào costmap (m)", "first_costmap_dist_median_m", "{:.2f}m"),
        ("Thời điểm người vào costmap (s)", "first_costmap_t_median_s", "{:.2f}s"),
        ("Cost làn trái tại vị trí người (d~2m)", "cost_left_lane_at_2m_median", "{}"),
        ("Cost làn phải tại vị trí người (d~2m)", "cost_right_lane_at_2m_median", "{}"),
        ("Min cost profile làn trái (d~2m)", "min_cost_left_lane_profile_at_2m_median", "{}"),
        ("Min cost profile làn phải (d~2m)", "min_cost_right_lane_profile_at_2m_median", "{}"),
    ]

    for label, key, fmt in rows:
        row_str = f"{label:<42} | "
        for c_key in ["corridor_090_H1", "corridor_090_H3", "corridor_120_H1", "corridor_120_H3"]:
            val = summary.get(c_key, {}).get(key)
            val_str = fmt.format(val) if val is not None else "N/A"
            row_str += f"{val_str:<16} | "
        print(row_str, flush=True)
    print("=" * 120, flush=True)


def main():
    parser = argparse.ArgumentParser(description="Step 2 Benchmark Runner")
    parser.add_argument("--config", type=str, required=True, choices=["2a", "2b"])
    parser.add_argument("--run_single", action="store_true")
    parser.add_argument("--corridor", type=str, default="090")
    parser.add_argument("--persona", type=str, default="H1")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--v_human", type=float, default=1.15)
    parser.add_argument("--domain_id", type=int, default=85)
    parser.add_argument("--out_json", type=str, default="/tmp/res_single.json")
    parser.add_argument("--aggregate_only", action="store_true")
    args = parser.parse_args()

    out_final_json = os.path.join(ROOT_DIR, f"experiments/gazebo_benchmark/results/mppi_ablation/a3_3_step{args.config}_results.json")

    if args.run_single:
        run_single_trial(args.config, args.corridor, args.persona, args.seed, args.v_human, args.domain_id, args.out_json)
        os._exit(0)

    cells = [
        {"corridor": "090", "persona": "H1", "seed_base": 101, "name": "090_H1"},
        {"corridor": "090", "persona": "H3", "seed_base": 201, "name": "090_H3"},
        {"corridor": "120", "persona": "H1", "seed_base": 301, "name": "120_H1"},
        {"corridor": "120", "persona": "H3", "seed_base": 401, "name": "120_H3"},
    ]

    print("=" * 90, flush=True)
    print(f"STEP {args.config.upper()} BENCHMARK: INFLATION & NEAR COLLISION ABLATION", flush=True)
    print("=" * 90, flush=True)

    if not args.aggregate_only:
        runs_to_do = []
        for c in cells:
            c_type = c["corridor"]
            persona = c["persona"]
            s_base = c["seed_base"]
            for i in range(10):
                seed = s_base + i
                rng = np.random.RandomState(seed)
                v_h = round(float(rng.uniform(1.0, 1.3)), 3)
                out_f = f"/tmp/res_a3_3_step{args.config}_{c_type}_{persona}_{seed}.json"
                runs_to_do.append((c_type, persona, seed, v_h, out_f))

        print(f">>> Executing {len(runs_to_do)} trials sequentially for Step {args.config}...", flush=True)
        for idx, (c_type, persona, seed, v_h, out_f) in enumerate(runs_to_do, 1):
            if os.path.exists(out_f):
                try:
                    with open(out_f) as fp:
                        res = json.load(fp)
                    if not res.get("error") and res.get("time_to_collision") != 0.0:
                        print(f"[{idx}/{len(runs_to_do)}] Seed {seed} ({c_type}_{persona}) already done. Skipping.", flush=True)
                        continue
                except Exception:
                    pass

            print(f"\n[{idx}/{len(runs_to_do)}] Running Step {args.config}: Corridor {c_type}, Persona {persona}, Seed {seed} (v_human={v_h:.3f} m/s)...", flush=True)
            cmd = [
                sys.executable, __file__,
                "--config", str(args.config),
                "--run_single",
                "--corridor", str(c_type),
                "--persona", str(persona),
                "--seed", str(seed),
                "--v_human", str(v_h),
                "--domain_id", str(args.domain_id),
                "--out_json", str(out_f)
            ]
            t_s = time.time()
            try:
                subprocess.run(cmd, check=True)
            except Exception as e:
                print(f"[ERROR] Trial failed for seed {seed}: {e}", flush=True)
            t_dur = time.time() - t_s
            if os.path.exists(out_f):
                with open(out_f) as fp:
                    d = json.load(fp)
                print(f"  -> Done in {t_dur:.1f}s | Goal: {d.get('goal_reached')} | Contact: {d.get('collision_contact')} | "
                      f"dmin_clear: {d.get('dmin_edge_to_edge')}m | Detour: {d.get('detour_around_human')} (y={d.get('y_at_min_dclear')}m) | "
                      f"Left lane cost 2m: {d.get('cost_left_lane_at_2m')} | Right lane cost 2m: {d.get('cost_right_lane_at_2m')}", flush=True)
            time.sleep(1.0)

    # Gather results
    trials = []
    for c in cells:
        c_type = c["corridor"]
        persona = c["persona"]
        s_base = c["seed_base"]
        for i in range(10):
            seed = s_base + i
            out_file = f"/tmp/res_a3_3_step{args.config}_{c_type}_{persona}_{seed}.json"
            if os.path.exists(out_file):
                with open(out_file) as fp:
                    trials.append(json.load(fp))

    aggregate_and_report(trials, args.config, out_final_json)


if __name__ == '__main__':
    main()
