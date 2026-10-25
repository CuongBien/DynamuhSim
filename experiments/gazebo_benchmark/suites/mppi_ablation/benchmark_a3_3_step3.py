#!/usr/bin/env python3
"""
Step 3 Benchmark Runner (3a, 3b, 3c):
- Step 3a: time_steps = 50 (model_dt 0.05, horizon 2.5s), accumulated on 2b.
- Step 3b: time_steps = 80 (model_dt 0.05, horizon 4.0s), accumulated on 3a.
- Step 3c: From 3b, cost_weight of PathAlignCritic (1.5), PathFollowCritic (3.0),
           PreferForwardCritic (2.0) halved. CostCritic (16.0) kept.
- Timer: max duration 40s sim-time strictly measured from goal acceptance.
- Metrics measured:
  - Footprint contact collision rate (%)
  - Near-miss (sượt d_clear < 0.02m) rate (%)
  - Goal reach rate (%) & time to goal (s)
  - d_clear min (m), max lateral dev |y| (m)
  - Detour rate (|y| >= 0.30m) (%)
  - Yield/stop rate (%)
  - Controller compute time per cycle: Mean & p95 (ms) via LD_PRELOAD profiler
  - Real cmd_vel sim frequency (Hz)
  - Steering start time (vy or |y| > 0.05m) & distance to human at that moment
  - Global path y position at human x when human distance is ~2m
  - Lane costs at 2m (left & right lane)
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
from experiments.gazebo_benchmark.core.mppi_monitor import Step3InteractionMonitor
from experiments.gazebo_benchmark.core.mppi_trial import run_trial


def run_single_trial(config_mode: str, corridor_type: str, persona: str, seed: int, v_human: float, domain_id: int, out_json: str):
    return run_trial(ROOT_DIR, config_mode, corridor_type, persona, seed, v_human, domain_id, out_json,
                     monitor_factory=Step3InteractionMonitor)


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

            near_misses = sum(1 for t in c_trials if t.get("near_miss_under_2cm"))
            near_miss_rate = round(near_misses / n * 100.0, 1)

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

            ctrl_means = [t["controller_calc_ms_mean"] for t in c_trials if t.get("controller_calc_ms_mean") is not None]
            ctrl_mean_val = round(float(np.mean(ctrl_means)), 2) if ctrl_means else None

            ctrl_p95s = [t["controller_calc_ms_p95"] for t in c_trials if t.get("controller_calc_ms_p95") is not None]
            ctrl_p95_val = round(float(np.median(ctrl_p95s)), 2) if ctrl_p95s else None

            steer_times = [t["t_steer_s"] for t in c_trials if t.get("t_steer_s") is not None]
            steer_time_med = round(float(np.median(steer_times)), 2) if steer_times else None

            steer_dists = [t["d_human_at_steer_m"] for t in c_trials if t.get("d_human_at_steer_m") is not None]
            steer_dist_med = round(float(np.median(steer_dists)), 2) if steer_dists else None

            first_costs = [t["first_costmap_dist_m"] for t in c_trials if t.get("first_costmap_dist_m") is not None]
            first_cost_med = round(float(np.median(first_costs)), 2) if first_costs else None

            first_ts = [t["first_costmap_t_s"] for t in c_trials if t.get("first_costmap_t_s") is not None]
            first_t_med = round(float(np.median(first_ts)), 2) if first_ts else None

            costs_left_2m = [t["cost_left_lane_at_2m"] for t in c_trials if t.get("cost_left_lane_at_2m") is not None]
            cost_left_2m_med = round(float(np.median(costs_left_2m)), 1) if costs_left_2m else None

            costs_right_2m = [t["cost_right_lane_at_2m"] for t in c_trials if t.get("cost_right_lane_at_2m") is not None]
            cost_right_2m_med = round(float(np.median(costs_right_2m)), 1) if costs_right_2m else None

            plan_ys = [t["global_path_y_at_2m"] for t in c_trials if t.get("global_path_y_at_2m") is not None]
            plan_y_med = round(float(np.median(plan_ys)), 4) if plan_ys else None

            summary[key] = {
                "n_trials": n,
                "contact_collision_rate_pct": contact_rate,
                "near_miss_under_2cm_rate_pct": near_miss_rate,
                "goal_reach_rate_pct": goal_rate,
                "detour_rate_pct": detour_rate,
                "dmin_clear_median_m": dmin_clear_med,
                "max_lateral_dev_median_m": max_lat_med,
                "yielded_stop_rate_pct": yielded_rate,
                "frozen_at_start_count": frozen_starts,
                "time_to_goal_median_s": t_goal_med,
                "controller_calc_ms_mean": ctrl_mean_val,
                "controller_calc_ms_p95": ctrl_p95_val,
                "cmd_vel_sim_hz_mean": hz_mean_val,
                "steer_start_time_median_s": steer_time_med,
                "steer_start_human_dist_median_m": steer_dist_med,
                "first_costmap_dist_median_m": first_cost_med,
                "first_costmap_t_median_s": first_t_med,
                "cost_left_lane_at_2m_median": cost_left_2m_med,
                "cost_right_lane_at_2m_median": cost_right_2m_med,
                "global_path_y_at_2m_median": plan_y_med,
            }

    final_payload = {
        "step": f"Step {config_mode}",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "hardware": "AMD Ryzen 7 7735H, RTX 4060 Laptop GPU",
        "timer_rule": "max_duration_s = 40.0s sim-time strictly measured from goal acceptance",
        "trials": trials,
        "summary": summary,
    }

    with open(out_file, "w") as fp:
        json.dump(final_payload, fp, indent=2)
    print(f"\n[OK] Raw results saved to: {out_file}", flush=True)

    print("\n" + "=" * 125, flush=True)
    print(f"{f'BẢNG TỔNG HỢP A3.3 BƯỚC {config_mode.upper()}':^125}", flush=True)
    print("=" * 125, flush=True)
    print(f"{'Chỉ số':<45} | {'0.90m H1':<16} | {'0.90m H3':<16} | {'1.20m H1':<16} | {'1.20m H3':<16}")
    print("-" * 125, flush=True)

    rows = [
        ("Tỉ lệ va chạm footprint (%)", "contact_collision_rate_pct", "{:.1f}%"),
        ("Tỉ lệ sượt (0 < d_clear < 0.02m) (%)", "near_miss_under_2cm_rate_pct", "{:.1f}%"),
        ("Tỉ lệ tới đích (%)", "goal_reach_rate_pct", "{:.1f}%"),
        ("d_clear nhỏ nhất mép-mép (m)", "dmin_clear_median_m", "{:.4f}m"),
        ("Độ lệch ngang lớn nhất của robot (m)", "max_lateral_dev_median_m", "{:.3f}m"),
        ("Tỉ lệ robot đi vòng qua người (|y|>=0.30m) (%)", "detour_rate_pct", "{:.1f}%"),
        ("Tỉ lệ robot dừng/nhường (%)", "yielded_stop_rate_pct", "{:.1f}%"),
        ("Thời gian tới đích trung vị (s)", "time_to_goal_median_s", "{:.2f}s"),
        ("Thời gian tính controller mỗi chu kỳ (Mean, ms)", "controller_calc_ms_mean", "{:.2f} ms"),
        ("Thời gian tính controller mỗi chu kỳ (p95, ms)", "controller_calc_ms_p95", "{:.2f} ms"),
        ("Tần số cmd_vel sim (Mean Hz)", "cmd_vel_sim_hz_mean", "{:.1f} Hz"),
        ("Thời điểm bắt đầu đánh lái (s)", "steer_start_time_median_s", "{}"),
        ("Khoảng cách tới người khi bắt đầu đánh lái (m)", "steer_start_human_dist_median_m", "{}"),
        ("Cự ly người vào costmap (m)", "first_costmap_dist_median_m", "{:.2f}m"),
        ("Thời điểm người vào costmap (s)", "first_costmap_t_median_s", "{:.2f}s"),
        ("Cost làn trái tại vị trí người (d~2m)", "cost_left_lane_at_2m_median", "{}"),
        ("Cost làn phải tại vị trí người (d~2m)", "cost_right_lane_at_2m_median", "{}"),
        ("Vị trí y đường đi toàn cục (d~2m, m)", "global_path_y_at_2m_median", "{}"),
    ]

    for label, key, fmt in rows:
        row_str = f"{label:<45} | "
        for c_key in ["corridor_090_H1", "corridor_090_H3", "corridor_120_H1", "corridor_120_H3"]:
            val = summary.get(c_key, {}).get(key)
            val_str = fmt.format(val) if val is not None else "N/A"
            row_str += f"{val_str:<16} | "
        print(row_str, flush=True)
    print("=" * 125, flush=True)


def main():
    parser = argparse.ArgumentParser(description="Step 3 Benchmark Runner")
    parser.add_argument("--config", type=str, required=True, choices=["3a", "3b", "3c"])
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

    print("=" * 95, flush=True)
    print(f"STEP {args.config.upper()} BENCHMARK: HORIZON & CRITIC WEIGHT ABLATION", flush=True)
    print("Timer rule: max_duration_s = 40.0s sim-time strictly measured from goal acceptance", flush=True)
    print("=" * 95, flush=True)

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
                print(f"  -> Done in {t_dur:.1f}s | Goal: {d.get('goal_reached')} | Contact: {d.get('collision_contact')} | Near-miss: {d.get('near_miss_under_2cm')} | "
                      f"dmin_clear: {d.get('dmin_edge_to_edge')}m | Detour: {d.get('detour_around_human')} (y={d.get('y_at_min_dclear')}m) | "
                      f"Steer: {d.get('t_steer_s')}s (d={d.get('d_human_at_steer_m')}m) | Ctrl mean: {d.get('controller_calc_ms_mean')}ms (p95={d.get('controller_calc_ms_p95')}ms) | Plan y: {d.get('global_path_y_at_2m')}", flush=True)
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
