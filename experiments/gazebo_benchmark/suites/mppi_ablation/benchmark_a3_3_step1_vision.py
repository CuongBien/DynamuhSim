#!/usr/bin/env python3
"""
Step 1: Sensing Horizon Ablation on A3.3 Benchmark.
Changes applied (Step 1):
- Lidar max range increased from 3.5m to 8.0m (in model_mecanum_step1.sdf).
- obstacle_max_range & raytrace_max_range increased from 3.5m to 7.0m (in nav2_mppi_step1.yaml).
- Local costmap width & height increased from 6x6m to 10x10m (in nav2_mppi_step1.yaml).
- All other parameters (8 critics, batch 280, time_steps 30, inflation 0.35/5.0, footprint 0.24x0.20) are UNCHANGED.
- Runs 40 trials (corridor 0.9m & 1.2m x H1 & H3 x 10 seeds: 101-110, 201-210, 301-310, 401-410).
- RTF = 1.0 lockstep.
- Measures:
  - Footprint contact collision rate (truncated at first contact)
  - Goal reach rate & time to goal
  - dmin edge-to-edge (d_clear)
  - Max lateral deviation (|y|)
  - Robot stopped/yielded rate (v < 0.05 m/s >= 1s, after t=2s)
  - cmd_vel sim frequency (mean & min)
  - First human entry into costmap: distance (d_center) and sim time (t_sim)
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
from experiments.gazebo_benchmark.core.mppi_monitor import Step1InteractionMonitor
from experiments.gazebo_benchmark.core.mppi_trial import run_trial
OUTPUT_JSON = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/mppi_ablation/a3_3_step1_vision_results.json")
BASELINE_JSON = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/correctness_a3/a3_3_rtf1_real_interaction_results.json")


def run_single_trial(corridor_type: str, persona: str, seed: int, v_human: float, domain_id: int, out_json: str):
    return run_trial(ROOT_DIR, "1", corridor_type, persona, seed, v_human, domain_id, out_json,
                     monitor_factory=Step1InteractionMonitor)


def aggregate_and_report(trials: list):
    cells = [
        {"corridor": "090", "persona": "H1", "name": "090_H1"},
        {"corridor": "090", "persona": "H3", "name": "090_H3"},
        {"corridor": "120", "persona": "H1", "name": "120_H1"},
        {"corridor": "120", "persona": "H3", "name": "120_H3"},
    ]

    summary = {}
    for c in cells:
        c_corr = f"corridor_{c['corridor']}"
        c_pers = c["persona"]
        c_key = f"{c_corr}_{c_pers}"
        m = [t for t in trials if t.get("corridor") == c_corr and t.get("persona") == c_pers]
        if m:
            d_clears = [t["dmin_edge_to_edge"] for t in m if t.get("dmin_edge_to_edge") is not None]
            contact_colls = [1 if t.get("collision_contact") else 0 for t in m]
            goals = [1 if t.get("goal_reached") else 0 for t in m]
            lat_devs = [t["max_lateral_dev"] for t in m if t.get("max_lateral_dev") is not None]
            yielded = [1 if t.get("robot_stopped_or_yielded") else 0 for t in m]
            cmd_hz_means = [t["cmd_vel_sim_hz_mean"] for t in m if t.get("cmd_vel_sim_hz_mean") is not None]
            times = [t["time_to_goal"] for t in m if t.get("time_to_goal") is not None]
            cm_dists = [t["first_costmap_dist_m"] for t in m if t.get("first_costmap_dist_m") is not None]
            cm_times = [t["first_costmap_t_s"] for t in m if t.get("first_costmap_t_s") is not None]

            summary[c_key] = {
                "n_runs": len(m),
                "contact_collision_rate_pct": round(float(np.mean(contact_colls) * 100), 1),
                "goal_reach_rate_pct": round(float(np.mean(goals) * 100), 1),
                "dmin_clear_median_m": round(float(np.median(d_clears)), 4) if d_clears else None,
                "dmin_clear_min_m": round(float(np.min(d_clears)), 4) if d_clears else None,
                "max_lateral_dev_median_m": round(float(np.median(lat_devs)), 4) if lat_devs else None,
                "yielded_stop_rate_pct": round(float(np.mean(yielded) * 100), 1),
                "time_to_goal_median_s": round(float(np.median(times)), 2) if times else None,
                "cmd_vel_sim_hz_mean": round(float(np.mean(cmd_hz_means)), 2) if cmd_hz_means else None,
                "first_costmap_dist_median_m": round(float(np.median(cm_dists)), 3) if cm_dists else None,
                "first_costmap_t_median_s": round(float(np.median(cm_times)), 2) if cm_times else None,
            }

    # Load baseline Step 0 summary
    base_summary = {}
    if os.path.exists(BASELINE_JSON):
        try:
            with open(BASELINE_JSON) as fp:
                b_data = json.load(fp)
                base_summary = b_data.get("summary", {})
        except Exception:
            pass

    full_output = {
        "step": "Step 1: Sensing Horizon (8m Lidar, 7m Obstacle Range, 10x10m Local Costmap)",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "hardware": {
            "cpu": "AMD Ryzen 7 7735H (8 cores, 16 threads)",
            "gpu": "NVIDIA GeForce RTX 4060 Laptop GPU"
        },
        "trials": trials,
        "summary": summary,
        "baseline_summary_step0": base_summary
    }

    with open(OUTPUT_JSON, "w") as fp:
        json.dump(full_output, fp, indent=2)

    print(f"\n[OK] Raw Step 1 results saved to: {OUTPUT_JSON}", flush=True)

    print("\n" + "=" * 105, flush=True)
    print(f"{'BẢNG SO SÁNH A3.3: BƯỚC 0 (CẤU HÌNH GỐC) VS BƯỚC 1 (TẦM NHÌN 8M/10M)':^105}", flush=True)
    print("=" * 105, flush=True)
    print(f"{'Chỉ số':<35} | {'0.90m H1':<15} | {'0.90m H3':<15} | {'1.20m H1':<15} | {'1.20m H3':<15}")
    print("-" * 105, flush=True)

    rows = [
        ("Tỉ lệ va chạm footprint (%)", "contact_collision_rate_pct", "{:.1f}%"),
        ("Tỉ lệ tới đích (%)", "goal_reach_rate_pct", "{:.1f}%"),
        ("d_clear nhỏ nhất mép-mép (m)", "dmin_clear_median_m", "{:.4f}m"),
        ("Độ lệch ngang max robot (m)", "max_lateral_dev_median_m", "{:.3f}m"),
        ("Tỉ lệ robot dừng/nhường (%)", "yielded_stop_rate_pct", "{:.1f}%"),
        ("Thời gian tới đích trung vị (s)", "time_to_goal_median_s", "{:.2f}s"),
        ("Tần số cmd_vel sim (Mean Hz)", "cmd_vel_sim_hz_mean", "{:.1f} Hz"),
        ("Cự ly người vào costmap (m)", "first_costmap_dist_median_m", "{:.2f}m"),
        ("Thời điểm người vào costmap (s)", "first_costmap_t_median_s", "{:.2f}s"),
    ]

    for label, key, fmt in rows:
        row_str = f"{label:<35} | "
        for c_key in ["corridor_090_H1", "corridor_090_H3", "corridor_120_H1", "corridor_120_H3"]:
            v_new = summary.get(c_key, {}).get(key)
            v_old = base_summary.get(c_key, {}).get(key)

            str_new = fmt.format(v_new) if v_new is not None else "N/A"
            if v_old is not None:
                str_old = fmt.format(v_old)
                cell_str = f"{str_old} -> {str_new}"
            else:
                cell_str = f"{str_new}"
            row_str += f"{cell_str:<15} | "
        print(row_str, flush=True)
    print("=" * 105, flush=True)


def main():
    parser = argparse.ArgumentParser(description="Step 1 Benchmark Runner")
    parser.add_argument("--run_single", action="store_true")
    parser.add_argument("--corridor", type=str, default="090")
    parser.add_argument("--persona", type=str, default="H1")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--v_human", type=float, default=1.15)
    parser.add_argument("--domain_id", type=int, default=80)
    parser.add_argument("--out_json", type=str, default="/tmp/res_single.json")
    parser.add_argument("--aggregate_only", action="store_true")
    args = parser.parse_args()

    if args.run_single:
        run_single_trial(args.corridor, args.persona, args.seed, args.v_human, args.domain_id, args.out_json)
        os._exit(0)

    cells = [
        {"corridor": "090", "persona": "H1", "seed_base": 101, "name": "090_H1"},
        {"corridor": "090", "persona": "H3", "seed_base": 201, "name": "090_H3"},
        {"corridor": "120", "persona": "H1", "seed_base": 301, "name": "120_H1"},
        {"corridor": "120", "persona": "H3", "seed_base": 401, "name": "120_H3"},
    ]

    print("=" * 80, flush=True)
    print("STEP 1 BENCHMARK: SENSING HORIZON (8M LIDAR, 7M OBSTACLE RANGE, 10x10M COSTMAP)", flush=True)
    print("=" * 80, flush=True)

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
                out_f = f"/tmp/res_a3_3_step1_{c_type}_{persona}_{seed}.json"
                runs_to_do.append((c_type, persona, seed, v_h, out_f))

        print(f">>> Executing {len(runs_to_do)} trials sequentially...", flush=True)
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

            print(f"\n[{idx}/{len(runs_to_do)}] Running Corridor {c_type}, Persona {persona}, Seed {seed} (v_human={v_h:.3f} m/s)...", flush=True)
            cmd = [
                sys.executable, __file__,
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
                      f"dmin_clear: {d.get('dmin_edge_to_edge')}m | Costmap Entry: {d.get('first_costmap_dist_m')}m at {d.get('first_costmap_t_s')}s | Mean Hz: {d.get('cmd_vel_sim_hz_mean')}", flush=True)
            time.sleep(1.0)

    # Gather results
    trials = []
    for c in cells:
        c_type = c["corridor"]
        persona = c["persona"]
        s_base = c["seed_base"]
        for i in range(10):
            seed = s_base + i
            out_file = f"/tmp/res_a3_3_step1_{c_type}_{persona}_{seed}.json"
            if os.path.exists(out_file):
                with open(out_file) as fp:
                    trials.append(json.load(fp))

    aggregate_and_report(trials)


if __name__ == '__main__':
    main()
