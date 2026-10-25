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
import signal
import subprocess
import json
import re
import argparse
import numpy as np

import rclpy

from pathlib import Path as FilePath

ROOT_DIR = str(FilePath(__file__).resolve().parents[4])
sys.path.insert(0, ROOT_DIR)
from experiments.gazebo_benchmark.core.mppi_monitor import Step2InteractionMonitor
ROBOT_MODEL_SDF = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/robot_models/model_mecanum_step1.sdf")


def run_single_trial(config_mode: str, corridor_type: str, persona: str, seed: int, v_human: float, domain_id: int, out_json: str):
    gen_dir = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated")
    os.makedirs(gen_dir, exist_ok=True)

    nav2_yaml = os.path.join(ROOT_DIR, f"experiments/gazebo_benchmark/configs/nav2_mppi/nav2_mppi_step{config_mode}.yaml")

    # 1. Generate scenario JSON
    scen_file = os.path.join(gen_dir, f"scenario_a3_3_step{config_mode}_{corridor_type}_{persona}_{seed}.json")
    scen_data = {
        "episode_id": seed,
        "corridor_mode": True,
        "humans": [
            {
                "id": 1,
                "name": "human_01",
                "mode": "autonomous",
                "speed": v_human,
                "max_speed": v_human,
                "persona": persona,
                "loop": False,
                "wait_for_robot": True,
                "start": {"x": -7.0, "y": 0.0, "yaw": 3.14159265},
                "waypoints": [[-7.0, 0.0], [-15.0, 0.0]]
            }
        ]
    }
    with open(scen_file, "w") as f:
        json.dump(scen_data, f, indent=2)

    # 2. Patch SDF world: real_time_factor = 1.0
    base_sdf_path = os.path.join(ROOT_DIR, f"src/custom_corridor/worlds/corridor_{corridor_type}.sdf")
    with open(base_sdf_path, "r") as f:
        sdf = f.read()

    sdf = re.sub(r'<real_time_factor>.*?</real_time_factor>', '<real_time_factor>1.0</real_time_factor>', sdf)
    sdf = re.sub(
        r'<randomize_politeness>.*?</randomize_politeness>',
        f'<randomize_politeness>false</randomize_politeness>\n      <seed>{seed}</seed>\n      <default_scenario_file>{scen_file}</default_scenario_file>',
        sdf
    )
    if f"<seed>{seed}</seed>" not in sdf:
        sdf = re.sub(r'<seed>\d+</seed>', f'<seed>{seed}</seed>', sdf)

    proxy_z = "0.80" if corridor_type == "090" else "0.875"
    sdf = re.sub(r'<actor name=\"human_01_visual\">\s*<pose>.*?</pose>', '<actor name=\"human_01_visual\">\n      <pose>-7.0 0.0 0.0 0 0 3.14159</pose>', sdf)
    sdf = re.sub(r'<model name=\"human_01_proxy\">\s*<pose>.*?</pose>', f'<model name=\"human_01_proxy\">\n      <pose>-7.0 0.0 {proxy_z} 0 0 3.14159</pose>', sdf)

    trial_sdf = os.path.join(gen_dir, f"world_a3_3_step{config_mode}_{corridor_type}_{persona}_{seed}.sdf")
    with open(trial_sdf, "w") as f:
        f.write(sdf)

    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.dirname(os.path.dirname(plugin_share)) + "/lib"

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["ROS_DOMAIN_ID"] = str(domain_id)
    partition = f"a3_3_step{config_mode}_{corridor_type}_{persona}_{seed}_{domain_id}"
    env["GZ_PARTITION"] = partition

    all_procs = []
    nav2_log_path = f"/tmp/nav2_a3_3_step{config_mode}_{corridor_type}_{persona}_{seed}_{domain_id}.log"
    log_fp = open(nav2_log_path, "w")

    width_arg = "0.90" if corridor_type == "090" else "1.20"
    map_arg = f"corridor_{corridor_type}"
    corridor_half_w = 0.45 if corridor_type == "090" else 0.60

    result = {}
    try:
        # Launch Sim with Step 1 Robot Model (8m Lidar)
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={width_arg}",
            f"custom_world:={trial_sdf}",
            f"custom_robot_model:={ROBOT_MODEL_SDF}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        gz_p = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(gz_p)
        time.sleep(4.5)

        # Launch Nav2 with Step 2 Config
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={nav2_yaml}",
            f"map:={map_arg}",
            "robot_model:=mecanum"
        ]
        nav2_p = subprocess.Popen(nav2_cmd, env=env, stdout=log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
        all_procs.append(nav2_p)

        t0 = time.time()
        nav2_ready = False
        while time.time() - t0 < 60.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path) as fp:
                    content = fp.read()
                    if content.count("Managed nodes are active") >= 2:
                        nav2_ready = True
                        break
            time.sleep(0.4)

        if not nav2_ready:
            result = {"error": "nav2_activation_timeout", "goal_reached": False}
        else:
            time.sleep(1.0)
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)
            os.environ["GZ_PARTITION"] = partition
            rclpy.init()
            monitor = Step2InteractionMonitor(target_gx=-7.0, target_gy=0.0, max_duration_s=25.0, corridor_half_width=corridor_half_w)
            result = monitor.run_navigation()
            monitor.destroy_node()
            rclpy.shutdown()

    finally:
        for p in all_procs:
            try: os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception: pass
        time.sleep(1.0)
        for p in all_procs:
            try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception: pass
        log_fp.close()
        subprocess.run(["pkill", "-9", "-f", f"domain_id={domain_id}|gz sim|corridor_tb3|nav2_corridor"], stderr=subprocess.DEVNULL)
        time.sleep(1.0)

    result["seed"] = seed
    result["corridor"] = f"corridor_{corridor_type}"
    result["corridor_width_m"] = 0.90 if corridor_type == "090" else 1.20
    result["persona"] = persona
    result["v_human"] = v_human
    result["rtf_setting"] = 1.0
    result["step"] = f"Step {config_mode}"

    with open(out_json, "w") as fp:
        json.dump(result, fp, indent=2)


def cleanup_all():
    subprocess.run(["pkill", "-9", "-f", "gz sim|corridor_tb3|nav2_corridor|rviz2|robot_state_publisher|component_container_isolated|gz-sim"], stderr=subprocess.DEVNULL)
    time.sleep(1.5)


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
        cleanup_all()
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
            cleanup_all()
            cmd = [
                sys.executable, __file__,
                "--config", str(args.config),
                "--run_single",
                "--corridor", str(c_type),
                "--persona", str(persona),
                "--seed", str(seed),
                "--v_human", str(v_h),
                "--domain_id", "85",
                "--out_json", str(out_f)
            ]
            t_s = time.time()
            try:
                subprocess.run(cmd, check=True)
            except Exception as e:
                print(f"[ERROR] Trial failed for seed {seed}: {e}", flush=True)
            cleanup_all()
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
