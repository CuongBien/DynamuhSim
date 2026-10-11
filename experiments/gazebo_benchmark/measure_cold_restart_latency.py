#!/usr/bin/env python3
"""
Benchmark: Cold Restart Latency Under Real Parallel Load.
Definition:
- Start point (t0): The exact timestamp when calling the launch command for Gazebo simulation for the episode.
- End point (tend): The exact timestamp when BOTH conditions are met:
  1. Nav2 lifecycle reaches ACTIVE state ("Managed nodes are active").
  2. The first depth camera frame (/camera/depth/image_rect_raw) is received.
- Zero artificial delay/stagger included in the measurement.
- Real parallel load: 4 instances running and restarting continuously in parallel.
- Trials: At least 5 trials per configuration (1 instance and 4 instances, N=5, with Nav2 MPPI).
- Metrics reported: Mean, Std, Min, Max, and recalculated ep/h & hours for 10,000 episodes.
"""

import os
import sys
import time
import json
import signal
import subprocess
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))



class DepthListener(Node):
    def __init__(self, domain_id: int):
        super().__init__(f'depth_listener_{domain_id}')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)])
        self.first_depth_wall_time = None
        self.sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self._depth_cb,
            10
        )

    def _depth_cb(self, msg: Image):
        if self.first_depth_wall_time is None:
            self.first_depth_wall_time = time.time()


def cleanup_processes(pids=None):
    if pids:
        for p in pids:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.1)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
    time.sleep(1.0)
    for name in [
        "gz sim", "gz-sim", "ros_gz_bridge", "parameter_bridge", "image_bridge",
        "controller_server", "planner_server", "bt_navigator", "behavior_server",
        "waypoint_follower", "lifecycle_manager", "amcl", "map_server", "robot_state_publisher"
    ]:
        subprocess.run(["pkill", "-9", "-f", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def measure_single_instance_trial(domain_id: int, partition_name: str, trial_idx: int) -> dict:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = partition_name
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"

    world_file = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/generated_configs/world_n5_actor_and_cylinder.sdf")
    params_file = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_mecanum.yaml")
    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    nav2_log_path = f"/tmp/nav2_cold_restart_{domain_id}_{trial_idx}.log"
    if os.path.exists(nav2_log_path):
        os.remove(nav2_log_path)
    nav2_log_file = open(nav2_log_path, "w")

    procs = []

    # Initialize ROS node listener on this domain
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    rclpy.init()
    listener = DepthListener(domain_id)

    # EXACT start point: when launch command is invoked
    t0 = time.time()

    try:
        # Launch Gazebo Sim with Mecanum & Sensors (no artificial delay)
        gz_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            "width:=0.90",
            f"custom_world:={world_file}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        p_gz = subprocess.Popen(gz_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        procs.append(p_gz)

        # Launch Nav2 MPPI stack concurrently (zero artificial delay)
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={params_file}",
            "map:=corridor_090",
            "robot_model:=mecanum"
        ]
        p_nav = subprocess.Popen(nav2_cmd, env=env, stdout=nav2_log_file, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
        procs.append(p_nav)

        t_nav2_active = None
        t_first_depth = None

        timeout_s = 30.0
        while (time.time() - t0) < timeout_s and (t_nav2_active is None or t_first_depth is None):
            rclpy.spin_once(listener, timeout_sec=0.02)
            if t_first_depth is None and listener.first_depth_wall_time is not None:
                t_first_depth = listener.first_depth_wall_time

            if t_nav2_active is None and os.path.exists(nav2_log_path):
                with open(nav2_log_path, "r", errors="ignore") as fp:
                    if "Managed nodes are active" in fp.read():
                        t_nav2_active = time.time()

        if t_nav2_active is None or t_first_depth is None:
            tend = time.time()
            success = False
        else:
            tend = max(t_nav2_active, t_first_depth)
            success = True

        latency = tend - t0
        latency_nav2 = (t_nav2_active - t0) if t_nav2_active else timeout_s
        latency_depth = (t_first_depth - t0) if t_first_depth else timeout_s

        result = {
            "domain_id": domain_id,
            "trial_idx": trial_idx,
            "success": success,
            "cold_restart_latency_s": round(latency, 3),
            "nav2_active_latency_s": round(latency_nav2, 3),
            "first_depth_latency_s": round(latency_depth, 3),
            "limiting_condition": "nav2_active" if latency_nav2 >= latency_depth else "first_depth"
        }

    finally:
        cleanup_processes(procs)
        try:
            listener.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    return result


def worker_measure_cycle(domain_id: int, partition_name: str, cycle_idx: int, out_file: str):
    res = measure_single_instance_trial(domain_id, partition_name, cycle_idx)
    with open(out_file, "w") as fp:
        json.dump(res, fp)


def run_benchmark():
    parser = argparse.ArgumentParser(description="Cold Restart Latency Benchmark")
    parser.add_argument("--trials", type=int, default=5, help="Number of trials per configuration (default: 5)")
    parser.add_argument("--worker-domain", type=int, default=None)
    parser.add_argument("--worker-partition", type=str, default=None)
    parser.add_argument("--worker-cycle", type=int, default=None)
    parser.add_argument("--worker-out", type=str, default=None)
    args = parser.parse_args()

    # Inner worker dispatch for parallel 4-instance execution
    if args.worker_domain is not None:
        worker_measure_cycle(args.worker_domain, args.worker_partition, args.worker_cycle, args.worker_out)
        sys.exit(0)

    print("=" * 80)
    print("BENCHMARK COLD RESTART LATENCY THEO ĐỊNH NGHĨA MỚI")
    print("Mốc: t0 (gọi lệnh khởi động) -> max(Nav2 active, First depth frame)")
    print("Không giãn cách nhân tạo | Đo dưới tải thật")
    print("=" * 80)

    results = {}

    # -------------------------------------------------------------
    # 1. Đo Cấu hình 1 Instance (N=5, có Nav2)
    # -------------------------------------------------------------
    print(f"\n>>> 1. ĐO CẤU HÌNH 1 INSTANCE (N=5, có Nav2 MPPI) - {args.trials} LẦN CHẠY...")
    inst1_trials = []
    cleanup_processes()

    for trial_idx in range(args.trials):
        print(f"  [1 Instance] Lần chạy {trial_idx + 1}/{args.trials}...")
        res = measure_single_instance_trial(domain_id=85, partition_name="gz_part_cold_1inst", trial_idx=trial_idx)
        inst1_trials.append(res)
        print(f"    -> Cold restart: {res['cold_restart_latency_s']:.2f} s "
              f"(Nav2 active: {res['nav2_active_latency_s']:.2f} s, Depth: {res['first_depth_latency_s']:.2f} s, Giới hạn bởi: {res['limiting_condition']})")

    lats_1 = [r["cold_restart_latency_s"] for r in inst1_trials if r["success"]]
    mean_lat_1 = float(np.mean(lats_1))
    std_lat_1 = float(np.std(lats_1))
    min_lat_1 = float(np.min(lats_1))
    max_lat_1 = float(np.max(lats_1))

    # Recalculate throughput with RTF from A4 (Mean RTF = 2.82)
    rtf_1 = 2.816
    time_ep_1 = (20.0 / rtf_1) + mean_lat_1
    ep_h_1 = (3600.0 / time_ep_1) * 1
    h_10k_1 = 10000.0 / ep_h_1

    results["1_instance"] = {
        "num_instances": 1,
        "num_humans": 5,
        "with_nav2": True,
        "num_trials": len(inst1_trials),
        "mean_latency_s": round(mean_lat_1, 3),
        "std_latency_s": round(std_lat_1, 3),
        "min_latency_s": round(min_lat_1, 3),
        "max_latency_s": round(max_lat_1, 3),
        "rtf_measured_a4": rtf_1,
        "time_per_ep_20s_s": round(time_ep_1, 2),
        "episodes_per_hour": round(ep_h_1, 1),
        "hours_for_10k_episodes": round(h_10k_1, 2),
        "trials": inst1_trials
    }

    print(f"\n[Kết quả 1 Instance]: Mean = {mean_lat_1:.2f} s ± {std_lat_1:.2f} s | Min = {min_lat_1:.2f} s | Max = {max_lat_1:.2f} s")
    print(f"  Thông lượng tính lại: {ep_h_1:.1f} ep/h | 10.000 episodes: {h_10k_1:.2f} giờ")

    # -------------------------------------------------------------
    # 2. Đo Cấu hình 4 Instances Dưới Tải Thật (4 instances chạy song song liên tục restart)
    # -------------------------------------------------------------
    print(f"\n>>> 2. ĐO CẤU HÌNH 4 INSTANCES DƯỚI TẢI THẬT (Chạy song song, liên tục restart qua {args.trials} chu kỳ)...")
    inst4_trials = []
    cleanup_processes()

    for cycle_idx in range(args.trials):
        print(f"  [4 Instances - Tải Song Song] Chu kỳ {cycle_idx + 1}/{args.trials} (4 instances đồng thời khởi động)...")
        cycle_procs = []
        out_files = []

        for i in range(4):
            dom_id = 90 + i
            part_name = f"gz_part_cold_4inst_{i}"
            out_f = f"/tmp/cold_restart_4inst_cycle_{cycle_idx}_inst_{i}.json"
            if os.path.exists(out_f):
                os.remove(out_f)
            out_files.append(out_f)

            cmd = [
                sys.executable, os.path.abspath(__file__),
                "--worker-domain", str(dom_id),
                "--worker-partition", part_name,
                "--worker-cycle", str(cycle_idx),
                "--worker-out", out_f
            ]
            p = subprocess.Popen(cmd, preexec_fn=os.setsid)
            cycle_procs.append(p)

        # Wait for all 4 concurrent instances in this cycle to complete
        for p in cycle_procs:
            p.wait(timeout=45.0)

        # Collect results of this cycle
        for i, out_f in enumerate(out_files):
            if os.path.exists(out_f):
                with open(out_f) as fp:
                    r = json.load(fp)
                inst4_trials.append(r)
                print(f"    Instance {i}: Cold restart = {r['cold_restart_latency_s']:.2f} s "
                      f"(Nav2: {r['nav2_active_latency_s']:.2f} s, Depth: {r['first_depth_latency_s']:.2f} s)")

        cleanup_processes()

    lats_4 = [r["cold_restart_latency_s"] for r in inst4_trials if r["success"]]
    mean_lat_4 = float(np.mean(lats_4))
    std_lat_4 = float(np.std(lats_4))
    min_lat_4 = float(np.min(lats_4))
    max_lat_4 = float(np.max(lats_4))

    # Recalculate throughput with RTF from A4 (Instance RTF = 1.11, Total RTF = 4.46)
    rtf_inst_4 = 1.114
    time_ep_4 = (20.0 / rtf_inst_4) + mean_lat_4
    ep_h_4 = (3600.0 / time_ep_4) * 4
    h_10k_4 = 10000.0 / ep_h_4

    results["4_instances_parallel_load"] = {
        "num_instances": 4,
        "num_humans": 5,
        "with_nav2": True,
        "num_trials_total": len(inst4_trials),
        "mean_latency_s": round(mean_lat_4, 3),
        "std_latency_s": round(std_lat_4, 3),
        "min_latency_s": round(min_lat_4, 3),
        "max_latency_s": round(max_lat_4, 3),
        "rtf_measured_a4_per_inst": rtf_inst_4,
        "time_per_ep_20s_s": round(time_ep_4, 2),
        "episodes_per_hour": round(ep_h_4, 1),
        "hours_for_10k_episodes": round(h_10k_4, 2),
        "trials": inst4_trials
    }

    print(f"\n[Kết quả 4 Instances dưới tải song song]: Mean = {mean_lat_4:.2f} s ± {std_lat_4:.2f} s | Min = {min_lat_4:.2f} s | Max = {max_lat_4:.2f} s")
    print(f"  Thông lượng tính lại: {ep_h_4:.1f} ep/h | 10.000 episodes: {h_10k_4:.2f} giờ")

    # Export raw JSON results
    out_json = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/cold_restart_benchmark_results.json")
    with open(out_json, "w") as fp:
        json.dump(results, fp, indent=2)
    print(f"\nĐã lưu kết quả JSON thô vào: {out_json}")


if __name__ == "__main__":
    run_benchmark()
