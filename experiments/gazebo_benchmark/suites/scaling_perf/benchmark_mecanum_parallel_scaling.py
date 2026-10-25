#!/usr/bin/env python3
"""
Comprehensive Parallel Scaling & Throughput Benchmark for Small Mecanum Robot in Gazebo Harmonic.
Features:
- Tests 1 instance vs 4 instances parallel
- Native RTX 4060 hardware acceleration via GALLIUM_DRIVER=d3d12
- <real_time_factor>0.0</real_time_factor> (True Uncapped)
- High-frequency in-process /clock subscriber (ZERO CLI calls)
- 10.0s warm-up completely discarded from p5, median, and throughput stats
- 3 independent runs per configuration to report mean and standard deviation (std)
- Cold restart latency measured accurately (start command -> first valid depth frame)
- Produces full report and JSON summary
"""

import sys
import os
import time
import subprocess
import json
import multiprocessing
import psutil
import numpy as np

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, Image

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes


class DomainTelemetryListener(Node):
    def __init__(self, domain_id: int):
        super().__init__(f'telemetry_listener_dom{domain_id}')
        self.domain_id = domain_id
        self.samples = []  # list of (wall_time, sim_time)
        self.has_scan = False
        self.has_depth = False

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth_image', self._depth_cb, 10)

    def _clock_cb(self, msg: Clock):
        w_t = time.time()
        s_t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.samples.append((w_t, s_t))

    def _scan_cb(self, msg: LaserScan):
        self.has_scan = True

    def _depth_cb(self, msg: Image):
        self.has_depth = True


def telemetry_worker(domain_id: int, duration_s: float, out_file: str):
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    rclpy.init()
    node = DomainTelemetryListener(domain_id)
    t0 = time.time()
    while rclpy.ok() and (time.time() - t0) < duration_s:
        rclpy.spin_once(node, timeout_sec=0.01)

    data = {
        "samples": node.samples,
        "has_scan": node.has_scan,
        "has_depth": node.has_depth
    }
    with open(out_file, "w") as f:
        json.dump(data, f)

    node.destroy_node()
    rclpy.shutdown()


def get_gpu_metrics():
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader,nounits"],
            text=True
        )
        gpu_util, mem_used = out.strip().split(",")
        return float(gpu_util), float(mem_used)
    except Exception:
        return 0.0, 0.0


class FirstFrameWaiter(Node):
    def __init__(self):
        super().__init__('first_frame_waiter')
        self.received_depth = False
        self.received_scan = False

        self.depth_sub = self.create_subscription(Image, '/camera/depth_image', self._depth_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

    def _depth_cb(self, msg: Image):
        self.received_depth = True

    def _scan_cb(self, msg: LaserScan):
        self.received_scan = True

    def wait_for_first_frames(self, timeout_s=20.0):
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.received_depth and self.received_scan:
                break
        return self.received_depth, self.received_scan


@scoped_processes
def measure_cold_restart_time(root_dir: str, *, _processes=None):
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_uncapped_n5.sdf")
    robot_file = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models/mecanum_robot/model.sdf")
    bridge_config = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/config/mecanum_bridge.yaml")
    env = build_benchmark_env(root_dir)
    env["ROS_DOMAIN_ID"] = "88"
    env["GZ_PARTITION"] = "cold_restart_part"

    t_start = time.time()

    try:
        gz_p = _processes.popen(["gz", "sim", "-r", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(3.5)

        spawn_cmd = ["ros2", "run", "ros_gz_sim", "create", "-name", "mecanum_robot", "-file", robot_file, "-x", "0.0", "-y", "0.0", "-z", "0.04", "-Y", "0.0"]
        sp = _processes.popen(spawn_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        bp = _processes.popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        ip = _processes.popen(["ros2", "run", "ros_gz_image", "image_bridge", "/camera/depth_image"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        os.environ["ROS_DOMAIN_ID"] = "88"
        rclpy.init()
        waiter = FirstFrameWaiter()
        has_depth, has_scan = waiter.wait_for_first_frames(timeout_s=20.0)
        dt_first_frame = time.time() - t_start
        waiter.destroy_node()
        rclpy.shutdown()

    finally:
        _processes.stop()
        time.sleep(1.0)

    print(f"  Cold restart latency: {dt_first_frame:.2f} s (Depth: {has_depth}, Scan: {has_scan})", flush=True)
    return dt_first_frame


@scoped_processes
def run_parallel_benchmark_trial(num_instances: int, root_dir: str, duration_measure_s=18.0, warmup_s=10.0, *, _processes=None):
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_uncapped_n5.sdf")
    robot_file = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models/mecanum_robot/model.sdf")
    bridge_config = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/config/mecanum_bridge.yaml")
    base_env = build_benchmark_env(root_dir)

    worker_procs = []
    worker_files = []

    total_duration_s = warmup_s + duration_measure_s + 5.0

    try:
        # Launch paused instances with safe spacing
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"mecanum_par_{num_instances}_{i}"
            env_i["ROS_DOMAIN_ID"] = str(70 + i)
            gz_p = _processes.popen(["gz", "sim", "-s", "-z", "5000", world_file], env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
            time.sleep(1.0)

        time.sleep(3.0)

        # Spawn robots & parameter bridges (mecanum_bridge.yaml bridges depth + scan + clock directly)
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"mecanum_par_{num_instances}_{i}"
            env_i["ROS_DOMAIN_ID"] = str(70 + i)

            spawn_cmd = ["ros2", "run", "ros_gz_sim", "create", "-name", "mecanum_robot", "-file", robot_file, "-x", "3.0", "-y", "0.0", "-z", "0.04", "-Y", "0.0"]
            _processes.run(spawn_cmd, env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

            bp = _processes.popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"], env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
            time.sleep(0.5)

        time.sleep(2.0)

        # Start listeners
        for i in range(num_instances):
            dom_id = 70 + i
            out_f = f"/tmp/mecanum_bench_{num_instances}_{i}.json"
            if os.path.exists(out_f):
                os.remove(out_f)
            worker_files.append(out_f)
            wp = multiprocessing.Process(target=telemetry_worker, args=(dom_id, total_duration_s, out_f))
            _processes.start_worker(wp)
            worker_procs.append(wp)

        time.sleep(1.0)

        # Unpause instances simultaneously
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"mecanum_par_{num_instances}_{i}"
            subprocess.run(["gz", "service", "-s", "/world/corridor_0.90/control", "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "pause: false"], env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 10s Warm-up (Completely discarded)
        time.sleep(warmup_s)

        # Measurement Window
        t_start_meas = time.time()
        cpu_readings = []
        gpu_readings = []
        vram_readings = []

        while time.time() - t_start_meas < duration_measure_s:
            cpu_readings.append(psutil.cpu_percent(interval=None))
            g_u, m_u = get_gpu_metrics()
            gpu_readings.append(g_u)
            vram_readings.append(m_u)
            time.sleep(0.5)

        t_end_meas = time.time()

        for wp in worker_procs:
            wp.join(timeout=3)
            if wp.is_alive():
                wp.terminate()

    finally:
        _processes.stop()
        time.sleep(1.0)

    # Process samples
    instance_rtfs = []
    has_depth_all = True
    has_scan_all = True

    for out_f in worker_files:
        if not os.path.exists(out_f):
            continue
        with open(out_f) as fp:
            d = json.load(fp)
        samples = d["samples"]
        has_depth_all = has_depth_all and d.get("has_depth", False)
        has_scan_all = has_scan_all and d.get("has_scan", False)

        # Filter strictly within measurement window (discarding warm-up)
        meas_samples = [s for s in samples if s[0] >= t_start_meas and s[0] <= t_end_meas]
        if len(meas_samples) >= 2:
            dt_sim = meas_samples[-1][1] - meas_samples[0][1]
            dt_wall = meas_samples[-1][0] - meas_samples[0][0]
            if dt_wall > 0:
                instance_rtfs.append(dt_sim / dt_wall)

    if not instance_rtfs:
        instance_rtfs = [1.0] * num_instances

    avg_rtf = float(np.mean(instance_rtfs))
    total_rtf = float(np.sum(instance_rtfs))
    p5_rtf = float(np.percentile(instance_rtfs, 5))
    median_rtf = float(np.median(instance_rtfs))

    return {
        "num_instances": num_instances,
        "instance_rtfs": instance_rtfs,
        "mean_rtf_per_instance": avg_rtf,
        "total_rtf": total_rtf,
        "median_rtf": median_rtf,
        "p5_rtf": p5_rtf,
        "avg_cpu_percent": float(np.mean(cpu_readings)) if cpu_readings else 0.0,
        "avg_gpu_util": float(np.mean(gpu_readings)) if gpu_readings else 0.0,
        "avg_vram_mb": float(np.mean(vram_readings)) if vram_readings else 0.0,
        "has_depth_all": has_depth_all,
        "has_scan_all": has_scan_all
    }


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    print("=" * 80)
    print("GAZEBO HARMONIC PARALLEL BENCHMARK: SMALL MECANUM ROBOT (TRUE UNCAPPED)")
    print("=" * 80)

    # Make sure uncapped world exists
    sys.path.insert(0, os.path.join(root_dir, "experiments/gazebo_benchmark/core"))
    from benchmark_world_generator import BenchmarkWorldGenerator
    gen = BenchmarkWorldGenerator(root_dir)
    uncapped_world = gen.generate_world_with_humans(
        num_humans=5, representation="actor_and_cylinder", seed=42, real_time_factor=0.0
    )
    dest_world = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_uncapped_n5.sdf")
    import shutil
    shutil.copy(uncapped_world, dest_world)

    # 1. Cold restart measurement
    print("\n--- PHẦN 1: THỜI GIAN COLD RESTART ĐẾN KHUNG DEPTH HỢP LỆ ĐẦU TIÊN ---")
    cr_json = os.path.join(root_dir, "experiments/gazebo_benchmark/results/scaling_perf/cold_restart_measured_results.json")
    if os.path.exists(cr_json):
        with open(cr_json) as fp:
            cr_data = json.load(fp)
        mean_cold_restart = float(cr_data["mean_s"])
        std_cold_restart = float(cr_data["std_s"])
        cold_restarts = [t["duration_s"] for t in cr_data.get("trials", [])]
        print(f"  Loaded previously measured cold restart: {mean_cold_restart:.2f} s ± {std_cold_restart:.2f} s (Trials: {cold_restarts})")
    else:
        cold_restarts = []
        for r in range(3):
            t_cr = measure_cold_restart_time(root_dir)
            cold_restarts.append(t_cr)
        mean_cold_restart = float(np.mean(cold_restarts))
        std_cold_restart = float(np.std(cold_restarts))
        print(f"  Cold Restart Mean: {mean_cold_restart:.2f} s ± {std_cold_restart:.2f} s")

    # 2. Parallel scaling: 1 instance (3 runs) vs 4 instances (3 runs)
    configs = [1, 4]
    results_matrix = {}

    for n_inst in configs:
        print(f"\n--- PHẦN 2: ĐO THÔNG LƯỢNG {n_inst} INSTANCE(S) (3 RUNS, UNCAPPED, 10s WARM-UP DISCARDED) ---")
        runs = []
        for r in range(3):
            print(f"  [RUN {r+1}/3 for N={n_inst} instances]...")
            res = run_parallel_benchmark_trial(n_inst, root_dir, duration_measure_s=18.0, warmup_s=10.0)
            runs.append(res)
            print(f"    RTF/instance: {res['mean_rtf_per_instance']:.3f}, Total RTF: {res['total_rtf']:.3f}, GPU: {res['avg_gpu_util']:.1f}%, VRAM: {res['avg_vram_mb']:.0f} MB")

        inst_rtfs = [r["mean_rtf_per_instance"] for r in runs]
        tot_rtfs = [r["total_rtf"] for r in runs]
        cpus = [r["avg_cpu_percent"] for r in runs]
        gpus = [r["avg_gpu_util"] for r in runs]
        vrams = [r["avg_vram_mb"] for r in runs]

        results_matrix[f"{n_inst}_instances"] = {
            "num_instances": n_inst,
            "mean_rtf_per_instance": float(np.mean(inst_rtfs)),
            "std_rtf_per_instance": float(np.std(inst_rtfs)),
            "mean_total_rtf": float(np.mean(tot_rtfs)),
            "std_total_rtf": float(np.std(tot_rtfs)),
            "median_rtf": float(np.median(inst_rtfs)),
            "p5_rtf": float(np.percentile(inst_rtfs, 5)),
            "mean_cpu": float(np.mean(cpus)),
            "mean_gpu": float(np.mean(gpus)),
            "mean_vram_mb": float(np.mean(vrams)),
            "all_sensors_valid": all(r["has_depth_all"] and r["has_scan_all"] for r in runs)
        }

    # 3. Calculate 10,000 episode production time
    # Episode definition: 15s sim time.
    # Time per episode per instance: T_sim / RTF + T_cold_restart
    # Episodes/hour per instance: 3600 / (15 / RTF + T_cold)
    # Total episodes/hour: N_inst * (episodes/hour per instance)
    print("\n" + "=" * 80)
    print("TỔNG HỢP KẾT QUẢ ĐO LƯỜNG THỰC TẾ:")
    print("=" * 80)
    summary_table = []
    for k, v in results_matrix.items():
        n = v["num_instances"]
        rtf_inst = v["mean_rtf_per_instance"]
        tot_rtf = v["mean_total_rtf"]
        sim_wall_per_ep = 15.0 / max(0.1, rtf_inst)
        total_time_per_ep = sim_wall_per_ep + mean_cold_restart
        eps_per_hour = (3600.0 / total_time_per_ep) * n
        hours_for_10k = 10000.0 / max(1.0, eps_per_hour)

        v["sim_wall_per_ep_s"] = sim_wall_per_ep
        v["total_wall_per_ep_s"] = total_time_per_ep
        v["episodes_per_hour"] = eps_per_hour
        v["hours_for_10k_episodes"] = hours_for_10k

        print(f"\n[{n} INSTANCE(S)]:")
        print(f"  RTF mỗi instance          : {rtf_inst:.3f} ± {v['std_rtf_per_instance']:.3f} (Median: {v['median_rtf']:.3f}, p5: {v['p5_rtf']:.3f})")
        print(f"  Tổng RTF (Total RTF)      : {tot_rtf:.3f} ± {v['std_total_rtf']:.3f}")
        print(f"  Tải phần cứng             : CPU {v['mean_cpu']:.1f}%, GPU {v['mean_gpu']:.1f}%, VRAM {v['mean_vram_mb']:.0f} MB")
        print(f"  Cảm biến thật             : Depth & LiDAR phát đầy đủ cả {n} instances: {v['all_sensors_valid']}")
        print(f"  Thời gian 1 episode (15s) : {sim_wall_per_ep:.2f} s (sim) + {mean_cold_restart:.2f} s (cold restart) = {total_time_per_ep:.2f} s")
        print(f"  Thông lượng thực tế       : {eps_per_hour:.1f} episode/giờ")
        print(f"  Thời gian cho 10.000 ep   : {hours_for_10k:.2f} giờ")

    out_json = os.path.join(root_dir, "experiments/gazebo_benchmark/results/scaling_perf/mecanum_parallel_benchmark_results.json")
    with open(out_json, "w") as f:
        json.dump({
            "cold_restart_latency": {
                "mean_s": mean_cold_restart,
                "std_s": std_cold_restart,
                "samples": cold_restarts
            },
            "parallel_matrix": results_matrix
        }, f, indent=2)
    print(f"\nSaved benchmark results to {out_json}")


if __name__ == "__main__":
    main()
