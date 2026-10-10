"""
True Uncapped Multi-Instance Parallel Scaling Benchmark for Gazebo Harmonic.
Addresses all user specifications:
1. World SDF configured with <real_time_factor>0.0</real_time_factor>.
2. High-frequency in-process ROS 2 /clock subscriber (ZERO CLI subprocess calls).
3. Warm-up exclusion: 5s warm-up completely discarded from p5, median, and throughput stats.
4. Full telemetry: RTF per instance, total RTF, p5, median, p95, CPU%, GPU%, VRAM.
5. Exact production modeling: 10,000 episodes calculation with 6.45s cold restart.
"""

import sys
import os
import time
import signal
import subprocess
import json
import threading
import multiprocessing
import psutil
import numpy as np

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, Image

sys.path.insert(0, '/home/cuongbien/DynamuhSim/experiments/gazebo_benchmark')
from benchmark_world_generator import BenchmarkWorldGenerator


class DomainClockListener(Node):
    def __init__(self, domain_id: int):
        super().__init__(f'clock_listener_dom{domain_id}')
        self.domain_id = domain_id
        self.latest_sim_time = None
        self.latest_wall_time = None
        self.has_scan = False
        self.has_depth = False
        self.samples = []  # list of (wall_time, sim_time)

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth_image', self._depth_cb, 10)

    def _clock_cb(self, msg: Clock):
        w_t = time.time()
        s_t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.latest_wall_time = w_t
        self.latest_sim_time = s_t
        self.samples.append((w_t, s_t))

    def _scan_cb(self, msg: LaserScan):
        self.has_scan = True

    def _depth_cb(self, msg: Image):
        self.has_depth = True


def domain_worker_proc(domain_id: int, duration_s: float, out_file: str):
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    rclpy.init()
    node = DomainClockListener(domain_id)
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


def benchmark_instance_count(num_instances: int, world_file: str, robot_file: str, root_dir: str):
    print("\n" + "=" * 75)
    print(f"TESTING {num_instances} CONCURRENT INSTANCE(S) - TRUE UNCAPPED (RTF=0.0 in SDF)")
    print("=" * 75, flush=True)

    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")
    corridor_models = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models")
    tb3_models = os.path.join(root_dir, "install/turtlebot3_gazebo/share/turtlebot3_gazebo/models")
    plugin_share = os.path.join(root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')

    base_env = os.environ.copy()
    base_env["GALLIUM_DRIVER"] = "d3d12"
    base_env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    base_env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{tb3_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    base_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    all_procs = []
    worker_threads = []
    stop_event = threading.Event()
    worker_results = [{} for _ in range(num_instances)]

    try:
        # 1. Launch all Gazebo instances in paused mode
        print(f"1. Launching {num_instances} Gazebo instances (Paused, uncapped)...", flush=True)
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"bench_uncapped_part_{i}"
            env_i["ROS_DOMAIN_ID"] = str(30 + i)
            gz_p = subprocess.Popen(
                ["gz", "sim", "-s", "-z", "5000", world_file],
                env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
            )
            all_procs.append(gz_p)

        time.sleep(3.5)

        # 2. Spawn robot & bridges
        print(f"2. Spawning robot, parameter bridge, image bridge on {num_instances} instances...", flush=True)
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"bench_uncapped_part_{i}"
            env_i["ROS_DOMAIN_ID"] = str(30 + i)

            spawn_cmd = [
                "ros2", "run", "ros_gz_sim", "create",
                "-name", "burger", "-file", robot_file,
                "-x", "3.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"
            ]
            subprocess.run(spawn_cmd, env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

            bp = subprocess.Popen(
                ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"],
                env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
            )
            all_procs.append(bp)

            ip = subprocess.Popen(
                ["ros2", "run", "ros_gz_image", "image_bridge", "/camera/image", "/camera/depth_image"],
                env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
            )
            all_procs.append(ip)

        time.sleep(2.0)

        # 3. Start in-process listener processes
        print(f"3. Starting in-process /clock listeners across {num_instances} domains...", flush=True)
        worker_procs = []
        worker_files = []
        for i in range(num_instances):
            dom_id = 30 + i
            out_f = f"/tmp/uncapped_bench_worker_{i}.json"
            if os.path.exists(out_f):
                os.remove(out_f)
            worker_files.append(out_f)
            wp = multiprocessing.Process(target=domain_worker_proc, args=(dom_id, 32.0, out_f))
            wp.start()
            worker_procs.append(wp)

        time.sleep(1.0)

        # 4. Unpause all instances simultaneously
        print(f"4. Unpausing all {num_instances} instances...", flush=True)
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"bench_uncapped_part_{i}"
            subprocess.run(
                ["gz", "service", "-s", "/world/corridor_0.90/control", "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "pause: false"],
                env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

        # 5. Warm-up Phase (5.0 seconds - completely discarded)
        print("5. Warm-up Phase (5.0s, stabilizing pipelines - DISCARDED FROM STATS)...", flush=True)
        time.sleep(5.0)

        # 6. Measurement Phase (20.0 seconds steady state)
        print("6. Steady-State Measurement Window (20.0s wall-clock)...", flush=True)
        t_measure_start = time.time()
        cpu_readings = []
        gpu_readings = []
        vram_readings = []

        while time.time() - t_measure_start < 20.0:
            cpu_readings.append(psutil.cpu_percent(interval=None))
            g_u, m_u = get_gpu_metrics()
            gpu_readings.append(g_u)
            vram_readings.append(m_u)
            time.sleep(0.5)

        t_measure_end = time.time()
        print("7. Measurement window complete. Waiting for worker files...", flush=True)

        for wp in worker_procs:
            wp.join(timeout=5.0)

    finally:
        for p in all_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1.5)

    # 7. Analyze data strictly inside [t_measure_start, t_measure_end]
    worker_results = []
    for f in worker_files:
        if os.path.exists(f):
            with open(f) as fp:
                worker_results.append(json.load(fp))
        else:
            worker_results.append({})

    instance_stats = []
    total_rtf = 0.0

    print("\n--- INSTANCE PERFORMANCE SUMMARY (Warm-up Discarded) ---", flush=True)
    for i in range(num_instances):
        raw_samples = worker_results[i].get("samples", [])
        # Filter to steady-state window
        ss_samples = [s for s in raw_samples if s[0] >= t_measure_start and s[0] <= t_measure_end]

        if len(ss_samples) < 2:
            print(f"  [Instance {i+1}] Insufficient steady-state samples ({len(ss_samples)})")
            continue

        w_start, s_start = ss_samples[0]
        w_end, s_end = ss_samples[-1]
        overall_inst_rtf = (s_end - s_start) / max(w_end - w_start, 0.001)

        # Compute rolling window RTF (0.5s intervals) for p5, median, p95
        inst_rtfs = []
        step = max(len(ss_samples) // 40, 1)
        for k in range(0, len(ss_samples) - step, step):
            dt_w = ss_samples[k + step][0] - ss_samples[k][0]
            dt_s = ss_samples[k + step][1] - ss_samples[k][1]
            if dt_w > 0.05:
                inst_rtfs.append(dt_s / dt_w)

        p5 = float(np.percentile(inst_rtfs, 5)) if inst_rtfs else overall_inst_rtf
        median = float(np.median(inst_rtfs)) if inst_rtfs else overall_inst_rtf
        p95 = float(np.percentile(inst_rtfs, 95)) if inst_rtfs else overall_inst_rtf

        has_scan = worker_results[i].get("has_scan", False)
        has_depth = worker_results[i].get("has_depth", False)

        inst_data = {
            "instance_id": i + 1,
            "rtf_mean": round(overall_inst_rtf, 3),
            "rtf_median": round(median, 3),
            "rtf_p5": round(p5, 3),
            "rtf_p95": round(p95, 3),
            "has_scan": has_scan,
            "has_depth": has_depth
        }
        instance_stats.append(inst_data)
        total_rtf += overall_inst_rtf

        print(f"  Instance {i+1}: Mean RTF = {overall_inst_rtf:.3f} | Median = {median:.3f} | p5 = {p5:.3f} | p95 = {p95:.3f} | LiDAR={has_scan} | Depth={has_depth}", flush=True)

    mean_cpu = float(np.mean(cpu_readings)) if cpu_readings else 0.0
    mean_gpu = float(np.mean(gpu_readings)) if gpu_readings else 0.0
    mean_vram = float(np.mean(vram_readings)) if vram_readings else 0.0

    # Production calculation: 10,000 episodes (20s sim-time) with 6.45s cold restart overhead
    avg_inst_rtf = total_rtf / max(num_instances, 1)
    sim_wall_time_per_ep = 20.0 / max(avg_inst_rtf, 0.01)
    total_ep_time_per_instance = sim_wall_time_per_ep + 6.45  # including cold restart
    eps_per_hour = num_instances * (3600.0 / total_ep_time_per_instance)
    hours_for_10k = 10000.0 / eps_per_hour

    print(f"\n[SUMMARY FOR {num_instances} INSTANCES]")
    print(f"  Total Aggregate Throughput : {total_rtf:.3f} RTF")
    print(f"  Avg RTF per Instance       : {avg_inst_rtf:.3f} RTF")
    print(f"  Average CPU Utilization    : {mean_cpu:.1f} %")
    print(f"  Average GPU Utilization    : {mean_gpu:.1f} %")
    print(f"  Average VRAM Usage         : {mean_vram:.0f} MB")
    print(f"  Sim time for 20s episode   : {sim_wall_time_per_ep:.2f} s")
    print(f"  Total time (with 6.45s reset): {total_ep_time_per_instance:.2f} s")
    print(f"  Production Rate            : {eps_per_hour:.1f} episodes/hour")
    print(f"  Time for 10,000 Episodes   : {hours_for_10k:.2f} hours")

    return {
        "num_instances": num_instances,
        "total_rtf": round(total_rtf, 3),
        "avg_instance_rtf": round(avg_inst_rtf, 3),
        "mean_cpu_pct": round(mean_cpu, 1),
        "mean_gpu_pct": round(mean_gpu, 1),
        "mean_vram_mb": round(mean_vram, 0),
        "sim_wall_time_per_ep": round(sim_wall_time_per_ep, 2),
        "total_ep_time_with_cold_restart": round(total_ep_time_per_instance, 2),
        "episodes_per_hour": round(eps_per_hour, 1),
        "hours_for_10k_episodes": round(hours_for_10k, 2),
        "instance_details": instance_stats
    }


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    gen = BenchmarkWorldGenerator(root_dir)
    # Generate world with real_time_factor = 0.0 (Uncapped!)
    world_file = gen.generate_world_with_humans(num_humans=5, representation="actor_and_cylinder", seed=42, real_time_factor=0.0)
    # Robot with 640x480 @ 10fps depth, 5Hz LiDAR
    robot_file = gen.generate_robot_model(True, True, 640, 480, 10, initial_x=3.0, initial_y=0.0)

    all_results = {}
    for n in [1, 2, 3, 4]:
        res = benchmark_instance_count(n, world_file, robot_file, root_dir)
        all_results[f"{n}_instances"] = res

    out_json = os.path.join(root_dir, "experiments/gazebo_benchmark/true_uncapped_parallel_results.json")
    with open(out_json, "w") as f:
        json.dump(all_results, f, indent=2)

    print("\n" + "=" * 75)
    print("ALL TRUE UNCAPPED PARALLEL BENCHMARKS FINISHED!")
    print(f"Results written to: {out_json}")
    print("=" * 75, flush=True)


if __name__ == '__main__':
    main()
