"""
Rigorous Multi-Instance Parallel Scaling Benchmark for Gazebo Harmonic (Section 7).
Tests 1, 2, 3, and 4 concurrent Gazebo instances with isolated GZ_PARTITION and ROS_DOMAIN_ID.
Features:
- Hardware-accelerated Direct3D 12 rendering on NVIDIA RTX 4060 dGPU.
- Direct parse of native gz topic `/clock` (with -e flag, zero hardcoded fallback).
- ROS 2 topic listener per domain verifying actual reception of /scan and /camera/depth_image.
- Configuration: 640x480 @ 10 Hz depth camera, LDS-01 5 Hz LiDAR, headless.
"""

import sys
import os
import time
import signal
import subprocess
import json
import psutil
import re
import numpy as np

sys.path.insert(0, '/home/cuongbien/DynamuhSim/experiments/gazebo_benchmark')
from benchmark_world_generator import BenchmarkWorldGenerator


class ParallelInstance:
    def __init__(self, instance_id: int, root_dir: str, world_file: str, robot_file: str):
        self.instance_id = instance_id
        self.root_dir = root_dir
        self.world_file = world_file
        self.robot_file = robot_file
        self.gz_partition = f"gz_bench_p{instance_id}"
        self.ros_domain_id = str(20 + instance_id)
        self.procs = []

    def start(self):
        bridge_config = os.path.join(self.root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")
        plugin_share = os.path.join(self.root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
        plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
        corridor_models = os.path.join(self.root_dir, "install/custom_corridor/share/custom_corridor/models")
        tb3_models = os.path.join(self.root_dir, "install/turtlebot3_gazebo/share/turtlebot3_gazebo/models")

        gz_resource_path = f"{corridor_models}:{tb3_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
        gz_plugin_path = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

        env = os.environ.copy()
        env["GALLIUM_DRIVER"] = "d3d12"
        env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
        env["GZ_PARTITION"] = self.gz_partition
        env["ROS_DOMAIN_ID"] = self.ros_domain_id
        env["GZ_SIM_RESOURCE_PATH"] = gz_resource_path
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = gz_plugin_path

        # 1. Start Gazebo Sim headless
        gz_proc = subprocess.Popen(
            ["gz", "sim", "-r", "-s", self.world_file],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        self.procs.append(gz_proc)
        time.sleep(3.5)

        # 2. Spawn robot
        spawn_cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "burger",
            "-file", self.robot_file,
            "-x", "3.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"
        ]
        subprocess.run(spawn_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

        # 3. Start Parameter Bridge
        bridge_proc = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
        )
        self.procs.append(bridge_proc)

        # 4. Start Image Bridge
        image_proc = subprocess.Popen(
            ["ros2", "run", "ros_gz_image", "image_bridge", "/camera/image", "/camera/depth_image"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
        )
        self.procs.append(image_proc)
        time.sleep(1.5)

    def get_sim_time(self) -> float:
        """Queries true simulation time directly from native Gazebo /clock topic with -e."""
        env = os.environ.copy()
        env["GZ_PARTITION"] = self.gz_partition
        try:
            out = subprocess.check_output(
                ["gz", "topic", "-e", "-t", "/clock", "-n", "1"],
                env=env, text=True, timeout=3.0
            )
            # Find the sim block:
            # sim {
            #   sec: 12
            #   nsec: 340000000
            # }
            sim_block = re.search(r"sim\s*\{([^}]+)\}", out)
            if sim_block:
                block_text = sim_block.group(1)
                sec_m = re.search(r"sec:\s*(\d+)", block_text)
                nsec_m = re.search(r"nsec:\s*(\d+)", block_text)
                sec = int(sec_m.group(1)) if sec_m else 0
                nsec = int(nsec_m.group(1)) if nsec_m else 0
                return sec + nsec * 1e-9
            return 0.0
        except Exception as e:
            print(f"  [ERROR] get_sim_time failed for partition {self.gz_partition}: {e}")
            return 0.0

    def verify_topics(self) -> tuple[float, float]:
        """Verifies actual message receipt on ROS 2 topics (/scan and /camera/depth_image)."""
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = self.ros_domain_id
        # Quick python test node to count messages over 1.5s
        code = (
            "import rclpy, time\n"
            "from sensor_msgs.msg import LaserScan, Image\n"
            "rclpy.init()\n"
            "node = rclpy.create_node('probe_node')\n"
            "counts = {'scan': 0, 'depth': 0}\n"
            "node.create_subscription(LaserScan, '/scan', lambda m: counts.update({'scan': counts['scan']+1}), 10)\n"
            "node.create_subscription(Image, '/camera/depth_image', lambda m: counts.update({'depth': counts['depth']+1}), 10)\n"
            "t0 = time.time()\n"
            "while time.time() - t0 < 1.5:\n"
            "    rclpy.spin_once(node, timeout_sec=0.05)\n"
            "node.destroy_node()\n"
            "rclpy.shutdown()\n"
            "print(f\"{counts['scan']},{counts['depth']}\")\n"
        )
        try:
            res = subprocess.check_output(["python3", "-c", code], env=env, text=True, timeout=3.5).strip()
            scan_c, depth_c = res.split(',')
            return float(scan_c) / 1.5, float(depth_c) / 1.5
        except Exception:
            return 0.0, 0.0

    def stop(self):
        for p in self.procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.2)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        self.procs = []


def query_gpu():
    try:
        res = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader,nounits"],
            text=True, timeout=1.0
        ).strip().split(',')
        return int(res[0]), int(res[1])
    except Exception:
        return 0, 0


def run_parallel_benchmark(root_dir="/home/cuongbien/DynamuhSim", test_instances=[1, 2, 3, 4], duration_s=20.0, num_humans=5, depth_fps=10):
    generator = BenchmarkWorldGenerator(root_dir)
    world_file = generator.generate_world_with_humans(num_humans, "actor_and_cylinder")
    robot_file = generator.generate_robot_model(True, True, 640, 480, depth_fps)

    results = {}

    for n_inst in test_instances:
        print("\n" + "=" * 70)
        print(f"RUNNING PARALLEL BENCHMARK: {n_inst} CONCURRENT INSTANCE(S)")
        print(f"World: N={num_humans} humans | Depth: 640x480 @ {depth_fps} Hz | LiDAR: 5 Hz | D3D12 NVIDIA")
        print("=" * 70)

        instances = [ParallelInstance(i, root_dir, world_file, robot_file) for i in range(n_inst)]
        
        try:
            for inst in instances:
                print(f"Starting instance {inst.instance_id} (Domain {inst.ros_domain_id}, Partition {inst.gz_partition})...")
                inst.start()

            print(f"All {n_inst} instances launched. Warming up for 5s...")
            time.sleep(5.0)

            # 1. Verify that each instance is actually publishing sensor messages!
            print("Verifying sensor activity on all instances...")
            sensor_stats = []
            for inst in instances:
                scan_hz, depth_hz = inst.verify_topics()
                sensor_stats.append((scan_hz, depth_hz))
                print(f"  Instance {inst.instance_id} (Domain {inst.ros_domain_id}): /scan={scan_hz:.1f}Hz, /camera/depth_image={depth_hz:.1f}Hz")

            print(f"Measuring sim progress for {duration_s}s...")
            start_wall = time.time()
            vram_samples = []
            gpu_samples = []
            cpu_samples = []

            # Background thread to monitor hardware
            def sample_hw():
                while time.time() - start_wall < duration_s:
                    g_util, vram = query_gpu()
                    c_util = psutil.cpu_percent(interval=1.0)
                    gpu_samples.append(g_util)
                    vram_samples.append(vram)
                    cpu_samples.append(c_util)

            import threading
            hw_thread = threading.Thread(target=sample_hw)
            hw_thread.start()

            t0_sims = [inst.get_sim_time() for inst in instances]
            t0_wall = time.time()

            time.sleep(duration_s)

            t1_wall = time.time()
            t1_sims = [inst.get_sim_time() for inst in instances]
            hw_thread.join()

            d_wall = t1_wall - t0_wall
            rtfs = [(t1 - t0) / d_wall if d_wall > 0 else 0.0 for t0, t1 in zip(t0_sims, t1_sims)]
            total_rtf = sum(rtfs)

            result = {
                "num_instances": n_inst,
                "individual_rtfs": rtfs,
                "total_rtf": float(total_rtf),
                "mean_rtf": float(np.mean(rtfs)),
                "std_rtf": float(np.std(rtfs)),
                "sensor_rates": sensor_stats,
                "vram_mb_mean": float(np.mean(vram_samples)) if vram_samples else 0.0,
                "vram_mb_peak": int(np.max(vram_samples)) if vram_samples else 0,
                "gpu_util_mean": float(np.mean(gpu_samples)) if gpu_samples else 0.0,
                "cpu_util_mean": float(np.mean(cpu_samples)) if cpu_samples else 0.0,
            }
            results[f"instances_{n_inst}"] = result

            print(f"\n[MEASURED RESULTS: {n_inst} INSTANCES]")
            for idx, rtf in enumerate(rtfs):
                print(f"  Instance {idx} RTF: {rtf:.3f} | Scan: {sensor_stats[idx][0]:.1f} Hz | Depth: {sensor_stats[idx][1]:.1f} Hz")
            print(f"  Total RTF (Throughput): {total_rtf:.3f}")
            print(f"  VRAM Mean/Peak: {result['vram_mb_mean']:.1f} MB / {result['vram_mb_peak']} MB")
            print(f"  CPU Total: {result['cpu_util_mean']:.1f}% | GPU Util: {result['gpu_util_mean']:.1f}%")

        finally:
            print(f"Cleaning up {n_inst} instances...")
            for inst in instances:
                inst.stop()
            time.sleep(2.0)

    # Save results
    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/parallel_scaling_results.json")
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nParallel scaling results written to {out_file}")
    return results


if __name__ == '__main__':
    run_parallel_benchmark()
