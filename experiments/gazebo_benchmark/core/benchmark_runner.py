"""
Automated Test Runner for Gazebo Harmonic Performance Benchmark.
Executes the standardized Section 5 measurement protocol:
1. Starts Gazebo with specified world & robot config.
2. 10s warm-up.
3. 120s profiling with fixed robot cmd_vel.
4. Samples RTF, topic Hz, CPU%, GPU%, VRAM, RAM every 1s.
5. Shuts down cleanly between runs.
6. Repeated N times (default 3) per condition, reporting median, p5, mean, std.
"""

import sys
import os
import time
import subprocess
import argparse
import json
import numpy as np

# Add tools to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from experiments.gazebo_benchmark.core.benchmark_world_generator import BenchmarkWorldGenerator
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes
from experiments.gazebo_benchmark.core.sim_env import (
    build_benchmark_env, spawn_benchmark_robot, start_parameter_bridge,
)


class GazeboBenchmarkRunner:
    def __init__(self, root_dir="/home/cuongbien/DynamuhSim"):
        self.root_dir = root_dir
        self.generator = BenchmarkWorldGenerator(root_dir)
        self.results_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/scaling_perf/benchmark_results.json")
        self.results = self._load_results()

    def _load_results(self) -> dict:
        if os.path.exists(self.results_file):
            try:
                with open(self.results_file, "r") as f:
                    return json.load(f)
            except Exception:
                return {}
        return {}

    def _save_results(self):
        with open(self.results_file, "w") as f:
            json.dump(self.results, f, indent=2)

    @scoped_processes
    def run_trial(
        self,
        world_file: str,
        robot_file: str,
        warmup_s: float = 10.0,
        duration_s: float = 120.0,
        enable_gui: bool = False,
        enable_lidar: bool = True,
        enable_depth: bool = True,
        update_rate: int = None,
        *, _processes=None,
    ) -> dict:
        """
        Runs a single trial of the benchmark.
        """
        print(f"\n[BENCHMARK TRIAL] Warm-up: {warmup_s}s | Duration: {duration_s}s | GUI: {enable_gui} | UpdateRate: {update_rate or 'Default'}")
        print(f"  World: {os.path.basename(world_file)}")
        print(f"  Robot: {os.path.basename(robot_file)}")

        bridge_config = os.path.join(self.root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")

        # 1. Start Gazebo
        gz_cmd = ["gz", "sim", "-r"]
        if not enable_gui:
            gz_cmd.append("-s")
        if update_rate is not None and update_rate > 0:
            gz_cmd.extend(["-z", str(update_rate)])
        gz_cmd.append(world_file)

        env = build_benchmark_env(self.root_dir, turtlebot3=True)

        print("  -> Starting Gazebo Sim...")
        _processes.popen(
            gz_cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
            text=True
        )

        time.sleep(4.0)

        # 2. Spawn Robot
        print("  -> Spawning Robot...")
        spawn_benchmark_robot(robot_file, env, processes=_processes)

        # 3. Start ROS parameter bridge
        print("  -> Starting ROS Parameter Bridge...")
        start_parameter_bridge(bridge_config, env, processes=_processes)

        if enable_depth:
            print("  -> Starting ROS Image Bridge for Depth...")
            _processes.popen(
                ["ros2", "run", "ros_gz_image", "image_bridge", "/camera/image", "/camera/depth_image"],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )

        time.sleep(2.0)

        # 4. Run Profiler in separate Python process to capture exact ROS 2 topic rates
        profiler_cmd = (
            f"source {self.root_dir}/setup_env.sh && python3 -c \""
            f"import rclpy, json\n"
            f"from benchmark_profiler import BenchmarkProfiler\n"
            f"rclpy.init()\n"
            f"p = BenchmarkProfiler(target_duration={duration_s}, warmup_duration={warmup_s})\n"
            f"summary = p.run_profiling()\n"
            f"p.destroy_node()\n"
            f"rclpy.shutdown()\n"
            f"print('__SUMMARY_JSON__:' + json.dumps(summary))\n"
            f"\""
        )

        summary = {}
        try:
            print(f"  -> Profiling started for {warmup_s + duration_s}s total...")
            prof_proc = _processes.popen(
                ["bash", "-c", profiler_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=os.path.join(self.root_dir, "experiments/gazebo_benchmark"),
            )
            output, _ = prof_proc.communicate(timeout=warmup_s + duration_s + 30.0)
            # Extract summary JSON from output
            for line in output.splitlines():
                if "__SUMMARY_JSON__:" in line:
                    json_str = line.split("__SUMMARY_JSON__:")[1]
                    summary = json.loads(json_str)
                    break
        except Exception as e:
            print(f"  [ERROR] Profiling failed: {e}")
        finally:
            print("  -> Cleaning up processes...")

        return summary

    def run_condition(
        self,
        name: str,
        num_humans: int = 5,
        representation: str = "actor_and_cylinder",
        enable_lidar: bool = True,
        enable_depth: bool = True,
        depth_width: int = 640,
        depth_height: int = 480,
        depth_fps: int = 30,
        enable_gui: bool = False,
        warmup_s: float = 10.0,
        duration_s: float = 120.0,
        num_runs: int = 3,
        update_rate: int = None,
    ) -> dict:
        """
        Runs N repetitions of a condition and aggregates statistics.
        """
        print("\n" + "=" * 75)
        print(f"STARTING BENCHMARK CONDITION: {name.upper()}")
        print(f"Humans: {num_humans} ({representation}) | Lidar: {enable_lidar} | Depth: {enable_depth} ({depth_width}x{depth_height}@{depth_fps}fps) | GUI: {enable_gui} | UpdateRate: {update_rate or 'Default'}")
        print("=" * 75)

        # Generate world and robot models
        world_file = self.generator.generate_world_with_humans(
            num_humans=num_humans,
            representation=representation,
        )
        robot_file = self.generator.generate_robot_model(
            enable_lidar=enable_lidar,
            enable_depth=enable_depth,
            depth_width=depth_width,
            depth_height=depth_height,
            depth_fps=depth_fps,
        )

        trial_summaries = []
        for run_idx in range(num_runs):
            print(f"\n--- Run {run_idx + 1} of {num_runs} ---")
            summary = self.run_trial(
                world_file=world_file,
                robot_file=robot_file,
                warmup_s=warmup_s,
                duration_s=duration_s,
                enable_gui=enable_gui,
                enable_lidar=enable_lidar,
                enable_depth=enable_depth,
                update_rate=update_rate,
            )
            if summary:
                trial_summaries.append(summary)
                print(f"  Run {run_idx + 1} Result: RTF Median = {summary.get('rtf_median', 0.0):.3f} | Scan Hz(wall/sim) = {summary.get('scan_hz_wall_mean', summary.get('scan_hz_mean', 0.0)):.1f}/{summary.get('scan_hz_sim_mean', 0.0):.1f} | Depth Hz(wall/sim) = {summary.get('depth_hz_wall_mean', summary.get('depth_hz_mean', 0.0)):.1f}/{summary.get('depth_hz_sim_mean', 0.0):.1f}")
            else:
                print(f"  [WARN] Run {run_idx + 1} yielded empty summary.")

        # Aggregate across runs
        if not trial_summaries:
            print(f"[ERROR] All runs failed for {name}!")
            return {}

        rtf_medians = [s['rtf_median'] for s in trial_summaries]
        rtf_p5s = [s['rtf_p5'] for s in trial_summaries]
        rtf_means = [s['rtf_mean'] for s in trial_summaries]
        scan_hzs_wall = [s.get('scan_hz_wall_mean', s.get('scan_hz_mean', 0.0)) for s in trial_summaries]
        scan_hzs_sim = [s.get('scan_hz_sim_mean', 0.0) for s in trial_summaries]
        depth_hzs_wall = [s.get('depth_hz_wall_mean', s.get('depth_hz_mean', 0.0)) for s in trial_summaries]
        depth_hzs_sim = [s.get('depth_hz_sim_mean', 0.0) for s in trial_summaries]
        gpu_utils = [s['gpu_util_mean'] for s in trial_summaries]
        vrams = [s['vram_mb_mean'] for s in trial_summaries]
        rams = [s['ram_mb_mean'] for s in trial_summaries]
        cpu_totals = [s.get('cpu_total_mean', 0.0) for s in trial_summaries]
        cpu_gzs = [s.get('cpu_gz_mean', 0.0) for s in trial_summaries]

        aggregated = {
            'name': name,
            'num_humans': num_humans,
            'representation': representation,
            'enable_lidar': enable_lidar,
            'enable_depth': enable_depth,
            'depth_resolution': f"{depth_width}x{depth_height}",
            'depth_fps_target': depth_fps,
            'enable_gui': enable_gui,
            'update_rate': update_rate,
            'runs_completed': len(trial_summaries),
            'rtf_median_mean': float(np.mean(rtf_medians)),
            'rtf_median_std': float(np.std(rtf_medians)),
            'rtf_p5_mean': float(np.mean(rtf_p5s)),
            'rtf_p5_std': float(np.std(rtf_p5s)),
            'rtf_mean_mean': float(np.mean(rtf_means)),
            'rtf_mean_std': float(np.std(rtf_means)),
            'scan_hz_wall_mean': float(np.mean(scan_hzs_wall)),
            'scan_hz_wall_std': float(np.std(scan_hzs_wall)),
            'scan_hz_sim_mean': float(np.mean(scan_hzs_sim)),
            'scan_hz_sim_std': float(np.std(scan_hzs_sim)),
            'depth_hz_wall_mean': float(np.mean(depth_hzs_wall)),
            'depth_hz_wall_std': float(np.std(depth_hzs_wall)),
            'depth_hz_sim_mean': float(np.mean(depth_hzs_sim)),
            'depth_hz_sim_std': float(np.std(depth_hzs_sim)),
            'gpu_util_mean': float(np.mean(gpu_utils)),
            'gpu_util_std': float(np.std(gpu_utils)),
            'vram_mb_mean': float(np.mean(vrams)),
            'ram_mb_mean': float(np.mean(rams)),
            'cpu_total_mean': float(np.mean(cpu_totals)),
            'cpu_gz_mean': float(np.mean(cpu_gzs)),
        }

        self.results[name] = aggregated
        self._save_results()

        print("\n" + "-" * 75)
        print(f"[AGGREGATED RESULT: {name.upper()}]")
        print(f"  RTF Median:     {aggregated['rtf_median_mean']:.3f} +/- {aggregated['rtf_median_std']:.3f}")
        print(f"  RTF p5 (worst): {aggregated['rtf_p5_mean']:.3f} +/- {aggregated['rtf_p5_std']:.3f}")
        print(f"  LiDAR Rate:     Wall: {aggregated['scan_hz_wall_mean']:.1f} Hz | Sim: {aggregated['scan_hz_sim_mean']:.1f} Hz (nominal 5 Hz)")
        print(f"  Depth Rate:     Wall: {aggregated['depth_hz_wall_mean']:.1f} Hz | Sim: {aggregated['depth_hz_sim_mean']:.1f} Hz (nominal {depth_fps} Hz)")
        print(f"  GPU Utilization:{aggregated['gpu_util_mean']:.1f}%")
        print(f"  VRAM Used:      {aggregated['vram_mb_mean']:.1f} MB")
        print(f"  RAM Used:       {aggregated['ram_mb_mean']:.1f} MB")
        print(f"  CPU Total / gz: {aggregated['cpu_total_mean']:.1f}% / {aggregated['cpu_gz_mean']:.1f}%")
        print("-" * 75)

        return aggregated


def main():
    parser = argparse.ArgumentParser(description="Gazebo Harmonic Benchmark Runner")
    parser.add_argument("--condition", type=str, default="baseline", 
                        help="Condition or group name: baseline, sensors, humans, representation, depth_rates, uncapped, all, quick_test, or specific name")
    parser.add_argument("--warmup", type=float, default=5.0, help="Warmup duration in seconds")
    parser.add_argument("--duration", type=float, default=25.0, help="Test duration in seconds")
    parser.add_argument("--runs", type=int, default=2, help="Number of repetitions per condition")
    args = parser.parse_args()

    runner = GazeboBenchmarkRunner()

    cond = args.condition
    if cond == "quick_test":
        runner.run_condition("quick_test_baseline", num_humans=5, warmup_s=5.0, duration_s=15.0, num_runs=1)
    elif cond == "baseline":
        runner.run_condition("baseline", num_humans=5, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
    elif cond in ["uncapped", "uncapped_all"]:
        # Uncapped RTF measurements (Priority 1)
        runner.run_condition("uncapped_no_sensors", num_humans=5, enable_lidar=False, enable_depth=False, update_rate=5000, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("uncapped_lidar_only", num_humans=5, enable_lidar=True, enable_depth=False, update_rate=5000, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("uncapped_baseline", num_humans=5, enable_lidar=True, enable_depth=True, update_rate=5000, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("uncapped_depth_320x240_15fps", num_humans=5, enable_lidar=True, enable_depth=True, depth_width=320, depth_height=240, depth_fps=15, update_rate=5000, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
    elif cond in ["depth_matrix", "depth_grid"]:
        # Thorough Depth matrix: separating resolution from frequency
        runner.run_condition("D1_depth_320x240_15fps", num_humans=5, depth_width=320, depth_height=240, depth_fps=15, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("D2_depth_320x240_30fps", num_humans=5, depth_width=320, depth_height=240, depth_fps=30, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("D3_depth_320x240_10fps", num_humans=5, depth_width=320, depth_height=240, depth_fps=10, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("D4_depth_640x480_10fps", num_humans=5, depth_width=640, depth_height=480, depth_fps=10, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("D5_depth_640x480_15fps", num_humans=5, depth_width=640, depth_height=480, depth_fps=15, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
    elif cond in ["humans", "group_B"]:
        runner.run_condition("B1_n0", num_humans=0, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("B2_n2", num_humans=2, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("B3_n10", num_humans=10, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("B4_n24", num_humans=24, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
    elif cond == "B4_n24":
        runner.run_condition("B4_n24", num_humans=24, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
    elif cond in ["crowd_depth10hz", "crowd_10hz"]:
        # User requested crowd conditions at Depth 10 Hz on RTX 4060 dGPU
        runner.run_condition("N10_depth10hz", num_humans=10, depth_fps=10, representation="actor_and_cylinder", warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("N24_depth10hz", num_humans=24, depth_fps=10, representation="actor_and_cylinder", warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("N24_cylinder_depth10hz", num_humans=24, depth_fps=10, representation="cylinder_only", warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
    elif cond in ["sensors", "group_A"]:
        runner.run_condition("A1_no_sensors", num_humans=5, enable_lidar=False, enable_depth=False, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("A2_lidar_only", num_humans=5, enable_lidar=True, enable_depth=False, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
        runner.run_condition("A3_depth_only", num_humans=5, enable_lidar=False, enable_depth=True, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)
    else:
        # Generic single run
        runner.run_condition(cond, num_humans=5, warmup_s=args.warmup, duration_s=args.duration, num_runs=args.runs)


if __name__ == '__main__':
    main()

