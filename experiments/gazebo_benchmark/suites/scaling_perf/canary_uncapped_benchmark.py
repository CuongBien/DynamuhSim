"""
Canary Uncapped Performance Benchmark on NVIDIA RTX 4060 dGPU (D3D12).
Executes with closed background apps in an idle environment.
Runs 3 repetitions for each core condition with update_rate=5000 (uncapped).
Reports: RTF Mean +/- Std, RTF Median, RTF p5, Sim/Wall Hz, GPU%, VRAM, CPU%.
"""

import sys
import os
import json

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
from experiments.gazebo_benchmark.core.benchmark_runner import GazeboBenchmarkRunner


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    runner = GazeboBenchmarkRunner(root_dir)

    conditions = [
        {
            "name": "canary_baseline_n5_10hz",
            "num_humans": 5,
            "representation": "actor_and_cylinder",
            "depth_fps": 10,
            "depth_width": 640,
            "depth_height": 480,
        },
        {
            "name": "canary_crowd_n10_10hz",
            "num_humans": 10,
            "representation": "actor_and_cylinder",
            "depth_fps": 10,
            "depth_width": 640,
            "depth_height": 480,
        },
        {
            "name": "canary_crowd_n24_10hz_mesh",
            "num_humans": 24,
            "representation": "actor_and_cylinder",
            "depth_fps": 10,
            "depth_width": 640,
            "depth_height": 480,
        },
        {
            "name": "canary_crowd_n24_10hz_cylinder_only",
            "num_humans": 24,
            "representation": "cylinder_only",
            "depth_fps": 10,
            "depth_width": 640,
            "depth_height": 480,
        },
    ]

    all_results = {}

    print("=" * 75)
    print("STARTING CANARY UNCAPPED BENCHMARK (NVIDIA RTX 4060 dGPU, -z 5000)")
    print("Environment: Idle (Chrome closed), 3 runs per condition, 15s duration")
    print("=" * 75)

    for cfg in conditions:
        print("\n" + "#" * 75)
        print(f"CANARY CONDITION: {cfg['name']}")
        print("#" * 75)

        agg = runner.run_condition(
            name=cfg["name"],
            num_humans=cfg["num_humans"],
            representation=cfg["representation"],
            enable_lidar=True,
            enable_depth=True,
            depth_width=cfg["depth_width"],
            depth_height=cfg["depth_height"],
            depth_fps=cfg["depth_fps"],
            enable_gui=False,
            warmup_s=5.0,
            duration_s=15.0,
            num_runs=3,
            update_rate=5000,  # Uncapped hardware rate
        )
        all_results[cfg["name"]] = agg

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/scaling_perf/canary_uncapped_results.json")
    with open(out_file, "w") as f:
        json.dump(all_results, f, indent=2)

    print("\n" + "=" * 75)
    print("CANARY UNCAPPED SUMMARY TABLE")
    print("=" * 75)
    for name, res in all_results.items():
        print(f"Condition: {name}")
        print(f"  RTF Mean +/- Std: {res['rtf_mean_mean']:.3f} +/- {res['rtf_mean_std']:.3f}")
        print(f"  RTF Median:       {res['rtf_median_mean']:.3f} +/- {res['rtf_median_std']:.3f}")
        print(f"  RTF p5 (worst):   {res['rtf_p5_mean']:.3f} +/- {res['rtf_p5_std']:.3f}")
        print(f"  LiDAR Sim Hz:     {res['scan_hz_sim_mean']:.1f} Hz (Wall: {res['scan_hz_wall_mean']:.1f} Hz)")
        print(f"  Depth Sim Hz:     {res['depth_hz_sim_mean']:.1f} Hz (Wall: {res['depth_hz_wall_mean']:.1f} Hz)")
        print(f"  GPU % / VRAM:     {res['gpu_util_mean']:.1f}% / {res['vram_mb_mean']:.1f} MB")
        print(f"  CPU Total / gz:   {res['cpu_total_mean']:.1f}% / {res['cpu_gz_mean']:.1f}%")
        print("-" * 75)


if __name__ == '__main__':
    main()
