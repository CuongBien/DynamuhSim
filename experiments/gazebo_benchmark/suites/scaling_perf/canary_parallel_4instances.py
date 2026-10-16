"""
Canary Parallel Scaling Benchmark (4 Instances, Uncapped & Locked, 3 Runs).
Measures empirical throughput with closed background apps.
Ensures verified ROS 2 /scan and /camera/depth_image reception on every domain.
"""

import sys
import os
import time
import json
import numpy as np

sys.path.insert(0, '/home/cuongbien/DynamuhSim/experiments/gazebo_benchmark/core')
from parallel_scaling_benchmark import run_parallel_benchmark


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    print("=" * 75)
    print("STARTING CANARY PARALLEL SCALING (1 to 4 Instances, Depth 10 Hz)")
    print("Environment: Idle, NVIDIA RTX 4060 dGPU (D3D12)")
    print("=" * 75)

    # Run for 1, 2, 3, 4 instances with 20s measurement window
    results = run_parallel_benchmark(
        root_dir=root_dir,
        test_instances=[1, 2, 3, 4],
        duration_s=20.0,
        num_humans=10,
        depth_fps=10
    )

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/scaling_perf/canary_parallel_results.json")
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nCanary parallel results written to {out_file}")


if __name__ == '__main__':
    main()
