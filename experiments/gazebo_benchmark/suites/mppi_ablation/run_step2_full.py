#!/usr/bin/env python3
"""
Master runner for Step 2:
Executes Step 2a (40 trials), followed sequentially by Step 2b (40 trials).
"""

import os
import sys
import time
import subprocess

ROOT_DIR = "/home/cuongbien/DynamuhSim"
BENCHMARK_SCRIPT = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/suites/mppi_ablation/benchmark_a3_3_step2.py")

def main():
    print("=" * 90, flush=True)
    print("STARTING FULL STEP 2 BENCHMARK SUITE (STEP 2a followed by STEP 2b)", flush=True)
    print("=" * 90, flush=True)

    t0 = time.time()

    # 1. Step 2a
    print("\n>>> [1/2] RUNNING STEP 2a (40 trials, RTF=1.0)...", flush=True)
    subprocess.run([sys.executable, BENCHMARK_SCRIPT, "--config", "2a"], check=True)

    # Short pause to ensure clean transition
    time.sleep(3.0)

    # 2. Step 2b
    print("\n>>> [2/2] RUNNING STEP 2b (40 trials, RTF=1.0)...", flush=True)
    subprocess.run([sys.executable, BENCHMARK_SCRIPT, "--config", "2b"], check=True)

    elapsed = time.time() - t0
    print(f"\n[ALL COMPLETE] Full Step 2 suite finished successfully in {elapsed/60.0:.1f} minutes!", flush=True)

if __name__ == '__main__':
    main()
