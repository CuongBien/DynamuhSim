#!/usr/bin/env python3
"""
Master runner for Step 3:
Executes Step 3a (40 trials), followed by Step 3b (40 trials), followed by Step 3c (40 trials).
Timer: 40s sim-time strictly timed from goal send.
"""

import os
import sys
import time
import subprocess

ROOT_DIR = "/home/cuongbien/DynamuhSim"
BENCHMARK_SCRIPT = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/benchmark_a3_3_step3.py")

def main():
    print("=" * 95, flush=True)
    print("STARTING FULL STEP 3 BENCHMARK SUITE (STEP 3a -> STEP 3b -> STEP 3c)", flush=True)
    print("Timer rule: max 40.0s sim-time strictly measured from goal acceptance", flush=True)
    print("=" * 95, flush=True)

    t0 = time.time()

    # 1. Step 3a
    print("\n>>> [1/3] RUNNING STEP 3a (time_steps = 50, horizon 2.5s, 40 trials, RTF=1.0)...", flush=True)
    subprocess.run([sys.executable, BENCHMARK_SCRIPT, "--config", "3a"], check=True)
    time.sleep(3.0)

    # 2. Step 3b
    print("\n>>> [2/3] RUNNING STEP 3b (time_steps = 80, horizon 4.0s, 40 trials, RTF=1.0)...", flush=True)
    subprocess.run([sys.executable, BENCHMARK_SCRIPT, "--config", "3b"], check=True)
    time.sleep(3.0)

    # 3. Step 3c
    print("\n>>> [3/3] RUNNING STEP 3c (time_steps = 80, halved PathAlign/Follow/Forward weights, 40 trials, RTF=1.0)...", flush=True)
    subprocess.run([sys.executable, BENCHMARK_SCRIPT, "--config", "3c"], check=True)

    elapsed = time.time() - t0
    print(f"\n[ALL COMPLETE] Full Step 3 suite finished successfully in {elapsed/60.0:.1f} minutes!", flush=True)

if __name__ == '__main__':
    main()
