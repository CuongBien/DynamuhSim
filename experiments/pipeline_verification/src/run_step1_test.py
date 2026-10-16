#!/usr/bin/env python3
"""
Orchestrator for Step 1 Measurement:
1. Launches Gazebo Harmonic with corridor_090.sdf and robot at x=3.0, y=0.0 (facing human at x=5.0).
2. Runs measure_gazebo_lidar.py to capture scan frames and analyze profile.
3. Shuts down Gazebo cleanly.
4. Generates experiments/pipeline_verification/STEP1_LIDAR_MEASUREMENT_REPORT.md.
"""

import os
import sys
import time
import subprocess
import signal
import numpy as np


def main():
    src_dir = os.path.dirname(os.path.abspath(__file__))
    exp_dir = os.path.dirname(src_dir)
    root_dir = os.path.dirname(os.path.dirname(exp_dir))
    report_file = os.path.join(exp_dir, "reports/STEP1_LIDAR_MEASUREMENT_REPORT.md")
    
    print("[1/4] Starting Gazebo Harmonic headless with robot at x=3.0, y=0.0...")
    launch_cmd = (
        f"source {root_dir}/setup_env.sh && "
        f"ros2 launch custom_corridor corridor_tb3.launch.py "
        f"width:=0.90 gui:=false rviz:=false x:=3.0 y:=0.0 yaw:=0.0"
    )
    
    gz_proc = subprocess.Popen(
        ["bash", "-c", launch_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True
    )
    
    try:
        # Wait 8 seconds for Gazebo and ROS 2 bridges to be ready
        print("[2/4] Waiting for Gazebo and ROS bridge to initialize...")
        time.sleep(8)
        
        # Run inspector
        print("[3/4] Running measure_gazebo_lidar.py to collect /scan...")
        measure_cmd = f"source {root_dir}/setup_env.sh && python3 {src_dir}/measure_gazebo_lidar.py"
        meas_proc = subprocess.run(
            ["bash", "-c", measure_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=25
        )
        print("--- Measurement Output ---")
        print(meas_proc.stdout)
        print("--------------------------")
        
    finally:
        print("[4/4] Terminating Gazebo process tree...")
        try:
            os.killpg(os.getpgid(gz_proc.pid), signal.SIGTERM)
            time.sleep(2)
            os.killpg(os.getpgid(gz_proc.pid), signal.SIGKILL)
        except Exception:
            pass
        gz_proc.wait()
        
    print("[DONE] Test execution finished.")


if __name__ == '__main__':
    main()
