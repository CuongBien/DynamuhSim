#!/usr/bin/env python3
"""Run the Step 1 LiDAR measurement with owned Gazebo and inspector processes."""
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from experiments.gazebo_benchmark.core.mppi_trial import ProcessScope


def main():
    with ProcessScope() as processes:
        print("[1/4] Starting Gazebo Harmonic headless with robot at x=3.0, y=0.0...")
        processes.popen(
            ["bash", "-c", 'source "$1/setup_env.sh" && exec ros2 launch custom_corridor '
             'corridor_tb3.launch.py width:=0.90 gui:=false rviz:=false x:=3.0 y:=0.0 yaw:=0.0',
             "step1", str(ROOT)],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, text=True,
        )
        try:
            print("[2/4] Waiting for Gazebo and ROS bridge to initialize...")
            time.sleep(8)
            print("[3/4] Running measure_gazebo_lidar.py to collect /scan...")
            inspector = processes.popen(
                ["bash", "-c", 'source "$1/setup_env.sh" && exec python3 "$2"', "step1",
                 str(ROOT), str(Path(__file__).resolve().with_name("measure_gazebo_lidar.py"))],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )
            output, _ = inspector.communicate(timeout=25)
            print("--- Measurement Output ---")
            print(output)
            print("--------------------------")
        finally:
            print("[4/4] Terminating Gazebo process tree...")
    print("[DONE] Test execution finished.")


if __name__ == "__main__":
    main()
