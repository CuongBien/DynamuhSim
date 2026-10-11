#!/usr/bin/env python3
"""
Accurately measures cold restart duration from command execution to first valid depth frame.
Uses ros2 launch corridor_tb3.launch.py with small mecanum robot.
"""

import sys
import os
import time
import signal
import subprocess
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


class DepthFrameWaiter(Node):
    def __init__(self):
        super().__init__('depth_frame_waiter')
        self.received_depth = False
        self.received_scan = False
        self.t_first_depth = None
        self.t_first_scan = None
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', self._depth_cb, 10)
        self.depth_sub2 = self.create_subscription(Image, '/camera/depth_image', self._depth_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

    def _depth_cb(self, msg: Image):
        if not self.received_depth:
            self.received_depth = True
            self.t_first_depth = time.time()

    def _scan_cb(self, msg: LaserScan):
        if not self.received_scan:
            self.received_scan = True
            self.t_first_scan = time.time()


def run_measurement(trial_idx: int):
    domain_id = str(80 + trial_idx)
    os.environ["ROS_DOMAIN_ID"] = domain_id
    os.environ["GALLIUM_DRIVER"] = "d3d12"
    os.environ["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"

    cmd = [
        "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
        "width:=0.90", "obstacle:=none", "gui:=false", "rviz:=false",
        "robot_model:=mecanum", "x:=-13.0", "y:=0.0", "yaw:=0.0"
    ]

    t_start = time.time()
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

    rclpy.init()
    waiter = DepthFrameWaiter()

    try:
        while rclpy.ok() and (time.time() - t_start) < 25.0:
            rclpy.spin_once(waiter, timeout_sec=0.05)
            if waiter.received_depth and waiter.received_scan:
                break

        dt = (waiter.t_first_depth - t_start) if waiter.t_first_depth else (time.time() - t_start)
    finally:
        waiter.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "image_bridge"], stderr=subprocess.DEVNULL)
        time.sleep(1.0)

    print(f"  Trial {trial_idx + 1}: {dt:.2f} s (Depth: {waiter.received_depth}, Scan: {waiter.received_scan})")
    return dt, waiter.received_depth, waiter.received_scan


def main():
    print("=" * 70)
    print("ACCURATE COLD RESTART LATENCY BENCHMARK (3 RUNS)")
    print("Definition: Start of command -> First valid sensor frame")
    print("=" * 70)

    results = []
    for i in range(3):
        dt, d_ok, s_ok = run_measurement(i)
        results.append({"trial": i + 1, "duration_s": dt, "depth_ok": d_ok, "scan_ok": s_ok})

    durations = [r["duration_s"] for r in results]
    mean_dt = float(sum(durations) / len(durations))
    std_dt = float((sum((x - mean_dt)**2 for x in durations) / len(durations)) ** 0.5)

    print("=" * 70)
    print(f"COLD RESTART RESULT: {mean_dt:.2f} s ± {std_dt:.2f} s (Min: {min(durations):.2f} s, Max: {max(durations):.2f} s)")
    print("=" * 70)

    with open("experiments/gazebo_benchmark/cold_restart_measured_results.json", "w") as f:
        json.dump({"mean_s": mean_dt, "std_s": std_dt, "trials": results}, f, indent=2)


if __name__ == "__main__":
    main()
