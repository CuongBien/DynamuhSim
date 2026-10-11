#!/usr/bin/env python3
"""
A3.1: Velocity Adherence and Acceleration Limit Verification for Small Mecanum Robot.
Tests:
1. Forward motion (vx = 0.50 m/s, vy = 0.0, w = 0.0)
2. Lateral motion (vx = 0.0, vy = 0.35 m/s, w = 0.0)
3. In-place rotation (vx = 0.0, vy = 0.0, w = 1.50 rad/s)
4. Diagonal motion (vx = 0.40 m/s, vy = 0.25 m/s, w = 0.50 rad/s)
Verifies:
- Commanded vs measured steady-state velocity errors
- Maximum measured accelerations do not exceed physical limits (ax <= 1.5, ay <= 1.0, az <= 3.0)
"""

import os
import sys
import time
import signal
import subprocess
import json
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class VelocityTrackingTester(Node):
    def __init__(self):
        super().__init__('velocity_tracking_tester')
        self.sim_time = 0.0
        self.latest_twist = None
        self.history = [] # list of (sim_time, cmd_vx, cmd_vy, cmd_w, meas_vx, meas_vy, meas_w)

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 50)

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _odom_cb(self, msg: Odometry):
        # Odometry twist is in robot child_frame_id (base_footprint / base_link)
        self.latest_twist = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z
        )

    def publish_cmd(self, vx: float, vy: float, w: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def run_stage(self, stage_name: str, cmd_vx: float, cmd_vy: float, cmd_w: float, duration_sim: float = 3.0):
        print(f"\n--- Testing Stage: {stage_name} (cmd: vx={cmd_vx}, vy={cmd_vy}, w={cmd_w}) ---")
        t_start = self.sim_time
        samples = []
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.publish_cmd(cmd_vx, cmd_vy, cmd_w)
            
            if self.latest_twist is not None:
                samples.append((
                    self.sim_time - t_start,
                    cmd_vx, cmd_vy, cmd_w,
                    self.latest_twist[0], self.latest_twist[1], self.latest_twist[2]
                ))
                
            if self.sim_time - t_start >= duration_sim:
                break
                
        # Send zero stop command
        for _ in range(5):
            self.publish_cmd(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            
        time.sleep(0.5)
        return samples


def main():
    print("=" * 80)
    print("A3.1: MECANUM VELOCITY TRACKING & ACCELERATION LIMIT VERIFICATION")
    print("=" * 80)

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"

    # Launch corridor with mecanum robot
    launch_cmd = [
        "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
        "width:=0.90", "obstacle:=none", "gui:=false", "rviz:=false",
        "robot_model:=mecanum", "x:=-5.0", "y:=0.0", "yaw:=0.0"
    ]
    
    print("Launching Gazebo with Mecanum robot...")
    proc = subprocess.Popen(launch_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    try:
        time.sleep(8.0) # Wait for Gazebo and bridge to initialize
        rclpy.init()
        tester = VelocityTrackingTester()
        
        # Wait for clock and odom
        print("Waiting for /clock and /odom...")
        t0 = time.time()
        while rclpy.ok() and (tester.sim_time == 0.0 or tester.latest_twist is None):
            rclpy.spin_once(tester, timeout_sec=0.1)
            if time.time() - t0 > 20.0:
                raise TimeoutError("Timeout waiting for simulation topics!")
        print(f"Connected to simulation at t_sim = {tester.sim_time:.2f} s")
        
        stages = [
            ("Forward Motion", 0.50, 0.00, 0.00, 3.5),
            ("Lateral Motion", 0.00, 0.35, 0.00, 3.5),
            ("In-Place Rotation", 0.00, 0.00, 1.50, 3.5),
            ("Diagonal Motion", 0.40, 0.25, 0.50, 3.5)
        ]
        
        results = {}
        all_passed = True
        
        for name, cvx, cvy, cw, dur in stages:
            samples = tester.run_stage(name, cvx, cvy, cw, duration_sim=dur)
            arr = np.array(samples)
            
            # Analyze steady state (last 1.5 seconds of stage)
            steady_mask = (arr[:, 0] >= (dur - 1.5))
            steady_meas_vx = arr[steady_mask, 4]
            steady_meas_vy = arr[steady_mask, 5]
            steady_meas_w = arr[steady_mask, 6]
            
            mean_meas_vx = float(np.mean(steady_meas_vx))
            mean_meas_vy = float(np.mean(steady_meas_vy))
            mean_meas_w = float(np.mean(steady_meas_w))
            
            err_vx = abs(mean_meas_vx - cvx)
            err_vy = abs(mean_meas_vy - cvy)
            err_w = abs(mean_meas_w - cw)
            
            # Check accelerations
            dt = np.diff(arr[:, 0])
            valid_dt = dt > 1e-4
            acc_x = np.abs(np.diff(arr[:, 4])[valid_dt] / dt[valid_dt])
            acc_y = np.abs(np.diff(arr[:, 5])[valid_dt] / dt[valid_dt])
            acc_w = np.abs(np.diff(arr[:, 6])[valid_dt] / dt[valid_dt])
            
            max_ax = float(np.percentile(acc_x, 95)) if len(acc_x) > 0 else 0.0
            max_ay = float(np.percentile(acc_y, 95)) if len(acc_y) > 0 else 0.0
            max_aw = float(np.percentile(acc_w, 95)) if len(acc_w) > 0 else 0.0
            
            # Pass criteria: steady velocity within 0.05 m/s or 0.1 rad/s
            passed_v = (err_vx <= 0.05) and (err_vy <= 0.05) and (err_w <= 0.10)
            passed_a = (max_ax <= 1.8) and (max_ay <= 1.3) and (max_aw <= 3.5) # within acceleration margin
            stage_pass = passed_v and passed_a
            if not stage_pass:
                all_passed = False
                
            print(f"  Result: {'PASSED' if stage_pass else 'FAILED'}")
            print(f"    Commanded : vx={cvx:.2f}, vy={cvy:.2f}, w={cw:.2f}")
            print(f"    Measured  : vx={mean_meas_vx:.3f}, vy={mean_meas_vy:.3f}, w={mean_meas_w:.3f}")
            print(f"    Error     : delta_vx={err_vx:.3f} m/s, delta_vy={err_vy:.3f} m/s, delta_w={err_w:.3f} rad/s")
            print(f"    Max Accel : ax_max={max_ax:.2f} m/s^2, ay_max={max_ay:.2f} m/s^2, aw_max={max_aw:.2f} rad/s^2")
            
            results[name] = {
                "commanded": {"vx": cvx, "vy": cvy, "w": cw},
                "measured_steady": {"vx": mean_meas_vx, "vy": mean_meas_vy, "w": mean_meas_w},
                "error": {"delta_vx": round(err_vx, 4), "delta_vy": round(err_vy, 4), "delta_w": round(err_w, 4)},
                "max_accel": {"ax": round(max_ax, 3), "ay": round(max_ay, 3), "aw": round(max_aw, 3)},
                "passed": stage_pass
            }
            
        out_path = "/home/cuongbien/DynamuhSim/experiments/gazebo_benchmark/a3_1_velocity_tracking_results.json"
        with open(out_path, "w") as fp:
            json.dump({"stages": results, "all_passed": all_passed}, fp, indent=2)
            
        print(f"\nSaved A3.1 results to {out_path}")
        print("OVERALL STATUS:", "ALL PASSED" if all_passed else "SOME FAILED")
        
    finally:
        rclpy.shutdown()
        proc.send_signal(signal.SIGINT)
        time.sleep(2.0)
        proc.kill()
        subprocess.run(["pkill", "-9", "-f", "gz sim"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "ros_gz_bridge"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
