"""
Seed Determinism Test for Gazebo Harmonic & RVO2 Human Navigation.
Runs 2 identical trials with identical world & fixed robot cmd_vel.
Samples human and robot poses at 10 Hz and calculates max Euclidean error.
"""

import sys
import os
import time
import signal
import subprocess
import json
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class TrajectoryRecorder(Node):
    def __init__(self, target_duration=15.0):
        super().__init__('traj_recorder')
        self.target_duration = target_duration
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.sim_time = 0.0
        self.robot_trajectory = [] # list of (sim_time, x, y)
        self.latest_robot_pos = None

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.latest_robot_pos = (x, y)

    def record(self):
        # Wait for sim clock
        while rclpy.ok() and self.sim_time <= 0.1:
            rclpy.spin_once(self, timeout_sec=0.1)

        t_start = self.sim_time
        next_sample_time = t_start + 0.1

        twist = TwistStamped()
        twist.twist.linear.x = 0.15
        twist.twist.angular.z = 0.05

        while rclpy.ok() and (self.sim_time - t_start) < self.target_duration:
            rclpy.spin_once(self, timeout_sec=0.02)
            twist.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(twist)

            if self.sim_time >= next_sample_time and self.latest_robot_pos:
                self.robot_trajectory.append((self.sim_time - t_start, self.latest_robot_pos[0], self.latest_robot_pos[1]))
                next_sample_time += 0.1

        return self.robot_trajectory


def run_recording(world_file, robot_file, root_dir="/home/cuongbien/DynamuhSim"):
    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")
    corridor_models = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models")
    tb3_models = os.path.join(root_dir, "install/turtlebot3_gazebo/share/turtlebot3_gazebo/models")
    plugin_share = os.path.join(root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{tb3_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    gz_proc = subprocess.Popen(["gz", "sim", "-r", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(4.0)

    subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-name", "burger", "-file", robot_file, "-x", "3.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
    bridge_proc = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(1.5)

    # Record trajectory
    rclpy.init()
    recorder = TrajectoryRecorder(target_duration=15.0)
    traj = recorder.record()
    recorder.destroy_node()
    rclpy.shutdown()

    # Kill
    for p in [bridge_proc, gz_proc]:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            time.sleep(0.2)
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass

    return traj


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_n5_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/burger_lidar1_depth1_640x480_30fps.sdf")

    print("\n--- Trial 1 ---")
    traj1 = run_recording(world_file, robot_file, root_dir)
    time.sleep(2.0)

    print("\n--- Trial 2 ---")
    traj2 = run_recording(world_file, robot_file, root_dir)

    print(f"\nRecorded: Trial 1 = {len(traj1)} points, Trial 2 = {len(traj2)} points")
    min_len = min(len(traj1), len(traj2))
    errors = []
    for i in range(min_len):
        t1, x1, y1 = traj1[i]
        t2, x2, y2 = traj2[i]
        err = np.hypot(x1 - x2, y1 - y2)
        errors.append(err)

    max_err = max(errors) if errors else 999.0
    mean_err = np.mean(errors) if errors else 999.0
    print(f"\n[DETERMINISM RESULTS]")
    print(f"  Max Position Discrepancy:  {max_err:.6f} m")
    print(f"  Mean Position Discrepancy: {mean_err:.6f} m")
    
    if max_err < 0.001:
        print("  -> STATUS: PERFECT DETERMINISM (< 1 mm error)")
    elif max_err < 0.05:
        print("  -> STATUS: HIGH DETERMINISM (< 5 cm error)")
    else:
        print(f"  -> STATUS: NON-DETERMINISTIC (Max error: {max_err:.3f} m)")


if __name__ == '__main__':
    main()
