"""
Seed Determinism Benchmark (5 Runs, 20s, N=10 Humans).
Verifies strict trajectory reproducibility across 5 independent launches of Gazebo Harmonic.
Computes pairwise max position discrepancy, mean position discrepancy, and standard deviation.
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


class TrajectoryCollector(Node):
    def __init__(self, target_duration=20.0):
        super().__init__('traj_collector')
        self.target_duration = target_duration
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.sim_time = 0.0
        self.trajectory = []  # list of (sim_time, x, y)
        self.latest_pos = None

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.latest_pos = (x, y)

    def collect(self):
        # Wait for simulation clock
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

            if self.sim_time >= next_sample_time and self.latest_pos:
                self.trajectory.append((self.sim_time - t_start, self.latest_pos[0], self.latest_pos[1]))
                next_sample_time += 0.1

        return self.trajectory


def run_single_trial(run_idx, world_file, robot_file, root_dir="/home/cuongbien/DynamuhSim"):
    print(f"\n--- Launching Run {run_idx + 1} of 5 ---")
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

    spawn_cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-name", "burger",
        "-file", robot_file,
        "-x", "3.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"
    ]
    subprocess.run(spawn_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

    bridge_proc = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
    )
    time.sleep(1.5)

    rclpy.init()
    collector = TrajectoryCollector(target_duration=20.0)
    traj = collector.collect()
    collector.destroy_node()
    rclpy.shutdown()

    # Teardown
    for p in [bridge_proc, gz_proc]:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            time.sleep(0.2)
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass

    time.sleep(1.0)
    print(f"  Run {run_idx + 1} finished: collected {len(traj)} trajectory waypoints.")
    return traj


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_n10_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/burger_lidar1_depth1_640x480_10fps.sdf")

    print("=" * 70)
    print("STARTING 5-RUN SEED DETERMINISM BENCHMARK (N=10 Humans, 20s duration)")
    print("=" * 70)

    all_runs = []
    for r in range(5):
        traj = run_single_trial(r, world_file, robot_file, root_dir)
        all_runs.append(traj)

    # Compute pairwise errors against Run 0 (Baseline Reference)
    min_len = min(len(t) for t in all_runs)
    pairwise_max_errors = []
    pairwise_mean_errors = []

    ref_traj = all_runs[0][:min_len]

    print("\n" + "-" * 70)
    print("PAIRWISE TRAJECTORY COMPARISON (Against Run 1 Reference)")
    print("-" * 70)
    for idx, run_traj in enumerate(all_runs[1:], start=2):
        pts = run_traj[:min_len]
        errors = [np.hypot(p[1] - r[1], p[2] - r[2]) for p, r in zip(pts, ref_traj)]
        max_e = float(np.max(errors))
        mean_e = float(np.mean(errors))
        pairwise_max_errors.append(max_e)
        pairwise_mean_errors.append(mean_e)
        print(f"  Run {idx} vs Run 1: Max Discrepancy = {max_e*1000:.2f} mm | Mean = {mean_e*1000:.2f} mm")

    overall_max_err = max(pairwise_max_errors)
    overall_mean_err = np.mean(pairwise_mean_errors)
    overall_std_err = np.std(pairwise_max_errors)

    results = {
        "num_runs": 5,
        "duration_s": 20.0,
        "num_humans": 10,
        "num_waypoints": min_len,
        "overall_max_discrepancy_m": float(overall_max_err),
        "overall_mean_discrepancy_m": float(overall_mean_err),
        "overall_std_max_discrepancy_m": float(overall_std_err),
        "pairwise_max_errors_m": pairwise_max_errors,
        "pairwise_mean_errors_m": pairwise_mean_errors,
    }

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/seed_determinism_5runs_results.json")
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)

    print("\n" + "=" * 70)
    print("FINAL 5-RUN SEED DETERMINISM RESULTS (N=10 Humans, 20s)")
    print("=" * 70)
    print(f"  Maximum Position Error: {overall_max_err*1000:.2f} mm ({overall_max_err:.5f} m)")
    print(f"  Mean Position Error:    {overall_mean_err*1000:.2f} mm ({overall_mean_err:.5f} m)")
    print(f"  Std Dev of Max Error:   {overall_std_err*1000:.2f} mm ({overall_std_err:.5f} m)")
    if overall_max_err < 0.02:
        print("  -> EVALUATION: HIGHLY DETERMINISTIC (< 2 cm across 5 runs)")
    else:
        print(f"  -> EVALUATION: DRIFT DETECTED ({overall_max_err:.3f} m)")
    print("=" * 70)


if __name__ == '__main__':
    main()
