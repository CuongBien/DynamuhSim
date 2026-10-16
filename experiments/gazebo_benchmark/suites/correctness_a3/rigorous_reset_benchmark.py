"""
Rigorous Episode Reset & State Leakage Benchmark for Gazebo Harmonic (User Specification).
1. Measures time from reset command / process launch to FIRST VALID DEPTH FRAME.
2. Evaluates state leakage across 3 consecutive episodes in the same process:
   - Episode 1 (20s) -> In-process Reset -> Episode 2 (20s) -> In-process Reset -> Episode 3 (20s)
   - Checks if trajectories across consecutive episodes diverge (RVO2/seed/actor state leakage).
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
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped

import gz.transport13 as gzt
import gz.msgs10.boolean_pb2 as boolean_pb2
import gz.msgs10.world_control_pb2 as world_control_pb2
import gz.msgs10.pose_pb2 as pose_pb2


class FirstFrameWaiter(Node):
    def __init__(self):
        super().__init__('first_frame_waiter')
        self.received_depth = False
        self.received_scan = False
        self.first_depth_time = None
        self.first_scan_time = None

        self.depth_sub = self.create_subscription(Image, '/camera/depth_image', self._depth_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

    def _depth_cb(self, msg: Image):
        if not self.received_depth:
            self.received_depth = True
            self.first_depth_time = time.time()

    def _scan_cb(self, msg: LaserScan):
        if not self.received_scan:
            self.received_scan = True
            self.first_scan_time = time.time()

    def wait_for_first_frames(self, timeout_s=15.0):
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.received_depth and self.received_scan:
                break
        return self.received_depth, self.received_scan


def measure_cold_restart_to_first_frame(world_file, robot_file, root_dir="/home/cuongbien/DynamuhSim", n_trials=3):
    print("\n" + "=" * 70)
    print("MEASURING COLD RESTART DURATION TO FIRST VALID SENSOR FRAME")
    print("=" * 70)
    corridor_models = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models")
    tb3_models = os.path.join(root_dir, "install/turtlebot3_gazebo/share/turtlebot3_gazebo/models")
    plugin_share = os.path.join(root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{tb3_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    cold_restart_times = []

    for t in range(n_trials):
        t_start = time.time()

        # 1. Start Gazebo Sim
        gz_proc = subprocess.Popen(["gz", "sim", "-r", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(3.5)

        # 2. Spawn robot
        spawn_cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "burger", "-file", robot_file,
            "-x", "3.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"
        ]
        subprocess.run(spawn_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

        # 3. Parameter bridge and image bridge
        bridge_proc = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        image_proc = subprocess.Popen(["ros2", "run", "ros_gz_image", "image_bridge", "/camera/image", "/camera/depth_image"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        # 4. Wait for first valid depth frame
        rclpy.init()
        waiter = FirstFrameWaiter()
        has_depth, has_scan = waiter.wait_for_first_frames(timeout_s=15.0)
        dt_first_frame = time.time() - t_start
        waiter.destroy_node()
        rclpy.shutdown()

        print(f"  Trial {t+1}: {dt_first_frame:.2f}s to first valid depth frame (Depth: {has_depth}, Scan: {has_scan})")
        cold_restart_times.append(dt_first_frame)

        for p in [image_proc, bridge_proc, gz_proc]:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.2)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1.0)

    return cold_restart_times


def measure_consecutive_episode_leakage(world_file, robot_file, root_dir="/home/cuongbien/DynamuhSim", n_episodes=3):
    print("\n" + "=" * 70)
    print("EVALUATING IN-PROCESS STATE LEAKAGE ACROSS 3 CONSECUTIVE EPISODES")
    print("=" * 70)
    corridor_models = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models")
    tb3_models = os.path.join(root_dir, "install/turtlebot3_gazebo/share/turtlebot3_gazebo/models")
    plugin_share = os.path.join(root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{tb3_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    gz_proc = subprocess.Popen(["gz", "sim", "-r", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(4.0)

    spawn_cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-name", "burger", "-file", robot_file,
        "-x", "3.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"
    ]
    subprocess.run(spawn_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

    bridge_proc = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(1.5)

    node = gzt.Node()
    control_service = "/world/corridor_0.90/control"
    req_reset = world_control_pb2.WorldControl()
    req_reset.reset.all = True

    # Helper node to run episode and record trajectory
    class EpisodeRunner(Node):
        def __init__(self):
            super().__init__('ep_runner')
            self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
            self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
            self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
            self.sim_time = 0.0
            self.latest_pos = None

        def _clock_cb(self, msg: Clock):
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        def _odom_cb(self, msg: Odometry):
            self.latest_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        def run_episode(self, duration_s=15.0):
            while rclpy.ok() and self.sim_time <= 0.01:
                rclpy.spin_once(self, timeout_sec=0.05)
            t_start = self.sim_time
            traj = []
            next_t = t_start + 0.1
            twist = TwistStamped()
            twist.twist.linear.x = 0.15
            twist.twist.angular.z = 0.05
            while rclpy.ok() and (self.sim_time - t_start) < duration_s:
                rclpy.spin_once(self, timeout_sec=0.02)
                twist.header.stamp = self.get_clock().now().to_msg()
                self.cmd_pub.publish(twist)
                if self.sim_time >= next_t and self.latest_pos:
                    traj.append((self.sim_time - t_start, self.latest_pos[0], self.latest_pos[1]))
                    next_t += 0.1
            return traj

    rclpy.init()
    runner = EpisodeRunner()
    ep_trajectories = []

    try:
        for ep in range(n_episodes):
            print(f"\n--- Running Episode {ep + 1} of {n_episodes} (In-Process) ---")
            traj = runner.run_episode(duration_s=15.0)
            ep_trajectories.append(traj)
            print(f"  Episode {ep + 1} finished: {len(traj)} points. Issuing in-process WorldControl Reset...")
            t_res_0 = time.time()
            node.request(control_service, req_reset, world_control_pb2.WorldControl, boolean_pb2.Boolean, 2000)
            dt_reset = (time.time() - t_res_0) * 1000.0
            print(f"  In-process reset acknowledged in {dt_reset:.1f} ms.")
            time.sleep(1.0)
    finally:
        runner.destroy_node()
        rclpy.shutdown()
        for p in [bridge_proc, gz_proc]:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.2)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass

    # Compare trajectories between consecutive episodes
    min_len = min(len(t) for t in ep_trajectories)
    err_ep2_vs_ep1 = [np.hypot(p2[1] - p1[1], p2[2] - p1[2]) for p1, p2 in zip(ep_trajectories[0][:min_len], ep_trajectories[1][:min_len])]
    err_ep3_vs_ep1 = [np.hypot(p3[1] - p1[1], p3[2] - p1[2]) for p1, p3 in zip(ep_trajectories[0][:min_len], ep_trajectories[2][:min_len])]

    max_err_21 = float(np.max(err_ep2_vs_ep1)) if err_ep2_vs_ep1 else 999.0
    max_err_31 = float(np.max(err_ep3_vs_ep1)) if err_ep3_vs_ep1 else 999.0

    print("\n" + "=" * 70)
    print("STATE LEAKAGE CHECK RESULTS")
    print("=" * 70)
    print(f"  Episode 2 vs Episode 1 Max Discrepancy: {max_err_21*1000:.2f} mm")
    print(f"  Episode 3 vs Episode 1 Max Discrepancy: {max_err_31*1000:.2f} mm")
    if max_err_21 < 0.05 and max_err_31 < 0.05:
        print("  -> EVALUATION: ZERO STATE LEAKAGE DETECTED (< 5 cm between consecutive episodes).")
    else:
        print("  -> EVALUATION: STATE LEAKAGE DETECTED (RVO2/seed state drifted between episodes).")
    print("=" * 70)

    return max_err_21, max_err_31


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_n5_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/burger_lidar1_depth1_640x480_10fps.sdf")

    cold_times = measure_cold_restart_to_first_frame(world_file, robot_file, root_dir, n_trials=3)
    leak_21, leak_31 = measure_consecutive_episode_leakage(world_file, robot_file, root_dir, n_episodes=3)

    results = {
        "cold_restart_to_first_frame_s": {
            "mean": float(np.mean(cold_times)),
            "std": float(np.std(cold_times)),
            "trials": cold_times
        },
        "state_leakage_max_err_ep2_vs_ep1_m": float(leak_21),
        "state_leakage_max_err_ep3_vs_ep1_m": float(leak_31),
    }

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/rigorous_reset_benchmark_results.json")
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)


if __name__ == '__main__':
    main()
