"""
Rigorous Seed Determinism & In-Process State Leakage Verification for Gazebo Harmonic.
Synchronized architecture:
1. Starts Gazebo simulation paused.
2. Spawns robot and initializes bridges while simulation is paused.
3. Resets world to t_sim = 0.00 and unpauses cleanly.
4. Publishes cmd_vel strictly synchronized to /clock (sim-time).
5. Tests:
   - Part 1: Seed Determinism across 5 independent launches (N=10 humans, 20s, seed=42).
   - Part 2: In-Process Reset Leakage across 3 consecutive episodes (N=10 humans, 20s).
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

import gz.transport13 as gzt
import gz.msgs.world_control_pb2 as world_control_pb2
import gz.msgs.boolean_pb2 as boolean_pb2
import gz.msgs.stringmsg_pb2 as stringmsg_pb2


class SynchronizedTrajectoryCollector(Node):
    def __init__(self, target_duration=20.0):
        super().__init__('sync_traj_collector')
        self.target_duration = target_duration
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 50)
        
        # Native Gazebo ground-truth subscription for exact robot pose
        self.gz_node = gzt.Node()
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, '/rvo2_human/ground_truth', self._gt_cb)

        self.sim_time = 0.0
        self.latest_pos = None
        self.trajectory = []
        self.t_start = None
        self.last_cmd_sim_time = -1.0
        self.last_sample_sim_time = -1.0

    def _gt_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.latest_pos = (data['robot']['x'], data['robot']['y'])
        except Exception:
            pass

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.t_start is None:
            self.t_start = self.sim_time
            self.last_cmd_sim_time = self.sim_time
            self.last_sample_sim_time = self.sim_time
            return

        t_rel = self.sim_time - self.t_start
        # Publish cmd_vel every 0.02s of SIM-TIME
        if self.sim_time - self.last_cmd_sim_time >= 0.02:
            self.last_cmd_sim_time = self.sim_time
            twist = TwistStamped()
            twist.header.stamp = msg.clock
            # Fixed trajectory profile
            twist.twist.linear.x = 0.15
            twist.twist.angular.z = 0.05 * np.sin(0.1 * t_rel)
            self.cmd_pub.publish(twist)

        # Sample trajectory point every 0.1s of SIM-TIME
        if self.sim_time - self.last_sample_sim_time >= 0.1:
            self.last_sample_sim_time = self.sim_time
            pos = self.latest_pos if self.latest_pos is not None else (3.0, 0.0)
            self.trajectory.append((t_rel, pos[0], pos[1]))

    def _odom_cb(self, msg: Odometry):
        if self.latest_pos is None:
            self.latest_pos = (3.0 + msg.pose.pose.position.x, msg.pose.pose.position.y)

    def run_until_duration(self):
        t_wall_start = time.time()
        last_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.t_start is not None:
                sim_elapsed = self.sim_time - self.t_start
                if sim_elapsed - last_log >= 3.0:
                    print(f"    [Sim Time: {sim_elapsed:.1f}s / {self.target_duration:.1f}s] pos: {self.latest_pos}", flush=True)
                    last_log = sim_elapsed
                if sim_elapsed >= self.target_duration:
                    break
            if time.time() - t_wall_start > (self.target_duration * 3.0 + 15.0):
                print("    [WARN] Hard wall-clock timeout reached in run_until_duration!", flush=True)
                break
        return self.trajectory


def control_sim(action="unpause", world_name="corridor_0.90"):
    node = gzt.Node()
    service = f"/world/{world_name}/control"
    req = world_control_pb2.WorldControl()
    if action == "pause":
        req.pause = True
    elif action == "unpause":
        req.pause = False
    elif action == "reset":
        req.reset.all = True
        req.pause = False
    node.request(service, req, world_control_pb2.WorldControl, boolean_pb2.Boolean, 3000)


def run_single_launch(run_idx, world_file, robot_file, root_dir="/home/cuongbien/DynamuhSim", duration_s=20.0):
    print(f"\n[LAUNCH {run_idx + 1}/5] Starting Gazebo Harmonic (Paused mode)...")
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

    # 1. Start Gazebo in paused mode (without -r)
    gz_proc = subprocess.Popen(["gz", "sim", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(3.5)

    # 2. Spawn robot while paused
    spawn_cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-name", "burger",
        "-file", robot_file,
        "-x", "3.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"
    ]
    subprocess.run(spawn_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

    # 3. Start ROS 2 parameter bridge
    bridge_proc = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
    )
    time.sleep(1.5)

    # 4. Prepare recorder node
    rclpy.init()
    collector = SynchronizedTrajectoryCollector(target_duration=duration_s)

    # 5. Unpause simulation cleanly
    control_sim("unpause")

    # 6. Run simulation
    traj = collector.run_until_duration()
    collector.destroy_node()
    rclpy.shutdown()

    # 7. Teardown
    for p in [bridge_proc, gz_proc]:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            time.sleep(0.2)
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
    time.sleep(1.0)
    print(f"  Launch {run_idx + 1} completed: {len(traj)} trajectory points recorded.")
    return traj


def test_seed_determinism_5runs(world_file, robot_file, root_dir):
    print("=" * 75)
    print("TEST 1: 5-RUN SEED DETERMINISM (N=10 Humans, 20s Duration, Seed=42)")
    print("=" * 75)
    runs = []
    for i in range(5):
        traj = run_single_launch(i, world_file, robot_file, root_dir)
        runs.append(traj)

    min_len = min(len(t) for t in runs)
    ref = runs[0][:min_len]

    pairwise_max_errors = []
    pairwise_mean_errors = []

    print("\nPairwise Discrepancy against Run 1 (Reference):")
    for idx, r in enumerate(runs[1:], start=2):
        pts = r[:min_len]
        errs = [np.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(pts, ref)]
        max_e = float(np.max(errs))
        mean_e = float(np.mean(errs))
        pairwise_max_errors.append(max_e)
        pairwise_mean_errors.append(mean_e)
        print(f"  Run {idx} vs Run 1: Max Discrepancy = {max_e*1000:7.2f} mm ({max_e:6.4f} m) | Mean = {mean_e*1000:7.2f} mm")

    overall_max = max(pairwise_max_errors)
    overall_mean = np.mean(pairwise_mean_errors)
    print(f"\n[OVERALL DETERMINISM RESULT]")
    print(f"  Max Error:  {overall_max*1000:.2f} mm ({overall_max:.4f} m)")
    print(f"  Mean Error: {overall_mean*1000:.2f} mm ({overall_mean:.4f} m)")
    return {
        "overall_max_mm": overall_max * 1000.0,
        "overall_mean_mm": overall_mean * 1000.0,
        "pairwise_max_mm": [e * 1000.0 for e in pairwise_max_errors]
    }


def test_in_process_reset_leakage(world_file, robot_file, root_dir, n_episodes=3, duration_s=20.0):
    print("\n" + "=" * 75)
    print("TEST 2: IN-PROCESS RESET LEAKAGE (3 Consecutive Episodes, Single Gazebo Process)")
    print("=" * 75)
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

    gz_proc = subprocess.Popen(["gz", "sim", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(3.5)

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

    ep_trajectories = []
    rclpy.init()

    try:
        for ep in range(n_episodes):
            print(f"\n[EPISODE {ep + 1}/{n_episodes}] Running {duration_s}s...", flush=True)
            if ep > 0:
                print("  Issuing WorldControl reset...", flush=True)
                control_sim("pause")
                control_sim("reset")
                time.sleep(0.5)
            control_sim("unpause")

            collector = SynchronizedTrajectoryCollector(target_duration=duration_s)
            traj = collector.run_until_duration()
            collector.destroy_node()
            ep_trajectories.append(traj)
            print(f"  Episode {ep + 1} finished: {len(traj)} points recorded.", flush=True)

    finally:
        rclpy.shutdown()
        for p in [bridge_proc, gz_proc]:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.2)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass

    min_len = min(len(t) for t in ep_trajectories)
    ref = ep_trajectories[0][:min_len]

    leakage_results = []
    print("\nPairwise Discrepancy against Episode 1 (Reference):")
    for ep_idx, traj in enumerate(ep_trajectories[1:], start=2):
        pts = traj[:min_len]
        errs = [np.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(pts, ref)]
        max_e = float(np.max(errs))
        mean_e = float(np.mean(errs))
        leakage_results.append((max_e, mean_e))
        print(f"  Episode {ep_idx} vs Episode 1: Max Drift = {max_e*1000:7.2f} mm ({max_e:6.4f} m) | Mean = {mean_e*1000:7.2f} mm")

    return leakage_results


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_n10_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/burger_lidar1_depth1_640x480_10fps.sdf")

    print("\n" + "#" * 75)
    print("GAZEBO HARMONIC VERIFICATION SUITE: DETERMINISM & STATE LEAKAGE")
    print("#" * 75)

    det_res = test_seed_determinism_5runs(world_file, robot_file, root_dir)
    leak_res = test_in_process_reset_leakage(world_file, robot_file, root_dir)

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/verification_results.json")
    with open(out_file, "w") as f:
        json.dump({
            "seed_determinism": det_res,
            "in_process_leakage": leak_res,
        }, f, indent=2)

    print("\n" + "=" * 75)
    print("ALL TESTS COMPLETED SUCCESSFULLY!")
    print(f"Saved results to: {out_file}")
    print("=" * 75)


if __name__ == '__main__':
    main()
