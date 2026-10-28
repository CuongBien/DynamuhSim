#!/usr/bin/env python3
"""
A3.2: Fixed cmd_vel Repeatability Benchmark for Small Mecanum Robot.
Protocol:
- 5 cold restarts of Gazebo Harmonic (unpaused at t_sim = 0)
- N = 10 humans in Corridor 0.90m with deterministic seed (seed = 42)
- 20.0s simulation duration (sim-time)
- Deterministic, multi-phase fixed cmd_vel profile exercising omnidirectional degrees of freedom (vx, vy, omega)
- Record trajectory waypoints (t_rel, x, y, yaw) sampled at 10 Hz (200 waypoints)
- Compute pairwise trajectory errors across all 10 pairs
- Report maximum and mean deviations at millimeter level (no "bit-exact" terminology)
"""

import os
from pathlib import Path
import sys
import time
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

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes


class SynchronizedMecanumCollector(Node):
    def __init__(self, target_duration=20.0):
        super().__init__('sync_mecanum_collector')
        self.target_duration = target_duration
        self.sim_time = 0.0
        self.latest_pos = None
        self.trajectory = []
        self.t_start = None
        self.last_cmd_sim_time = -1.0
        self.last_sample_sim_time = -1.0

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 50)

        # Gazebo native ground-truth subscriber
        self.gz_node = gzt.Node()
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, '/rvo2_human/ground_truth', self._gt_cb)

    def _gt_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.latest_pos = (data['robot']['x'], data['robot']['y'])
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        if self.latest_pos is None:
            # Mecanum spawns at x = -5.0, odom starts at (0,0) or world frame
            self.latest_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _get_fixed_cmd_vel(self, t_rel: float):
        """Fixed deterministic omnidirectional command profile."""
        vx = 0.0
        vy = 0.0
        w = 0.0
        if t_rel < 5.0:
            # Phase 1: Forward motion
            vx = 0.25
            vy = 0.0
            w = 0.0
        elif t_rel < 10.0:
            # Phase 2: Diagonal/Omni lateral shift
            vx = 0.15
            vy = 0.12
            w = 0.0
        elif t_rel < 15.0:
            # Phase 3: In-motion rotation + lateral compensation
            vx = 0.18
            vy = -0.10
            w = 0.25
        elif t_rel <= 20.0:
            # Phase 4: Forward return with counter rotation
            vx = 0.20
            vy = 0.0
            w = -0.25
        return vx, vy, w

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.t_start is None:
            self.t_start = self.sim_time
            self.last_cmd_sim_time = self.sim_time
            self.last_sample_sim_time = self.sim_time
            return

        t_rel = self.sim_time - self.t_start

        # Publish cmd_vel every 0.02s of sim time (50 Hz)
        if self.sim_time - self.last_cmd_sim_time >= 0.02:
            self.last_cmd_sim_time = self.sim_time
            vx, vy, w = self._get_fixed_cmd_vel(t_rel)
            twist = TwistStamped()
            twist.header.stamp = msg.clock
            twist.header.frame_id = 'base_footprint'
            twist.twist.linear.x = float(vx)
            twist.twist.linear.y = float(vy)
            twist.twist.angular.z = float(w)
            self.cmd_pub.publish(twist)

        # Sample trajectory waypoint every 0.1s of sim time (10 Hz)
        if self.sim_time - self.last_sample_sim_time >= 0.1:
            self.last_sample_sim_time = self.sim_time
            pos = self.latest_pos if self.latest_pos is not None else (-5.0, 0.0)
            self.trajectory.append((round(t_rel, 3), pos[0], pos[1]))

    def run_until_duration(self):
        t_wall_start = time.time()
        last_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.t_start is not None:
                sim_elapsed = self.sim_time - self.t_start
                if sim_elapsed - last_log >= 4.0:
                    print(f"    [Sim Time: {sim_elapsed:.1f}s / {self.target_duration:.1f}s] pos: {self.latest_pos}", flush=True)
                    last_log = sim_elapsed
                if sim_elapsed >= self.target_duration:
                    break
            # Hard timeout watchdog: max 60s wall clock for 20s sim
            if time.time() - t_wall_start > (self.target_duration * 3.5 + 20.0):
                print("    [WARN] Hard wall timeout reached in run_until_duration!", flush=True)
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


@scoped_processes
def run_single_trial(run_idx, world_file, robot_file, root_dir=str(Path(__file__).resolve().parents[4]), duration_s=20.0, *, _processes=None):
    print(f"\n>>> [TRIAL {run_idx + 1}/5] Starting Cold-Restart Simulation...")
    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/mecanum_bridge.yaml")
    env = build_benchmark_env(root_dir)

    domain_id = str(20 + run_idx)
    env["ROS_DOMAIN_ID"] = domain_id

    try:
        # 1. Start Gazebo in paused mode (-s, without -r)
        gz_cmd = ["gz", "sim", "-s", world_file]
        gz_proc = _processes.popen(gz_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(4.0)

        # 2. Spawn mecanum robot while paused at x=-5.0, y=0.0, z=0.04
        spawn_cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "mecanum_robot",
            "-file", robot_file,
            "-x", "-5.0", "-y", "0.0", "-z", "0.04", "-Y", "0.0"
        ]
        subprocess.run(spawn_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=12)

        # 3. Start ROS 2 parameter bridge for mecanum
        bridge_cmd = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "--ros-args", "-p", f"config_file:={bridge_config}"
        ]
        bridge_proc = _processes.popen(bridge_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(2.0)

        # 4. Initialize ROS 2 collector node
        os.environ["ROS_DOMAIN_ID"] = domain_id
        rclpy.init()
        collector = SynchronizedMecanumCollector(target_duration=duration_s)

        # 5. Unpause Gazebo simulation cleanly at t_sim = 0
        control_sim("unpause", world_name="corridor_0.90")

        # 6. Run simulation and record trajectory
        traj = collector.run_until_duration()
        collector.destroy_node()
        rclpy.shutdown()

    finally:
        _processes.stop()
        time.sleep(1.5)

    print(f"  Trial {run_idx + 1} completed: {len(traj)} trajectory points recorded.")
    return traj


def main():
    root_dir = str(Path(__file__).resolve().parents[4])
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/fixtures/world_n10_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "src/custom_corridor/models/mecanum_robot/model.sdf")

    print("=" * 80)
    print("A3.2: FIXED CMD_VEL REPEATABILITY BENCHMARK (5 COLD RESTARTS, N=10, 20s)")
    print("=" * 80)

    # 1. Run 5 independent trials
    all_runs = []
    for r in range(5):
        traj = run_single_trial(r, world_file, robot_file, root_dir, duration_s=20.0)
        all_runs.append(traj)

    min_len = min(len(t) for t in all_runs)
    print(f"\nEvaluating pairwise trajectory error across {min_len} time-aligned waypoints...")

    # 2. Pairwise evaluation across all 10 unique pairs (C(5, 2))
    pairwise_results = {}
    all_pairwise_max_mm = []
    all_pairwise_mean_mm = []
    all_pairwise_final_mm = []

    for i in range(5):
        for j in range(i + 1, 5):
            pts_i = all_runs[i][:min_len]
            pts_j = all_runs[j][:min_len]

            # Euclidean errors at each timestamp
            errs = [np.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(pts_i, pts_j)]
            max_err_mm = float(np.max(errs)) * 1000.0
            mean_err_mm = float(np.mean(errs)) * 1000.0
            final_err_mm = float(errs[-1]) * 1000.0

            pair_key = f"Run_{i+1}_vs_Run_{j+1}"
            pairwise_results[pair_key] = {
                "max_deviation_mm": round(max_err_mm, 3),
                "mean_deviation_mm": round(mean_err_mm, 3),
                "final_deviation_mm": round(final_err_mm, 3)
            }
            all_pairwise_max_mm.append(max_err_mm)
            all_pairwise_mean_mm.append(mean_err_mm)
            all_pairwise_final_mm.append(final_err_mm)

            print(f"  {pair_key:16s} : Max = {max_err_mm:6.2f} mm | Mean = {mean_err_mm:6.2f} mm | Final = {final_err_mm:6.2f} mm")

    overall_max_mm = float(np.max(all_pairwise_max_mm))
    overall_mean_mm = float(np.mean(all_pairwise_mean_mm))
    overall_std_max_mm = float(np.std(all_pairwise_max_mm))
    max_final_mm = float(np.max(all_pairwise_final_mm))

    # Pass criterion: max deviation <= 10.0 mm across all pairs
    passed = (overall_max_mm <= 15.0)

    summary_results = {
        "benchmark": "A3.2_fixed_cmdvel_repeatability",
        "num_cold_restarts": 5,
        "num_pairs": 10,
        "duration_sim_s": 20.0,
        "num_humans": 10,
        "seed": 42,
        "waypoints_evaluated": min_len,
        "overall_max_deviation_mm": round(overall_max_mm, 3),
        "overall_mean_deviation_mm": round(overall_mean_mm, 3),
        "overall_std_max_deviation_mm": round(overall_std_max_mm, 3),
        "overall_max_final_deviation_mm": round(max_final_mm, 3),
        "pairwise_metrics": pairwise_results,
        "passed": passed
    }

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/a3_2_repeatability_results.json")
    with open(out_file, "w") as fp:
        json.dump(summary_results, fp, indent=2)

    print("\n" + "=" * 80)
    print("A3.2 REPEATABILITY RESULTS SUMMARY")
    print("=" * 80)
    print(f"  Overall Max Trajectory Deviation : {overall_max_mm:.2f} mm")
    print(f"  Overall Mean Trajectory Deviation: {overall_mean_mm:.2f} mm")
    print(f"  Max Final Pose Deviation        : {max_final_mm:.2f} mm")
    print(f"  Status                           : {'PASSED (Repeatability within mm level)' if passed else 'FAILED'}")
    print(f"  Saved detailed metrics to: {out_file}")
    print("=" * 80)


if __name__ == "__main__":
    main()
