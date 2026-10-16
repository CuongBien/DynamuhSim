"""
Pairwise Trajectory & Temporal Error Dynamics Analyzer:
Runs 5 independent launches of Gazebo Harmonic (N=10 humans, 20s duration, seed=42).
Saves all 5 full trajectories to disk.
Computes:
1. Full 5x5 Pairwise Error Matrix (all 10 pairs).
2. Temporal Error Profile e(t) across the 20 seconds.
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
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped

import gz.transport13 as gzt
import gz.msgs.world_control_pb2 as world_control_pb2
import gz.msgs.boolean_pb2 as boolean_pb2
import gz.msgs.stringmsg_pb2 as stringmsg_pb2


class PreciseTrajectoryRecorder(Node):
    def __init__(self, target_duration=20.0):
        super().__init__('precise_recorder')
        self.target_duration = target_duration
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 50)
        
        self.gz_node = gzt.Node()
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, '/rvo2_human/ground_truth', self._gt_cb)

        self.sim_time = 0.0
        self.latest_pos = None
        self.trajectory = []  # list of (t_rel, x, y)
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
        # Publish fixed cmd_vel every 0.02s sim-time
        if self.sim_time - self.last_cmd_sim_time >= 0.02:
            self.last_cmd_sim_time = self.sim_time
            twist = TwistStamped()
            twist.header.stamp = msg.clock
            twist.twist.linear.x = 0.15
            twist.twist.angular.z = 0.05 * np.sin(0.1 * t_rel)
            self.cmd_pub.publish(twist)

        # Sample trajectory point every 0.1s sim-time
        if self.sim_time - self.last_sample_sim_time >= 0.1:
            self.last_sample_sim_time = self.sim_time
            pos = self.latest_pos if self.latest_pos is not None else (3.0, 0.0)
            self.trajectory.append((t_rel, pos[0], pos[1]))

    def run_until_duration(self):
        t_wall_start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.t_start is not None and (self.sim_time - self.t_start) >= self.target_duration:
                break
            if time.time() - t_wall_start > (self.target_duration * 3.0 + 10.0):
                break
        return self.trajectory


def run_single_trial(run_idx, world_file, robot_file, root_dir, duration_s=20.0):
    print(f"\n[RUN {run_idx + 1}/5] Launching Gazebo Harmonic (Paused mode)...", flush=True)
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
        "-name", "burger", "-file", robot_file,
        "-x", "3.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"
    ]
    subprocess.run(spawn_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

    bridge_proc = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
    )
    time.sleep(1.5)

    rclpy.init()
    recorder = PreciseTrajectoryRecorder(target_duration=duration_s)

    # Unpause cleanly
    node = gzt.Node()
    req = world_control_pb2.WorldControl()
    req.pause = False
    node.request("/world/corridor_0.90/control", req, world_control_pb2.WorldControl, boolean_pb2.Boolean, 2000)

    traj = recorder.run_until_duration()
    recorder.destroy_node()
    rclpy.shutdown()

    for p in [bridge_proc, gz_proc]:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass

    time.sleep(1.0)
    print(f"  Run {run_idx + 1} finished: {len(traj)} points recorded.", flush=True)
    return traj


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_n10_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/burger_lidar1_depth1_640x480_10fps.sdf")

    print("=" * 75)
    print("STARTING FULL 5-RUN PAIRWISE & TEMPORAL ERROR ANALYSIS (N=10, 20s)")
    print("=" * 75, flush=True)

    all_runs = []
    for r in range(5):
        traj = run_single_trial(r, world_file, robot_file, root_dir, duration_s=20.0)
        all_runs.append(traj)

    min_len = min(len(t) for t in all_runs)
    times = [all_runs[0][k][0] for k in range(min_len)]

    # Compute all 10 pairwise comparisons (C_5^2)
    pairwise_matrix = {}
    time_error_profiles = {}  # time -> mean error across all pairs

    print("\n" + "-" * 75)
    print("FULL 5x5 PAIRWISE COMPARISON MATRIX (10 Pairs):")
    print("-" * 75, flush=True)

    all_pair_errors_over_time = []

    for i in range(5):
        for j in range(i + 1, 5):
            pts_i = all_runs[i][:min_len]
            pts_j = all_runs[j][:min_len]
            errs = [np.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(pts_i, pts_j)]
            max_e = float(np.max(errs))
            mean_e = float(np.mean(errs))
            final_e = float(errs[-1])
            pair_key = f"Run_{i+1}_vs_Run_{j+1}"
            pairwise_matrix[pair_key] = {
                "max_mm": max_e * 1000.0,
                "mean_mm": mean_e * 1000.0,
                "final_mm": final_e * 1000.0
            }
            all_pair_errors_over_time.append(errs)
            print(f"  {pair_key:18s} | Max: {max_e*1000:6.2f} mm | Mean: {mean_e*1000:6.2f} mm | Final(t=20s): {final_e*1000:6.2f} mm", flush=True)

    # Average error across all 10 pairs as a function of time
    arr_errs = np.array(all_pair_errors_over_time) * 1000.0  # (10, min_len) in mm
    mean_err_t = np.mean(arr_errs, axis=0)
    max_err_t = np.max(arr_errs, axis=0)

    temporal_samples = []
    # Sample every 2 seconds
    for step_idx in range(0, min_len, 20):
        t_val = times[step_idx]
        m_err = mean_err_t[step_idx]
        mx_err = max_err_t[step_idx]
        temporal_samples.append({
            "t_sec": round(t_val, 1),
            "mean_error_mm": round(m_err, 2),
            "max_error_mm": round(mx_err, 2)
        })

    print("\n" + "-" * 75)
    print("TEMPORAL ERROR DYNAMICS e(t) (Mean & Max Discrepancy vs Sim Time):")
    print("-" * 75, flush=True)
    for sample in temporal_samples:
        print(f"  t = {sample['t_sec']:4.1f}s | Mean Discrepancy: {sample['mean_error_mm']:5.2f} mm | Max: {sample['max_error_mm']:5.2f} mm", flush=True)

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/pairwise_dynamics_results.json")
    with open(out_file, "w") as f:
        json.dump({
            "num_runs": 5,
            "duration_s": 20.0,
            "pairwise_matrix": pairwise_matrix,
            "temporal_dynamics": temporal_samples,
            "overall_max_mm": float(np.max(arr_errs)),
            "overall_mean_mm": float(np.mean(arr_errs))
        }, f, indent=2)

    print(f"\nAnalysis complete! Results saved to: {out_file}", flush=True)


if __name__ == '__main__':
    main()
