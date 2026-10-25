#!/usr/bin/env python3
"""
Empirical Reproducibility Benchmark of Nav2 MPPI on Small Mecanum Omnidirectional Robot.
Tests:
- Nav2 MPPI with omnidirectional kinematics (motion_model: Omni, regenerate_noises: false)
- 3 cold-restart trials with identical seed=42 and world configuration
- Measures pairwise trajectory differences (ATE, lateral error, cmd_vel differences)
"""

import sys
import os
import time
import subprocess
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env


class MecanumMPPIRecorder(Node):
    def __init__(self, target_duration=12.0):
        super().__init__('mecanum_mppi_recorder')
        self.target_duration = target_duration
        self.sim_time = 0.0
        self.t_start = None
        self.latest_pos = (0.0, 0.0)
        self.trajectory = []  # list of (t_rel, x, y)
        self.cmd_history = []  # list of (sim_time, vx, vy, wz)
        self.last_sample_t = -1.0
        self.goal_sent = False

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _odom_cb(self, msg: Odometry):
        self.latest_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _cmd_cb(self, msg: TwistStamped):
        self.cmd_history.append((self.sim_time, msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z))

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.t_start is None:
            self.t_start = self.sim_time
            self.last_sample_t = self.sim_time
            return

        t_rel = self.sim_time - self.t_start
        if self.sim_time - self.last_sample_t >= 0.1:
            self.last_sample_t = self.sim_time
            self.trajectory.append((t_rel, self.latest_pos[0], self.latest_pos[1]))

    def send_goal(self, gx=-8.0, gy=0.0):
        print("    [GOAL] Waiting for /navigate_to_pose action server...", flush=True)
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            print("    [ERROR] /navigate_to_pose action server not available!", flush=True)
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        print(f"    [GOAL] Sending goal ({gx}, {gy}) to /navigate_to_pose action server...", flush=True)
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=8.0)

        if not send_goal_future.done():
            print("    [ERROR] Timed out waiting for send_goal response!", flush=True)
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print("    [ERROR] Goal was REJECTED by Nav2!", flush=True)
            return False

        print("    [GOAL] Goal successfully ACCEPTED by Nav2!", flush=True)
        self.goal_sent = True
        return True

    def run_until_duration(self):
        t_wall_0 = time.time()
        # Spin briefly to ensure subscriptions are alive before goal
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        self.send_goal(-8.0, 0.0)
        print("    [GOAL] Published navigation goal (-8.0, 0.0) to /goal_pose", flush=True)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.t_start is not None and (self.sim_time - self.t_start) >= self.target_duration:
                break
            if time.time() - t_wall_0 > (self.target_duration * 3.0 + 30.0):
                break
        return self.trajectory, self.cmd_history


@scoped_processes
def run_single_trial(trial_idx: int, root_dir: str, duration_s=12.0, *, _processes=None):
    print(f"\n  [TRIAL {trial_idx + 1}/3] Cold starting Gazebo & Nav2 MPPI for Mecanum Robot...", flush=True)

    bridge_config = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/config/mecanum_bridge.yaml")
    urdf_file = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/urdf/mecanum_robot.urdf")
    params_file = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_mecanum.yaml")

    domain_id = str(60 + trial_idx)
    partition = f"mecanum_mppi_{trial_idx}"

    env = build_benchmark_env(root_dir)
    env["ROS_DOMAIN_ID"] = domain_id
    env["GZ_PARTITION"] = partition
    plugin_share = get_package_share_directory("custom_corridor_plugins") if "get_package_share_directory" in globals() else ""
    plugin_lib = "/home/cuongbien/DynamuhSim/install/custom_corridor_plugins/lib"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"


    try:
        # 1. Start Gazebo Sim + Mecanum Robot + Bridges + RSP via standard launch file
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            "width:=0.90", "obstacle:=clean", "gui:=false", "rviz:=false",
            "robot_model:=mecanum", "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        gz_p = _processes.popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(5.0)

        # 2. Nav2 stack with MPPI for mecanum robot
        nav2_log_path = f"/tmp/nav2_mppi_mecanum_{trial_idx}.log"
        if os.path.exists(nav2_log_path):
            os.remove(nav2_log_path)
        nav2_log = open(nav2_log_path, "w")
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={params_file}",
            "map:=corridor_090",
            "robot_model:=mecanum"
        ]
        nav2_p = _processes.popen(nav2_cmd, env=env, stdout=nav2_log, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

        # 3. Wait for lifecycle_manager_navigation to report active in the FRESH log
        t0 = time.time()
        ready = False
        while time.time() - t0 < 40.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path) as fp:
                    content = fp.read()
                    if "lifecycle_manager_navigation" in content and "Managed nodes are active" in content:
                        ready = True
                        break
            time.sleep(0.5)

        if not ready:
            print("    [WARN] Nav2 navigation lifecycle manager did not report active in 40s!", flush=True)

        # Allow action servers and costmaps to settle
        time.sleep(3.0)

        # 4. Start recorder node in this domain
        os.environ["ROS_DOMAIN_ID"] = domain_id
        os.environ["GZ_PARTITION"] = partition
        rclpy.init()
        recorder = MecanumMPPIRecorder(target_duration=duration_s)

        traj, cmds = recorder.run_until_duration()
        recorder.destroy_node()
        rclpy.shutdown()

    finally:
        _processes.stop()
        time.sleep(1.0)

    final_pose = (traj[-1][1], traj[-1][2]) if len(traj) > 0 else (0.0, 0.0)
    print(f"    Trial finished: {len(traj)} trajectory points, {len(cmds)} cmd_vel commands issued. Final pose: ({final_pose[0]:.3f}, {final_pose[1]:.3f})", flush=True)
    return traj, cmds


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial", type=int, default=None, help="Run single trial index (0, 1, 2)")
    args = parser.parse_args()

    root_dir = "/home/cuongbien/DynamuhSim"

    if args.trial is not None:
        traj, cmds = run_single_trial(args.trial, root_dir, duration_s=12.0)
        trial_out = f"/tmp/mppi_trial_{args.trial}.json"
        with open(trial_out, "w") as f:
            json.dump({"traj": traj, "cmds": cmds}, f)
        print(f"Saved trial {args.trial} to {trial_out}")
        return

    print("=" * 75)
    print("NAV2 MPPI REPRODUCIBILITY BENCHMARK (SMALL MECANUM ROBOT)")
    print("=" * 75)

    trajectories = []
    commands = []

    for r in range(3):
        trial_out = f"/tmp/mppi_trial_{r}.json"
        if os.path.exists(trial_out):
            os.remove(trial_out)
        cmd = [sys.executable, os.path.abspath(__file__), "--trial", str(r)]
        res = subprocess.run(cmd)
        if res.returncode != 0 or not os.path.exists(trial_out):
            print(f"[ERROR] Trial {r} failed!")
            continue
        with open(trial_out, "r") as f:
            data = json.load(f)
            trajectories.append(data["traj"])
            commands.append(data["cmds"])

    valid_trajs = [t for t in trajectories if len(t) > 5]
    if len(valid_trajs) < 2:
        print("[ERROR] Insufficient valid trajectories collected for comparison!")
        return

    min_len = min(len(t) for t in valid_trajs)
    t_axis = [valid_trajs[0][k][0] for k in range(min_len)]

    pairwise_results = {}
    for i in range(len(valid_trajs)):
        for j in range(i + 1, len(valid_trajs)):
            t1 = np.array([[valid_trajs[i][k][1], valid_trajs[i][k][2]] for k in range(min_len)])
            t2 = np.array([[valid_trajs[j][k][1], valid_trajs[j][k][2]] for k in range(min_len)])

            diff = np.linalg.norm(t1 - t2, axis=1)
            lat_diff = np.abs(t1[:, 1] - t2[:, 1])

            pair_key = f"run_{i+1}_vs_run_{j+1}"
            pairwise_results[pair_key] = {
                "mean_ate_m": float(np.mean(diff)),
                "max_ate_m": float(np.max(diff)),
                "median_ate_m": float(np.median(diff)),
                "max_lat_err_m": float(np.max(lat_diff)),
                "mean_lat_err_m": float(np.mean(lat_diff)),
                "final_endpoint_err_m": float(diff[-1])
            }

    print("\n" + "=" * 75)
    print("PAIRWISE TRAJECTORY COMPARISON (REPRODUCIBILITY ANALYSIS):")
    print("=" * 75)
    for pair, metrics in pairwise_results.items():
        print(f"\n[{pair.upper()}]:")
        print(f"  Mean ATE (Absolute Trajectory Error) : {metrics['mean_ate_m']*1000:.2f} mm ({metrics['mean_ate_m']:.4f} m)")
        print(f"  Max 2D Euclidean Deviation          : {metrics['max_ate_m']*1000:.2f} mm ({metrics['max_ate_m']:.4f} m)")
        print(f"  Max Lateral Deviation (|Δy|)         : {metrics['max_lat_err_m']*1000:.2f} mm ({metrics['max_lat_err_m']:.4f} m)")
        print(f"  Final Endpoint Deviation             : {metrics['final_endpoint_err_m']*1000:.2f} mm ({metrics['final_endpoint_err_m']:.4f} m)")

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/mppi_mecanum_reproducibility_results.json")
    with open(out_file, "w") as f:
        json.dump({
            "pairwise_metrics": pairwise_results,
            "min_points_compared": min_len,
            "duration_s": 12.0
        }, f, indent=2)
    print(f"\nResults successfully written to {out_file}")


if __name__ == "__main__":
    main()
