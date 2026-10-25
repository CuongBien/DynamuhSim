#!/usr/bin/env python3
"""
A3.3: Nav2 MPPI Goal Navigation Benchmark with N=5 Humans in Corridor 0.90m.
Protocol:
- 10 cold-restart runs in corridor_090
- N = 5 humans distributed along the 5.0m navigation path
- Mecanum robot with Nav2 MPPI Omni controller
- Spawn at (-13.0, 0.0), Goal at (-8.0, 0.0)
- Evaluate: Success rate (reach within 0.25m), collision rate (d < 0.44m), deadlock rate
- Target: Success rate >= 9/10 (90%)
- Records and exports cmd_vel from Run 1 for A3.4 Replay Benchmark
"""

import os
from pathlib import Path
import sys
import time
import signal
import subprocess
import json
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

import gz.transport13 as gzt
import gz.msgs.stringmsg_pb2 as stringmsg_pb2

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env


class Nav2GoalTester(Node):
    def __init__(self, target_gx=-8.0, target_gy=0.0, max_duration_s=25.0):
        super().__init__('nav2_goal_tester')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_gx = target_gx
        self.target_gy = target_gy
        self.max_duration_s = max_duration_s

        self.sim_time = 0.0
        self.t_start = None
        self.robot_pos = (-13.0, 0.0)
        self.robot_vel = (0.0, 0.0)
        self.humans_pos = {}  # id -> (x, y)
        self.trajectory = []  # list of (t_rel, rx, ry, min_h_dist, min_wall_dist)
        self.cmd_history = []  # list of (sim_time, vx, vy, w)
        self.goal_sent = False
        self.goal_reached = False
        self.collision_detected = False
        self.deadlock_detected = False
        self.last_moving_time = None

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.gz_node = gzt.Node()
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, '/rvo2_human/ground_truth', self._gt_cb)

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _cmd_cb(self, msg: TwistStamped):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        w = msg.twist.angular.z
        self.cmd_history.append((self.sim_time, vx, vy, w))

    def _odom_cb(self, msg: Odometry):
        if self.robot_pos == (-13.0, 0.0):
            self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _gt_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.robot_pos = (data['robot']['x'], data['robot']['y'])
                self.robot_vel = (data['robot'].get('vx', 0.0), data['robot'].get('vy', 0.0))

            if 'humans' in data:
                for h in data['humans']:
                    hid = h.get('id', 0)
                    hx = h.get('x', None)
                    hy = h.get('y', None)
                    if hx is not None and hy is not None:
                        self.humans_pos[hid] = (hx, hy)
        except Exception:
            pass

    def send_goal(self):
        print("    [NAV2] Waiting for /navigate_to_pose action server...", flush=True)
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            print("    [ERROR] /navigate_to_pose action server not available!", flush=True)
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target_gx)
        goal_msg.pose.pose.position.y = float(self.target_gy)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        for attempt in range(5):
            print(f"    [NAV2] Sending goal ({self.target_gx}, {self.target_gy}) (attempt {attempt+1})...", flush=True)
            send_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=6.0)

            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle.accepted:
                    print("    [NAV2] Goal successfully ACCEPTED by Nav2!", flush=True)
                    self.goal_sent = True
                    return True
                else:
                    print("    [WARN] Goal was rejected by Nav2. Retrying in 1s...", flush=True)
            time.sleep(1.0)

        print("    [ERROR] All attempts to send goal failed!", flush=True)
        return False

    def execute_navigation(self):
        # Spin to establish subscriptions
        t_spin0 = time.time()
        while time.time() - t_spin0 < 2.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.send_goal():
            return {
                "success": False,
                "reason": "goal_rejected_or_server_down",
                "trajectory": [],
                "cmd_history": []
            }

        self.t_start = self.sim_time
        self.last_moving_time = self.sim_time
        t_wall_start = time.time()
        last_sample_t = -1.0

        min_human_dist_overall = 999.0
        min_wall_dist_overall = 999.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            t_rel = self.sim_time - self.t_start

            # Distance to goal
            dist_to_goal = np.hypot(self.robot_pos[0] - self.target_gx, self.robot_pos[1] - self.target_gy)

            # Check min distance to humans
            cur_min_h_dist = 999.0
            for hid, (hx, hy) in self.humans_pos.items():
                d = np.hypot(self.robot_pos[0] - hx, self.robot_pos[1] - hy)
                if d < cur_min_h_dist:
                    cur_min_h_dist = d
            if cur_min_h_dist < min_human_dist_overall:
                min_human_dist_overall = cur_min_h_dist

            # Corridor side wall distance (half width = 0.45m)
            wall_dist = 0.45 - abs(self.robot_pos[1])
            if wall_dist < min_wall_dist_overall:
                min_wall_dist_overall = wall_dist

            # Physical collision check: d_min < r_robot(0.16) + r_human(0.28) = 0.44m
            if cur_min_h_dist < 0.44:
                self.collision_detected = True

            # Wall collision check: robot edge penetrates wall
            if wall_dist < 0.08:
                self.collision_detected = True

            # Sample trajectory at 10 Hz
            if self.sim_time - last_sample_t >= 0.1:
                last_sample_t = self.sim_time
                self.trajectory.append((round(t_rel, 3), round(self.robot_pos[0], 3), round(self.robot_pos[1], 3), round(cur_min_h_dist, 3), round(wall_dist, 3)))

            # Check if goal reached
            if dist_to_goal <= 0.25:
                print(f"    [SUCCESS] Goal reached at t_sim = {t_rel:.2f}s! Dist to goal: {dist_to_goal:.3f}m", flush=True)
                self.goal_reached = True
                break

            # Speed check for deadlock
            speed = np.hypot(self.robot_vel[0], self.robot_vel[1])
            if speed > 0.03:
                self.last_moving_time = self.sim_time
            elif (self.sim_time - self.last_moving_time) > 10.0 and dist_to_goal > 0.5:
                print(f"    [DEADLOCK] Robot stationary for > 10s at dist {dist_to_goal:.2f}m!", flush=True)
                self.deadlock_detected = True
                break

            # Timeout check
            if t_rel >= self.max_duration_s:
                print(f"    [TIMEOUT] Reached max duration {self.max_duration_s}s. Dist to goal: {dist_to_goal:.3f}m", flush=True)
                break

            # Hard wall-clock timeout
            if time.time() - t_wall_start > (self.max_duration_s * 3.5 + 25.0):
                print("    [TIMEOUT] Hard wall timeout exceeded!", flush=True)
                break

        time_taken = self.sim_time - self.t_start if self.t_start else 0.0
        final_dist_to_goal = np.hypot(self.robot_pos[0] - self.target_gx, self.robot_pos[1] - self.target_gy)

        return {
            "success": self.goal_reached and not self.collision_detected,
            "goal_reached": self.goal_reached,
            "collision_detected": self.collision_detected,
            "deadlock_detected": self.deadlock_detected,
            "time_taken_s": round(time_taken, 2),
            "final_pos": [round(self.robot_pos[0], 3), round(self.robot_pos[1], 3)],
            "final_dist_to_goal_m": round(final_dist_to_goal, 3),
            "min_human_dist_m": round(min_human_dist_overall, 3) if min_human_dist_overall < 900 else None,
            "min_wall_dist_m": round(min_wall_dist_overall, 3),
            "trajectory": self.trajectory,
            "cmd_history": self.cmd_history
        }


def run_single_trial(run_idx: int, root_dir: str):
    print(f"\n========================================================")
    print(f">>> [RUN {run_idx + 1}/10] Testing Nav2 MPPI Goal Navigation...")
    print(f"========================================================")

    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_n5_actor_and_cylinder.sdf")
    params_file = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_mecanum.yaml")
    env = build_benchmark_env(root_dir)

    domain_id = str(60 + run_idx)
    env["ROS_DOMAIN_ID"] = domain_id
    partition = f"nav2_a3_3_{run_idx}"
    env["GZ_PARTITION"] = partition

    all_procs = []
    nav2_log_path = f"/tmp/nav2_a3_3_run_{run_idx}.log"
    if os.path.exists(nav2_log_path):
        os.remove(nav2_log_path)

    try:
        # 1. Start Gazebo Sim + Mecanum Robot + Bridges + RSP via standard launch file
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            "width:=0.90",
            f"custom_world:={world_file}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        gz_p = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(gz_p)
        time.sleep(5.0)

        # 2. Launch Nav2 stack
        nav2_log_file = open(nav2_log_path, "w")
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={params_file}",
            "map:=corridor_090",
            "robot_model:=mecanum"
        ]
        nav2_p = subprocess.Popen(nav2_cmd, env=env, stdout=nav2_log_file, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
        all_procs.append(nav2_p)

        # 3. Wait for Nav2 lifecycle manager to become active
        t0 = time.time()
        nav2_ready = False
        while time.time() - t0 < 45.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path) as fp:
                    content = fp.read()
                    if "lifecycle_manager_navigation" in content and "Managed nodes are active" in content:
                        nav2_ready = True
                        print(f"  Nav2 lifecycle manager became active in {time.time()-t0:.1f}s!", flush=True)
                        break
            time.sleep(0.5)

        if not nav2_ready:
            print("  [ERROR] Nav2 failed to activate within 45s!", flush=True)
            return {"success": False, "reason": "nav2_activation_timeout"}

        # Allow action servers and costmaps to settle
        time.sleep(3.5)

        # 4. Execute goal navigation
        os.environ["ROS_DOMAIN_ID"] = domain_id
        os.environ["GZ_PARTITION"] = partition
        rclpy.init()
        tester = Nav2GoalTester(target_gx=-8.0, target_gy=0.0, max_duration_s=25.0)
        result = tester.execute_navigation()
        tester.destroy_node()
        rclpy.shutdown()

    finally:
        for p in all_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.3)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1.5)

    trial_out = f"/tmp/nav2_a3_3_run_{run_idx}_result.json"
    with open(trial_out, "w") as fp:
        json.dump(result, fp, indent=2)

    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=int, default=None, help="Run single index (0-9)")
    args = parser.parse_args()

    root_dir = "/home/cuongbien/DynamuhSim"

    if args.run is not None:
        run_single_trial(args.run, root_dir)
        return

    print("=" * 80)
    print("A3.3: NAV2 MPPI GOAL NAVIGATION BENCHMARK (10 RUNS, N=5 HUMANS, 5m CORRIDOR)")
    print("=" * 80)

    results = []
    success_count = 0
    collision_count = 0
    deadlock_count = 0

    first_run_cmdvel = None
    first_run_traj = None

    for r in range(10):
        trial_out = f"/tmp/nav2_a3_3_run_{r}_result.json"
        if os.path.exists(trial_out):
            os.remove(trial_out)

        cmd = [sys.executable, os.path.abspath(__file__), "--run", str(r)]
        sub_res = subprocess.run(cmd)

        if sub_res.returncode != 0 or not os.path.exists(trial_out):
            print(f"[ERROR] Run {r+1} failed to execute or produce result file!")
            results.append({"success": False, "reason": "subprocess_failed"})
            continue

        with open(trial_out, "r") as fp:
            res = json.load(fp)

        results.append(res)
        if res.get("success", False):
            success_count += 1
        if res.get("collision_detected", False):
            collision_count += 1
        if res.get("deadlock_detected", False):
            deadlock_count += 1

        if r == 0 and "cmd_history" in res and len(res["cmd_history"]) > 0:
            first_run_cmdvel = res["cmd_history"]
            first_run_traj = res.get("trajectory", [])

        print(f"  Run {r+1}/10: Success={res.get('success')}, Reached={res.get('goal_reached')}, "
              f"Collision={res.get('collision_detected')}, Time={res.get('time_taken_s')}s, "
              f"d_min_human={res.get('min_human_dist_m')}m")

    success_rate = success_count / 10.0
    collision_rate = collision_count / 10.0
    deadlock_rate = deadlock_count / 10.0
    passed = (success_rate >= 0.90)

    summary = {
        "benchmark": "A3.3_nav2_goal_navigation",
        "num_runs": 10,
        "success_count": success_count,
        "success_rate": success_rate,
        "collision_count": collision_count,
        "collision_rate": collision_rate,
        "deadlock_count": deadlock_count,
        "deadlock_rate": deadlock_rate,
        "passed": passed,
        "runs": results
    }

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/a3_3_nav2_goal_navigation_results.json")
    with open(out_file, "w") as fp:
        json.dump(summary, fp, indent=2)

    # Save Run 1 cmd_vel for A3.4 replay
    if first_run_cmdvel is not None:
        replay_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/a3_4_recorded_cmdvel.json")
        with open(replay_file, "w") as fp:
            json.dump({
                "source": "A3.3_Run_1",
                "trajectory_original": first_run_traj,
                "cmd_vel_history": first_run_cmdvel
            }, fp, indent=2)
        print(f"\nSaved Run 1 cmd_vel ({len(first_run_cmdvel)} commands) to {replay_file} for A3.4 Replay.")

    print("\n" + "=" * 80)
    print("A3.3 BENCHMARK RESULTS SUMMARY")
    print("=" * 80)
    print(f"  Total Runs     : 10")
    print(f"  Success Rate   : {success_rate * 100:.1f}% ({success_count}/10)")
    print(f"  Collision Rate : {collision_rate * 100:.1f}% ({collision_count}/10)")
    print(f"  Deadlock Rate  : {deadlock_rate * 100:.1f}% ({deadlock_count}/10)")
    print(f"  Criterion      : Success Rate >= 90% -> {'PASSED' if passed else 'FAILED'}")
    print(f"  Report saved to: {out_file}")
    print("=" * 80)


if __name__ == "__main__":
    main()
