#!/usr/bin/env python3
"""
A3.4: MPPI cmd_vel Replay Benchmark with Real Head-on Interaction (Seed 101).

Protocol:
1. Records or loads a real head-on interaction episode in corridor_090 with H1 (Seed 101).
   - Ground truth robot pose recorded from /rvo2_human/ground_truth
   - Execution cmd_vel recorded with Nav2 sim-time stamps (header.stamp)
   - Synchronized reference T0 set to the exact sim-time of the first motion command
2. Executes 5 independent cold-restart replay trials in Gazebo Harmonic.
   - Dispatches recorded cmd_vel sequence synchronized to sim-time relative to T0
   - Records replayed ground-truth trajectory waypoints
   - Evaluates time-aligned trajectory deviation vs original run
3. Success Criteria:
   - Mean trajectory deviation < 10.0 mm (1.0 cm)
   - Max trajectory deviation < 20.0 mm (2.0 cm)
4. Outputs:
   - Raw JSON: experiments/gazebo_benchmark/results/correctness_a3/a3_4_replay_results.json
   - Recorded cmd_vel: experiments/gazebo_benchmark/results/correctness_a3/a3_4_recorded_cmdvel.json
"""

import os
from pathlib import Path
import sys
import time
import subprocess
import json
import re
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes


class InteractionRecorder(Node):
    def __init__(self, target_gx=-7.0, target_gy=0.0):
        super().__init__('interaction_recorder')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_gx = target_gx
        self.target_gy = target_gy

        self.sim_time = 0.0
        self.t_goal_accepted = None
        self.t_first_cmd = None
        self.robot_pos = (-13.0, 0.0)
        self.robot_vel = (0.0, 0.0)
        self.human_pos = None

        self.recorded_cmds = []
        self.recorded_gt = []
        self.goal_reached = False

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 100)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _cmd_cb(self, msg: TwistStamped):
        stamp_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        w = msg.twist.angular.z
        if self.t_first_cmd is None and (abs(vx) > 0.001 or abs(vy) > 0.001 or abs(w) > 0.001):
            self.t_first_cmd = stamp_t
        self.recorded_cmds.append({
            "sim_time": round(self.sim_time, 4),
            "stamp_time": round(stamp_t, 4),
            "vx": float(vx),
            "vy": float(vy),
            "w": float(w)
        })

    def _gt_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            t_gt = data.get("timestamp", self.sim_time)
            if 'robot' in data and data['robot']['x'] is not None:
                rx = data['robot']['x']
                ry = data['robot']['y']
                rvx = data['robot'].get('vx', 0.0)
                rvy = data['robot'].get('vy', 0.0)
                self.robot_pos = (rx, ry)
                self.robot_vel = (rvx, rvy)
                if self.t_goal_accepted is not None:
                    self.recorded_gt.append({
                        "t_sim": round(t_gt, 4),
                        "rx": round(rx, 4),
                        "ry": round(ry, 4),
                        "vx": round(rvx, 4),
                        "vy": round(rvy, 4)
                    })
            if 'humans' in data and len(data['humans']) > 0:
                h = data['humans'][0]
                self.human_pos = (h.get('x'), h.get('y'))
        except Exception:
            pass

    def send_goal(self):
        print("    [RECORDER] Waiting for Nav2 action server...", flush=True)
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            print("    [ERROR] /navigate_to_pose action server not available!", flush=True)
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target_gx)
        goal_msg.pose.pose.position.y = float(self.target_gy)
        goal_msg.pose.pose.orientation.w = 1.0

        for attempt in range(5):
            print(f"    [RECORDER] Sending goal attempt {attempt+1}...", flush=True)
            send_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=6.0)
            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle.accepted:
                    print("    [RECORDER] Goal accepted by Nav2!", flush=True)
                    self.t_goal_accepted = self.sim_time
                    return True
            time.sleep(1.0)
        return False

    def run(self, max_duration=25.0):
        t_w0 = time.time()
        while time.time() - t_w0 < 2.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.send_goal():
            return False

        t_start_sim = self.sim_time
        t_wall_0 = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            t_rel = self.sim_time - t_start_sim
            dist = np.hypot(self.robot_pos[0] - self.target_gx, self.robot_pos[1] - self.target_gy)

            if dist <= 0.25:
                print(f"    [RECORDER] Goal reached at t_sim = {t_rel:.2f}s!", flush=True)
                self.goal_reached = True
                break

            if t_rel >= max_duration or time.time() - t_wall_0 > (max_duration * 3.0 + 20.0):
                print(f"    [RECORDER] Episode ended at t_sim = {t_rel:.2f}s.", flush=True)
                break

        time.sleep(0.5)
        return True


class PrecisionReplayer(Node):
    def __init__(self, recorded_cmds, t0=None, duration_s=15.0):
        super().__init__('precision_replayer')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.duration_s = duration_s

        if t0 is None:
            t0 = recorded_cmds[0]["stamp_time"]
        self.cmds = []
        for c in recorded_cmds:
            if c["stamp_time"] >= t0:
                rel_t = max(0.0, c["stamp_time"] - t0)
                self.cmds.append((rel_t, c["vx"], c["vy"], c["w"]))

        self.sim_time = 0.0
        self.t_start = None
        self.cmd_idx = 0
        self.last_pub_sim_time = -1.0
        self.replayed_gt = []
        self.dispatched_cmds_log = []

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 50)

    def _gt_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            t_gt = data.get("timestamp", self.sim_time)
            if 'robot' in data and data['robot']['x'] is not None and self.t_start is not None:
                rx = data['robot']['x']
                ry = data['robot']['y']
                rvx = data['robot'].get('vx', 0.0)
                rvy = data['robot'].get('vy', 0.0)
                self.replayed_gt.append({
                    "t_rel": round(t_gt - self.t_start, 4),
                    "rx": round(rx, 4),
                    "ry": round(ry, 4),
                    "vx": round(rvx, 4),
                    "vy": round(rvy, 4)
                })
        except Exception:
            pass

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.t_start is None:
            self.t_start = self.sim_time
            self.last_pub_sim_time = self.sim_time

        t_rel = self.sim_time - self.t_start

        # Advance scheduled command
        prev_idx = self.cmd_idx
        while self.cmd_idx < len(self.cmds) - 1 and self.cmds[self.cmd_idx + 1][0] <= t_rel:
            self.cmd_idx += 1

        # Dispatch cmd_vel at 50 Hz sim-time or upon index advancement
        if (self.sim_time - self.last_pub_sim_time >= 0.02) or (self.cmd_idx != prev_idx):
            self.last_pub_sim_time = self.sim_time
            sched_t, vx, vy, w = self.cmds[self.cmd_idx]
            twist = TwistStamped()
            twist.header.stamp = msg.clock
            twist.header.frame_id = 'base_footprint'
            twist.twist.linear.x = float(vx)
            twist.twist.linear.y = float(vy)
            twist.twist.angular.z = float(w)
            self.cmd_pub.publish(twist)

            if self.cmd_idx != prev_idx:
                self.dispatched_cmds_log.append({
                    "cmd_idx": self.cmd_idx,
                    "sched_rel_t": round(sched_t, 4),
                    "actual_rel_t": round(t_rel, 4),
                    "dispatch_delay_ms": round((t_rel - sched_t) * 1000.0, 2),
                    "vx": round(vx, 4),
                    "vy": round(vy, 4),
                    "w": round(w, 4)
                })

    def run(self):
        t_wall_0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.t_start is not None and (self.sim_time - self.t_start) >= self.duration_s:
                break
            if time.time() - t_wall_0 > (self.duration_s * 3.0 + 20.0):
                break
        return self.replayed_gt, self.dispatched_cmds_log


@scoped_processes
def record_fresh_episode(root_dir: str, seed: int = 101, *, _processes=None) -> dict:
    print(f"\n>>> Recording fresh MPPI interaction episode (corridor_090, H1, Seed {seed})...")
    gen_dir = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated")
    os.makedirs(gen_dir, exist_ok=True)

    v_human = 1.155
    scen_file = os.path.join(gen_dir, f"scenario_a3_4_record_{seed}.json")
    scen_data = {
        "episode_id": seed,
        "corridor_mode": True,
        "humans": [
            {
                "id": 1,
                "name": "human_01",
                "mode": "autonomous",
                "speed": v_human,
                "max_speed": v_human,
                "persona": "H1",
                "loop": False,
                "wait_for_robot": True,
                "start": {"x": -7.0, "y": 0.0, "yaw": 3.14159265},
                "waypoints": [[-7.0, 0.0], [-15.0, 0.0]]
            }
        ]
    }
    with open(scen_file, "w") as f:
        json.dump(scen_data, f, indent=2)

    base_sdf_path = os.path.join(root_dir, "src/custom_corridor/worlds/corridor_090.sdf")
    with open(base_sdf_path, "r") as f:
        sdf = f.read()

    sdf = re.sub(r'<real_time_factor>.*?</real_time_factor>', '<real_time_factor>0.0</real_time_factor>', sdf)
    sdf = re.sub(
        r'<randomize_politeness>.*?</randomize_politeness>',
        f'<randomize_politeness>false</randomize_politeness>\n      <seed>{seed}</seed>\n      <default_scenario_file>{scen_file}</default_scenario_file>',
        sdf
    )
    if f"<seed>{seed}</seed>" not in sdf:
        sdf = re.sub(r'<seed>\d+</seed>', f'<seed>{seed}</seed>', sdf)

    sdf = re.sub(r'<actor name=\"human_01_visual\">\s*<pose>.*?</pose>', '<actor name=\"human_01_visual\">\n      <pose>-7.0 0.0 0.0 0 0 3.14159</pose>', sdf)
    sdf = re.sub(r'<model name=\"human_01_proxy\">\s*<pose>.*?</pose>', '<model name=\"human_01_proxy\">\n      <pose>-7.0 0.0 0.80 0 0 3.14159</pose>', sdf)

    trial_sdf = os.path.join(gen_dir, f"world_a3_4_record_{seed}.sdf")
    with open(trial_sdf, "w") as f:
        f.write(sdf)

    env = build_benchmark_env(root_dir)

    domain_id = "77"
    env["ROS_DOMAIN_ID"] = domain_id
    partition = f"a3_4_record_{seed}"
    env["GZ_PARTITION"] = partition

    nav2_log_path = f"/tmp/nav2_a3_4_record_{seed}.log"
    if os.path.exists(nav2_log_path):
        os.remove(nav2_log_path)

    params_file = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_pilot.yaml")

    try:
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            "width:=0.90",
            f"custom_world:={trial_sdf}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        gz_p = _processes.popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(4.5)

        nav2_log_file = open(nav2_log_path, "w")
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={params_file}",
            "map:=corridor_090",
            "robot_model:=mecanum"
        ]
        nav2_p = _processes.popen(nav2_cmd, env=env, stdout=nav2_log_file, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

        t0 = time.time()
        nav2_ready = False
        while time.time() - t0 < 35.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path) as fp:
                    content = fp.read()
                    if "lifecycle_manager_navigation" in content and "Managed nodes are active" in content:
                        nav2_ready = True
                        print(f"    Nav2 became active in {time.time()-t0:.1f}s!", flush=True)
                        break
            time.sleep(0.4)

        if not nav2_ready:
            print("    [ERROR] Nav2 failed to activate!")
            return None

        time.sleep(1.5)

        os.environ["ROS_DOMAIN_ID"] = domain_id
        os.environ["GZ_PARTITION"] = partition
        rclpy.init()
        recorder = InteractionRecorder(target_gx=-7.0, target_gy=0.0)
        success = recorder.run(max_duration=25.0)

        data = {
            "source": f"corridor_090_H1_seed_{seed}",
            "t_goal_accepted": recorder.t_goal_accepted,
            "t_first_cmd": recorder.t_first_cmd,
            "planning_latency_s": round(recorder.t_first_cmd - recorder.t_goal_accepted, 4) if (recorder.t_first_cmd and recorder.t_goal_accepted) else 0.0,
            "recorded_cmds": recorder.recorded_cmds,
            "recorded_gt": recorder.recorded_gt
        }

        recorder.destroy_node()
        rclpy.shutdown()

        rec_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/a3_4_recorded_cmdvel.json")
        with open(rec_file, "w") as f:
            json.dump(data, f, indent=2)

        print(f"    Saved recorded episode to {rec_file}!")
        print(f"    t_goal_accepted: {recorder.t_goal_accepted:.3f}s")
        print(f"    t_first_cmd:     {recorder.t_first_cmd:.3f}s (Planning Latency: {data['planning_latency_s']:.3f}s)")
        print(f"    Commands count:  {len(recorder.recorded_cmds)}")
        print(f"    GT points count: {len(recorder.recorded_gt)}")
        return data

    finally:
        _processes.stop()
        time.sleep(1.5)


@scoped_processes
def run_single_replay_trial(root_dir: str, rec_data: dict, trial_idx: int, *, _processes=None) -> dict:
    seed = 101
    recorded_cmds = rec_data['recorded_cmds']
    t0 = rec_data['t_first_cmd']
    orig_gt = [pt for pt in rec_data['recorded_gt'] if pt['t_sim'] >= t0]
    duration_s = orig_gt[-1]['t_sim'] - t0

    gen_dir = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated")
    os.makedirs(gen_dir, exist_ok=True)
    trial_sdf = os.path.join(gen_dir, f"world_a3_4_record_{seed}.sdf")
    if not os.path.exists(trial_sdf):
        base_sdf_path = os.path.join(root_dir, "src/custom_corridor/worlds/corridor_090.sdf")
        with open(base_sdf_path, "r") as f:
            base_sdf = f.read()
        base_sdf = re.sub(r'<real_time_factor>.*?</real_time_factor>', '<real_time_factor>0.0</real_time_factor>', base_sdf)
        base_sdf = re.sub(r'<randomize_politeness>.*?</randomize_politeness>', f'<randomize_politeness>false</randomize_politeness>\n      <seed>{seed}</seed>', base_sdf)
        base_sdf = re.sub(r'<actor name=\"human_01_visual\">\s*<pose>.*?</pose>', '<actor name=\"human_01_visual\">\n      <pose>-7.0 0.0 0.0 0 0 3.14159</pose>', base_sdf)
        base_sdf = re.sub(r'<model name=\"human_01_proxy\">\s*<pose>.*?</pose>', '<model name=\"human_01_proxy\">\n      <pose>-7.0 0.0 0.80 0 0 3.14159</pose>', base_sdf)
        with open(trial_sdf, "w") as f:
            f.write(base_sdf)

    env = build_benchmark_env(root_dir)

    domain_id = str(71 + trial_idx)
    env["ROS_DOMAIN_ID"] = domain_id
    partition = f"a3_4_replay_{trial_idx}_{seed}"
    env["GZ_PARTITION"] = partition

    try:
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            "width:=0.90",
            f"custom_world:={trial_sdf}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        gz_p = _processes.popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(4.5)

        os.environ["ROS_DOMAIN_ID"] = domain_id
        os.environ["GZ_PARTITION"] = partition
        rclpy.init()
        replayer = PrecisionReplayer(recorded_cmds, t0=t0, duration_s=duration_s)
        replay_gt, dispatched_cmds_log = replayer.run()
        replayer.destroy_node()
        rclpy.shutdown()

    finally:
        _processes.stop()
        time.sleep(1.5)

    # Evaluate trajectory deviation against orig_gt
    t_eval = np.arange(0.0, duration_s, 0.1)

    orig_rel_t = np.array([pt['t_sim'] - t0 for pt in orig_gt])
    orig_rx = np.array([pt['rx'] for pt in orig_gt])
    orig_ry = np.array([pt['ry'] for pt in orig_gt])

    rep_rel_t = np.array([pt['t_rel'] for pt in replay_gt])
    rep_rx = np.array([pt['rx'] for pt in replay_gt])
    rep_ry = np.array([pt['ry'] for pt in replay_gt])

    ox_interp = np.interp(t_eval, orig_rel_t, orig_rx)
    oy_interp = np.interp(t_eval, orig_rel_t, orig_ry)

    rx_interp = np.interp(t_eval, rep_rel_t, rep_rx)
    ry_interp = np.interp(t_eval, rep_rel_t, rep_ry)

    dx_mm = (rx_interp - ox_interp) * 1000.0
    dy_mm = (ry_interp - oy_interp) * 1000.0
    dist_mm = np.hypot(dx_mm, dy_mm)

    time_series = []
    for k in range(0, len(t_eval), 10):  # sample every 1.0s
        time_series.append({
            "t_s": round(float(t_eval[k]), 1),
            "orig_x": round(float(ox_interp[k]), 4),
            "orig_y": round(float(oy_interp[k]), 4),
            "rep_x": round(float(rx_interp[k]), 4),
            "rep_y": round(float(ry_interp[k]), 4),
            "dx_longitudinal_mm": round(float(dx_mm[k]), 2),
            "dy_lateral_mm": round(float(dy_mm[k]), 2),
            "total_error_mm": round(float(dist_mm[k]), 2)
        })

    mean_err = float(np.mean(dist_mm))
    max_err = float(np.max(dist_mm))
    final_err = float(dist_mm[-1])
    max_lat_err = float(np.max(np.abs(dy_mm)))
    passed = (mean_err < 10.0 and max_err < 20.0)

    res = {
        "trial_id": trial_idx,
        "waypoints_evaluated": len(t_eval),
        "duration_s": round(duration_s, 2),
        "mean_error_mm": round(mean_err, 3),
        "max_error_mm": round(max_err, 3),
        "final_error_mm": round(final_err, 3),
        "max_lateral_error_mm": round(max_lat_err, 3),
        "passed": passed,
        "time_series_sample": time_series,
        "dispatched_cmds_sample": dispatched_cmds_log[:5]
    }
    return res


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    rec_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/a3_4_recorded_cmdvel.json")

    # Step 1: Ensure fresh recording exists from real H1 interaction (Seed 101)
    if not os.path.exists(rec_file):
        rec_data = record_fresh_episode(root_dir, seed=101)
    else:
        with open(rec_file, "r") as f:
            rec_data = json.load(f)
        if "t_first_cmd" not in rec_data or "recorded_gt" not in rec_data:
            rec_data = record_fresh_episode(root_dir, seed=101)

    print("\n================================================================================")
    print("=== A3.4: REPLAY BENCHMARK EXECUTION (5 REPLAY RUNS) ===")
    print("================================================================================")
    print(f"Original Episode: {rec_data.get('source', 'seed_101')}")
    print(f"Reference T0 (first command): {rec_data['t_first_cmd']:.4f}s")
    print(f"Planning latency before first command: {rec_data.get('planning_latency_s', 0.0):.4f}s")
    print("Criteria: Mean error < 10.0 mm (1.0 cm), Max error < 20.0 mm (2.0 cm)\n")

    trials_results = []
    for i in range(1, 6):
        print(f">>> [Run {i}/5] Executing cold-restart replay trial...")
        trial_res = run_single_replay_trial(root_dir, rec_data, trial_idx=i)
        trials_results.append(trial_res)
        print(f"    Trial {i} result: Mean={trial_res['mean_error_mm']:.2f} mm | Max={trial_res['max_error_mm']:.2f} mm | Final={trial_res['final_error_mm']:.2f} mm | MaxLat={trial_res['max_lateral_error_mm']:.2f} mm | Passed: {trial_res['passed']}")

    # Aggregated Summary
    means = [t["mean_error_mm"] for t in trials_results]
    maxs = [t["max_error_mm"] for t in trials_results]
    finals = [t["final_error_mm"] for t in trials_results]
    lats = [t["max_lateral_error_mm"] for t in trials_results]
    all_passed = all(t["passed"] for t in trials_results)

    summary = {
        "n_trials": len(trials_results),
        "mean_error_overall_mm": {
            "mean": round(float(np.mean(means)), 3),
            "std": round(float(np.std(means)), 3),
            "min": round(float(np.min(means)), 3),
            "max": round(float(np.max(means)), 3),
        },
        "max_error_overall_mm": {
            "mean": round(float(np.mean(maxs)), 3),
            "std": round(float(np.std(maxs)), 3),
            "min": round(float(np.min(maxs)), 3),
            "max": round(float(np.max(maxs)), 3),
        },
        "final_error_overall_mm": {
            "mean": round(float(np.mean(finals)), 3),
            "std": round(float(np.std(finals)), 3),
            "min": round(float(np.min(finals)), 3),
            "max": round(float(np.max(finals)), 3),
        },
        "max_lateral_error_overall_mm": {
            "mean": round(float(np.mean(lats)), 3),
            "std": round(float(np.std(lats)), 3),
            "min": round(float(np.min(lats)), 3),
            "max": round(float(np.max(lats)), 3),
        },
        "criteria_passed": all_passed,
        "criteria": {
            "mean_error_threshold_mm": 10.0,
            "max_error_threshold_mm": 20.0
        }
    }

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/a3_4_replay_results.json")
    full_output = {
        "benchmark": "A3.4_precision_cmdvel_replay",
        "scenario": "corridor_090_H1_seed_101",
        "summary": summary,
        "trials": trials_results
    }
    with open(out_file, "w") as f:
        json.dump(full_output, f, indent=2)

    print("\n================================================================================")
    print("=== A3.4 REPLAY BENCHMARK FINAL SUMMARY (5 RUNS) ===")
    print("================================================================================")
    print(f"Overall Status              : {'PASSED (All 5 runs satisfy criteria)' if all_passed else 'FAILED'}")
    print(f"Mean Trajectory Deviation   : {summary['mean_error_overall_mm']['mean']:.2f} ± {summary['mean_error_overall_mm']['std']:.2f} mm [Min: {summary['mean_error_overall_mm']['min']:.2f}, Max: {summary['mean_error_overall_mm']['max']:.2f} mm]")
    print(f"Max Trajectory Deviation    : {summary['max_error_overall_mm']['mean']:.2f} ± {summary['max_error_overall_mm']['std']:.2f} mm [Min: {summary['max_error_overall_mm']['min']:.2f}, Max: {summary['max_error_overall_mm']['max']:.2f} mm]")
    print(f"Final Pose Deviation        : {summary['final_error_overall_mm']['mean']:.2f} ± {summary['final_error_overall_mm']['std']:.2f} mm")
    print(f"Max Lateral Deviation       : {summary['max_lateral_error_overall_mm']['mean']:.2f} ± {summary['max_lateral_error_overall_mm']['std']:.2f} mm")
    print(f"Output saved to             : {out_file}")
    print("================================================================================\n")


if __name__ == "__main__":
    main()
