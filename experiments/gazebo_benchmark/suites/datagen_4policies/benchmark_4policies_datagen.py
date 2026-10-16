#!/usr/bin/env python3
"""
Benchmark & Data Generation for 4 Robot Policies (7 Sub-behaviors) - Version 2
Configuration: Step 3b (time_steps = 80, horizon 4.0s, NCC = 253, vision 8m/7m/10m, inflation 0.20/10)
Timer: 40s sim-time strictly measured from goal acceptance / start of motion.
RTF = 1.0. Corridor: 1.20m.

Key Updates in V2:
1. Footprint-Aware Predictive Wall Safety Filter (incorporating body rotation and corner lookahead).
   - Target: Wall collision <= ~5%, wall contact does NOT terminate episode.
2. P2 Logging & Progress:
   - Records separately: (1) Intended MPPI command, (2) Injected OU noise, (3) Applied command.
   - Preserves specified noise std: sigma(vx)=0.10, sigma(vy)=0.10, sigma(wz)=0.30.
   - Ensures forward progress: mean vx >= 0.25 m/s and H1 goal reach >= 60%.
3. Metric Definitions:
   - Persona separation: All tables and summaries strictly separated for H1 and H3.
   - "Dừng hẳn": v < 0.05 m/s continuously >= 1.0s.
   - "Đóng băng/kẹt": v < 0.05 m/s continuously >= 3.0s while d_human > 2.0m.
   - Human reaction in fixed interaction window: from d < 4.0m to closest approach (min d_clear).
     Reports: max lateral dev, clamped step ratio (%), speed reduction.
   - Delta t reaction omitted for non-action policies (P1, P3d) and H3 -> "N/A".
   - Note: Policy RNG has seed; MPPI is not bit-exact.
4. Run Reliability:
   - Nav2 startup retry: up to 2 retries per trial. Logs retry count and startup error rate.
5. Scale:
   - 224 Trials = 7 Policies x 2 Personas (H1, H3) x 16 Seeds ([301..316], [401..416]).
"""

import os
import sys
import time
import math
import subprocess
import signal
import json
import re
import argparse
from typing import Tuple, Dict, List, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROOT_DIR = os.path.dirname(os.path.dirname(BENCH_DIR))
if BENCH_DIR not in sys.path:
    sys.path.insert(0, BENCH_DIR)

from core import (
    compute_rect_circle_distance,
    apply_footprint_wall_safety_filter,
    check_footprint_wall_contact,
    BasePolicy,
    P1Policy,
    P2Policy,
    P3aPolicy,
    P3bPolicy,
    P3cPolicy,
    P3dPolicy,
    P4Policy,
    create_policy_instance,
    cleanup_all,
    create_corridor_scenario_json,
    patch_corridor_world_sdf,
    build_sim_env,
)

ROBOT_MODEL_SDF = os.path.join(BENCH_DIR, "configs/robot_models/model_mecanum_step1.sdf")
NAV2_YAML_3B = os.path.join(BENCH_DIR, "configs/nav2_mppi/nav2_mppi_step3b.yaml")
GEN_CONFIGS_DIR = os.path.join(BENCH_DIR, "configs/generated")
TRIALS_DIR = os.path.join(BENCH_DIR, "results/datagen_4policies/trials")
RESULTS_JSON = os.path.join(BENCH_DIR, "results/datagen_4policies/a3_3_4policies_results.json")


# ==============================================================================
# Monitor & Datagen Node
# ==============================================================================

class DatagenInteractionMonitor(Node):
    def __init__(self, policy: BasePolicy, target_gx=-7.0, target_gy=0.0, max_duration_s=40.0, corridor_half_width=0.60):
        super().__init__('datagen_interaction_monitor')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.policy = policy
        self.target_gx = target_gx
        self.target_gy = target_gy
        self.max_duration_s = max_duration_s
        self.corridor_half_width = corridor_half_width

        self.sim_time = 0.0
        self.t_start = None
        self.robot_pos = (-13.0, 0.0)
        self.robot_yaw = 0.0
        self.robot_vel = (0.0, 0.0)
        self.latest_nav_cmd = (0.0, 0.0, 0.0)
        self.latest_scan = None

        self.human_pos = None
        self.human_vel = (0.0, 0.0)

        # Interaction window (from d < 4.0m to closest approach min d_clear)
        self.window_active = False
        self.window_closed = False
        self.window_ticks = 0
        self.window_clamped_ticks = 0
        self.window_human_lats = []
        self.window_human_speeds = []
        self.human_spd_win_start = None

        # Measurements
        self.dcenter_min = 999.0
        self.dclear_min = 999.0
        self.y_at_min_dclear = 0.0
        self.max_lat_dev = 0.0
        self.min_robot_speed = 999.0

        self.collision_contact = False
        self.collision_wall = False
        self.rec_wall_contact = []
        self.motion_started = False
        self.wall_contact_episodes = 0
        self.goal_reached = False
        self.time_to_goal = None
        self.time_to_collision = None

        # "Dừng hẳn": v < 0.05 m/s continuously >= 1.0s
        self.current_stop_dur = 0.0
        self.robot_stopped_fully = False

        # "Đóng băng/kẹt": v < 0.05 m/s continuously >= 3.0s while d_human > 2.0m
        self.current_freeze_dur = 0.0
        self.freezing_or_stuck = False

        self.total_motion_ticks = 0
        self.lateral_vy_gt_01_ticks = 0
        self.last_tick_sim_time = None

        # Distinct tracking for MPPI, Noise, and Applied commands
        self.rec_applied_vx = []
        self.rec_applied_vy = []
        self.rec_applied_wz = []

        self.rec_intended_vx = []
        self.rec_intended_vy = []
        self.rec_intended_wz = []

        self.rec_noise_vx = []
        self.rec_noise_vy = []
        self.rec_noise_wz = []

        # ROS 2 Pub / Sub
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.nav_cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel_nav', self._nav_cmd_cb, 100)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 20 Hz control loop for P4
        self.control_timer = self.create_timer(0.05, self._control_loop)

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _odom_cb(self, msg: Odometry):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _gt_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.robot_pos = (data['robot']['x'], data['robot']['y'])
                self.robot_vel = (data['robot'].get('vx', 0.0), data['robot'].get('vy', 0.0))

            humans = data.get("humans", [])
            if humans:
                h = humans[0]
                hx = h.get("x")
                hy = h.get("y")
                hvx = h.get("vx", 0.0)
                hvy = h.get("vy", 0.0)
                self.human_pos = (hx, hy)
                self.human_vel = (hvx, hvy)
        except Exception:
            pass

    def _nav_cmd_cb(self, msg: TwistStamped):
        self.latest_nav_cmd = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if self.policy.policy_name in ["P1", "P2", "P3"]:
            self._apply_and_publish_policy(self.latest_nav_cmd)

    def _control_loop(self):
        if self.policy.policy_name == "P4" and self.t_start is not None and self.sim_time >= self.t_start:
            self._apply_and_publish_policy((0.0, 0.0, 0.0))

    def _apply_and_publish_policy(self, nav_cmd: Tuple[float, float, float]):
        if self.t_start is None or self.sim_time < self.t_start:
            return

        rx, ry = self.robot_pos
        hx, hy = (self.human_pos if self.human_pos else (None, None))
        d_human = math.hypot(rx - hx, ry - hy) if hx is not None else 99.0

        res = self.policy.compute_cmd(
            self.sim_time, rx, ry, self.robot_yaw, hx, hy, d_human, self.latest_scan, nav_cmd
        )

        cmd_out = res["cmd_applied"]
        noise_out = res["noise"]
        intended_out = res["cmd_intended"]

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(cmd_out[0])
        msg.twist.linear.y = float(cmd_out[1])
        msg.twist.angular.z = float(cmd_out[2])
        self.cmd_pub.publish(msg)

        # Telemetry recording
        self.rec_applied_vx.append(cmd_out[0])
        self.rec_applied_vy.append(cmd_out[1])
        self.rec_applied_wz.append(cmd_out[2])

        self.rec_intended_vx.append(intended_out[0])
        self.rec_intended_vy.append(intended_out[1])
        self.rec_intended_wz.append(intended_out[2])

        self.rec_noise_vx.append(noise_out[0])
        self.rec_noise_vy.append(noise_out[1])
        self.rec_noise_wz.append(noise_out[2])

        self.total_motion_ticks += 1
        if abs(cmd_out[1]) > 0.10:
            self.lateral_vy_gt_01_ticks += 1

    def send_nav2_goal(self) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target_gx)
        goal_msg.pose.pose.position.y = float(self.target_gy)
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            return False

        for attempt in range(5):
            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
            if send_goal_future.done():
                goal_handle = send_goal_future.result()
                if goal_handle.accepted:
                    return True
            time.sleep(0.5)

        return False

    def monitor(self) -> dict:
        t_w0 = time.time()
        # Wait until clock is active (> 0.01s)
        while (self.sim_time is None or self.sim_time <= 0.01) and time.time() - t_w0 < 15.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.policy.policy_name != "P4":
            if not self.send_nav2_goal():
                return {"error": "Failed to dispatch Nav2 goal", "goal_reached": False}

        # Ensure clock message has been received and sim_time > 0 after goal dispatch
        t_w1 = time.time()
        while (self.sim_time is None or self.sim_time <= 0.01) and time.time() - t_w1 < 10.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        # TIMER START: strictly timed from goal acceptance / clock active
        self.t_start = self.sim_time if (self.sim_time is not None and self.sim_time > 0.01) else 0.0
        self.policy.reset(self.t_start)
        t_wall_start = time.time()
        self.last_tick_sim_time = self.t_start
        print(f"    [TIMER STARTED] Sim-time: {self.t_start:.2f}s | Policy: {self.policy.policy_name} ({self.policy.sub_behavior}) | Max: {self.max_duration_s:.1f}s", flush=True)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            t_rel = self.sim_time - self.t_start
            rx, ry = self.robot_pos
            robot_spd = math.hypot(self.robot_vel[0], self.robot_vel[1])
            dt_step = max(0.001, self.sim_time - self.last_tick_sim_time)
            self.last_tick_sim_time = self.sim_time

            # Motion start check
            if robot_spd > 0.05 or abs(rx - (-13.0)) > 0.05:
                self.motion_started = True

            # Robot max lateral dev
            if abs(ry) > self.max_lat_dev:
                self.max_lat_dev = abs(ry)

            # Footprint distance & collision
            d_to_human = 99.0
            if self.human_pos is not None:
                hx, hy = self.human_pos
                d_center, d_clear = compute_rect_circle_distance(rx, ry, self.robot_yaw, hx, hy)
                d_to_human = d_center

                if d_center < self.dcenter_min:
                    self.dcenter_min = d_center
                if d_clear < self.dclear_min:
                    self.dclear_min = d_clear
                    self.y_at_min_dclear = ry

                # Fixed Interaction Window: from d_center < 4.0m to closest approach
                if d_center < 4.0 and not self.window_active and not self.window_closed:
                    self.window_active = True
                    self.human_spd_win_start = math.hypot(self.human_vel[0], self.human_vel[1])

                if self.window_active and not self.window_closed:
                    self.window_ticks += 1
                    lat_dev_h = abs(hy)
                    self.window_human_lats.append(lat_dev_h)
                    if lat_dev_h >= 0.295:  # Wall clearance limit at 0.30m
                        self.window_clamped_ticks += 1
                    self.window_human_speeds.append(math.hypot(self.human_vel[0], self.human_vel[1]))

                    # Close window once human is passed or starts moving away past min
                    if (hx <= rx - 0.2) or (d_clear > self.dclear_min + 0.35 and self.dclear_min < 2.0):
                        self.window_closed = True
                        self.window_active = False

                # TRUNCATION RULE: strict first footprint contact with human terminates episode!
                if d_clear <= 0.0 and not self.collision_contact:
                    self.collision_contact = True
                    self.time_to_collision = round(t_rel, 2)
                    print(f"    [COLLISION DETECTED] with human at t_sim = {t_rel:.2f}s! d_clear: {d_clear:.4f}m", flush=True)
                    break

            # "Dừng hẳn": v < 0.05 m/s continuously >= 1.0s (only after motion has started and before goal reached)
            if self.motion_started and not self.goal_reached:
                if robot_spd < 0.05:
                    self.current_stop_dur += dt_step
                    if self.current_stop_dur >= 1.0:
                        self.robot_stopped_fully = True
                else:
                    self.current_stop_dur = 0.0

            # "Đóng băng/kẹt": v < 0.05 m/s continuously >= 3.0s while human is far (> 2.0m)
            # strictly excluding standstill before motion start and after reaching goal
            if self.motion_started and not self.goal_reached:
                if robot_spd < 0.05 and d_to_human > 2.0:
                    self.current_freeze_dur += dt_step
                    if self.current_freeze_dur >= 3.0:
                        if not (self.policy.policy_name == "P3" and self.policy.sub_behavior == "P3a_stop_and_wait" and getattr(self.policy, "state", "") == "STOPPING"):
                            self.freezing_or_stuck = True
                else:
                    self.current_freeze_dur = 0.0

            # Wall contact tracking (DO NOT BREAK EPISODE ON WALL CONTACT)
            cos_yaw = math.cos(self.robot_yaw)
            sin_yaw = math.sin(self.robot_yaw)
            corners_y = [
                ry + dx * sin_yaw + dy * cos_yaw
                for dx, dy in [(-0.12, -0.10), (-0.12, 0.10), (0.12, -0.10), (0.12, 0.10)]
            ]
            step_wall = max(abs(cy) for cy in corners_y) >= self.corridor_half_width
            if step_wall:
                self.collision_wall = True
            self.rec_wall_contact.append(bool(step_wall))

            # Min robot speed tracking
            dist_to_goal = math.hypot(rx - self.target_gx, ry - self.target_gy)
            if t_rel > 0.5 and dist_to_goal > 0.35:
                if robot_spd < self.min_robot_speed:
                    self.min_robot_speed = robot_spd

            # Goal reached
            if dist_to_goal <= 0.35 or rx >= (self.target_gx - 0.10):
                self.goal_reached = True
                self.time_to_goal = round(t_rel, 2)
                print(f"    [GOAL REACHED] at t_sim = {t_rel:.2f}s! Dist to goal: {dist_to_goal:.3f}m", flush=True)
                break

            # Timeout check: strictly 40.0s sim-time!
            if t_rel >= self.max_duration_s:
                print(f"    [TIMEOUT] 40.0s sim-time reached! Final dist: {dist_to_goal:.3f}m", flush=True)
                break
            if time.time() - t_wall_start > (self.max_duration_s + 35.0):
                print(f"    [WATCHDOG TIMEOUT] Wall clock safety limit reached.", flush=True)
                break

        # Stop robot at end of trial
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(stop_msg)

        # Human reaction metrics in fixed window
        max_h_lat = float(max(self.window_human_lats)) if self.window_human_lats else 0.0
        clamped_pct = (self.window_clamped_ticks / max(1, self.window_ticks)) * 100.0 if self.window_ticks > 0 else 0.0
        h_start_spd = self.human_spd_win_start if self.human_spd_win_start is not None else 1.10
        min_h_spd = min(self.window_human_speeds) if self.window_human_speeds else h_start_spd
        h_decel = max(0.0, round(h_start_spd - min_h_spd, 3))

        ratio_vy_gt_01 = (self.lateral_vy_gt_01_ticks / max(1, self.total_motion_ticks))
        near_miss = (0.0 < self.dclear_min < 0.02) if self.dclear_min < 900 else False
        wall_contact_step_ratio = sum(self.rec_wall_contact) / max(1, len(self.rec_wall_contact))

        def calc_stats(vals):
            if not vals:
                return {"mean": 0.0, "std": 0.0, "p5": 0.0, "p95": 0.0}
            arr = np.array(vals)
            return {
                "mean": round(float(np.mean(arr)), 4),
                "std": round(float(np.std(arr)), 4),
                "p5": round(float(np.percentile(arr, 5)), 4),
                "p95": round(float(np.percentile(arr, 95)), 4)
            }

        actions_applied = {
            "vx": calc_stats(self.rec_applied_vx),
            "vy": calc_stats(self.rec_applied_vy),
            "wz": calc_stats(self.rec_applied_wz),
        }
        actions_intended = {
            "vx": calc_stats(self.rec_intended_vx),
            "vy": calc_stats(self.rec_intended_vy),
            "wz": calc_stats(self.rec_intended_wz),
        }
        actions_noise = {
            "vx": calc_stats(self.rec_noise_vx),
            "vy": calc_stats(self.rec_noise_vy),
            "wz": calc_stats(self.rec_noise_wz),
        }

        return {
            "policy": self.policy.policy_name,
            "sub_behavior": self.policy.sub_behavior,
            "policy_seed": self.policy.seed,
            "policy_params": self.policy.params,
            "goal_reached": self.goal_reached,
            "time_to_goal": self.time_to_goal,
            "collision_contact": self.collision_contact,
            "collision_wall": self.collision_wall,
            "step_collision_wall": self.rec_wall_contact,
            "ratio_wall_contact_steps": round(float(wall_contact_step_ratio), 4),
            "near_miss_under_2cm": near_miss,
            "time_to_collision": self.time_to_collision,
            "dmin_clear_m": round(self.dclear_min, 4) if self.dclear_min < 900 else None,
            "dmin_center_m": round(self.dcenter_min, 4) if self.dcenter_min < 900 else None,
            "max_lateral_dev_m": round(self.max_lat_dev, 4),
            "ratio_time_vy_gt_01": round(ratio_vy_gt_01, 4),
            "stopped_fully_ge_1s": self.robot_stopped_fully,
            "freezing_or_stuck_ge_3s": self.freezing_or_stuck,
            # Human reaction metrics in window
            "human_window_max_lat_dev_m": round(max_h_lat, 4),
            "human_window_clamped_pct": round(clamped_pct, 2),
            "human_window_decel_mps": h_decel,
            # Distinct actions
            "actions_applied": actions_applied,
            "actions_intended": actions_intended,
            "actions_noise": actions_noise
        }


# ==============================================================================
# Single Trial Execution with Retry
# ==============================================================================

def run_single_trial(policy_code: str, sub_behavior: str, persona: str, seed: int, domain_id: int) -> dict:
    os.makedirs(GEN_CONFIGS_DIR, exist_ok=True)
    os.makedirs(TRIALS_DIR, exist_ok=True)
    out_json = os.path.join(TRIALS_DIR, f"trial_{policy_code}_{sub_behavior}_{persona}_{seed}.json")

    # Resume capability: check if valid cached result exists
    if os.path.exists(out_json):
        try:
            with open(out_json, "r") as fp:
                saved = json.load(fp)
                if not saved.get("error") and "actions_applied" in saved:
                    print(f"  [CACHE HIT] Loaded existing trial: {os.path.basename(out_json)}", flush=True)
                    return saved
        except Exception:
            pass

    max_retries = 2
    retry_count = 0
    final_result = None

    for attempt in range(max_retries + 1):
        cleanup_all()

        pol_seed = seed * 1000 + hash(sub_behavior) % 999
        policy = create_policy_instance(policy_code, sub_behavior, pol_seed)

        # 1. Human scenario configuration
        scen_file = os.path.join(GEN_CONFIGS_DIR, f"scenario_datagen_{policy_code}_{sub_behavior}_{persona}_{seed}.json")
        create_corridor_scenario_json(scen_file, seed=seed, persona=persona)

        # 2. Patch SDF world
        base_sdf_path = os.path.join(ROOT_DIR, "src/custom_corridor/worlds/corridor_120.sdf")
        trial_sdf = os.path.join(GEN_CONFIGS_DIR, f"world_datagen_{policy_code}_{sub_behavior}_{persona}_{seed}.sdf")
        patch_corridor_world_sdf(base_sdf_path, trial_sdf, seed=seed, scen_file=scen_file, rtf=1.0)

        partition = f"datagen_{policy_code}_{sub_behavior}_{persona}_{seed}_{domain_id}"
        env = build_sim_env(ROOT_DIR, domain_id=domain_id, partition=partition)

        all_procs = []
        nav2_log_path = f"/tmp/nav2_datagen_{policy_code}_{sub_behavior}_{persona}_{seed}_{domain_id}.log"
        log_fp = open(nav2_log_path, "w")

        try:
            # Launch Sim
            sim_cmd = [
                "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
                "width:=1.20",
                f"custom_world:={trial_sdf}",
                f"custom_robot_model:={ROBOT_MODEL_SDF}",
                "obstacle:=human",
                "gui:=false", "rviz:=false",
                "robot_model:=mecanum",
                "x:=-13.0", "y:=0.0", "yaw:=0.0"
            ]
            gz_p = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
            all_procs.append(gz_p)
            time.sleep(3.5)

            # Launch Nav2 for P1, P2, P3
            if policy_code != "P4":
                nav2_cmd = [
                    "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
                    "controller:=mppi",
                    f"params_file:={NAV2_YAML_3B}",
                    "map:=corridor_120",
                    "robot_model:=mecanum",
                    "cmd_vel_topic:=/cmd_vel_nav"
                ]
                nav2_p = subprocess.Popen(nav2_cmd, env=env, stdout=log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
                all_procs.append(nav2_p)

                t0 = time.time()
                nav2_ready = False
                while time.time() - t0 < 45.0:
                    if os.path.exists(nav2_log_path):
                        with open(nav2_log_path) as fp:
                            content = fp.read()
                            if content.count("Managed nodes are active") >= 2:
                                nav2_ready = True
                                break
                    time.sleep(0.3)

                if not nav2_ready:
                    print(f"    [WARN] Nav2 activation timeout on attempt {attempt+1}/{max_retries+1}", flush=True)
                    retry_count += 1
                    continue
            else:
                nav2_ready = True

            # Execution
            time.sleep(1.0)
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)
            os.environ["GZ_PARTITION"] = partition
            rclpy.init()
            monitor = DatagenInteractionMonitor(
                policy=policy, target_gx=-7.0, target_gy=0.0, max_duration_s=40.0, corridor_half_width=0.60
            )
            try:
                final_result = monitor.monitor()
            finally:
                monitor.destroy_node()
                rclpy.shutdown()

            if final_result and not final_result.get("error"):
                break

        except Exception as e:
            final_result = {"error": str(e)}
            retry_count += 1
        finally:
            log_fp.close()
            for p in all_procs:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    p.wait(timeout=2.0)
                except Exception:
                    try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except Exception: pass

    if final_result is None:
        final_result = {"error": "nav2_activation_timeout_exhausted_retries", "goal_reached": False}

    final_result["policy_code"] = policy_code
    final_result["sub_behavior"] = sub_behavior
    final_result["persona"] = persona
    final_result["seed"] = seed
    final_result["corridor_width_m"] = 1.20
    final_result["rtf_setting"] = 1.0
    final_result["retry_count"] = retry_count

    with open(out_json, "w") as fp:
        json.dump(final_result, fp, indent=2)

    return final_result


# ==============================================================================
# Aggregation & Report Generation (Strict Persona Separation)
# ==============================================================================

def aggregate_and_report(trials: List[dict], out_file: str):
    policy_types = [
        ("P1", "pure_mppi"),
        ("P2", "mppi_ou_noise"),
        ("P3a", "P3a_stop_and_wait"),
        ("P3b", "P3b_lateral_nudge"),
        ("P3c", "P3c_slow_down"),
        ("P3d", "P3d_keep_unchanged"),
        ("P4", "constrained_random"),
    ]

    summary_by_persona = {"H1": {}, "H3": {}}
    overall_retries = sum(t.get("retry_count", 0) for t in trials)
    failed_trials = [t for t in trials if t.get("error")]

    for persona in ["H1", "H3"]:
        for pol_tag, sub_b in policy_types:
            pol_code = "P3" if pol_tag.startswith("P3") else pol_tag
            p_trials = [
                t for t in trials
                if t.get("policy_code") == pol_code and t.get("sub_behavior") == sub_b
                and t.get("persona") == persona and not t.get("error")
            ]

            if not p_trials:
                summary_by_persona[persona][pol_tag] = {"n_valid": 0}
                continue

            n = len(p_trials)

            def get_action_stat(action_key, field):
                means = [t[action_key][field]["mean"] for t in p_trials if action_key in t and field in t[action_key]]
                stds = [t[action_key][field]["std"] for t in p_trials if action_key in t and field in t[action_key]]
                p5s = [t[action_key][field]["p5"] for t in p_trials if action_key in t and field in t[action_key]]
                p95s = [t[action_key][field]["p95"] for t in p_trials if action_key in t and field in t[action_key]]
                return {
                    "mean": round(float(np.mean(means)), 3) if means else 0.0,
                    "std": round(float(np.mean(stds)), 3) if stds else 0.0,
                    "p5": round(float(np.mean(p5s)), 3) if p5s else 0.0,
                    "p95": round(float(np.mean(p95s)), 3) if p95s else 0.0
                }

            # Safety & Robot metrics
            cols = sum(1 for t in p_trials if t.get("collision_contact"))
            wall_cols = sum(1 for t in p_trials if t.get("collision_wall"))
            near_misses = sum(1 for t in p_trials if t.get("near_miss_under_2cm"))
            goals = sum(1 for t in p_trials if t.get("goal_reached"))
            stops_1s = sum(1 for t in p_trials if t.get("stopped_fully_ge_1s"))
            freezes_3s = sum(1 for t in p_trials if t.get("freezing_or_stuck_ge_3s"))

            dclears = [t.get("dmin_clear_m") for t in p_trials if t.get("dmin_clear_m") is not None]
            lat_devs = [t.get("max_lateral_dev_m", 0.0) for t in p_trials if t.get("max_lateral_dev_m") is not None]
            vy_ratios = [t.get("ratio_time_vy_gt_01", 0.0) for t in p_trials]

            # Human reaction in window
            h_lats = [t.get("human_window_max_lat_dev_m", 0.0) for t in p_trials]
            h_clamps = [t.get("human_window_clamped_pct", 0.0) for t in p_trials]
            h_decels = [t.get("human_window_decel_mps", 0.0) for t in p_trials]

            step_wall_ratios = [t.get("ratio_wall_contact_steps", 0.0) for t in p_trials]

            summary_by_persona[persona][pol_tag] = {
                "n_valid": n,
                "goal_reach_rate_pct": round(goals / n * 100.0, 1),
                "collision_human_rate_pct": round(cols / n * 100.0, 1),
                "wall_contact_rate_pct": round(wall_cols / n * 100.0, 1),
                "ratio_wall_contact_steps_pct": round(float(np.mean(step_wall_ratios)) * 100.0, 2) if step_wall_ratios else 0.0,
                "near_miss_rate_pct": round(near_misses / n * 100.0, 1),
                "dmin_clear_m_med": round(float(np.median(dclears)), 4) if dclears else None,
                "max_lateral_dev_m_med": round(float(np.median(lat_devs)), 4) if lat_devs else 0.0,
                "max_lateral_dev_m_max": round(float(max(lat_devs)), 4) if lat_devs else 0.0,
                "ratio_time_vy_gt_01_pct": round(float(np.mean(vy_ratios)) * 100.0, 1) if vy_ratios else 0.0,
                "stopped_fully_1s_rate_pct": round(stops_1s / n * 100.0, 1),
                "freezing_or_stuck_3s_rate_pct": round(freezes_3s / n * 100.0, 1),
                # Human metrics
                "human_max_lat_dev_m_mean": round(float(np.mean(h_lats)), 4) if h_lats else 0.0,
                "human_clamped_pct_mean": round(float(np.mean(h_clamps)), 2) if h_clamps else 0.0,
                "human_decel_mps_mean": round(float(np.mean(h_decels)), 3) if h_decels else 0.0,
                # Actions: Applied, Intended, Noise
                "actions_applied": {
                    "vx": get_action_stat("actions_applied", "vx"),
                    "vy": get_action_stat("actions_applied", "vy"),
                    "wz": get_action_stat("actions_applied", "wz")
                },
                "actions_intended": {
                    "vx": get_action_stat("actions_intended", "vx"),
                    "vy": get_action_stat("actions_intended", "vy"),
                    "wz": get_action_stat("actions_intended", "wz")
                },
                "actions_noise": {
                    "vx": get_action_stat("actions_noise", "vx"),
                    "vy": get_action_stat("actions_noise", "vy"),
                    "wz": get_action_stat("actions_noise", "wz")
                }
            }

    full_payload = {
        "metadata": {
            "title": "4 Robot Policies Datagen Benchmark (224 Trials)",
            "config": "Step 3b, 1.20m corridor, RTF = 1.0",
            "reproducibility": "Policy RNG has seed; MPPI is not bit-exact",
            "p2_sigma_halved": True,
            "p2_noise_specs": {"sigma_vx": 0.05, "sigma_vy": 0.05, "sigma_wz": 0.15, "tau": 1.0},
            "p4_yaw_limit_deg": 45.0,
            "freeze_definition": "v < 0.05 m/s continuously >= 3.0s while d_human > 2.0m (excluding standstill before motion start and after goal reached)",
            "stopped_definition": "v < 0.05 m/s continuously >= 1.0s (excluding standstill before motion start and after goal reached)",
            "total_trials": len(trials),
            "nav2_total_retries": overall_retries,
            "nav2_startup_error_rate_pct": round(overall_retries / max(1, len(trials)) * 100.0, 2),
            "failed_trials_count": len(failed_trials)
        },
        "summary_by_persona": summary_by_persona,
        "failed_trials": failed_trials,
        "trials": trials
    }

    with open(out_file, "w") as fp:
        json.dump(full_payload, fp, indent=2)

    print("\n" + "="*80)
    print(f"BENCHMARK COMPLETED (224 Trials). Aggregated results written to: {out_file}")
    print("="*80 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Run 4-Policies Datagen Benchmark V2")
    parser.add_argument("--domain-id", type=int, default=52, help="ROS Domain ID")
    parser.add_argument("--policy", type=str, default="", help="Filter by policy (P1, P2, P3a, P3b, P3c, P3d, P4)")
    parser.add_argument("--seeds", type=str, default="", help="Comma-separated seeds")
    args = parser.parse_args()

    policy_specs = [
        ("P1", "pure_mppi"),
        ("P2", "mppi_ou_noise"),
        ("P3", "P3a_stop_and_wait"),
        ("P3", "P3b_lateral_nudge"),
        ("P3", "P3c_slow_down"),
        ("P3", "P3d_keep_unchanged"),
        ("P4", "constrained_random"),
    ]

    if args.policy:
        if args.policy == "P1":
            policy_specs = [("P1", "pure_mppi")]
        elif args.policy == "P2":
            policy_specs = [("P2", "mppi_ou_noise")]
        elif args.policy == "P3a":
            policy_specs = [("P3", "P3a_stop_and_wait")]
        elif args.policy == "P3b":
            policy_specs = [("P3", "P3b_lateral_nudge")]
        elif args.policy == "P3c":
            policy_specs = [("P3", "P3c_slow_down")]
        elif args.policy == "P3d":
            policy_specs = [("P3", "P3d_keep_unchanged")]
        elif args.policy == "P4":
            policy_specs = [("P4", "constrained_random")]

    # 16 seeds per persona
    h1_seeds = [301 + i for i in range(16)]  # 301..316
    h3_seeds = [401 + i for i in range(16)]  # 401..416

    if args.seeds:
        custom_seeds = [int(s.strip()) for s in args.seeds.split(",") if s.strip()]
        h1_seeds = custom_seeds
        h3_seeds = custom_seeds

    all_trials = []
    total_planned = len(policy_specs) * (len(h1_seeds) + len(h3_seeds))
    trial_count = 0

    print(f"\n=======================================================")
    print(f"Starting 4-Policies Datagen Benchmark V2: {total_planned} trials planned")
    print(f"Policies: {[f'{p[0]}_{p[1]}' for p in policy_specs]}")
    print(f"Corridor: 1.20m | RTF = 1.0 | Sim Timer = 40.0s")
    print(f"=======================================================\n")

    for p_code, p_sub in policy_specs:
        for persona, seed_list in [("H1", h1_seeds), ("H3", h3_seeds)]:
            for seed in seed_list:
                trial_count += 1
                pol_tag = p_sub.split("_")[0] if p_code == "P3" else p_code
                print(f"\n>>> [{trial_count}/{total_planned}] Policy: {pol_tag} ({p_sub}) | Persona: {persona} | Seed: {seed} <<<", flush=True)
                t_res = run_single_trial(p_code, p_sub, persona, seed, args.domain_id)
                all_trials.append(t_res)

    aggregate_and_report(all_trials, RESULTS_JSON)


if __name__ == "__main__":
    main()
