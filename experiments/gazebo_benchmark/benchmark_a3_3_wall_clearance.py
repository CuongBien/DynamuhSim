#!/usr/bin/env python3
"""
A3.3 Benchmark - Wall Clearance & Anti-Penetration Verification
Robot Configuration: Step 3b (time_steps = 80, horizon 4.0s, NCC = 253, vision 8m/7m/10m, inflation 0.20/10)
Timer: 40s sim-time strictly measured from goal acceptance.
RTF = 1.0.

Strict Human Wall-Clearing Rule:
Max lateral deviation of human center = corridor_half_width - 0.28 - 0.02
- Corridor 0.90m: max_y = 0.45 - 0.28 - 0.02 = 0.15 m
- Corridor 1.20m: max_y = 0.60 - 0.28 - 0.02 = 0.30 m

Geometric Clearance:
Clearance = corridor_width - 0.56 - 0.20
- Corridor 0.90m: 0.14 m
- Corridor 1.20m: 0.44 m

Verification Metrics:
1. Human wall penetration step ratio (%) & max penetration depth (cm) (Target: < 1% steps, max depth < 1 cm)
2. Clamped step ratio (%)
3. Human lateral deviation in H1 before and after
4. Footprint contact collision rate, d_clear min, near-miss (< 0.02m) rate, goal reach rate
5. Remaining geometric clearance per corridor
"""

import os
import sys
import time
import math
import subprocess
import json
import re
import argparse
from typing import Tuple, Dict, List, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

ROOT_DIR = "/home/cuongbien/DynamuhSim"
ROBOT_MODEL_SDF = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/step_configs/model_mecanum_step1.sdf")
NAV2_YAML_3B = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/step_configs/nav2_mppi_step3b.yaml")
PROFILER_LIB = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/libcontroller_profiler.so")


def compute_rect_circle_distance(rx: float, ry: float, ryaw: float, hx: float, hy: float, rw=0.20, rl=0.24, hr=0.28) -> Tuple[float, float]:
    dx = hx - rx
    dy = hy - ry
    d_center = math.hypot(dx, dy)

    cos_y = math.cos(ryaw)
    sin_y = math.sin(ryaw)
    x_rel = dx * cos_y + dy * sin_y
    y_rel = -dx * sin_y + dy * cos_y

    half_l = rl / 2.0  # 0.12 m
    half_w = rw / 2.0  # 0.10 m

    x_cl = max(-half_l, min(half_l, x_rel))
    y_cl = max(-half_w, min(half_w, y_rel))
    d_rect = math.hypot(x_rel - x_cl, y_rel - y_cl)
    d_clear = d_rect - hr

    return d_center, d_clear


class WallClearanceInteractionMonitor(Node):
    def __init__(self, target_gx=-7.0, target_gy=0.0, max_duration_s=40.0, corridor_half_width=0.45):
        super().__init__('wall_clearance_interaction_monitor')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_gx = target_gx
        self.target_gy = target_gy
        self.max_duration_s = max_duration_s
        self.corridor_half_width = corridor_half_width
        self.allowed_human_y = max(0.05, self.corridor_half_width - 0.28 - 0.02)

        self.sim_time = 0.0
        self.t_start = None
        self.robot_pos = (-13.0, 0.0)
        self.robot_yaw = 0.0
        self.robot_vel = (0.0, 0.0)
        self.latest_cmd = (0.0, 0.0, 0.0)
        self.human_pos = None
        self.human_vel = (0.0, 0.0)

        # Measurements - Robot
        self.dcenter_min = 999.0
        self.dclear_min = 999.0
        self.y_at_min_dclear = 0.0
        self.max_lat_dev = 0.0
        self.min_robot_speed = 999.0

        self.collision_contact = False
        self.collision_wall = False
        self.goal_reached = False
        self.time_to_goal = None
        self.time_to_collision = None

        self.robot_stopped_or_yielded = False
        self.current_stop_duration = 0.0
        self.last_stop_check_time = None

        self.steer_detected = False
        self.t_steer_s = None
        self.d_human_at_steer_m = None

        # Human safety and wall clearance tracking
        self.total_human_ticks = 0
        self.penetration_ticks = 0
        self.max_penetration_depth_m = 0.0
        self.clamped_ticks = 0
        self.max_human_lat_dev = 0.0
        self.plugin_total_steps = 0
        self.plugin_clamped_steps = 0
        self.plugin_penetration_steps = 0
        self.plugin_max_penetration_depth = 0.0

        # cmd_vel tracking
        self.cmd_sim_times = []

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 100)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

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
                self.human_pos = (hx, hy)
                self.human_vel = (h.get("vx", 0.0), h.get("vy", 0.0))

                # Update plugin-reported stats if available
                if "total_steps" in h:
                    self.plugin_total_steps = h.get("total_steps", 0)
                    self.plugin_clamped_steps = h.get("clamped_steps", 0)
                    self.plugin_penetration_steps = h.get("penetration_steps", 0)
                    self.plugin_max_penetration_depth = h.get("max_penetration_depth", 0.0)

                # Independent Monitor Tracking
                self.total_human_ticks += 1
                lat_dev = abs(hy)
                if lat_dev > self.max_human_lat_dev:
                    self.max_human_lat_dev = lat_dev

                # Outer body edge = |y| + 0.28m
                outer_edge_y = lat_dev + 0.28
                if outer_edge_y > self.corridor_half_width:
                    self.penetration_ticks += 1
                    depth = outer_edge_y - self.corridor_half_width
                    if depth > self.max_penetration_depth_m:
                        self.max_penetration_depth_m = depth

                # Clamped ticks: human center touching or exceeding allowed threshold (0.15m or 0.30m)
                if lat_dev >= (self.allowed_human_y - 0.005):
                    self.clamped_ticks += 1
        except Exception:
            pass

    def _cmd_cb(self, msg: TwistStamped):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        wz = msg.twist.angular.z
        self.latest_cmd = (vx, vy, wz)
        if self.t_start is not None and self.sim_time >= self.t_start:
            self.cmd_sim_times.append(self.sim_time)

    def send_goal(self) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target_gx)
        goal_msg.pose.pose.position.y = float(self.target_gy)
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("NavigateToPose action server not available!")
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
        while time.time() - t_w0 < 2.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.04)

        if not self.send_goal():
            return {"error": "Failed to dispatch Nav2 goal", "goal_reached": False}

        # TIMER START: strictly timed from goal acceptance!
        self.t_start = self.sim_time
        t_wall_start = time.time()
        self.last_stop_check_time = self.sim_time
        print(f"    [TIMER STARTED] Sim-time: {self.t_start:.2f}s | Max duration: {self.max_duration_s:.1f}s", flush=True)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            t_rel = self.sim_time - self.t_start
            rx, ry = self.robot_pos
            robot_spd = math.hypot(self.robot_vel[0], self.robot_vel[1])

            # Robot max lateral dev
            if abs(ry) > self.max_lat_dev:
                self.max_lat_dev = abs(ry)

            # Footprint distance & collision
            if self.human_pos is not None:
                hx, hy = self.human_pos
                d_center, d_clear = compute_rect_circle_distance(rx, ry, self.robot_yaw, hx, hy)
                if d_center < self.dcenter_min:
                    self.dcenter_min = d_center
                if d_clear < self.dclear_min:
                    self.dclear_min = d_clear
                    self.y_at_min_dclear = ry

                if d_clear <= 0.0 and not self.collision_contact:
                    self.collision_contact = True
                    self.time_to_collision = round(t_rel, 2)
                    print(f"    [FOOTPRINT COLLISION DETECTED] at t_sim = {t_rel:.2f}s! d_clear: {d_clear:.4f}m", flush=True)
                    break

                # Steering detection
                if not self.steer_detected and t_rel > 0.5:
                    if abs(self.latest_cmd[1]) > 0.05 or abs(ry) > 0.05:
                        self.steer_detected = True
                        self.t_steer_s = round(t_rel, 2)
                        self.d_human_at_steer_m = round(d_center, 3)

            # Stopping / Yielding logic
            if t_rel >= 2.0:
                dt_step = 0.02 if self.last_stop_check_time is None else max(0.001, self.sim_time - self.last_stop_check_time)
                self.last_stop_check_time = self.sim_time
                if robot_spd < 0.05:
                    self.current_stop_duration += dt_step
                    if self.current_stop_duration >= 1.0:
                        self.robot_stopped_or_yielded = True
                else:
                    self.current_stop_duration = 0.0

            # Wall collision for robot
            cos_yaw = math.cos(self.robot_yaw)
            sin_yaw = math.sin(self.robot_yaw)
            corners_y = [
                ry + dx * sin_yaw + dy * cos_yaw
                for dx, dy in [(-0.12, -0.10), (-0.12, 0.10), (0.12, -0.10), (0.12, 0.10)]
            ]
            if max(abs(cy) for cy in corners_y) >= self.corridor_half_width:
                self.collision_wall = True

            # Min robot speed
            dist_to_goal = math.hypot(rx - self.target_gx, ry - self.target_gy)
            if t_rel > 0.5 and dist_to_goal > 0.35:
                if robot_spd < self.min_robot_speed:
                    self.min_robot_speed = robot_spd

            # Goal reached
            if dist_to_goal <= 0.35:
                self.goal_reached = True
                self.time_to_goal = round(t_rel, 2)
                print(f"    [GOAL REACHED] at t_sim = {t_rel:.2f}s! Dist to goal: {dist_to_goal:.3f}m", flush=True)
                break

            # Timeout check: strictly 40.0s sim-time from goal acceptance!
            if t_rel >= self.max_duration_s:
                print(f"    [TIMEOUT] 40.0s sim-time reached from goal dispatch! Final dist: {dist_to_goal:.3f}m", flush=True)
                break
            if time.time() - t_wall_start > (self.max_duration_s + 35.0):
                print(f"    [WATCHDOG TIMEOUT] Wall clock safety limit reached.", flush=True)
                break

        # Compute cmd_vel sim frequency
        cmd_hz_mean = 0.0
        if len(self.cmd_sim_times) >= 2:
            dur_sim = self.cmd_sim_times[-1] - self.cmd_sim_times[0]
            if dur_sim > 0.0:
                cmd_hz_mean = round((len(self.cmd_sim_times) - 1) / dur_sim, 2)

        # Detour around human flag: |y_robot| >= 0.30m at min d_clear
        detour_flag = abs(self.y_at_min_dclear) >= 0.30

        # Near-miss (sượt): 0.0 < d_clear < 0.02 m
        near_miss_flag = (0.0 < self.dclear_min < 0.02) if self.dclear_min < 900 else False

        # Human penetration ratios
        penetration_ratio_pct = round((self.penetration_ticks / max(1, self.total_human_ticks)) * 100.0, 2)
        plugin_penetration_ratio_pct = round((self.plugin_penetration_steps / max(1, self.plugin_total_steps)) * 100.0, 2) if self.plugin_total_steps > 0 else 0.0

        clamped_ratio_pct = round((self.plugin_clamped_steps / max(1, self.plugin_total_steps)) * 100.0, 2) if self.plugin_total_steps > 0 else round((self.clamped_ticks / max(1, self.total_human_ticks)) * 100.0, 2)

        return {
            "goal_reached": self.goal_reached,
            "time_to_goal": self.time_to_goal,
            "collision_contact": self.collision_contact,
            "near_miss_under_2cm": near_miss_flag,
            "collision_wall": self.collision_wall,
            "collision_occurred": self.collision_contact or self.collision_wall,
            "time_to_collision": self.time_to_collision,
            "dmin_edge_to_edge": round(self.dclear_min, 4) if self.dclear_min < 900 else None,
            "dmin_center": round(self.dcenter_min, 4) if self.dcenter_min < 900 else None,
            "y_at_min_dclear": round(self.y_at_min_dclear, 4),
            "detour_around_human": detour_flag,
            "max_lateral_dev": round(self.max_lat_dev, 4),
            "min_robot_speed": round(self.min_robot_speed, 4) if self.min_robot_speed < 900 else 0.0,
            "robot_stopped_or_yielded": self.robot_stopped_or_yielded,
            "steer_detected": self.steer_detected,
            "t_steer_s": self.t_steer_s,
            "d_human_at_steer_m": self.d_human_at_steer_m,
            "cmd_vel_sim_hz_mean": cmd_hz_mean,
            # Human safety & wall clearance metrics
            "human_max_lateral_dev_m": round(self.max_human_lat_dev, 4),
            "human_wall_penetration_step_pct": penetration_ratio_pct,
            "human_max_penetration_depth_m": round(max(self.max_penetration_depth_m, self.plugin_max_penetration_depth), 4),
            "human_max_penetration_depth_cm": round(max(self.max_penetration_depth_m, self.plugin_max_penetration_depth) * 100.0, 2),
            "human_clamped_step_pct": clamped_ratio_pct,
            "human_total_ticks": self.total_human_ticks,
            "human_penetration_ticks": self.penetration_ticks,
            "human_clamped_ticks": self.clamped_ticks,
            "plugin_total_steps": self.plugin_total_steps,
            "plugin_clamped_steps": self.plugin_clamped_steps,
            "plugin_penetration_steps": self.plugin_penetration_steps,
            "final_dist_to_goal": round(math.hypot(self.robot_pos[0] - self.target_gx, self.robot_pos[1] - self.target_gy), 3),
        }


def run_single_trial(corridor_type: str, persona: str, seed: int, v_human: float, domain_id: int, out_json: str):
    gen_dir = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/generated_configs")
    os.makedirs(gen_dir, exist_ok=True)

    nav2_yaml = NAV2_YAML_3B

    # 1. Generate scenario JSON
    scen_file = os.path.join(gen_dir, f"scenario_a3_3_wallfix_{corridor_type}_{persona}_{seed}.json")
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
                "persona": persona,
                "loop": False,
                "wait_for_robot": True,
                "start": {"x": -7.0, "y": 0.0, "yaw": 3.14159265},
                "waypoints": [[-7.0, 0.0], [-15.0, 0.0]]
            }
        ]
    }
    with open(scen_file, "w") as f:
        json.dump(scen_data, f, indent=2)

    # 2. Patch SDF world: real_time_factor = 1.0
    base_sdf_path = os.path.join(ROOT_DIR, f"src/custom_corridor/worlds/corridor_{corridor_type}.sdf")
    with open(base_sdf_path, "r") as f:
        sdf = f.read()

    sdf = re.sub(r'<real_time_factor>.*?</real_time_factor>', '<real_time_factor>1.0</real_time_factor>', sdf)
    sdf = re.sub(
        r'<randomize_politeness>.*?</randomize_politeness>',
        f'<randomize_politeness>false</randomize_politeness>\n      <seed>{seed}</seed>\n      <default_scenario_file>{scen_file}</default_scenario_file>',
        sdf
    )
    if f"<seed>{seed}</seed>" not in sdf:
        sdf = re.sub(r'<seed>\d+</seed>', f'<seed>{seed}</seed>', sdf)

    proxy_z = "0.80" if corridor_type == "090" else "0.875"
    sdf = re.sub(r'<actor name=\"human_01_visual\">\s*<pose>.*?</pose>', '<actor name=\"human_01_visual\">\n      <pose>-7.0 0.0 0.0 0 0 3.14159</pose>', sdf)
    sdf = re.sub(r'<model name=\"human_01_proxy\">\s*<pose>.*?</pose>', f'<model name=\"human_01_proxy\">\n      <pose>-7.0 0.0 {proxy_z} 0 0 3.14159</pose>', sdf)

    trial_sdf = os.path.join(gen_dir, f"world_a3_3_wallfix_{corridor_type}_{persona}_{seed}.sdf")
    with open(trial_sdf, "w") as f:
        f.write(sdf)

    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.dirname(os.path.dirname(plugin_share)) + "/lib"

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["ROS_DOMAIN_ID"] = str(domain_id)
    partition = f"a3_3_wallfix_{corridor_type}_{persona}_{seed}_{domain_id}"
    env["GZ_PARTITION"] = partition

    nav2_env = env.copy()

    all_procs = []
    nav2_log_path = f"/tmp/nav2_a3_3_wallfix_{corridor_type}_{persona}_{seed}_{domain_id}.log"
    log_fp = open(nav2_log_path, "w")

    width_arg = "0.90" if corridor_type == "090" else "1.20"
    map_arg = f"corridor_{corridor_type}"
    corridor_half_w = 0.45 if corridor_type == "090" else 0.60

    result = {}
    try:
        # Launch Sim with Step 1 Robot Model (8m Lidar)
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={width_arg}",
            f"custom_world:={trial_sdf}",
            f"custom_robot_model:={ROBOT_MODEL_SDF}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        gz_p = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(gz_p)
        time.sleep(4.5)

        # Launch Nav2 with Step 3b Config
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={nav2_yaml}",
            f"map:={map_arg}",
            "robot_model:=mecanum"
        ]
        nav2_p = subprocess.Popen(nav2_cmd, env=nav2_env, stdout=log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
        all_procs.append(nav2_p)

        t0 = time.time()
        nav2_ready = False
        while time.time() - t0 < 60.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path) as fp:
                    content = fp.read()
                    if content.count("Managed nodes are active") >= 2:
                        nav2_ready = True
                        break
            time.sleep(0.4)

        if not nav2_ready:
            result = {"error": "nav2_activation_timeout", "goal_reached": False}
        else:
            time.sleep(1.0)
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)
            os.environ["GZ_PARTITION"] = partition
            rclpy.init()
            monitor = WallClearanceInteractionMonitor(
                target_gx=-7.0, target_gy=0.0, max_duration_s=40.0, corridor_half_width=corridor_half_w
            )
            try:
                result = monitor.monitor()
            finally:
                monitor.destroy_node()
                rclpy.shutdown()

    except Exception as e:
        result = {"error": str(e)}
    finally:
        log_fp.close()
        for p in all_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                p.wait(timeout=3.0)
            except Exception:
                try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception: pass

    result["seed"] = seed
    result["corridor"] = corridor_type
    result["corridor_width_m"] = 0.90 if corridor_type == "090" else 1.20
    result["corridor_clearance_m"] = 0.14 if corridor_type == "090" else 0.44
    result["persona"] = persona
    result["v_human"] = v_human
    result["rtf_setting"] = 1.0
    result["step"] = "3b_wall_clearance_fix"

    with open(out_json, "w") as fp:
        json.dump(result, fp, indent=2)


def cleanup_all():
    subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
    subprocess.run(["killall", "-9", "map_server", "amcl", "planner_server", "controller_server",
                    "bt_navigator", "behavior_server", "waypoint_follower", "lifecycle_manager",
                    "parameter_bridge", "static_transform_publisher", "robot_state_publisher"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "nav2_"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "robot_state_publisher"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "corridor_tb3"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ros2"], stderr=subprocess.DEVNULL)
    subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def aggregate_and_report(trials: List[dict], out_file: str):
    corridor_cells = {
        "corridor_090_H1": [t for t in trials if t.get("corridor") == "090" and t.get("persona") == "H1"],
        "corridor_090_H3": [t for t in trials if t.get("corridor") == "090" and t.get("persona") == "H3"],
        "corridor_120_H1": [t for t in trials if t.get("corridor") == "120" and t.get("persona") == "H1"],
        "corridor_120_H3": [t for t in trials if t.get("corridor") == "120" and t.get("persona") == "H3"],
    }

    summary = {}
    for c_key, c_trials in corridor_cells.items():
        n = len(c_trials)
        if n == 0:
            summary[c_key] = {"n_trials": 0}
            continue

        valid_trials = [t for t in c_trials if not t.get("error")]
        n_valid = len(valid_trials)
        if n_valid == 0:
            summary[c_key] = {"n_trials": 0}
            continue

        contact_cols = sum(1 for t in valid_trials if t.get("collision_contact"))
        near_misses = sum(1 for t in valid_trials if t.get("near_miss_under_2cm"))
        goals = sum(1 for t in valid_trials if t.get("goal_reached"))
        detours = sum(1 for t in valid_trials if t.get("detour_around_human"))
        yields = sum(1 for t in valid_trials if t.get("robot_stopped_or_yielded"))

        dclears = [t["dmin_edge_to_edge"] for t in valid_trials if t.get("dmin_edge_to_edge") is not None]
        dclear_med = round(float(np.median(dclears)), 4) if dclears else None

        lat_devs = [t["max_lateral_dev"] for t in valid_trials if t.get("max_lateral_dev") is not None]
        lat_dev_med = round(float(np.median(lat_devs)), 4) if lat_devs else None

        goal_times = [t["time_to_goal"] for t in valid_trials if t.get("time_to_goal") is not None]
        goal_time_med = round(float(np.median(goal_times)), 2) if goal_times else None

        cmd_hzs = [t["cmd_vel_sim_hz_mean"] for t in valid_trials if t.get("cmd_vel_sim_hz_mean") is not None and t.get("cmd_vel_sim_hz_mean") > 0.0]
        cmd_hz_mean = round(float(np.mean(cmd_hzs)), 2) if cmd_hzs else None

        # Human safety & wall penetration stats
        pene_pcts = [t["human_wall_penetration_step_pct"] for t in valid_trials if t.get("human_wall_penetration_step_pct") is not None]
        pene_pct_mean = round(float(np.mean(pene_pcts)), 2) if pene_pcts else 0.0

        pene_depths_cm = [t["human_max_penetration_depth_cm"] for t in valid_trials if t.get("human_max_penetration_depth_cm") is not None]
        max_pene_depth_cm = round(float(max(pene_depths_cm)), 2) if pene_depths_cm else 0.0

        clamped_pcts = [t["human_clamped_step_pct"] for t in valid_trials if t.get("human_clamped_step_pct") is not None]
        clamped_pct_mean = round(float(np.mean(clamped_pcts)), 2) if clamped_pcts else 0.0

        human_lats = [t["human_max_lateral_dev_m"] for t in valid_trials if t.get("human_max_lateral_dev_m") is not None]
        human_lat_med = round(float(np.median(human_lats)), 4) if human_lats else None

        clearance_geom = 0.14 if "090" in c_key else 0.44

        summary[c_key] = {
            "n_trials": n_valid,
            "corridor_width_m": 0.90 if "090" in c_key else 1.20,
            "corridor_clearance_geom_m": clearance_geom,
            "contact_collision_rate_pct": round(contact_cols / n_valid * 100.0, 1),
            "near_miss_under_2cm_rate_pct": round(near_misses / n_valid * 100.0, 1),
            "goal_reach_rate_pct": round(goals / n_valid * 100.0, 1),
            "detour_rate_pct": round(detours / n_valid * 100.0, 1),
            "dmin_clear_median_m": dclear_med,
            "max_lateral_dev_median_m": lat_dev_med,
            "yielded_stop_rate_pct": round(yields / n_valid * 100.0, 1),
            "time_to_goal_median_s": goal_time_med,
            "cmd_vel_sim_hz_mean": cmd_hz_mean,
            # Human wall metrics
            "human_penetration_step_pct_mean": pene_pct_mean,
            "human_max_penetration_depth_cm": max_pene_depth_cm,
            "human_clamped_step_pct_mean": clamped_pct_mean,
            "human_max_lateral_dev_median_m": human_lat_med,
        }

    final_payload = {
        "benchmark": "A3.3 Wall Clearance Fix Verification",
        "robot_config": "Step 3b (time_steps = 80, horizon 4.0s, NCC = 253, vision 8m/7m/10m, inflation 0.20/10)",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "hardware": "AMD Ryzen 7 7735H, RTX 4060 Laptop GPU",
        "timer_rule": "max_duration_s = 40.0s sim-time strictly measured from goal acceptance",
        "geometric_clearances": {
            "corridor_090_m": 0.14,
            "corridor_120_m": 0.44
        },
        "trials": trials,
        "summary": summary,
    }

    with open(out_file, "w") as fp:
        json.dump(final_payload, fp, indent=2)
    print(f"\n[OK] Raw results saved to: {out_file}", flush=True)

    print("\n" + "=" * 135, flush=True)
    print(f"{'BẢNG TỔNG HỢP KIỂM TRA LỖI NGƯỜI LẤN TƯỜNG (A3.3, CẤU HÌNH 3B)':^135}", flush=True)
    print("=" * 135, flush=True)
    print(f"{'Chỉ số':<52} | {'0.90m H1':<17} | {'0.90m H3':<17} | {'1.20m H1':<17} | {'1.20m H3':<17}")
    print("-" * 135, flush=True)

    rows = [
        ("Khe hở hình học còn lại (W - 0.56 - 0.20, m)", "corridor_clearance_geom_m", "{:.2f} m"),
        ("Tỉ lệ bước người lấn tường (%) [Mục tiêu: < 1%]", "human_penetration_step_pct_mean", "{:.2f}%"),
        ("Độ sâu lấn tường lớn nhất (cm) [Mục tiêu: < 1cm]", "human_max_penetration_depth_cm", "{:.2f} cm"),
        ("Tỉ lệ bước phải kẹp (clamp) (%)", "human_clamped_step_pct_mean", "{:.2f}%"),
        ("Độ lệch ngang tối đa của người (m)", "human_max_lateral_dev_median_m", "{:.4f} m"),
        ("Tỉ lệ va chạm footprint (%)", "contact_collision_rate_pct", "{:.1f}%"),
        ("Tỉ lệ sượt (0 < d_clear < 0.02m) (%)", "near_miss_under_2cm_rate_pct", "{:.1f}%"),
        ("Tỉ lệ tới đích (%)", "goal_reach_rate_pct", "{:.1f}%"),
        ("d_clear nhỏ nhất mép-mép (m)", "dmin_clear_median_m", "{:.4f} m"),
        ("Độ lệch ngang lớn nhất của robot (m)", "max_lateral_dev_median_m", "{:.3f} m"),
        ("Tỉ lệ robot đi vòng qua người (|y|>=0.30m) (%)", "detour_rate_pct", "{:.1f}%"),
        ("Tỉ lệ robot dừng/nhường (%)", "yielded_stop_rate_pct", "{:.1f}%"),
        ("Thời gian tới đích trung vị (s)", "time_to_goal_median_s", "{:.2f} s"),
        ("Tần số cmd_vel sim thực (Mean Hz)", "cmd_vel_sim_hz_mean", "{:.1f} Hz"),
    ]

    for label, key, fmt in rows:
        row_str = f"{label:<52} | "
        for c_key in ["corridor_090_H1", "corridor_090_H3", "corridor_120_H1", "corridor_120_H3"]:
            val = summary.get(c_key, {}).get(key)
            val_str = fmt.format(val) if val is not None else "N/A"
            row_str += f"{val_str:<17} | "
        print(row_str, flush=True)
    print("=" * 135, flush=True)


def main():
    parser = argparse.ArgumentParser(description="Wall Clearance Benchmark Runner")
    parser.add_argument("--run_single", action="store_true")
    parser.add_argument("--corridor", type=str, default="090")
    parser.add_argument("--persona", type=str, default="H1")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--v_human", type=float, default=1.15)
    parser.add_argument("--domain_id", type=int, default=85)
    parser.add_argument("--out_json", type=str, default="/tmp/res_single_wall.json")
    parser.add_argument("--aggregate_only", action="store_true")
    args = parser.parse_args()

    out_final_json = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/a3_3_wall_clearance_results.json")

    if args.run_single:
        run_single_trial(args.corridor, args.persona, args.seed, args.v_human, args.domain_id, args.out_json)
        os._exit(0)

    cells = [
        {"corridor": "090", "persona": "H1", "seed_base": 101, "name": "090_H1"},
        {"corridor": "090", "persona": "H3", "seed_base": 201, "name": "090_H3"},
        {"corridor": "120", "persona": "H1", "seed_base": 301, "name": "120_H1"},
        {"corridor": "120", "persona": "H3", "seed_base": 401, "name": "120_H3"},
    ]

    print("=" * 105, flush=True)
    print("A3.3 BENCHMARK: HUMAN WALL CLEARANCE & ANTI-PENETRATION FIX (ROBOT CONFIG 3b)", flush=True)
    print("Timer rule: max_duration_s = 40.0s sim-time strictly measured from goal acceptance", flush=True)
    print("Rule: max lateral dev of human = corridor_half_width - 0.28 - 0.02 (0.15m at 0.9m, 0.30m at 1.2m)", flush=True)
    print("=" * 105, flush=True)

    if not args.aggregate_only:
        cleanup_all()
        runs_to_do = []
        for c in cells:
            c_type = c["corridor"]
            persona = c["persona"]
            s_base = c["seed_base"]
            for i in range(10):
                seed = s_base + i
                rng = np.random.RandomState(seed)
                v_h = round(float(rng.uniform(1.0, 1.3)), 3)
                out_f = f"/tmp/res_a3_3_wallfix_{c_type}_{persona}_{seed}.json"
                runs_to_do.append((c_type, persona, seed, v_h, out_f))

        print(f">>> Executing {len(runs_to_do)} trials sequentially with Robot Config 3b...", flush=True)
        for idx, (c_type, persona, seed, v_h, out_f) in enumerate(runs_to_do, 1):
            if os.path.exists(out_f):
                try:
                    with open(out_f) as fp:
                        res = json.load(fp)
                    if not res.get("error") and res.get("time_to_collision") != 0.0:
                        print(f"[{idx}/{len(runs_to_do)}] Seed {seed} ({c_type}_{persona}) already done. Skipping.", flush=True)
                        continue
                except Exception:
                    pass

            print(f"\n[{idx}/{len(runs_to_do)}] Running Wall Fix: Corridor {c_type}, Persona {persona}, Seed {seed} (v_human={v_h:.3f} m/s)...", flush=True)
            cleanup_all()
            cmd = [
                sys.executable, __file__,
                "--run_single",
                "--corridor", str(c_type),
                "--persona", str(persona),
                "--seed", str(seed),
                "--v_human", str(v_h),
                "--domain_id", "85",
                "--out_json", str(out_f)
            ]
            t_s = time.time()
            try:
                subprocess.run(cmd, check=True)
            except Exception as e:
                print(f"[ERROR] Trial failed for seed {seed}: {e}", flush=True)
            cleanup_all()
            t_dur = time.time() - t_s
            if os.path.exists(out_f):
                with open(out_f) as fp:
                    d = json.load(fp)
                print(f"  -> Done in {t_dur:.1f}s | Goal: {d.get('goal_reached')} | Contact: {d.get('collision_contact')} | "
                      f"dmin_clear: {d.get('dmin_edge_to_edge')}m | "
                      f"Human lat: {d.get('human_max_lateral_dev_m')}m | Pene: {d.get('human_wall_penetration_step_pct')}% (max {d.get('human_max_penetration_depth_cm')}cm) | "
                      f"Clamped: {d.get('human_clamped_step_pct')}%", flush=True)
            time.sleep(1.0)

    # Gather results
    trials = []
    for c in cells:
        c_type = c["corridor"]
        persona = c["persona"]
        s_base = c["seed_base"]
        for i in range(10):
            seed = s_base + i
            out_file = f"/tmp/res_a3_3_wallfix_{c_type}_{persona}_{seed}.json"
            if os.path.exists(out_file):
                with open(out_file) as fp:
                    trials.append(json.load(fp))

    aggregate_and_report(trials, out_final_json)


if __name__ == '__main__':
    main()
