#!/usr/bin/env python3
"""
Step 2 Benchmark Runner (2a and 2b):
- Step 2a: inflation_radius = 0.20, cost_scaling_factor = 10.0 in local and global costmaps.
           near_collision_cost = 140 (unchanged), footprint 0.24x0.20 (unchanged).
- Step 2b: From 2a, near_collision_cost = 253 (Nav2 default).
- Model: Lidar 8m (model_mecanum_step1.sdf), local costmap 10x10m, obstacle range 7m (from Step 1).
- 40 trials: corridor 0.9m & 1.2m x H1 & H3 x 10 seeds (101-110, 201-210, 301-310, 401-410). RTF = 1.0.
"""

import os
import sys
import time
import math
import signal
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
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

ROOT_DIR = "/home/cuongbien/DynamuhSim"
ROBOT_MODEL_SDF = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/robot_models/model_mecanum_step1.sdf")


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


class Step2InteractionMonitor(Node):
    def __init__(self, target_gx=-7.0, target_gy=0.0, max_duration_s=25.0, corridor_half_width=0.45):
        super().__init__('step2_interaction_monitor')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_gx = target_gx
        self.target_gy = target_gy
        self.max_duration_s = max_duration_s
        self.corridor_half_width = corridor_half_width

        self.sim_time = 0.0
        self.t_start = None
        self.robot_pos = (-13.0, 0.0)
        self.robot_yaw = 0.0
        self.robot_vel = (0.0, 0.0)
        self.human_pos = None
        self.human_vel = (0.0, 0.0)

        # Measurements
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

        # Stopping / Yielding logic (v < 0.05 m/s continuously >= 1.0 s, after t = 2.0 s)
        self.robot_stopped_or_yielded = False
        self.current_stop_duration = 0.0
        self.last_stop_check_time = None

        # Frozen at start logic (v < 0.02 m/s continuously during first 2.0 s)
        self.max_speed_first_2s = 0.0
        self.frozen_at_start = False

        # First costmap detection tracking
        self.first_costmap_dist = None
        self.first_costmap_t = None
        self.latest_costmap = None

        # 2m Snapshot tracking
        self.snapshot_2m = None

        # cmd_vel tracking
        self.cmd_sim_times = []

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 100)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _costmap_cb(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def _odom_cb(self, msg: Odometry):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _cmd_cb(self, msg: TwistStamped):
        s_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if s_t == 0.0:
            s_t = self.sim_time
        self.cmd_sim_times.append(s_t)

    def _gt_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.robot_pos = (data['robot']['x'], data['robot']['y'])
                self.robot_vel = (data['robot'].get('vx', 0.0), data['robot'].get('vy', 0.0))

            if 'humans' in data and len(data['humans']) > 0:
                h = data['humans'][0]
                if h.get('x') is not None and h.get('y') is not None:
                    self.human_pos = (h['x'], h['y'])
                    self.human_vel = (h.get('vx', 0.0), h.get('vy', 0.0))
        except Exception:
            pass

    def get_cost_at_point(self, costmap: OccupancyGrid, px_odom: float, py_odom: float) -> Optional[int]:
        ox = costmap.info.origin.position.x
        oy = costmap.info.origin.position.y
        res = costmap.info.resolution
        w = costmap.info.width
        h = costmap.info.height
        mx = int(round((px_odom - ox) / res))
        my = int(round((py_odom - oy) / res))
        if 0 <= mx < w and 0 <= my < h:
            return int(costmap.data[my * w + mx])
        return None

    def evaluate_lanes_at_2m(self, costmap: OccupancyGrid, rx: float, ry: float, hx: float, hy: float) -> dict:
        """
        Evaluates cost at left lane (y = +0.32) and right lane (y = -0.32) at human x,
        and min cost along the lanes between robot x and human x.
        Robot spawned at x = -13.0 in map, which corresponds to 0.0 in odom frame.
        So odom_x = wx + 13.0, odom_y = wy.
        """
        hcx = hx + 13.0
        hcy = hy

        cost_left_at_human = self.get_cost_at_point(costmap, hcx, +0.32)
        cost_right_at_human = self.get_cost_at_point(costmap, hcx, -0.32)

        # Profile along lanes between robot x and human x
        x_min = min(rx + 13.0, hcx)
        x_max = max(rx + 13.0, hcx)
        xs = np.linspace(x_min, x_max, max(3, int((x_max - x_min) / 0.10) + 1))

        left_costs = []
        right_costs = []
        for xv in xs:
            cl = self.get_cost_at_point(costmap, xv, +0.32)
            cr = self.get_cost_at_point(costmap, xv, -0.32)
            if cl is not None and cl >= 0:
                left_costs.append(cl)
            if cr is not None and cr >= 0:
                right_costs.append(cr)

        return {
            "cost_left_lane_at_human": cost_left_at_human,
            "cost_right_lane_at_human": cost_right_at_human,
            "min_cost_left_lane_profile": int(np.min(left_costs)) if left_costs else None,
            "min_cost_right_lane_profile": int(np.min(right_costs)) if right_costs else None,
        }

    def send_nav_goal(self) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target_gx)
        goal_msg.pose.pose.position.y = float(self.target_gy)
        goal_msg.pose.pose.orientation.w = 1.0

        for attempt in range(5):
            future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done():
                handle = future.result()
                if handle.accepted:
                    return True
            time.sleep(0.5)
        return False

    def run_navigation(self) -> dict:
        t_w0 = time.time()
        while time.time() - t_w0 < 2.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.04)

        if not self.send_nav_goal():
            return {"error": "goal_rejected_or_timeout", "goal_reached": False}

        self.t_start = self.sim_time
        t_wall_start = time.time()
        self.last_stop_check_time = self.sim_time

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            t_rel = self.sim_time - self.t_start

            rx, ry = self.robot_pos
            ryaw = self.robot_yaw
            robot_spd = math.hypot(self.robot_vel[0], self.robot_vel[1])

            # Lateral deviation
            lat_dev = abs(ry)
            if lat_dev > self.max_lat_dev:
                self.max_lat_dev = lat_dev

            # Frozen at start check: max speed during first 2.0s
            if t_rel <= 2.0:
                if robot_spd > self.max_speed_first_2s:
                    self.max_speed_first_2s = robot_spd
            elif t_rel > 2.0 and not hasattr(self, '_checked_frozen'):
                self._checked_frozen = True
                if self.max_speed_first_2s < 0.02:
                    self.frozen_at_start = True

            # Stopping / Yielding check: v < 0.05 m/s continuously >= 1.0s, after t = 2.0s
            if t_rel >= 2.0:
                dt_stop = self.sim_time - self.last_stop_check_time if self.last_stop_check_time else 0.0
                if dt_stop > 0.0 and dt_stop < 0.5:
                    if robot_spd < 0.05:
                        self.current_stop_duration += dt_stop
                        if self.current_stop_duration >= 1.0:
                            self.robot_stopped_or_yielded = True
                    else:
                        self.current_stop_duration = 0.0
                self.last_stop_check_time = self.sim_time

            # Collision & encounter tracking
            cur_dc = 999.0
            cur_dclear = 999.0
            if self.human_pos is not None:
                hx, hy = self.human_pos
                cur_dc, cur_dclear = compute_rect_circle_distance(rx, ry, ryaw, hx, hy)
                if cur_dc < self.dcenter_min:
                    self.dcenter_min = cur_dc
                if cur_dclear < self.dclear_min:
                    self.dclear_min = cur_dclear
                    self.y_at_min_dclear = ry

                # First costmap detection check (search within 0.35m of human)
                if self.first_costmap_dist is None and self.latest_costmap is not None and t_rel > 0.5:
                    ox = self.latest_costmap.info.origin.position.x
                    oy = self.latest_costmap.info.origin.position.y
                    res = self.latest_costmap.info.resolution
                    w_cm = self.latest_costmap.info.width
                    h_cm = self.latest_costmap.info.height
                    hcx = hx + 13.0
                    hcy = hy
                    mx0 = int((hcx - ox) / res)
                    my0 = int((hcy - oy) / res)
                    r_c = int(0.35 / res)
                    max_c = 0
                    for dx_i in range(-r_c, r_c + 1):
                        for dy_i in range(-r_c, r_c + 1):
                            cx = mx0 + dx_i
                            cy = my0 + dy_i
                            if 0 <= cx < w_cm and 0 <= cy < h_cm:
                                val = self.latest_costmap.data[cy * w_cm + cx]
                                if val > max_c:
                                    max_c = val
                    if max_c >= 99:
                        self.first_costmap_dist = round(cur_dc, 3)
                        self.first_costmap_t = round(t_rel, 2)

                # Snapshot at d ~ 2m
                if cur_dc <= 2.10 and self.snapshot_2m is None and self.latest_costmap is not None and t_rel > 0.5:
                    self.snapshot_2m = self.evaluate_lanes_at_2m(self.latest_costmap, rx, ry, hx, hy)

                # Footprint contact collision: d_clear <= 0.0 m (only after t_rel > 0.5)
                if t_rel > 0.5 and cur_dclear <= 0.0:
                    self.collision_contact = True
                    self.time_to_collision = round(t_rel, 2)
                    print(f"    [COLLISION DETECTED] Footprint contact (d_clear={cur_dclear:.4f}m) at t_sim={t_rel:.2f}s! Truncating episode.", flush=True)
                    break

            # Wall collision check
            max_corner_y = 0.0
            for dx_c, dy_c in [(0.12, 0.10), (0.12, -0.10), (-0.12, 0.10), (-0.12, -0.10)]:
                c_y = ry + dx_c * math.sin(ryaw) + dy_c * math.cos(ryaw)
                if abs(c_y) > max_corner_y:
                    max_corner_y = abs(c_y)
            if max_corner_y >= self.corridor_half_width:
                self.collision_wall = True

            # Min robot speed during active encounter
            dist_to_goal = math.hypot(rx - self.target_gx, ry - self.target_gy)
            if t_rel > 0.5 and dist_to_goal > 0.35:
                if robot_spd < self.min_robot_speed:
                    self.min_robot_speed = robot_spd

            # Goal reached check
            if dist_to_goal <= 0.35:
                self.goal_reached = True
                self.time_to_goal = round(t_rel, 2)
                print(f"    [GOAL REACHED] at t_sim = {t_rel:.2f}s! Dist to goal: {dist_to_goal:.3f}m", flush=True)
                break

            # Timeout check
            if t_rel >= self.max_duration_s:
                break
            if time.time() - t_wall_start > (self.max_duration_s + 20.0):
                break

        # Compute cmd_vel sim frequencies
        cmd_hz_mean = 0.0
        cmd_hz_min = 0.0
        if len(self.cmd_sim_times) >= 2:
            dur_sim = self.cmd_sim_times[-1] - self.cmd_sim_times[0]
            if dur_sim > 0.0:
                cmd_hz_mean = round((len(self.cmd_sim_times) - 1) / dur_sim, 2)

            dts = [self.cmd_sim_times[i] - self.cmd_sim_times[i-1] for i in range(1, len(self.cmd_sim_times))]
            valid_dts = [dt for dt in dts if dt > 0.001]
            if valid_dts:
                cmd_hz_min = round(1.0 / max(valid_dts), 2)

        # Detour around human flag: |y_robot| >= 0.30m at min d_clear
        detour_flag = abs(self.y_at_min_dclear) >= 0.30

        # Snapshot fallback
        snap_left = None
        snap_right = None
        min_left_prof = None
        min_right_prof = None
        if self.snapshot_2m:
            snap_left = self.snapshot_2m.get("cost_left_lane_at_human")
            snap_right = self.snapshot_2m.get("cost_right_lane_at_human")
            min_left_prof = self.snapshot_2m.get("min_cost_left_lane_profile")
            min_right_prof = self.snapshot_2m.get("min_cost_right_lane_profile")

        return {
            "goal_reached": self.goal_reached,
            "time_to_goal": self.time_to_goal,
            "collision_contact": self.collision_contact,
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
            "frozen_at_start": self.frozen_at_start,
            "cmd_vel_count": len(self.cmd_sim_times),
            "cmd_vel_sim_hz_mean": cmd_hz_mean,
            "cmd_vel_sim_hz_min": cmd_hz_min,
            "first_costmap_dist_m": self.first_costmap_dist,
            "first_costmap_t_s": self.first_costmap_t,
            "cost_left_lane_at_2m": snap_left,
            "cost_right_lane_at_2m": snap_right,
            "min_cost_left_lane_profile_at_2m": min_left_prof,
            "min_cost_right_lane_profile_at_2m": min_right_prof,
            "final_dist_to_goal": round(math.hypot(self.robot_pos[0] - self.target_gx, self.robot_pos[1] - self.target_gy), 3),
        }


def run_single_trial(config_mode: str, corridor_type: str, persona: str, seed: int, v_human: float, domain_id: int, out_json: str):
    gen_dir = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated")
    os.makedirs(gen_dir, exist_ok=True)

    nav2_yaml = os.path.join(ROOT_DIR, f"experiments/gazebo_benchmark/configs/nav2_mppi/nav2_mppi_step{config_mode}.yaml")

    # 1. Generate scenario JSON
    scen_file = os.path.join(gen_dir, f"scenario_a3_3_step{config_mode}_{corridor_type}_{persona}_{seed}.json")
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

    trial_sdf = os.path.join(gen_dir, f"world_a3_3_step{config_mode}_{corridor_type}_{persona}_{seed}.sdf")
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
    partition = f"a3_3_step{config_mode}_{corridor_type}_{persona}_{seed}_{domain_id}"
    env["GZ_PARTITION"] = partition

    all_procs = []
    nav2_log_path = f"/tmp/nav2_a3_3_step{config_mode}_{corridor_type}_{persona}_{seed}_{domain_id}.log"
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

        # Launch Nav2 with Step 2 Config
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={nav2_yaml}",
            f"map:={map_arg}",
            "robot_model:=mecanum"
        ]
        nav2_p = subprocess.Popen(nav2_cmd, env=env, stdout=log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
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
            monitor = Step2InteractionMonitor(target_gx=-7.0, target_gy=0.0, max_duration_s=25.0, corridor_half_width=corridor_half_w)
            result = monitor.run_navigation()
            monitor.destroy_node()
            rclpy.shutdown()

    finally:
        for p in all_procs:
            try: os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception: pass
        time.sleep(1.0)
        for p in all_procs:
            try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception: pass
        log_fp.close()
        subprocess.run(["pkill", "-9", "-f", f"domain_id={domain_id}|gz sim|corridor_tb3|nav2_corridor"], stderr=subprocess.DEVNULL)
        time.sleep(1.0)

    result["seed"] = seed
    result["corridor"] = f"corridor_{corridor_type}"
    result["corridor_width_m"] = 0.90 if corridor_type == "090" else 1.20
    result["persona"] = persona
    result["v_human"] = v_human
    result["rtf_setting"] = 1.0
    result["step"] = f"Step {config_mode}"

    with open(out_json, "w") as fp:
        json.dump(result, fp, indent=2)


def cleanup_all():
    subprocess.run(["pkill", "-9", "-f", "gz sim|corridor_tb3|nav2_corridor|rviz2|robot_state_publisher|component_container_isolated|gz-sim"], stderr=subprocess.DEVNULL)
    time.sleep(1.5)


def aggregate_and_report(trials: list, config_mode: str, out_file: str):
    valid_trials = [t for t in trials if not t.get("error")]
    print(f"\n=======================================================", flush=True)
    print(f"STEP {config_mode.upper()} BENCHMARK AGGREGATION ({len(valid_trials)}/{len(trials)} Valid Trials)", flush=True)
    print(f"=======================================================", flush=True)

    summary = {}
    for c_type in ["corridor_090", "corridor_120"]:
        for persona in ["H1", "H3"]:
            key = f"{c_type}_{persona}"
            c_trials = [t for t in valid_trials if t["corridor"] == c_type and t["persona"] == persona]
            n = len(c_trials)
            if n == 0:
                continue

            contact_collisions = sum(1 for t in c_trials if t.get("collision_contact"))
            contact_rate = round(contact_collisions / n * 100.0, 1)

            goal_reaches = sum(1 for t in c_trials if t.get("goal_reached"))
            goal_rate = round(goal_reaches / n * 100.0, 1)

            detours = sum(1 for t in c_trials if t.get("detour_around_human"))
            detour_rate = round(detours / n * 100.0, 1)

            dmin_clears = [t["dmin_edge_to_edge"] for t in c_trials if t.get("dmin_edge_to_edge") is not None]
            dmin_clear_med = round(float(np.median(dmin_clears)), 4) if dmin_clears else None

            max_lat_devs = [t["max_lateral_dev"] for t in c_trials if t.get("max_lateral_dev") is not None]
            max_lat_med = round(float(np.median(max_lat_devs)), 4) if max_lat_devs else None

            yielded_stops = sum(1 for t in c_trials if t.get("robot_stopped_or_yielded"))
            yielded_rate = round(yielded_stops / n * 100.0, 1)

            frozen_starts = sum(1 for t in c_trials if t.get("frozen_at_start"))

            reach_times = [t["time_to_goal"] for t in c_trials if t.get("goal_reached") and t.get("time_to_goal") is not None]
            t_goal_med = round(float(np.median(reach_times)), 2) if reach_times else None

            hz_means = [t["cmd_vel_sim_hz_mean"] for t in c_trials if t.get("cmd_vel_sim_hz_mean") and t["cmd_vel_sim_hz_mean"] > 0]
            hz_mean_val = round(float(np.mean(hz_means)), 2) if hz_means else None

            first_costs = [t["first_costmap_dist_m"] for t in c_trials if t.get("first_costmap_dist_m") is not None]
            first_cost_med = round(float(np.median(first_costs)), 2) if first_costs else None

            first_ts = [t["first_costmap_t_s"] for t in c_trials if t.get("first_costmap_t_s") is not None]
            first_t_med = round(float(np.median(first_ts)), 2) if first_ts else None

            costs_left_2m = [t["cost_left_lane_at_2m"] for t in c_trials if t.get("cost_left_lane_at_2m") is not None]
            cost_left_2m_med = round(float(np.median(costs_left_2m)), 1) if costs_left_2m else None

            costs_right_2m = [t["cost_right_lane_at_2m"] for t in c_trials if t.get("cost_right_lane_at_2m") is not None]
            cost_right_2m_med = round(float(np.median(costs_right_2m)), 1) if costs_right_2m else None

            min_left_profs = [t["min_cost_left_lane_profile_at_2m"] for t in c_trials if t.get("min_cost_left_lane_profile_at_2m") is not None]
            min_left_prof_med = round(float(np.median(min_left_profs)), 1) if min_left_profs else None

            min_right_profs = [t["min_cost_right_lane_profile_at_2m"] for t in c_trials if t.get("min_cost_right_lane_profile_at_2m") is not None]
            min_right_prof_med = round(float(np.median(min_right_profs)), 1) if min_right_profs else None

            summary[key] = {
                "n_trials": n,
                "contact_collision_rate_pct": contact_rate,
                "goal_reach_rate_pct": goal_rate,
                "detour_rate_pct": detour_rate,
                "dmin_clear_median_m": dmin_clear_med,
                "max_lateral_dev_median_m": max_lat_med,
                "yielded_stop_rate_pct": yielded_rate,
                "frozen_at_start_count": frozen_starts,
                "time_to_goal_median_s": t_goal_med,
                "cmd_vel_sim_hz_mean": hz_mean_val,
                "first_costmap_dist_median_m": first_cost_med,
                "first_costmap_t_median_s": first_t_med,
                "cost_left_lane_at_2m_median": cost_left_2m_med,
                "cost_right_lane_at_2m_median": cost_right_2m_med,
                "min_cost_left_lane_profile_at_2m_median": min_left_prof_med,
                "min_cost_right_lane_profile_at_2m_median": min_right_prof_med,
            }

    final_payload = {
        "step": f"Step {config_mode}",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "hardware": "AMD Ryzen 7 7735H, RTX 4060 Laptop GPU",
        "trials": trials,
        "summary": summary,
    }

    with open(out_file, "w") as fp:
        json.dump(final_payload, fp, indent=2)
    print(f"\n[OK] Raw results saved to: {out_file}", flush=True)

    print("\n" + "=" * 120, flush=True)
    print(f"{f'BẢNG TỔNG HỢP A3.3 BƯỚC {config_mode.upper()}':^120}", flush=True)
    print("=" * 120, flush=True)
    print(f"{'Chỉ số':<42} | {'0.90m H1':<16} | {'0.90m H3':<16} | {'1.20m H1':<16} | {'1.20m H3':<16}")
    print("-" * 120, flush=True)

    rows = [
        ("Tỉ lệ va chạm footprint (%)", "contact_collision_rate_pct", "{:.1f}%"),
        ("Tỉ lệ tới đích (%)", "goal_reach_rate_pct", "{:.1f}%"),
        ("d_clear nhỏ nhất mép-mép (m)", "dmin_clear_median_m", "{:.4f}m"),
        ("Độ lệch ngang lớn nhất của robot (m)", "max_lateral_dev_median_m", "{:.3f}m"),
        ("Tỉ lệ robot đi vòng qua người (|y|>=0.30m) (%)", "detour_rate_pct", "{:.1f}%"),
        ("Tỉ lệ robot dừng/nhường (%)", "yielded_stop_rate_pct", "{:.1f}%"),
        ("Thời gian tới đích trung vị (s)", "time_to_goal_median_s", "{:.2f}s"),
        ("Tần số cmd_vel sim (Mean Hz)", "cmd_vel_sim_hz_mean", "{:.1f} Hz"),
        ("Cự ly người vào costmap (m)", "first_costmap_dist_median_m", "{:.2f}m"),
        ("Thời điểm người vào costmap (s)", "first_costmap_t_median_s", "{:.2f}s"),
        ("Cost làn trái tại vị trí người (d~2m)", "cost_left_lane_at_2m_median", "{}"),
        ("Cost làn phải tại vị trí người (d~2m)", "cost_right_lane_at_2m_median", "{}"),
        ("Min cost profile làn trái (d~2m)", "min_cost_left_lane_profile_at_2m_median", "{}"),
        ("Min cost profile làn phải (d~2m)", "min_cost_right_lane_profile_at_2m_median", "{}"),
    ]

    for label, key, fmt in rows:
        row_str = f"{label:<42} | "
        for c_key in ["corridor_090_H1", "corridor_090_H3", "corridor_120_H1", "corridor_120_H3"]:
            val = summary.get(c_key, {}).get(key)
            val_str = fmt.format(val) if val is not None else "N/A"
            row_str += f"{val_str:<16} | "
        print(row_str, flush=True)
    print("=" * 120, flush=True)


def main():
    parser = argparse.ArgumentParser(description="Step 2 Benchmark Runner")
    parser.add_argument("--config", type=str, required=True, choices=["2a", "2b"])
    parser.add_argument("--run_single", action="store_true")
    parser.add_argument("--corridor", type=str, default="090")
    parser.add_argument("--persona", type=str, default="H1")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--v_human", type=float, default=1.15)
    parser.add_argument("--domain_id", type=int, default=85)
    parser.add_argument("--out_json", type=str, default="/tmp/res_single.json")
    parser.add_argument("--aggregate_only", action="store_true")
    args = parser.parse_args()

    out_final_json = os.path.join(ROOT_DIR, f"experiments/gazebo_benchmark/results/mppi_ablation/a3_3_step{args.config}_results.json")

    if args.run_single:
        run_single_trial(args.config, args.corridor, args.persona, args.seed, args.v_human, args.domain_id, args.out_json)
        os._exit(0)

    cells = [
        {"corridor": "090", "persona": "H1", "seed_base": 101, "name": "090_H1"},
        {"corridor": "090", "persona": "H3", "seed_base": 201, "name": "090_H3"},
        {"corridor": "120", "persona": "H1", "seed_base": 301, "name": "120_H1"},
        {"corridor": "120", "persona": "H3", "seed_base": 401, "name": "120_H3"},
    ]

    print("=" * 90, flush=True)
    print(f"STEP {args.config.upper()} BENCHMARK: INFLATION & NEAR COLLISION ABLATION", flush=True)
    print("=" * 90, flush=True)

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
                out_f = f"/tmp/res_a3_3_step{args.config}_{c_type}_{persona}_{seed}.json"
                runs_to_do.append((c_type, persona, seed, v_h, out_f))

        print(f">>> Executing {len(runs_to_do)} trials sequentially for Step {args.config}...", flush=True)
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

            print(f"\n[{idx}/{len(runs_to_do)}] Running Step {args.config}: Corridor {c_type}, Persona {persona}, Seed {seed} (v_human={v_h:.3f} m/s)...", flush=True)
            cleanup_all()
            cmd = [
                sys.executable, __file__,
                "--config", str(args.config),
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
                      f"dmin_clear: {d.get('dmin_edge_to_edge')}m | Detour: {d.get('detour_around_human')} (y={d.get('y_at_min_dclear')}m) | "
                      f"Left lane cost 2m: {d.get('cost_left_lane_at_2m')} | Right lane cost 2m: {d.get('cost_right_lane_at_2m')}", flush=True)
            time.sleep(1.0)

    # Gather results
    trials = []
    for c in cells:
        c_type = c["corridor"]
        persona = c["persona"]
        s_base = c["seed_base"]
        for i in range(10):
            seed = s_base + i
            out_file = f"/tmp/res_a3_3_step{args.config}_{c_type}_{persona}_{seed}.json"
            if os.path.exists(out_file):
                with open(out_file) as fp:
                    trials.append(json.load(fp))

    aggregate_and_report(trials, args.config, out_final_json)


if __name__ == '__main__':
    main()
