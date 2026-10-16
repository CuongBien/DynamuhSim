#!/usr/bin/env python3
"""
A3.3 Real Interaction Benchmark with RTF = 1.0 (Real-Time Lockstep).

Key configurations:
- real_time_factor = 1.0 in world SDF.
- Baseline Nav2 MPPI config: 8 critics, batch_size 280, time_steps 30, controller_frequency 20.0 Hz.
- Local costmap 6x6 m, resolution 0.05 m, rectangular footprint 0.24 x 0.20 m.
- 40 runs:
  - corridor_090, H1: seeds 101-110 (10 runs)
  - corridor_090, H3: seeds 201-210 (10 runs)
  - corridor_120, H1: seeds 301-310 (10 runs)
  - corridor_120, H3: seeds 401-410 (10 runs)
- Metrics measured per run:
  - Exact cmd_vel sim-time frequency (mean & min).
  - Footprint contact collision flag (d_clear <= 0.0 m) vs cylinder human proxy (r = 0.28 m).
  - Episode truncation on first collision.
  - Goal reach status & time to goal.
  - Minimum edge-to-edge distance (dmin mép-mép).
  - Maximum robot lateral deviation (|y|).
  - Robot stopped/yielded rate (v < 0.05 m/s continuous >= 1.0 s, after t = 2.0 s).
  - Robot frozen at start flag (v < 0.02 m/s continuously during first 2 s).
- Outputs:
  - Raw JSON: experiments/gazebo_benchmark/results/correctness_a3/a3_3_rtf1_real_interaction_results.json
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
from typing import Tuple, Dict, List
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

ROOT_DIR = "/home/cuongbien/DynamuhSim"
OUTPUT_JSON = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/correctness_a3/a3_3_rtf1_real_interaction_results.json")
OLD_JSON = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/correctness_a3/a3_3_real_interaction_results.json")


def compute_rect_circle_distance(rx: float, ry: float, ryaw: float, hx: float, hy: float, rw=0.20, rl=0.24, hr=0.28) -> Tuple[float, float]:
    """
    Computes center-to-center distance and exact edge-to-edge clearance
    between oriented rectangular robot (rl x rw) and cylindrical human (radius hr).
    """
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


class A33RTF1InteractionMonitor(Node):
    def __init__(self, target_gx=-7.0, target_gy=0.0, max_duration_s=25.0, corridor_half_width=0.45):
        super().__init__('a3_3_rtf1_interaction_monitor')
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
        self.human_speed = 0.0

        # Measurements
        self.dcenter_min = 999.0
        self.dclear_min = 999.0
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

        # cmd_vel tracking
        self.cmd_sim_times = []  # sim timestamps of cmd_vel
        self.cmd_wall_times = []

        self.telemetry = []

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

    def _cmd_cb(self, msg: TwistStamped):
        s_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if s_t == 0.0:
            s_t = self.sim_time
        self.cmd_sim_times.append(s_t)
        self.cmd_wall_times.append(time.time())

    def _gt_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.robot_pos = (data['robot']['x'], data['robot']['y'])
                self.robot_vel = (data['robot'].get('vx', 0.0), data['robot'].get('vy', 0.0))

            if 'humans' in data and len(data['humans']) > 0:
                h = data['humans'][0]
                hx = h.get('x', None)
                hy = h.get('y', None)
                if hx is not None and hy is not None:
                    self.human_pos = (hx, hy)
                    self.human_vel = (h.get('vx', 0.0), h.get('vy', 0.0))
                    self.human_speed = h.get('speed', 0.0)
        except Exception:
            pass

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
        last_sample_t = -1.0
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

            # Collision with human check (only after t_rel > 0.5 to prevent transient false collisions at spawn)
            cur_dc = 999.0
            cur_dclear = 999.0
            if self.human_pos is not None and t_rel > 0.5:
                hx, hy = self.human_pos
                cur_dc, cur_dclear = compute_rect_circle_distance(rx, ry, ryaw, hx, hy)
                if cur_dc < self.dcenter_min:
                    self.dcenter_min = cur_dc
                if cur_dclear < self.dclear_min:
                    self.dclear_min = cur_dclear

                # Footprint contact collision: d_clear <= 0.0 m
                if cur_dclear <= 0.0:
                    self.collision_contact = True
                    self.time_to_collision = round(t_rel, 2)
                    print(f"    [COLLISION DETECTED] Footprint contact (d_clear={cur_dclear:.4f}m) at t_sim={t_rel:.2f}s! Truncating episode.", flush=True)
                    break

            # Wall collision check
            # Corners of 0.24x0.20m box: (+-0.12, +-0.10)
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

            # Telemetry record (10 Hz)
            if self.sim_time - last_sample_t >= 0.10:
                last_sample_t = self.sim_time
                self.telemetry.append({
                    "t": round(t_rel, 2),
                    "rx": round(rx, 3), "ry": round(ry, 3), "yaw": round(ryaw, 3), "rspd": round(robot_spd, 3),
                    "hx": round(self.human_pos[0], 3) if self.human_pos else None,
                    "hy": round(self.human_pos[1], 3) if self.human_pos else None,
                    "d_center": round(cur_dc, 3), "d_clear": round(cur_dclear, 3)
                })

            # Goal reached check
            if dist_to_goal <= 0.35:
                self.goal_reached = True
                self.time_to_goal = round(t_rel, 2)
                print(f"    [GOAL REACHED] at t_sim = {t_rel:.2f}s! Dist to goal: {dist_to_goal:.3f}m", flush=True)
                break

            # Timeout check
            if t_rel >= self.max_duration_s:
                print(f"    [TIMEOUT] Max duration {self.max_duration_s}s reached. Dist to goal: {dist_to_goal:.3f}m", flush=True)
                break

            if time.time() - t_wall_start > (self.max_duration_s + 25.0):
                print("    [HARD TIMEOUT] Wall timeout reached!", flush=True)
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
                # Minimum frequency is 1 / max(dt)
                cmd_hz_min = round(1.0 / max(valid_dts), 2)

        return {
            "goal_reached": self.goal_reached,
            "time_to_goal": self.time_to_goal,
            "collision_contact": self.collision_contact,
            "collision_wall": self.collision_wall,
            "collision_occurred": self.collision_contact or self.collision_wall,
            "time_to_collision": self.time_to_collision,
            "dmin_edge_to_edge": round(self.dclear_min, 4) if self.dclear_min < 900 else None,
            "dmin_center": round(self.dcenter_min, 4) if self.dcenter_min < 900 else None,
            "max_lateral_dev": round(self.max_lat_dev, 4),
            "min_robot_speed": round(self.min_robot_speed, 4) if self.min_robot_speed < 900 else 0.0,
            "robot_stopped_or_yielded": self.robot_stopped_or_yielded,
            "frozen_at_start": self.frozen_at_start,
            "cmd_vel_count": len(self.cmd_sim_times),
            "cmd_vel_sim_hz_mean": cmd_hz_mean,
            "cmd_vel_sim_hz_min": cmd_hz_min,
            "final_dist_to_goal": round(math.hypot(self.robot_pos[0] - self.target_gx, self.robot_pos[1] - self.target_gy), 3),
            "telemetry_count": len(self.telemetry)
        }


def run_single_trial_worker(corridor_type: str, persona: str, seed: int, v_human: float, domain_id: int, out_json: str):
    gen_dir = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated")
    os.makedirs(gen_dir, exist_ok=True)

    # 1. Generate scenario JSON
    scen_file = os.path.join(gen_dir, f"scenario_a3_3_rtf1_{corridor_type}_{persona}_{seed}.json")
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

    # 2. Patch SDF world: real_time_factor = 1.0!
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

    trial_sdf = os.path.join(gen_dir, f"world_a3_3_rtf1_{corridor_type}_{persona}_{seed}.sdf")
    with open(trial_sdf, "w") as f:
        f.write(sdf)

    # Environment
    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["ROS_DOMAIN_ID"] = str(domain_id)
    partition = f"a3_3_rtf1_{corridor_type}_{persona}_{seed}_{domain_id}"
    env["GZ_PARTITION"] = partition

    all_procs = []
    nav2_log_path = f"/tmp/nav2_a3_3_rtf1_{corridor_type}_{persona}_{seed}_{domain_id}.log"
    log_fp = open(nav2_log_path, "w")

    width_arg = "0.90" if corridor_type == "090" else "1.20"
    map_arg = f"corridor_{corridor_type}"
    # Use original pilot config (8 critics, batch 280, time_steps 30, costmap 6x6, footprint rectangular)
    params_file = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_pilot.yaml")
    corridor_half_w = 0.45 if corridor_type == "090" else 0.60

    result = {}
    try:
        # Launch Sim
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={width_arg}",
            f"custom_world:={trial_sdf}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        gz_p = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(gz_p)
        time.sleep(4.5)

        # Launch Nav2
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={params_file}",
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
            monitor = A33RTF1InteractionMonitor(target_gx=-7.0, target_gy=0.0, max_duration_s=25.0, corridor_half_width=corridor_half_w)
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

    result["seed"] = seed
    result["corridor"] = f"corridor_{corridor_type}"
    result["corridor_width_m"] = 0.90 if corridor_type == "090" else 1.20
    result["persona"] = persona
    result["v_human"] = v_human
    result["rtf_setting"] = 1.0

    with open(out_json, "w") as f:
        json.dump(result, f, indent=2)


def run_cell_sequential(cell: dict, domain_id: int):
    c_type = cell["corridor"]
    persona = cell["persona"]
    s_base = cell["seed_base"]
    cell_name = cell["name"]

    print(f"\n[Worker domain {domain_id}] Starting Cell {cell_name} (Corridor {c_type}, Persona {persona})...", flush=True)

    for i in range(10):
        seed = s_base + i
        rng = np.random.RandomState(seed)
        v_human = round(float(rng.uniform(1.0, 1.3)), 3)
        out_json = f"/tmp/res_a3_3_rtf1_{c_type}_{persona}_{seed}.json"

        cmd = [
            sys.executable, __file__,
            "--run_single",
            "--corridor", str(c_type),
            "--persona", str(persona),
            "--seed", str(seed),
            "--v_human", str(v_human),
            "--domain_id", str(domain_id),
            "--out_json", str(out_json)
        ]
        t_t0 = time.time()
        print(f"  [{cell_name}] Running seed {seed} (v_human={v_human:.3f} m/s)...", flush=True)
        subprocess.run(cmd, check=True)
        t_dur = time.time() - t_t0

        if os.path.exists(out_json):
            with open(out_json) as fp:
                res = json.load(fp)
            print(f"  [{cell_name}] Seed {seed} done in {t_dur:.1f}s -> Goal={res.get('goal_reached')}, "
                  f"Contact={res.get('collision_contact')}, dmin_clear={res.get('dmin_edge_to_edge')}m, "
                  f"cmd_vel_mean_hz={res.get('cmd_vel_sim_hz_mean')}, Yielded={res.get('robot_stopped_or_yielded')}, "
                  f"Frozen={res.get('frozen_at_start')}", flush=True)


def cleanup_lingering():
    subprocess.run(
        ["pkill", "-9", "-f", "ros2|custom_corridor|corridor_tb3|nav2_corridor|goal_pose_bridge|static_transform_publisher|robot_state_publisher|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|lifecycle_manager|map_server|amcl|parameter_bridge|gz sim"],
        stderr=subprocess.DEVNULL
    )
    time.sleep(1.5)


def main():
    parser = argparse.ArgumentParser(description="A3.3 RTF=1.0 Benchmark")
    parser.add_argument("--run_single", action="store_true")
    parser.add_argument("--corridor", type=str, default="090")
    parser.add_argument("--persona", type=str, default="H1")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--v_human", type=float, default=1.15)
    parser.add_argument("--domain_id", type=int, default=70)
    parser.add_argument("--out_json", type=str, default="/tmp/res_single.json")
    parser.add_argument("--cell_worker", type=str, default="")
    parser.add_argument("--run_missing", action="store_true", help="Run only missing or error seeds sequentially")
    parser.add_argument("--aggregate_only", action="store_true", help="Aggregate existing results without running")
    args = parser.parse_args()

    if args.run_single:
        run_single_trial_worker(args.corridor, args.persona, args.seed, args.v_human, args.domain_id, args.out_json)
        os._exit(0)

    cells = [
        {"corridor": "090", "persona": "H1", "seed_base": 101, "name": "090_H1", "domain_id": 71},
        {"corridor": "090", "persona": "H3", "seed_base": 201, "name": "090_H3", "domain_id": 72},
        {"corridor": "120", "persona": "H1", "seed_base": 301, "name": "120_H1", "domain_id": 73},
        {"corridor": "120", "persona": "H3", "seed_base": 401, "name": "120_H3", "domain_id": 74},
    ]

    if args.cell_worker:
        cell_configs = {c["name"]: c for c in cells}
        if args.cell_worker in cell_configs:
            c = cell_configs[args.cell_worker]
            run_cell_sequential(c, c["domain_id"])
        return

    print("=" * 80, flush=True)
    print("A3.3 REAL HEAD-ON INTERACTION BENCHMARK: RTF = 1.0 (40 TRIALS)", flush=True)
    print("=" * 80, flush=True)

    if args.run_missing:
        cleanup_lingering()
        print("\n>>> Checking all 40 trials to find missing/error runs...", flush=True)
        missing_runs = []
        for c in cells:
            c_type = c["corridor"]
            persona = c["persona"]
            s_base = c["seed_base"]
            for i in range(10):
                seed = s_base + i
                rng = np.random.RandomState(seed)
                v_h = round(float(rng.uniform(1.0, 1.3)), 3)
                out_f = f"/tmp/res_a3_3_rtf1_{c_type}_{persona}_{seed}.json"
                needs_run = False
                if not os.path.exists(out_f):
                    needs_run = True
                else:
                    try:
                        with open(out_f) as fp:
                            res = json.load(fp)
                        if res.get("error") or res.get("time_to_collision") == 0.0:
                            needs_run = True
                    except Exception:
                        needs_run = True
                if needs_run:
                    missing_runs.append((c_type, persona, seed, v_h, out_f))

        print(f">>> Found {len(missing_runs)} runs to execute sequentially.", flush=True)
        for idx, (c_type, persona, seed, v_h, out_f) in enumerate(missing_runs, 1):
            print(f"\n[{idx}/{len(missing_runs)}] Running Corridor {c_type}, Persona {persona}, Seed {seed} (v_human={v_h:.3f} m/s)...", flush=True)
            cleanup_lingering()
            cmd = [
                sys.executable, __file__,
                "--run_single",
                "--corridor", str(c_type),
                "--persona", str(persona),
                "--seed", str(seed),
                "--v_human", str(v_h),
                "--domain_id", "75",
                "--out_json", str(out_f)
            ]
            t_start = time.time()
            try:
                subprocess.run(cmd, check=True)
            except Exception as e:
                print(f"[ERROR] Execution failed for seed {seed}: {e}", flush=True)
            cleanup_lingering()
            t_dur = time.time() - t_start
            if os.path.exists(out_f):
                with open(out_f) as fp:
                    d = json.load(fp)
                print(f"  -> Finished in {t_dur:.1f}s | Goal: {d.get('goal_reached')} | Contact: {d.get('collision_contact')} | dmin_clear: {d.get('dmin_edge_to_edge')}m | Mean Hz: {d.get('cmd_vel_sim_hz_mean')}", flush=True)
            time.sleep(1.0)

    elif not args.aggregate_only:
        cleanup_lingering()
        print("\n>>> Launching 4 Cell Workers concurrently across domains 71, 72, 73, 74...", flush=True)
        workers = []
        for c in cells:
            cmd = [sys.executable, __file__, "--cell_worker", c["name"]]
            p = subprocess.Popen(cmd)
            workers.append((c["name"], p))
            time.sleep(3.5)

        for name, p in workers:
            p.wait()

    print("\n>>> Aggregating all 40 results...", flush=True)

    # Gather all 40 results
    trials = []
    for c in cells:
        c_type = c["corridor"]
        persona = c["persona"]
        s_base = c["seed_base"]
        for i in range(10):
            seed = s_base + i
            out_file = f"/tmp/res_a3_3_rtf1_{c_type}_{persona}_{seed}.json"
            if os.path.exists(out_file):
                with open(out_file) as fp:
                    trials.append(json.load(fp))
            else:
                print(f"[WARN] Missing output file: {out_file}", flush=True)

    # Compute cell summaries
    summary = {}
    for c in cells:
        c_corr = f"corridor_{c['corridor']}"
        c_pers = c["persona"]
        c_key = f"{c_corr}_{c_pers}"
        m = [t for t in trials if t.get("corridor") == c_corr and t.get("persona") == c_pers]
        if m:
            d_clears = [t["dmin_edge_to_edge"] for t in m if t.get("dmin_edge_to_edge") is not None]
            d_centers = [t["dmin_center"] for t in m if t.get("dmin_center") is not None]
            contact_colls = [1 if t.get("collision_contact") else 0 for t in m]
            goals = [1 if t.get("goal_reached") else 0 for t in m]
            lat_devs = [t["max_lateral_dev"] for t in m if t.get("max_lateral_dev") is not None]
            yielded = [1 if t.get("robot_stopped_or_yielded") else 0 for t in m]
            frozens = [1 if t.get("frozen_at_start") else 0 for t in m]
            cmd_hz_means = [t["cmd_vel_sim_hz_mean"] for t in m if t.get("cmd_vel_sim_hz_mean") is not None]
            cmd_hz_mins = [t["cmd_vel_sim_hz_min"] for t in m if t.get("cmd_vel_sim_hz_min") is not None]
            times = [t["time_to_goal"] for t in m if t.get("time_to_goal") is not None]

            summary[c_key] = {
                "n_runs": len(m),
                "contact_collision_rate_pct": round(float(np.mean(contact_colls) * 100), 1),
                "goal_reach_rate_pct": round(float(np.mean(goals) * 100), 1),
                "dmin_clear_median_m": round(float(np.median(d_clears)), 4) if d_clears else None,
                "dmin_clear_min_m": round(float(np.min(d_clears)), 4) if d_clears else None,
                "dmin_clear_max_m": round(float(np.max(d_clears)), 4) if d_clears else None,
                "dmin_center_median_m": round(float(np.median(d_centers)), 4) if d_centers else None,
                "max_lateral_dev_median_m": round(float(np.median(lat_devs)), 4) if lat_devs else None,
                "yielded_stop_rate_pct": round(float(np.mean(yielded) * 100), 1),
                "frozen_at_start_count": int(np.sum(frozens)),
                "cmd_vel_sim_hz_mean": round(float(np.mean(cmd_hz_means)), 2) if cmd_hz_means else None,
                "cmd_vel_sim_hz_min_median": round(float(np.median(cmd_hz_mins)), 2) if cmd_hz_mins else None,
                "time_to_goal_median_s": round(float(np.median(times)), 2) if times else None
            }

    full_output = {
        "benchmark": "A3.3 Real Interaction Benchmark with RTF = 1.0",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "hardware": {
            "cpu": "AMD Ryzen 7 7735H (8 cores, 16 threads)",
            "gpu": "NVIDIA GeForce RTX 4060 Laptop GPU"
        },
        "trials": trials,
        "summary": summary
    }

    # Load Old Results for comparison
    old_summary = {}
    if os.path.exists(OLD_JSON):
        try:
            with open(OLD_JSON) as fp:
                old_data = json.load(fp)
                old_summary = old_data.get("summary", {})
        except Exception:
            pass

    full_output["old_summary_rtf0"] = old_summary

    with open(OUTPUT_JSON, "w") as fp:
        json.dump(full_output, fp, indent=2)

    print(f"\n[OK] Raw results saved to: {OUTPUT_JSON}", flush=True)

    print("\n" + "=" * 95, flush=True)
    print(f"{'BẢNG SO SÁNH A3.3: TRƯỚC (RTF = 0.0) VS SAU (RTF = 1.0)':^95}", flush=True)
    print("=" * 95, flush=True)
    print(f"{'Chỉ số':<30} | {'0.90m H1':<14} | {'0.90m H3':<14} | {'1.20m H1':<14} | {'1.20m H3':<14}")
    print("-" * 95, flush=True)

    for metric_name, key_new, key_old, fmt in [
        ("Tỉ lệ va chạm footprint (%)", "contact_collision_rate_pct", "collision_rate_pct", "{:.1f}%"),
        ("Tỉ lệ tới đích (%)", "goal_reach_rate_pct", "goal_reach_rate_pct", "{:.1f}%"),
        ("dmin mép-mép trung vị (m)", "dmin_clear_median_m", None, "{:.3f}m"),
        ("dmin tâm-tâm trung vị (m)", "dmin_center_median_m", "dmin_median_m", "{:.3f}m"),
        ("Độ lệch ngang max trung vị (m)", "max_lateral_dev_median_m", "max_lateral_dev_median_m", "{:.3f}m"),
        ("Tỉ lệ robot dừng/nhường (%)", "yielded_stop_rate_pct", None, "{:.1f}%"),
        ("Số lần đóng băng xuất phát", "frozen_at_start_count", None, "{:d}"),
        ("Tần số cmd_vel sim (Mean Hz)", "cmd_vel_sim_hz_mean", None, "{:.1f} Hz"),
        ("Tần số cmd_vel sim (Min Hz)", "cmd_vel_sim_hz_min_median", None, "{:.1f} Hz"),
    ]:
        row_str = f"{metric_name:<30} | "
        for c_key in ["corridor_090_H1", "corridor_090_H3", "corridor_120_H1", "corridor_120_H3"]:
            val_new = summary.get(c_key, {}).get(key_new)
            val_old = old_summary.get(c_key, {}).get(key_old) if key_old else None
            
            str_new = fmt.format(val_new) if val_new is not None else "N/A"
            if val_old is not None:
                str_old = fmt.format(val_old)
                cell_str = f"{str_old} -> {str_new}"
            else:
                cell_str = f"{str_new}"
            row_str += f"{cell_str:<14} | "
        print(row_str, flush=True)
    print("=" * 95, flush=True)


if __name__ == '__main__':
    main()
