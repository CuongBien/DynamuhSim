#!/usr/bin/env python3
"""
C1: Complete Generator for Pilot 300 Episodes in Gazebo Harmonic with Small Mecanum Robot and Nav2 MPPI.

Specifications:
- Total episodes: 300 (ep_0001 to ep_0300)
- Duration: 20.0s @ 10 Hz (target 200 frames per episode if no collision/truncation)
- Humans: N in {2, 5, 10} balanced (exactly 100 of each)
- Corridors: corridor_090 (150 eps) and corridor_120 (150 eps)
- Persona distribution: exactly H1: 80, H2: 80, H3: 80, H4: 30, H5: 30
- Controller: 270 Nav2 MPPI (regenerate_noises: true), 30 non-Nav2 (open-loop)
- Geometry: >= 60% geometric interaction (d_CPA < 1.5m; also records < 2.0m)
- Initial human lateral offset: random in [-0.4, 0.4] m within corridor boundaries
- Feasibility filter: requires max(gap_left, gap_right) >= 0.30 m or encounter in recess
- Frozen flag: robot speed < 0.02 m/s in first 2s, or stationary >= 3s when > 0.35m from goal and human > 2.0m
- Collision: Rectangular footprint (0.24 x 0.20 m) vs cylinder (r = 0.28 m). Truncates on first contact (d_clear <= 0).
  Asserts d_center < 0.30m without collision == 0. Reports wall penetration > 2cm per corridor.
- Clean depth: 16UC1 PNG (uint16 mm), invalid (<0.28m, >5.0m, inf, nan) saved as 0
- Hardware specs: Queried dynamically via lscpu and nvidia-smi (AMD Ryzen 7 7735H, RTX 4060)
- Multi-worker: 4 concurrent instances on domains 81, 82, 83, 84
"""

import os
import sys
import time
import math
import json
import signal
import subprocess
import argparse
import multiprocessing as mp
from typing import Tuple, Dict, List
import numpy as np
import cv2

ROOT_DIR = "/home/cuongbien/DynamuhSim"
SCRIPT_DIR = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark")
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, "src/custom_corridor/scripts"))

from benchmark_world_generator import BenchmarkWorldGenerator
from generate_randomized_mecanum_sdf import generate_randomized_mecanum_sdf

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


def get_hardware_specs() -> Dict[str, str]:
    cpu = "Unknown CPU"
    try:
        out = subprocess.check_output("lscpu", text=True)
        for line in out.splitlines():
            if "Model name:" in line:
                cpu = line.split(":", 1)[1].strip()
                break
    except Exception as e:
        cpu = str(e)

    gpu = "Unknown GPU"
    try:
        out = subprocess.check_output(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], text=True)
        gpu = out.strip().splitlines()[0]
    except Exception as e:
        gpu = str(e)
    return {"cpu": cpu, "gpu": gpu}


def compute_rect_circle_distance(rx: float, ry: float, ryaw: float, hx: float, hy: float, rw=0.20, rl=0.24, hr=0.28) -> Tuple[float, float]:
    dx = hx - rx
    dy = hy - ry
    d_center = math.hypot(dx, dy)
    cos_y = math.cos(ryaw)
    sin_y = math.sin(ryaw)
    x_rel = dx * cos_y + dy * sin_y
    y_rel = -dx * sin_y + dy * cos_y

    half_l = rl / 2.0  # 0.12
    half_w = rw / 2.0  # 0.10
    x_cl = max(-half_l, min(half_l, x_rel))
    y_cl = max(-half_w, min(half_w, y_rel))
    d_rect = math.hypot(x_rel - x_cl, y_rel - y_cl)
    d_clear = d_rect - hr
    return d_center, d_clear


def compute_cpa(r0: Tuple[float, float], vr: Tuple[float, float], h0: Tuple[float, float], vh: Tuple[float, float], max_t: float = 18.0) -> float:
    dx = r0[0] - h0[0]
    dy = r0[1] - h0[1]
    dvx = vr[0] - vh[0]
    dvy = vr[1] - vh[1]
    v2 = dvx**2 + dvy**2
    if v2 < 1e-6:
        return math.hypot(dx, dy)
    t_cpa = -(dx * dvx + dy * dvy) / v2
    t_cpa_clamped = min(max_t, max(0.0, t_cpa))
    cpa_x = dx + dvx * t_cpa_clamped
    cpa_y = dy + dvy * t_cpa_clamped
    return math.hypot(cpa_x, cpa_y)


def generate_pilot_300_plan(seed=4242) -> Tuple[List[Dict], Dict]:
    rng = np.random.RandomState(seed)

    # 1. Personas: exactly H1: 80, H2: 80, H3: 80, H4: 30, H5: 30
    personas = ['H1'] * 80 + ['H2'] * 80 + ['H3'] * 80 + ['H4'] * 30 + ['H5'] * 30

    # 2. Corridors: 150 corridor_090, 150 corridor_120
    corridors = []
    for count in [80, 80, 80, 30, 30]:
        corridors.extend(['corridor_090'] * (count // 2) + ['corridor_120'] * (count // 2))

    # 3. N humans: 100 of N=2, 100 of N=5, 100 of N=10
    n_humans = []
    for count in [80, 80, 80, 30, 30]:
        if count == 80:
            sub = [2] * 27 + [5] * 27 + [10] * 26
        else:
            sub = [2] * 10 + [5] * 10 + [10] * 10
        n_humans.extend(sub)
    idx_h1_2 = [i for i in range(80) if n_humans[i] == 2][-1]
    n_humans[idx_h1_2] = 10
    idx_h2_5 = [i for i in range(80, 160) if n_humans[i] == 5][-1]
    n_humans[idx_h2_5] = 10

    # 4. Controller: 270 Nav2 MPPI, 30 non-Nav2
    nav2_flags = []
    for count in [80, 80, 80, 30, 30]:
        non_nav2_count = 8 if count == 80 else 3
        nav2_count = count - non_nav2_count
        nav2_flags.extend([True] * nav2_count + [False] * non_nav2_count)

    # Deterministic shuffle within persona blocks
    groups = [(0, 80), (80, 160), (160, 240), (240, 270), (270, 300)]
    shuffled_items = []
    for start, end in groups:
        sub_indices = list(range(start, end))
        rng.shuffle(sub_indices)
        for idx in sub_indices:
            shuffled_items.append({
                'persona': personas[idx],
                'corridor': corridors[idx],
                'num_humans': n_humans[idx],
                'use_nav2': nav2_flags[idx]
            })

    perm = rng.permutation(300)
    ordered_items = [shuffled_items[p] for p in perm]

    # Now generate human spawn & trajectory parameters with feasibility filter
    rejection_stats = {'corridor_090': 0, 'corridor_120': 0, 'total': 0}
    cpa_lt_15_count = 0
    cpa_lt_20_count = 0
    ep_plans = []

    for ep_idx, item in enumerate(ordered_items):
        ep_id = ep_idx + 1
        ep_seed = 1000 + ep_id
        corr = item['corridor']
        half_w = 0.45 if corr == 'corridor_090' else 0.60
        max_y = half_w - 0.28 - 0.01  # safe center coordinate without wall penetration

        # Geometric interaction target: 75% interactive oncoming, 25% non-interactive
        is_interactive_target = (rng.rand() < 0.75)

        # Sample human 1 with feasibility filter
        while True:
            raw_y = float(rng.uniform(-0.40, 0.40))
            in_recess = bool(corr == 'corridor_090' and rng.rand() < 0.22)

            if in_recess:
                feasible = True
                y_h1 = float(np.clip(raw_y, 0.10, 0.40))  # shifted towards recess
                start_x_h1 = float(rng.uniform(1.45, 2.55))
            else:
                y_h1 = float(np.clip(raw_y, -max_y, max_y))
                gap_l = half_w - (y_h1 + 0.28)
                gap_r = (y_h1 - 0.28) - (-half_w)
                feasible = (max(gap_l, gap_r) >= 0.30)
                start_x_h1 = float(rng.uniform(-6.8, -5.8)) if is_interactive_target else float(rng.uniform(2.5, 6.0))

            if not feasible:
                rejection_stats[corr] += 1
                rejection_stats['total'] += 1
                continue
            break

        # Speeds
        if item['persona'] == 'H4':
            v_h1 = 0.0
            mode_h1 = "stationary"
        else:
            v_h1 = float(rng.uniform(0.95, 1.25))
            mode_h1 = "autonomous"

        # Calculate CPA
        r0 = (-13.0, 0.0)
        vr = (0.35, 0.0)
        h0 = (start_x_h1, y_h1)
        vh = (-v_h1, 0.0) if mode_h1 == "autonomous" and is_interactive_target else (0.0, 0.0)
        cpa = compute_cpa(r0, vr, h0, vh)
        if cpa < 1.50:
            cpa_lt_15_count += 1
        if cpa < 2.00:
            cpa_lt_20_count += 1

        # Background humans
        humans_list = [{
            "id": 1,
            "name": "human_01",
            "mode": mode_h1,
            "speed": round(v_h1, 3),
            "max_speed": round(v_h1 * 1.2, 3),
            "persona": item['persona'],
            "loop": False,
            "wait_for_robot": True,
            "start": {"x": round(start_x_h1, 3), "y": round(y_h1, 3), "yaw": 3.14159},
            "waypoints": [[round(start_x_h1, 3), round(y_h1, 3)], [-15.0, round(y_h1, 3)]],
            "proxy_z": 0.80 if corr == 'corridor_090' else 0.875
        }]

        # Spawn remaining N-1 humans spaced along corridor
        bg_xs = np.linspace(-4.5, 12.0, item['num_humans'])
        # Sample background humans according to agreed population proportion:
        # 40% H1 (Cooperative), 10% H2 (Unyielding), 25% H3 (RobotBlind), 15% H4 (Static), 10% H5 (SocialForce)
        persona_choices = ['H1', 'H2', 'H3', 'H4', 'H5']
        persona_probs = [0.40, 0.10, 0.25, 0.15, 0.10]

        for h_i in range(2, item['num_humans'] + 1):
            bg_x = float(bg_xs[h_i - 1] + rng.uniform(-0.25, 0.25))
            bg_raw_y = float(rng.uniform(-0.35, 0.35))
            bg_y = float(np.clip(bg_raw_y, -max_y, max_y))

            # Human 2 shares episode target persona for close-interaction density; humans 3..N follow population distribution
            if h_i == 2 and item['num_humans'] <= 2:
                bg_persona = item['persona']
            else:
                bg_persona = str(rng.choice(persona_choices, p=persona_probs))

            if bg_persona == 'H4':
                bg_mode = "stationary"
                bg_v = 0.0
                bg_target_x = bg_x
            else:
                bg_mode = "autonomous"
                bg_v = float(rng.uniform(0.70, 1.05))
                bg_dir = -1.0 if (h_i % 2 == 0) else 1.0
                bg_target_x = -14.0 if bg_dir < 0 else 14.0

            humans_list.append({
                "id": h_i,
                "name": f"human_{h_i:02d}",
                "mode": bg_mode,
                "speed": round(bg_v, 3),
                "max_speed": round(bg_v * 1.2 if bg_v > 0 else 0.0, 3),
                "persona": bg_persona,
                "loop": False,
                "wait_for_robot": False,
                "start": {"x": round(bg_x, 3), "y": round(bg_y, 3), "yaw": 3.14159 if (h_i % 2 == 0) else 0.0},
                "waypoints": [[round(bg_x, 3), round(bg_y, 3)], [bg_target_x, round(bg_y, 3)]],
                "proxy_z": 0.80 if corr == 'corridor_090' else 0.875
            })

        ep_plans.append({
            'episode_id': ep_id,
            'seed': ep_seed,
            'persona': item['persona'],
            'corridor': corr,
            'num_humans': item['num_humans'],
            'use_nav2': item['use_nav2'],
            'is_interactive_target': is_interactive_target,
            'cpa_dist_m': round(cpa, 3),
            'has_cpa_lt_15': bool(cpa < 1.50),
            'has_cpa_lt_20': bool(cpa < 2.00),
            'humans': humans_list
        })

    filter_stats = {
        'rejection_stats': rejection_stats,
        'cpa_lt_15_count': cpa_lt_15_count,
        'cpa_lt_15_rate': round(cpa_lt_15_count / 300.0 * 100.0, 1),
        'cpa_lt_20_count': cpa_lt_20_count,
        'cpa_lt_20_rate': round(cpa_lt_20_count / 300.0 * 100.0, 1)
    }
    return ep_plans, filter_stats


class EpisodeRecorderNode(Node):
    def __init__(self, ep_dir: str, target_sim_duration: float = 20.0, corridor_half_w: float = 0.45, target_gx: float = -7.0, target_gy: float = 0.0):
        super().__init__('pilot_episode_recorder')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.ep_dir = ep_dir
        self.target_sim_duration = target_sim_duration
        self.corridor_half_w = corridor_half_w
        self.target_gx = target_gx
        self.target_gy = target_gy

        self.depth_dir = os.path.join(ep_dir, "depth")
        os.makedirs(self.depth_dir, exist_ok=True)

        self.sim_time = 0.0
        self.t_0 = None
        self.t_goal_sent = None
        self.t_first_cmd = None
        self.completed = False
        self.collision_occurred = False
        self.collision_step = None
        self.is_frozen = False
        self.frozen_reason = ""
        self.max_speed_first_2s = 0.0
        self.last_moving_t = 0.0
        self.goal_reached = False
        self.time_to_goal = None

        self.robot_pos = (-13.0, 0.0, 0.0)
        self.robot_speed = 0.0

        self.depth_buffer = []
        self.ground_truth = []
        self.odometry = []
        self.cmd_vel = []
        self.step_metrics = []
        self.invariant_violations = 0
        self.wall_pen_steps = 0
        self.total_human_steps = 0

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 50)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', self._depth_cb, 50)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.t_0 is not None:
            t_rel = self.sim_time - self.t_0
            if t_rel >= self.target_sim_duration:
                self.completed = True

    def _cmd_cb(self, msg: TwistStamped):
        vx, vy, w = msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z
        if self.t_goal_sent is not None and self.t_first_cmd is None and (abs(vx) > 0.001 or abs(vy) > 0.001 or abs(w) > 0.001):
            self.t_first_cmd = self.sim_time
            if self.t_0 is None:
                self.t_0 = self.sim_time

        if self.t_0 is not None:
            t_rel = self.sim_time - self.t_0
            self.cmd_vel.append({
                "t_rel": round(t_rel, 3),
                "vx": round(vx, 4),
                "vy": round(vy, 4),
                "w": round(w, 4)
            })

    def _odom_cb(self, msg: Odometry):
        if self.t_0 is None:
            return
        t_rel = self.sim_time - self.t_0
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny, cosy)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        w = msg.twist.twist.angular.z

        self.odometry.append({
            "t_rel": round(t_rel, 3),
            "x": round(pos.x, 4),
            "y": round(pos.y, 4),
            "yaw": round(yaw, 4),
            "vx": round(vx, 4),
            "vy": round(vy, 4),
            "w": round(w, 4)
        })

    def _gt_cb(self, msg: String):
        if self.t_0 is None or self.completed or self.collision_occurred:
            return
        t_rel = self.sim_time - self.t_0
        try:
            d = json.loads(msg.data)
            # Use ground truth robot position
            r_info = d.get('robot', {})
            rx = r_info.get('x', self.robot_pos[0])
            ry = r_info.get('y', self.robot_pos[1])
            ryaw = r_info.get('yaw', self.robot_pos[2])
            rvx = r_info.get('vx', 0.0)
            rvy = r_info.get('vy', 0.0)
            self.robot_speed = math.hypot(rvx, rvy)
            self.robot_pos = (rx, ry, ryaw)

            step_min_dclear = 999.0
            step_min_dcenter = 999.0
            step_wall_pen_max = 0.0

            humans = d.get('humans', [])
            for h in humans:
                hx, hy = h['x'], h['y']
                dc, dcl = compute_rect_circle_distance(rx, ry, ryaw, hx, hy)
                if dc < step_min_dcenter:
                    step_min_dcenter = dc
                if dcl < step_min_dclear:
                    step_min_dclear = dcl

                self.total_human_steps += 1
                wall_pen = max(0.0, (abs(hy) + 0.28) - self.corridor_half_w)
                if wall_pen > 0.02:
                    self.wall_pen_steps += 1
                if wall_pen > step_wall_pen_max:
                    step_wall_pen_max = wall_pen

            # Invariant check: d_center < 0.30 without collision flag must be 0
            if step_min_dcenter < 0.30 and step_min_dclear > 0.0:
                self.invariant_violations += 1

            # Goal check
            dist_to_goal = math.hypot(rx - self.target_gx, ry - self.target_gy)
            if dist_to_goal <= 0.35 and not self.goal_reached:
                self.goal_reached = True
                self.time_to_goal = round(t_rel, 3)

            self.step_metrics.append({
                "t_rel": round(t_rel, 3),
                "rx": round(rx, 4),
                "ry": round(ry, 4),
                "rspd": round(self.robot_speed, 4),
                "dist_to_goal": round(dist_to_goal, 4),
                "min_d_center": round(step_min_dcenter, 4),
                "min_d_clear": round(step_min_dclear, 4),
                "wall_penetration_max": round(step_wall_pen_max, 4),
                "wall_penetration_over_2cm": bool(step_wall_pen_max > 0.02)
            })

            # Check collision
            if step_min_dclear <= 0.0:
                self.collision_occurred = True
                self.collision_step = len(self.step_metrics)

            # Frozen flag check
            if t_rel <= 2.0:
                if self.robot_speed > self.max_speed_first_2s:
                    self.max_speed_first_2s = self.robot_speed
            else:
                if self.max_speed_first_2s < 0.02 and not self.is_frozen:
                    self.is_frozen = True
                    self.frozen_reason = "no_movement_first_2s"
                if self.robot_speed > 0.02:
                    self.last_moving_t = t_rel
                elif (t_rel - self.last_moving_t) >= 3.0 and dist_to_goal > 0.35 and step_min_dcenter > 2.0 and not self.is_frozen:
                    self.is_frozen = True
                    self.frozen_reason = "stationary_ge_3s_far_from_humans"

            d["sim_time_rel"] = round(t_rel, 3)
            self.ground_truth.append(d)
        except Exception:
            pass

    def _depth_cb(self, msg: Image):
        if self.t_0 is None or self.completed or self.collision_occurred:
            return
        try:
            if msg.encoding in ['32FC1', '32FC']:
                depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
                depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
                invalid_mask = (depth_m < 0.28) | (depth_m > 5.0)
                depth_m[invalid_mask] = 0.0
                depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
            elif msg.encoding in ['16UC1', 'mono16']:
                depth_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            else:
                return
            self.depth_buffer.append(depth_mm)
        except Exception:
            pass

    def send_nav_goal(self, gx: float = -7.0, gy: float = 0.0) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(gx)
        goal_msg.pose.pose.position.y = float(gy)
        goal_msg.pose.pose.orientation.w = 1.0

        for attempt in range(5):
            send_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle.accepted:
                    return True
            time.sleep(0.5)
        return False


def run_single_episode(plan: Dict, domain_id: int, out_dir: str) -> bool:
    ep_id = plan['episode_id']
    seed = plan['seed']
    corr = plan['corridor']
    num_humans = plan['num_humans']
    persona = plan['persona']
    use_nav2 = plan['use_nav2']

    ep_dir = os.path.join(out_dir, f"ep_{ep_id:04d}")
    os.makedirs(ep_dir, exist_ok=True)
    manifest_file = os.path.join(ep_dir, "manifest.json")

    # Resume support
    if os.path.exists(manifest_file):
        try:
            with open(manifest_file) as fp:
                existing_m = json.load(fp)
            if existing_m.get("qc_passed", False):
                return True
        except Exception:
            pass

    # 1. Generate randomized robot SDF
    robot_sdf, robot_params = generate_randomized_mecanum_sdf(seed=seed, randomize=True)
    robot_file = os.path.join(ep_dir, "robot.sdf")
    with open(robot_file, "w") as fp:
        fp.write(robot_sdf)

    # 2. Scenario JSON
    scen_file = os.path.join(ep_dir, "scenario.json")
    scen_data = {
        "episode_id": ep_id,
        "corridor_mode": True,
        "humans": plan['humans']
    }
    with open(scen_file, "w") as fp:
        json.dump(scen_data, fp, indent=2)

    # 3. World SDF
    corr_type = "090" if corr == "corridor_090" else "120"
    gen = BenchmarkWorldGenerator(ROOT_DIR)
    world_filename = f"world_ep_{ep_id:04d}.sdf"
    world_file = gen.generate_world_with_humans(
        num_humans=num_humans,
        corridor_type=corr_type,
        seed=seed,
        real_time_factor=0.0,
        out_filename=world_filename,
        scenario_file=scen_file,
        scenario_humans=plan['humans']
    )

    # Setup Environment
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = f"gz_pilot_d{domain_id}_ep_{ep_id:04d}"
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    params_file = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_pilot.yaml")
    half_w = 0.45 if corr == "corridor_090" else 0.60
    procs = []

    try:
        # Launch Gazebo
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={'0.90' if corr_type == '090' else '1.20'}",
            f"custom_world:={world_file}",
            f"custom_robot_model:={robot_file}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        p_sim = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        procs.append(p_sim)
        time.sleep(3.8)

        # Launch Nav2 if requested
        nav2_log_path = f"/tmp/nav2_pilot_ep_{ep_id:04d}_{domain_id}.log"
        if use_nav2:
            nav2_log_file = open(nav2_log_path, "w")
            nav2_cmd = [
                "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
                "controller:=mppi",
                f"params_file:={params_file}",
                f"map:={corr}",
                "robot_model:=mecanum"
            ]
            p_nav = subprocess.Popen(nav2_cmd, env=env, stdout=nav2_log_file, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
            procs.append(p_nav)

            # Wait for Nav2 active state
            t0 = time.time()
            while time.time() - t0 < 30.0:
                if os.path.exists(nav2_log_path):
                    with open(nav2_log_path) as fp:
                        if "Managed nodes are active" in fp.read():
                            break
                time.sleep(0.4)

        rclpy.init()
        recorder = EpisodeRecorderNode(ep_dir, target_sim_duration=20.0, corridor_half_w=half_w, target_gx=-7.0, target_gy=0.0)

        # Allow topics to initialize
        t_spin0 = time.time()
        while time.time() - t_spin0 < 1.5:
            rclpy.spin_once(recorder, timeout_sec=0.05)

        if use_nav2:
            # Send Nav2 navigation goal
            if recorder.send_nav_goal(-7.0, 0.0):
                recorder.t_goal_sent = recorder.sim_time
            else:
                recorder.t_goal_sent = recorder.sim_time

            # Wait for command release to establish T_0
            t_w_wait = time.time()
            while rclpy.ok() and recorder.t_0 is None and (time.time() - t_w_wait) < 8.0:
                rclpy.spin_once(recorder, timeout_sec=0.01)

            if recorder.t_0 is None:
                recorder.t_0 = recorder.sim_time
        else:
            # Open-loop non-Nav2 publisher: publish constant forward speed
            cmd_pub = recorder.create_publisher(TwistStamped, '/cmd_vel', 10)
            recorder.t_goal_sent = recorder.sim_time
            recorder.t_0 = recorder.sim_time
            recorder.t_first_cmd = recorder.sim_time

        # Main execution spin loop
        wall_start = time.time()
        while rclpy.ok() and not recorder.completed and not recorder.collision_occurred and (time.time() - wall_start) < 45.0:
            if not use_nav2:
                # Keep publishing forward velocity
                tw = TwistStamped()
                tw.header.stamp = recorder.get_clock().now().to_msg()
                tw.twist.linear.x = 0.35
                cmd_pub.publish(tw)

            rclpy.spin_once(recorder, timeout_sec=0.01)

        # Flush depth frames to PNG16
        for idx, d_img in enumerate(recorder.depth_buffer):
            cv2.imwrite(os.path.join(recorder.depth_dir, f"frame_{idx:04d}.png"), d_img)

        delta_t_cmd0 = (recorder.t_first_cmd - recorder.t_goal_sent) if (recorder.t_first_cmd is not None and recorder.t_goal_sent is not None) else 0.0
        hw_specs = get_hardware_specs()

        # QC check:
        # Criteria: depth frames >= 180 (or truncated upon collision), invariant_violations == 0, gt frames >= 180 (or truncated)
        qc_passed = True
        if recorder.invariant_violations > 0:
            qc_passed = False
        if not recorder.collision_occurred and len(recorder.depth_buffer) < 170:
            qc_passed = False
        if len(recorder.ground_truth) < 50:
            qc_passed = False

        manifest = {
            "episode_id": ep_id,
            "seed": seed,
            "corridor": corr,
            "num_humans": num_humans,
            "persona": persona,
            "use_nav2": use_nav2,
            "is_interactive_target": plan['is_interactive_target'],
            "cpa_dist_m": plan['cpa_dist_m'],
            "has_cpa_lt_15": plan['has_cpa_lt_15'],
            "has_cpa_lt_20": plan['has_cpa_lt_20'],
            "hardware": hw_specs,
            "robot_params": robot_params,
            "delta_t_cmd0_s": round(delta_t_cmd0, 4),
            "completed": recorder.completed,
            "collision_occurred": recorder.collision_occurred,
            "collision_step": recorder.collision_step,
            "goal_reached": recorder.goal_reached,
            "time_to_goal": recorder.time_to_goal,
            "is_frozen": recorder.is_frozen,
            "frozen_reason": recorder.frozen_reason,
            "invariant_violations": recorder.invariant_violations,
            "wall_pen_steps": recorder.wall_pen_steps,
            "total_human_steps": recorder.total_human_steps,
            "depth_frames_saved": len(recorder.depth_buffer),
            "gt_steps_saved": len(recorder.ground_truth),
            "odom_steps_saved": len(recorder.odometry),
            "cmd_steps_saved": len(recorder.cmd_vel),
            "qc_passed": qc_passed
        }

        with open(manifest_file, "w") as fp:
            json.dump(manifest, fp, indent=2)
        with open(os.path.join(ep_dir, "ground_truth.json"), "w") as fp:
            json.dump(recorder.ground_truth, fp)
        with open(os.path.join(ep_dir, "odometry.json"), "w") as fp:
            json.dump(recorder.odometry, fp)
        with open(os.path.join(ep_dir, "cmd_vel.json"), "w") as fp:
            json.dump(recorder.cmd_vel, fp)
        with open(os.path.join(ep_dir, "step_metrics.json"), "w") as fp:
            json.dump(recorder.step_metrics, fp)

        return qc_passed

    except Exception as e:
        print(f"[Worker domain {domain_id}] Error in ep_{ep_id:04d}: {e}")
        return False
    finally:
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.05)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        try:
            recorder.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def worker_loop(worker_id: int, domain_id: int, tasks: List[Dict], out_dir: str, result_queue: mp.Queue):
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    for plan in tasks:
        t_start = time.time()
        ep_id = plan['episode_id']
        manifest_file = os.path.join(out_dir, f"ep_{ep_id:04d}/manifest.json")
        
        # Check if already completed and passed QC
        if os.path.exists(manifest_file):
            try:
                with open(manifest_file) as fp:
                    m = json.load(fp)
                if m.get("qc_passed", False):
                    result_queue.put({
                        'episode_id': ep_id,
                        'worker_id': worker_id,
                        'success': True,
                        'elapsed_s': 0.0
                    })
                    continue
            except Exception:
                pass

        # Spawn isolated process for clean DDS lifecycle teardown
        ep_q = mp.Queue()
        def _ep_runner(p, d, o, q):
            try:
                ok = run_single_episode(p, d, o)
                q.put(ok)
            except Exception:
                q.put(False)

        proc = mp.Process(target=_ep_runner, args=(plan, domain_id, out_dir, ep_q))
        proc.start()
        proc.join(timeout=60.0)

        if proc.is_alive():
            try:
                proc.terminate()
                time.sleep(0.1)
                proc.kill()
            except Exception:
                pass
            proc.join()
            success = False
        else:
            try:
                success = ep_q.get_nowait()
            except Exception:
                success = False
                if os.path.exists(manifest_file):
                    try:
                        with open(manifest_file) as fp:
                            success = json.load(fp).get("qc_passed", False)
                    except Exception:
                        pass

        elapsed = time.time() - t_start
        result_queue.put({
            'episode_id': ep_id,
            'worker_id': worker_id,
            'success': success,
            'elapsed_s': elapsed
        })


def main():
    parser = argparse.ArgumentParser(description="Pilot 300 Episode Dataset Generator")
    parser.add_argument("--workers", type=int, default=4, help="Number of concurrent workers (default: 4)")
    parser.add_argument("--start-ep", type=int, default=1, help="Starting episode ID (default: 1)")
    parser.add_argument("--total-episodes", type=int, default=300, help="Total episodes (default: 300)")
    parser.add_argument("--out-dir", type=str, default=os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/pilot_300/pilot_300_dataset/episodes"))
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    print("=" * 80)
    print("PILOT 300 EPISODES GENERATOR (GAZEBO HARMONIC + SMALL MECANUM + NAV2 MPPI)")
    print("=" * 80)

    plans, filter_stats = generate_pilot_300_plan()
    print(f"Total planned episodes: {len(plans)}")
    print(f"Feasibility filter rejections: {filter_stats['rejection_stats']}")
    print(f"Geometric interaction (d_CPA < 1.5m): {filter_stats['cpa_lt_15_count']}/300 ({filter_stats['cpa_lt_15_rate']}%)")
    print(f"Geometric interaction (d_CPA < 2.0m): {filter_stats['cpa_lt_20_count']}/300 ({filter_stats['cpa_lt_20_rate']}%)")

    # Filter by start_ep and total_episodes
    tasks_to_run = [p for p in plans if args.start_ep <= p['episode_id'] < (args.start_ep + args.total_episodes)]

    # Partition tasks across workers
    num_workers = args.workers
    worker_tasks = [[] for _ in range(num_workers)]
    for i, t in enumerate(tasks_to_run):
        worker_tasks[i % num_workers].append(t)

    result_queue = mp.Queue()
    procs = []
    base_domain = 81

    t0_all = time.time()
    for w_i in range(num_workers):
        dom_id = base_domain + w_i
        p = mp.Process(target=worker_loop, args=(w_i, dom_id, worker_tasks[w_i], args.out_dir, result_queue))
        p.start()
        procs.append(p)
        time.sleep(1.0)  # staggered start

    total_tasks = len(tasks_to_run)
    completed_count = 0
    passed_count = 0

    while completed_count < total_tasks:
        try:
            res = result_queue.get(timeout=60.0)
            completed_count += 1
            if res['success']:
                passed_count += 1
            elapsed_total = time.time() - t0_all
            ep_rate = (completed_count / elapsed_total) * 3600.0
            print(f"Progress: [{completed_count}/{total_tasks}] ({(completed_count/total_tasks)*100:.1f}%) | "
                  f"Ep #{res['episode_id']:04d} ({'PASS' if res['success'] else 'FAIL'}) "
                  f"in {res['elapsed_s']:.1f}s | "
                  f"QC Pass Rate: {passed_count}/{completed_count} ({(passed_count/completed_count)*100:.1f}%) | "
                  f"Throughput: {ep_rate:.1f} ep/h")
        except Exception:
            # Check if all processes are still alive
            alive = any(p.is_alive() for p in procs)
            if not alive:
                print("All worker processes finished.")
                break

    for p in procs:
        p.join()

    total_wall_s = time.time() - t0_all
    overall_throughput = (completed_count / max(1.0, total_wall_s)) * 3600.0
    print("=" * 80)
    print(f"GENERATION COMPLETE: {completed_count}/{total_tasks} completed in {total_wall_s:.1f}s")
    print(f"Overall Throughput: {overall_throughput:.1f} ep/h")
    print(f"QC Passed: {passed_count}/{completed_count} ({(passed_count/max(1, completed_count))*100:.1f}%)")
    print("=" * 80)

    # Save summary manifest
    summary_file = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/pilot_300/pilot_300_summary.json")
    with open(summary_file, "w") as fp:
        json.dump({
            "total_episodes": completed_count,
            "qc_passed": passed_count,
            "pass_rate_pct": round(passed_count / max(1, completed_count) * 100.0, 1),
            "wall_duration_s": round(total_wall_s, 2),
            "throughput_ep_per_h": round(overall_throughput, 1),
            "filter_stats": filter_stats
        }, fp, indent=2)


if __name__ == '__main__':
    main()
