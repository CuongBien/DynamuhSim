#!/usr/bin/env python3
"""
Diagnostic Script: Diagnose why the robot does not dodge the blind human (H3) in 1.2 m corridor.
Configuration:
- Corridor 120, Persona H3, Seed 401
- RTF = 1.0 (real_time_factor = 1.0 in world SDF)
- Baseline Nav2 MPPI config (nav2_mppi_pilot.yaml - NO PARAMETER CHANGES)
- Logs every 0.1s sim-time:
  - Robot-human distance (d_center, d_clear)
  - Minimum lidar in forward cone +-20 deg
  - cmd_vel (vx, vy, omega)
- At d ~ 4m and d ~ 2m:
  - Save local costmap image
  - Report cell cost values: at human position, right behind human, and side lanes (y +- 0.32m)
  - Check longitudinal continuity of free cells
- Answers questions (a) and (b).
"""

import os
import sys
import time
import math
import subprocess
import json
import re
from typing import Tuple, Dict, List, Optional
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, PoseStamped, PointStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf2_ros
import tf2_geometry_msgs

from pathlib import Path as FilePath

ROOT_DIR = str(FilePath(__file__).resolve().parents[4])
sys.path.insert(0, ROOT_DIR)
from experiments.gazebo_benchmark.core.geometry import compute_rect_circle_distance
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes
OUTPUT_DIR = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/mppi_ablation")
TELEMETRY_JSON = os.path.join(OUTPUT_DIR, "diagnose_h3_telemetry.json")
TELEMETRY_CSV = os.path.join(OUTPUT_DIR, "diagnose_h3_telemetry.csv")
SUMMARY_JSON = os.path.join(OUTPUT_DIR, "diagnose_h3_summary.json")
IMG_4M = os.path.join(OUTPUT_DIR, "costmap_snapshot_4m.png")
IMG_2M = os.path.join(OUTPUT_DIR, "costmap_snapshot_2m.png")


class H3CorridorDiagnoser(Node):
    def __init__(self, target_gx=-7.0, target_gy=0.0):
        super().__init__('h3_corridor_diagnoser')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_gx = target_gx
        self.target_gy = target_gy

        self.sim_time = 0.0
        self.t_start = None

        # Robot & Human state
        self.robot_pos = (-13.0, 0.0)
        self.robot_yaw = 0.0
        self.robot_vel = (0.0, 0.0)
        self.human_pos = None
        self.human_vel = (0.0, 0.0)

        # Telemetry
        self.latest_scan = None
        self.latest_cmd = (0.0, 0.0, 0.0)  # vx, vy, w
        self.latest_costmap = None

        # Snapshots
        self.snapshot_4m = None
        self.snapshot_2m = None
        self.first_human_detection = None  # (dist, time, max_cost_near_human)

        self.telemetry = []

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 100)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 100)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, 10)

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
        self.latest_cmd = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _costmap_cb(self, msg: OccupancyGrid):
        self.latest_costmap = msg

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

    def extract_lidar_min_cone(self, cone_deg=20.0) -> float:
        """Finds minimum range in forward cone +- cone_deg (default +-20 deg)."""
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            return 999.0
        scan = self.latest_scan
        cone_rad = math.radians(cone_deg)
        valid_ranges = []
        for i, r in enumerate(scan.ranges):
            ang = scan.angle_min + i * scan.angle_increment
            # Normalize angle to [-pi, pi]
            ang_norm = math.atan2(math.sin(ang), math.cos(ang))
            if abs(ang_norm) <= cone_rad:
                if scan.range_min <= r <= scan.range_max and not math.isinf(r) and not math.isnan(r):
                    valid_ranges.append(r)
        return min(valid_ranges) if valid_ranges else 999.0

    def get_cost_at_point(self, costmap: OccupancyGrid, px_odom: float, py_odom: float) -> Optional[int]:
        ox = costmap.info.origin.position.x
        oy = costmap.info.origin.position.y
        res = costmap.info.resolution
        w = costmap.info.width
        h = costmap.info.height

        mx = int(round((px_odom - ox) / res))
        my = int(round((py_odom - oy) / res))

        if 0 <= mx < w and 0 <= my < h:
            idx = my * w + mx
            return int(costmap.data[idx])
        return None

    def transform_map_to_costmap_frame(self, costmap: OccupancyGrid, wx: float, wy: float) -> Optional[Tuple[float, float]]:
        frame = costmap.header.frame_id
        if frame == 'map':
            return wx, wy
        try:
            pt = PointStamped()
            pt.header.frame_id = 'map'
            pt.point.x = float(wx)
            pt.point.y = float(wy)
            pt.point.z = 0.0
            tf = self.tf_buffer.lookup_transform(frame, 'map', rclpy.time.Time())
            pt_out = tf2_geometry_msgs.do_transform_point(pt, tf)
            return pt_out.point.x, pt_out.point.y
        except Exception:
            # Fallback using odom offset if TF fails
            # Robot at spawn is -13.0 in map, 0.0 in odom
            return wx + 13.0, wy

    def evaluate_costmap_features(self, costmap: OccupancyGrid, rx: float, ry: float, hx: float, hy: float) -> dict:
        """
        Evaluates cell costs at human, behind human, and side lanes.
        Also evaluates longitudinal profiles along y = +-0.32m between robot and human.
        """
        # Transform points to costmap frame
        h_tf = self.transform_map_to_costmap_frame(costmap, hx, hy)
        if not h_tf:
            return {}
        h_cx, h_cy = h_tf

        # Human center
        cost_human = self.get_cost_at_point(costmap, h_cx, h_cy)

        # Right behind human (along human direction, which is walking -x, so behind is +0.35m in x)
        h_behind_tf = self.transform_map_to_costmap_frame(costmap, hx + 0.35, hy)
        cost_behind = self.get_cost_at_point(costmap, h_behind_tf[0], h_behind_tf[1]) if h_behind_tf else None

        # Left lane at x = hx, y = hy + 0.32
        h_left_tf = self.transform_map_to_costmap_frame(costmap, hx, hy + 0.32)
        cost_left = self.get_cost_at_point(costmap, h_left_tf[0], h_left_tf[1]) if h_left_tf else None

        # Right lane at x = hx, y = hy - 0.32
        h_right_tf = self.transform_map_to_costmap_frame(costmap, hx, hy - 0.32)
        cost_right = self.get_cost_at_point(costmap, h_right_tf[0], h_right_tf[1]) if h_right_tf else None

        # Longitudinal profile along lanes between robot x and human x
        # Sample points every 0.10m from rx to hx + 0.5m
        x_min = min(rx, hx)
        x_max = max(rx, hx) + 0.5
        xs = np.linspace(x_min, x_max, int((x_max - x_min) / 0.10) + 1)

        left_lane_costs = []
        right_lane_costs = []
        for x_val in xs:
            p_l = self.transform_map_to_costmap_frame(costmap, x_val, +0.32)
            if p_l:
                c_l = self.get_cost_at_point(costmap, p_l[0], p_l[1])
                if c_l is not None:
                    left_lane_costs.append(c_l)

            p_r = self.transform_map_to_costmap_frame(costmap, x_val, -0.32)
            if p_r:
                c_r = self.get_cost_at_point(costmap, p_r[0], p_r[1])
                if c_r is not None:
                    right_lane_costs.append(c_r)

        # Check for continuous free strip (all costs in profile <= threshold)
        # Threshold for inscribed / footprint collision in OccupancyGrid is 99 (internal 253)
        # Free cell is cost == 0, low cost is cost < 50
        free_strip_left_strict = all(c == 0 for c in left_lane_costs) if left_lane_costs else False
        free_strip_left_navigable = all(0 <= c < 99 for c in left_lane_costs) if left_lane_costs else False

        free_strip_right_strict = all(c == 0 for c in right_lane_costs) if right_lane_costs else False
        free_strip_right_navigable = all(0 <= c < 99 for c in right_lane_costs) if right_lane_costs else False

        return {
            "cost_at_human": cost_human,
            "cost_behind_human": cost_behind,
            "cost_left_lane_at_human": cost_left,
            "cost_right_lane_at_human": cost_right,
            "left_lane_profile": {
                "min": int(np.min(left_lane_costs)) if left_lane_costs else None,
                "max": int(np.max(left_lane_costs)) if left_lane_costs else None,
                "mean": float(np.mean(left_lane_costs)) if left_lane_costs else None,
                "has_continuous_strict_free_0": free_strip_left_strict,
                "has_continuous_navigable_under_99": free_strip_left_navigable,
                "count_cells": len(left_lane_costs),
                "cells": [int(c) for c in left_lane_costs]
            },
            "right_lane_profile": {
                "min": int(np.min(right_lane_costs)) if right_lane_costs else None,
                "max": int(np.max(right_lane_costs)) if right_lane_costs else None,
                "mean": float(np.mean(right_lane_costs)) if right_lane_costs else None,
                "has_continuous_strict_free_0": free_strip_right_strict,
                "has_continuous_navigable_under_99": free_strip_right_navigable,
                "count_cells": len(right_lane_costs),
                "cells": [int(c) for c in right_lane_costs]
            }
        }

    def render_costmap_image(self, costmap: OccupancyGrid, rx: float, ry: float, ryaw: float, hx: float, hy: float, title: str, save_path: str, feats: dict):
        w = costmap.info.width
        h = costmap.info.height
        res = costmap.info.resolution
        ox = costmap.info.origin.position.x
        oy = costmap.info.origin.position.y

        grid = np.array(costmap.data, dtype=np.int16).reshape((h, w))
        # Mask unknown (-1) as gray
        grid_plot = np.where(grid == -1, np.nan, grid)

        fig, ax = plt.subplots(figsize=(10, 8), dpi=150)
        extent = [ox, ox + w * res, oy, oy + h * res]
        im = ax.imshow(grid_plot, origin='lower', extent=extent, cmap='viridis', vmin=0, vmax=100)
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Costmap Value (0=Free, 99=Inscribed, 100=Lethal)')

        # Transform robot and human into costmap frame
        r_tf = self.transform_map_to_costmap_frame(costmap, rx, ry)
        h_tf = self.transform_map_to_costmap_frame(costmap, hx, hy)

        if r_tf:
            rcx, rcy = r_tf
            # Draw robot rectangular footprint (0.24 x 0.20 m)
            rect = patches.Rectangle((-0.12, -0.10), 0.24, 0.20, angle=math.degrees(ryaw),
                                     rotation_point=(0, 0), facecolor='cyan', edgecolor='blue', alpha=0.8, label='Robot Footprint (0.24x0.20m)')
            t_rect = matplotlib.transforms.Affine2D().rotate_around(rcx, rcy, ryaw).translate(rcx, rcy)
            # Use plot marker for robot center
            ax.plot(rcx, rcy, 'co', markersize=8, label='Robot Center')
            ax.arrow(rcx, rcy, 0.3 * math.cos(ryaw), 0.3 * math.sin(ryaw), head_width=0.06, head_length=0.08, fc='blue', ec='blue')

        if h_tf:
            hcx, hcy = h_tf
            # Draw human circle radius 0.28m
            circ = patches.Circle((hcx, hcy), 0.28, facecolor='red', edgecolor='darkred', alpha=0.5, label='Human Proxy (r=0.28m)')
            ax.add_patch(circ)
            ax.plot(hcx, hcy, 'r*', markersize=10, label='Human Center')

            # Annotate points
            ax.plot(hcx, hcy + 0.32, 'm^', markersize=8, label=f'Left Lane (+0.32m, cost={feats.get("cost_left_lane_at_human")})')
            ax.plot(hcx, hcy - 0.32, 'v', color='orange', markersize=8, label=f'Right Lane (-0.32m, cost={feats.get("cost_right_lane_at_human")})')
            ax.plot(hcx + 0.35, hcy, 's', color='lime', markersize=8, label=f'Behind Human (+0.35m, cost={feats.get("cost_behind_human")})')

        # Draw Corridor walls at y = +-0.60m
        wall_top_tf = self.transform_map_to_costmap_frame(costmap, rx, +0.60)
        wall_bot_tf = self.transform_map_to_costmap_frame(costmap, rx, -0.60)
        if wall_top_tf and wall_bot_tf:
            ax.axhline(wall_top_tf[1], color='black', linestyle='--', linewidth=1.5, label='Corridor Wall (y=+0.60m)')
            ax.axhline(wall_bot_tf[1], color='black', linestyle='--', linewidth=1.5, label='Corridor Wall (y=-0.60m)')

        # Annotate costs box
        info_text = (
            f"Distance: {math.hypot(rx - hx, ry - hy):.2f}m\n"
            f"Cost at Human: {feats.get('cost_at_human')}\n"
            f"Cost Behind Human (+0.35m): {feats.get('cost_behind_human')}\n"
            f"Left Lane (+0.32m): {feats.get('cost_left_lane_at_human')}\n"
            f"Right Lane (-0.32m): {feats.get('cost_right_lane_at_human')}\n"
            f"Left Lane Continuous Navigable (<99): {feats.get('left_lane_profile', {}).get('has_continuous_navigable_under_99')}\n"
            f"Right Lane Continuous Navigable (<99): {feats.get('right_lane_profile', {}).get('has_continuous_navigable_under_99')}"
        )
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.85, edgecolor='gray'))

        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.set_xlabel('X [m] (Costmap Frame)')
        ax.set_ylabel('Y [m] (Costmap Frame)')
        ax.legend(loc='lower left', fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.4)
        plt.tight_layout()
        plt.savefig(save_path)
        plt.close()
        print(f"[OK] Saved costmap visualization: {save_path}", flush=True)

    def send_nav_goal(self) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target_gx)
        goal_msg.pose.pose.position.y = float(self.target_gy)
        goal_msg.pose.pose.orientation.w = 1.0

        for _ in range(5):
            future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done():
                handle = future.result()
                if handle.accepted:
                    return True
            time.sleep(0.5)
        return False

    def run_diagnosis(self) -> dict:
        t_w0 = time.time()
        while time.time() - t_w0 < 2.5 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.04)

        if not self.send_nav_goal():
            return {"error": "goal_rejected_or_timeout"}

        self.t_start = self.sim_time
        t_wall_start = time.time()
        last_log_t = -1.0

        print(">>> Starting telemetry recording at 10 Hz sim...", flush=True)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            t_rel = self.sim_time - self.t_start
            if t_rel < 0.0:
                continue

            rx, ry = self.robot_pos
            ryaw = self.robot_yaw
            vx, vy, w = self.latest_cmd

            hx = self.human_pos[0] if self.human_pos else -7.0
            hy = self.human_pos[1] if self.human_pos else 0.0

            d_center, d_clear = compute_rect_circle_distance(rx, ry, ryaw, hx, hy)
            lidar_min_20deg = self.extract_lidar_min_cone(20.0)

            # Check if human appears in costmap
            if self.latest_costmap is not None:
                h_tf = self.transform_map_to_costmap_frame(self.latest_costmap, hx, hy)
                if h_tf:
                    # Check in 0.35m radius around human for any lethal (100) or high cost cell
                    ox = self.latest_costmap.info.origin.position.x
                    oy = self.latest_costmap.info.origin.position.y
                    res = self.latest_costmap.info.resolution
                    w_cm = self.latest_costmap.info.width
                    h_cm = self.latest_costmap.info.height
                    hcx, hcy = h_tf
                    r_cells = int(0.35 / res)
                    mx0 = int((hcx - ox) / res)
                    my0 = int((hcy - oy) / res)

                    max_c_near = 0
                    for dx_i in range(-r_cells, r_cells + 1):
                        for dy_i in range(-r_cells, r_cells + 1):
                            cx = mx0 + dx_i
                            cy = my0 + dy_i
                            if 0 <= cx < w_cm and 0 <= cy < h_cm:
                                val = self.latest_costmap.data[cy * w_cm + cx]
                                if val > max_c_near:
                                    max_c_near = val

                    if max_c_near >= 99 and self.first_human_detection is None and t_rel > 0.5:
                        self.first_human_detection = {
                            "d_center_m": round(d_center, 3),
                            "t_sim_s": round(t_rel, 2),
                            "max_cost_near_human": max_c_near
                        }
                        print(f"  [COSTMAP DETECTION] Human first marked in costmap at d = {d_center:.3f}m (t = {t_rel:.2f}s, max cost = {max_c_near})!", flush=True)

            # Snapshot at d ~ 4m
            if d_center <= 4.10 and self.snapshot_4m is None and self.latest_costmap is not None and t_rel > 0.5:
                feats = self.evaluate_costmap_features(self.latest_costmap, rx, ry, hx, hy)
                self.snapshot_4m = {
                    "t_sim": round(t_rel, 2),
                    "d_center": round(d_center, 3),
                    "d_clear": round(d_clear, 3),
                    "rx": round(rx, 3), "ry": round(ry, 3), "yaw": round(ryaw, 3),
                    "hx": round(hx, 3), "hy": round(hy, 3),
                    "lidar_min_cone_20deg": round(lidar_min_20deg, 3),
                    "cmd_vel": [round(vx, 3), round(vy, 3), round(w, 3)],
                    "features": feats
                }
                self.render_costmap_image(self.latest_costmap, rx, ry, ryaw, hx, hy,
                                          f"Local Costmap Snapshot at d ≈ 4.0 m (d = {d_center:.2f} m, t = {t_rel:.2f} s)",
                                          IMG_4M, feats)

            # Snapshot at d ~ 2m
            if d_center <= 2.10 and self.snapshot_2m is None and self.latest_costmap is not None and t_rel > 0.5:
                feats = self.evaluate_costmap_features(self.latest_costmap, rx, ry, hx, hy)
                self.snapshot_2m = {
                    "t_sim": round(t_rel, 2),
                    "d_center": round(d_center, 3),
                    "d_clear": round(d_clear, 3),
                    "rx": round(rx, 3), "ry": round(ry, 3), "yaw": round(ryaw, 3),
                    "hx": round(hx, 3), "hy": round(hy, 3),
                    "lidar_min_cone_20deg": round(lidar_min_20deg, 3),
                    "cmd_vel": [round(vx, 3), round(vy, 3), round(w, 3)],
                    "features": feats
                }
                self.render_costmap_image(self.latest_costmap, rx, ry, ryaw, hx, hy,
                                          f"Local Costmap Snapshot at d ≈ 2.0 m (d = {d_center:.2f} m, t = {t_rel:.2f} s)",
                                          IMG_2M, feats)

            # Telemetry record (10 Hz sim: every 0.1s)
            if self.sim_time - last_log_t >= 0.10:
                last_log_t = self.sim_time
                self.telemetry.append({
                    "t_sim": round(t_rel, 2),
                    "d_center": round(d_center, 3),
                    "d_clear": round(d_clear, 3),
                    "lidar_min_cone_20deg": round(lidar_min_20deg, 3) if lidar_min_20deg < 900 else None,
                    "cmd_vx": round(vx, 3),
                    "cmd_vy": round(vy, 3),
                    "cmd_w": round(w, 3),
                    "rx": round(rx, 3),
                    "ry": round(ry, 3),
                    "ryaw": round(ryaw, 3),
                    "hx": round(hx, 3),
                    "hy": round(hy, 3)
                })

            # Check collision
            if t_rel > 0.5 and d_clear <= 0.0:
                print(f"  [COLLISION] Contact at t_sim = {t_rel:.2f}s, d_center = {d_center:.3f}m, d_clear = {d_clear:.4f}m", flush=True)
                break

            if t_rel >= 15.0 or (time.time() - t_wall_start) > 40.0:
                break

        return {
            "telemetry_count": len(self.telemetry),
            "first_human_detection": self.first_human_detection,
            "snapshot_4m": self.snapshot_4m,
            "snapshot_2m": self.snapshot_2m
        }


@scoped_processes
def run_diagnose_pipeline(*, _processes=None):
    domain_id = 77
    corridor_type = "120"
    persona = "H3"
    seed = 401
    v_human = 1.15

    gen_dir = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated")
    os.makedirs(gen_dir, exist_ok=True)

    scen_file = os.path.join(gen_dir, f"scenario_diagnose_{corridor_type}_{persona}_{seed}.json")
    scen_data = {
        "episode_id": seed,
        "corridor_mode": True,
        "humans": [{
            "id": 1, "name": "human_01", "mode": "autonomous",
            "speed": v_human, "max_speed": v_human, "persona": persona,
            "loop": False, "wait_for_robot": True,
            "start": {"x": -7.0, "y": 0.0, "yaw": 3.14159265},
            "waypoints": [[-7.0, 0.0], [-15.0, 0.0]]
        }]
    }
    with open(scen_file, "w") as f:
        json.dump(scen_data, f, indent=2)

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

    proxy_z = "0.875"
    sdf = re.sub(r'<actor name=\"human_01_visual\">\s*<pose>.*?</pose>', '<actor name=\"human_01_visual\">\n      <pose>-7.0 0.0 0.0 0 0 3.14159</pose>', sdf)
    sdf = re.sub(r'<model name=\"human_01_proxy\">\s*<pose>.*?</pose>', f'<model name=\"human_01_proxy\">\n      <pose>-7.0 0.0 {proxy_z} 0 0 3.14159</pose>', sdf)

    trial_sdf = os.path.join(gen_dir, f"world_diagnose_{corridor_type}_{persona}_{seed}.sdf")
    with open(trial_sdf, "w") as f:
        f.write(sdf)

    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["ROS_DOMAIN_ID"] = str(domain_id)
    partition = f"diagnose_{corridor_type}_{persona}_{seed}_{domain_id}"
    env["GZ_PARTITION"] = partition

    nav2_log_path = f"/tmp/nav2_diagnose_{domain_id}.log"
    log_fp = open(nav2_log_path, "w")

    params_file = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_pilot.yaml")


    try:
        time.sleep(1.5)

        print(">>> 1. Launching Gazebo Corridor 1.20m (RTF = 1.0)...", flush=True)
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            "width:=1.20",
            f"custom_world:={trial_sdf}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        gz_p = _processes.popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(4.5)

        print(">>> 2. Launching Nav2 MPPI (Baseline pilot configs)...", flush=True)
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={params_file}",
            "map:=corridor_120",
            "robot_model:=mecanum"
        ]
        nav2_p = _processes.popen(nav2_cmd, env=env, stdout=log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

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
            print("[ERROR] Nav2 activation timed out!", flush=True)
            return

        print(">>> 3. Nav2 active! Initializing Diagnoser Node...", flush=True)
        time.sleep(1.0)
        os.environ["ROS_DOMAIN_ID"] = str(domain_id)
        os.environ["GZ_PARTITION"] = partition
        rclpy.init()
        diagnoser = H3CorridorDiagnoser(target_gx=-7.0, target_gy=0.0)
        diag_summary = diagnoser.run_diagnosis()
        telemetry = diagnoser.telemetry
        diagnoser.destroy_node()
        rclpy.shutdown()

    finally:
        _processes.stop()
        time.sleep(2.0)
        log_fp.close()

    # Save telemetry files
    with open(TELEMETRY_JSON, "w") as fp:
        json.dump(telemetry, fp, indent=2)

    import csv
    if telemetry:
        keys = telemetry[0].keys()
        with open(TELEMETRY_CSV, "w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=keys)
            writer.writeheader()
            writer.writerows(telemetry)

    with open(SUMMARY_JSON, "w") as fp:
        json.dump(diag_summary, fp, indent=2)

    print("\n" + "=" * 80)
    print("DIAGNOSTIC PIPELINE COMPLETE")
    print("=" * 80)
    print(f"Telemetry JSON: {TELEMETRY_JSON} ({len(telemetry)} frames)")
    print(f"Telemetry CSV:  {TELEMETRY_CSV}")
    print(f"Summary JSON:    {SUMMARY_JSON}")
    print(f"Snapshot 4m PNG: {IMG_4M}")
    print(f"Snapshot 2m PNG: {IMG_2M}")


if __name__ == '__main__':
    run_diagnose_pipeline()
