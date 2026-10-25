"""Shared MPPI interaction monitor; suite files select a measurement profile.

One callback/action/measurement implementation preserves the Step1/2/3 result
schemas, timer budgets and subscriptions. No processes are launched here.
"""
from dataclasses import dataclass
import json
import math
import time
from typing import Optional
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from .geometry import compute_rect_circle_distance, check_footprint_wall_contact


@dataclass(frozen=True)
class MonitorProfile:
    node_name: str
    max_duration_s: float
    watchdog_extra_s: float
    lanes: bool = False
    steering: bool = False


STEP1 = MonitorProfile("step1_interaction_monitor", 25., 20.)
STEP2 = MonitorProfile("step2_interaction_monitor", 25., 20., lanes=True)
STEP3 = MonitorProfile("step3_interaction_monitor", 40., 35., lanes=True, steering=True)


class InteractionMonitor(Node):
    def __init__(self, target_gx=-7.0, target_gy=0.0, max_duration_s=None, corridor_half_width=0.45, *, profile=STEP3):
        super().__init__(profile.node_name)
        self._profile = profile
        if max_duration_s is None:
            max_duration_s = profile.max_duration_s
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
        self.latest_cmd = (0.0, 0.0, 0.0)
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

        # Steering start detection (|vy| > 0.05 m/s or |y| > 0.05 m)
        self.steer_detected = False
        self.t_steer_s = None
        self.d_human_at_steer_m = None

        # First costmap detection tracking
        self.first_costmap_dist = None
        self.first_costmap_t = None
        self.latest_costmap = None

        # 2m Snapshot tracking & global path
        self.snapshot_2m = None
        self.latest_plan = None
        self.global_path_y_at_2m = None

        # cmd_vel tracking
        self.cmd_sim_times = []

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 100)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, 10)
        if self._profile.steering:
            self.plan_sub = self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _costmap_cb(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def _plan_cb(self, msg: Path):
        self.latest_plan = msg

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
        self.latest_cmd = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

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
        hcx = hx + 13.0
        cost_left_at_human = self.get_cost_at_point(costmap, hcx, +0.32)
        cost_right_at_human = self.get_cost_at_point(costmap, hcx, -0.32)

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

        # Timer starts STRICTLY upon goal acceptance!
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

            # Steering start detection (|vy| > 0.05 m/s or |ry| > 0.05 m)
            if self._profile.steering and not self.steer_detected and t_rel > 0.5:
                if abs(self.latest_cmd[1]) > 0.05 or abs(ry) > 0.05:
                    self.steer_detected = True
                    self.t_steer_s = round(t_rel, 2)
                    if self.human_pos is not None:
                        hx, hy = self.human_pos
                        dc, _ = compute_rect_circle_distance(rx, ry, ryaw, hx, hy)
                        self.d_human_at_steer_m = round(dc, 3)

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
                if self._profile.lanes and cur_dc <= 2.10 and self.snapshot_2m is None and self.latest_costmap is not None and t_rel > 0.5:
                    self.snapshot_2m = self.evaluate_lanes_at_2m(self.latest_costmap, rx, ry, hx, hy)
                    # Also extract global plan y at human x
                    if self._profile.steering and self.latest_plan is not None and len(self.latest_plan.poses) > 0:
                        closest_wp = min(self.latest_plan.poses, key=lambda p: abs(p.pose.position.x - hx))
                        self.global_path_y_at_2m = round(closest_wp.pose.position.y, 4)

                # Footprint contact collision: d_clear <= 0.0 m (only after t_rel > 0.5)
                if t_rel > 0.5 and cur_dclear <= 0.0:
                    self.collision_contact = True
                    self.time_to_collision = round(t_rel, 2)
                    print(f"    [COLLISION DETECTED] Footprint contact (d_clear={cur_dclear:.4f}m) at t_sim={t_rel:.2f}s! Truncating episode.", flush=True)
                    break

            # Wall collision check
            if check_footprint_wall_contact(ry, ryaw, self.corridor_half_width):
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

            # Profile timeout is measured from goal acceptance.
            if t_rel >= self.max_duration_s:
                if self._profile.steering:
                    print(f"    [TIMEOUT] 40.0s sim-time reached from goal dispatch! Final dist: {dist_to_goal:.3f}m", flush=True)
                break
            if time.time() - t_wall_start > (self.max_duration_s + self._profile.watchdog_extra_s):
                if self._profile.steering:
                    print(f"    [WATCHDOG TIMEOUT] Wall clock safety limit reached.", flush=True)
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

        # Near-miss (sượt): 0.0 < d_clear < 0.02 m
        near_miss_flag = (0.0 < self.dclear_min < 0.02) if self.dclear_min < 900 else False

        snap_left = None
        snap_right = None
        min_left_prof = None
        min_right_prof = None
        if self.snapshot_2m:
            snap_left = self.snapshot_2m.get("cost_left_lane_at_human")
            snap_right = self.snapshot_2m.get("cost_right_lane_at_human")
            min_left_prof = self.snapshot_2m.get("min_cost_left_lane_profile")
            min_right_prof = self.snapshot_2m.get("min_cost_right_lane_profile")

        result = {
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
            "frozen_at_start": self.frozen_at_start,
            "steer_detected": self.steer_detected,
            "t_steer_s": self.t_steer_s,
            "d_human_at_steer_m": self.d_human_at_steer_m,
            "cmd_vel_count": len(self.cmd_sim_times),
            "cmd_vel_sim_hz_mean": cmd_hz_mean,
            "cmd_vel_sim_hz_min": cmd_hz_min,
            "first_costmap_dist_m": self.first_costmap_dist,
            "first_costmap_t_s": self.first_costmap_t,
            "cost_left_lane_at_2m": snap_left,
            "cost_right_lane_at_2m": snap_right,
            "min_cost_left_lane_profile_at_2m": min_left_prof,
            "min_cost_right_lane_profile_at_2m": min_right_prof,
            "global_path_y_at_2m": self.global_path_y_at_2m,
            "final_dist_to_goal": round(math.hypot(self.robot_pos[0] - self.target_gx, self.robot_pos[1] - self.target_gy), 3),
        }
        if not self._profile.lanes:
            for key in ("y_at_min_dclear", "detour_around_human", "cost_left_lane_at_2m", "cost_right_lane_at_2m",
                        "min_cost_left_lane_profile_at_2m", "min_cost_right_lane_profile_at_2m"):
                del result[key]
        if not self._profile.steering:
            for key in ("near_miss_under_2cm", "steer_detected", "t_steer_s", "d_human_at_steer_m", "global_path_y_at_2m"):
                del result[key]
        return result


class Step1InteractionMonitor(InteractionMonitor):
    def __init__(self, target_gx=-7.0, target_gy=0.0, max_duration_s=25.0, corridor_half_width=0.45):
        super().__init__(target_gx, target_gy, max_duration_s, corridor_half_width, profile=STEP1)


class Step2InteractionMonitor(InteractionMonitor):
    def __init__(self, target_gx=-7.0, target_gy=0.0, max_duration_s=25.0, corridor_half_width=0.45):
        super().__init__(target_gx, target_gy, max_duration_s, corridor_half_width, profile=STEP2)


class Step3InteractionMonitor(InteractionMonitor):
    def __init__(self, target_gx=-7.0, target_gy=0.0, max_duration_s=40.0, corridor_half_width=0.45):
        super().__init__(target_gx, target_gy, max_duration_s, corridor_half_width, profile=STEP3)
