#!/usr/bin/env python3
"""
Quick investigation script for 3 seeds of 1.2m H3 (402, 403, 406):
Logs exact robot velocity, cmd_vel, and distance at every 0.05s up to the exact moment of collision.
"""
import os
import sys
import time
import math
import json
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

def compute_rect_circle_distance(rx: float, ry: float, ryaw: float, hx: float, hy: float, rw=0.20, rl=0.24, hr=0.28):
    dx = hx - rx
    dy = hy - ry
    cos_y = math.cos(ryaw)
    sin_y = math.sin(ryaw)
    x_rel = dx * cos_y + dy * sin_y
    y_rel = -dx * sin_y + dy * cos_y
    half_l = rl / 2.0
    half_w = rw / 2.0
    x_cl = max(-half_l, min(half_l, x_rel))
    y_cl = max(-half_w, min(half_w, y_rel))
    d_rect = math.hypot(x_rel - x_cl, y_rel - y_cl)
    return d_rect - hr

class CollisionVelocityInspector(Node):
    def __init__(self):
        super().__init__('collision_velocity_inspector')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.sim_time = 0.0
        self.robot_pos = (-13.0, 0.0)
        self.robot_yaw = 0.0
        self.robot_vel = (0.0, 0.0)
        self.human_pos = None
        self.human_vel = (0.0, 0.0)
        self.latest_cmd = (0.0, 0.0, 0.0)
        self.history = []

        self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 100)

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

    def _gt_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.robot_pos = (data['robot']['x'], data['robot']['y'])
                self.robot_vel = (data['robot'].get('vx', 0.0), data['robot'].get('vy', 0.0))
            humans = data.get("humans", [])
            if humans:
                self.human_pos = (humans[0].get('x', 0.0), humans[0].get('y', 0.0))
                self.human_vel = (humans[0].get('vx', 0.0), humans[0].get('vy', 0.0))
        except Exception:
            pass

    def inspect(self, max_duration=30.0) -> dict:
        t0 = time.time()
        while (self.human_pos is None or self.sim_time == 0.0) and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > 15.0:
                return {"error": "timeout_init"}

        t_start = self.sim_time
        collision_record = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            t_rel = self.sim_time - t_start
            if self.human_pos is not None:
                rx, ry = self.robot_pos
                hx, hy = self.human_pos
                d_clear = compute_rect_circle_distance(rx, ry, self.robot_yaw, hx, hy)
                r_spd = math.hypot(self.robot_vel[0], self.robot_vel[1])

                record = {
                    "t_rel": round(t_rel, 3),
                    "d_clear": round(d_clear, 4),
                    "rx": round(rx, 3), "ry": round(ry, 3),
                    "robot_vx": round(self.robot_vel[0], 4),
                    "robot_vy": round(self.robot_vel[1], 4),
                    "robot_speed": round(r_spd, 4),
                    "cmd_vx": round(self.latest_cmd[0], 4),
                    "cmd_vy": round(self.latest_cmd[1], 4),
                    "cmd_wz": round(self.latest_cmd[2], 4),
                    "hx": round(hx, 3), "hy": round(hy, 3),
                    "human_vx": round(self.human_vel[0], 4),
                }
                self.history.append(record)

                if d_clear <= 0.0 and collision_record is None:
                    collision_record = record
                    print(f"Collision detected at t={t_rel:.3f}s: robot_vel=({record['robot_vx']}, {record['robot_vy']}), speed={record['robot_speed']}", flush=True)
                    break

            if t_rel > max_duration:
                break

        return {
            "collision_record": collision_record,
            "final_record": self.history[-1] if self.history else None,
            "history_tail": self.history[-10:] if self.history else []
        }
