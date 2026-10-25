"""Clock-driven trajectory recording shared by the A3 determinism suites.

The synchronized profile retains its odometry fallback, progress logging and
longer watchdog. Command and sampling intervals use the historical comparisons.
"""
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import gz.transport13 as gzt
import gz.msgs.stringmsg_pb2 as stringmsg_pb2


class PreciseTrajectoryRecorder(Node):
    node_name = "precise_recorder"
    synchronized = False

    def __init__(self, target_duration=20.0):
        super().__init__(self.node_name)
        self.target_duration = target_duration
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        if self.synchronized:
            self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 50)

        self.gz_node = gzt.Node()
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, '/rvo2_human/ground_truth', self._gt_cb)

        self.sim_time = 0.0
        self.latest_pos = None
        self.trajectory = []  # list of (t_rel, x, y)
        self.t_start = None
        self.last_cmd_sim_time = -1.0
        self.last_sample_sim_time = -1.0

    def _gt_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.latest_pos = (data['robot']['x'], data['robot']['y'])
        except Exception:
            pass

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.t_start is None:
            self.t_start = self.sim_time
            self.last_cmd_sim_time = self.sim_time
            self.last_sample_sim_time = self.sim_time
            return

        t_rel = self.sim_time - self.t_start
        # Publish fixed cmd_vel every 0.02s sim-time
        if self.sim_time - self.last_cmd_sim_time >= 0.02:
            self.last_cmd_sim_time = self.sim_time
            twist = TwistStamped()
            twist.header.stamp = msg.clock
            twist.twist.linear.x = 0.15
            twist.twist.angular.z = 0.05 * np.sin(0.1 * t_rel)
            self.cmd_pub.publish(twist)

        # Sample trajectory point every 0.1s sim-time
        if self.sim_time - self.last_sample_sim_time >= 0.1:
            self.last_sample_sim_time = self.sim_time
            pos = self.latest_pos if self.latest_pos is not None else (3.0, 0.0)
            self.trajectory.append((t_rel, pos[0], pos[1]))

    def _odom_cb(self, msg: Odometry):
        if self.latest_pos is None:
            self.latest_pos = (3.0 + msg.pose.pose.position.x, msg.pose.pose.position.y)

    def run_until_duration(self):
        t_wall_start = time.time()
        last_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.t_start is not None:
                sim_elapsed = self.sim_time - self.t_start
                if self.synchronized and sim_elapsed - last_log >= 3.0:
                    print(f"    [Sim Time: {sim_elapsed:.1f}s / {self.target_duration:.1f}s] pos: {self.latest_pos}", flush=True)
                    last_log = sim_elapsed
                if sim_elapsed >= self.target_duration:
                    break
            if time.time() - t_wall_start > (self.target_duration * 3.0 + (15.0 if self.synchronized else 10.0)):
                if self.synchronized:
                    print("    [WARN] Hard wall-clock timeout reached in run_until_duration!", flush=True)
                break
        return self.trajectory


class SynchronizedTrajectoryCollector(PreciseTrajectoryRecorder):
    node_name = "sync_traj_collector"
    synchronized = True
