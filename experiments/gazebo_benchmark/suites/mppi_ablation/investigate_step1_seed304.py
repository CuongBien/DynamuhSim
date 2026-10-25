#!/usr/bin/env python3
"""
Telemetry logger for Step 1 Seed 304 to investigate why it timed out.
Logs position, cmd_vel, distance to goal, and cost at goal every 0.1s.
"""

import os, sys, time, math, signal, subprocess, json, re
import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes

ROOT_DIR = "/home/cuongbien/DynamuhSim"
ROBOT_MODEL_SDF = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/robot_models/model_mecanum_step1.sdf")
NAV2_PARAMS_YAML = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/nav2_mppi/nav2_mppi_step1.yaml")
WORLD_SDF = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated/world_a3_3_step1_120_H1_304.sdf")

class Seed304Logger(Node):
    def __init__(self):
        super().__init__('seed_304_logger')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.sim_time = 0.0
        self.robot_pos = (-13.0, 0.0)
        self.robot_yaw = 0.0
        self.latest_cmd = (0.0, 0.0, 0.0)
        self.human_pos = (-7.0, 0.0)
        self.latest_costmap = None
        self.latest_global_costmap = None
        self.odom_pos = (0.0, 0.0)
        self.telemetry = []

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 100)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._costmap_cb, 10)
        self.global_costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._global_costmap_cb, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _odom_cb(self, msg: Odometry):
        self.odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _cmd_cb(self, msg: TwistStamped):
        self.latest_cmd = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def _costmap_cb(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def _global_costmap_cb(self, msg: OccupancyGrid):
        self.latest_global_costmap = msg

    def _gt_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            if 'robot' in d and d['robot']['x'] is not None:
                self.robot_pos = (d['robot']['x'], d['robot']['y'])
            if 'humans' in d and len(d['humans']) > 0:
                h = d['humans'][0]
                if h.get('x') is not None and h.get('y') is not None:
                    self.human_pos = (h['x'], h['y'])
        except Exception:
            pass

    def get_local_cost_at_world(self, wx: float, wy: float):
        if self.latest_costmap is None:
            return None
        ox = self.latest_costmap.info.origin.position.x
        oy = self.latest_costmap.info.origin.position.y
        res = self.latest_costmap.info.resolution
        w = self.latest_costmap.info.width
        h = self.latest_costmap.info.height
        # In odom frame: wx + 13.0
        mx = int(round((wx + 13.0 - ox) / res))
        my = int(round((wy - oy) / res))
        if 0 <= mx < w and 0 <= my < h:
            return int(self.latest_costmap.data[my * w + mx])
        return None

    def get_global_cost_at_world(self, wx: float, wy: float):
        if self.latest_global_costmap is None:
            return None
        ox = self.latest_global_costmap.info.origin.position.x
        oy = self.latest_global_costmap.info.origin.position.y
        res = self.latest_global_costmap.info.resolution
        w = self.latest_global_costmap.info.width
        h = self.latest_global_costmap.info.height
        mx = int(round((wx - ox) / res))
        my = int(round((wy - oy) / res))
        if 0 <= mx < w and 0 <= my < h:
            return int(self.latest_global_costmap.data[my * w + mx])
        return None

    def send_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = -7.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0
        for _ in range(5):
            future = self.nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done() and future.result().accepted:
                return True
            time.sleep(0.5)
        return False

    def run(self):
        time.sleep(2.0)
        if not self.send_goal():
            print("Failed to send goal")
            return
        t0 = self.sim_time
        t_last = -1.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            t_rel = self.sim_time - t0
            if t_rel < 0: continue
            if self.sim_time - t_last >= 0.10:
                t_last = self.sim_time
                rx, ry = self.robot_pos
                vx, vy, w = self.latest_cmd
                hx, hy = self.human_pos
                d_goal = math.hypot(rx - (-7.0), ry - 0.0)
                cost_local = self.get_local_cost_at_world(-7.0, 0.0)
                cost_global = self.get_global_cost_at_world(-7.0, 0.0)
                self.telemetry.append({
                    "t_sim": round(t_rel, 2),
                    "rx": round(rx, 3), "ry": round(ry, 3), "yaw": round(self.robot_yaw, 3),
                    "vx": round(vx, 3), "vy": round(vy, 3), "w": round(w, 3),
                    "hx": round(hx, 3), "hy": round(hy, 3),
                    "d_goal": round(d_goal, 3),
                    "cost_local_at_goal": cost_local,
                    "cost_global_at_goal": cost_global
                })
            if d_goal <= 0.35:
                print(f"Goal reached at t_sim = {t_rel:.2f}s")
                break
            if t_rel >= 25.0:
                print("Max duration 25s reached")
                break

@scoped_processes
def main(*, _processes=None):
    domain_id = 89
    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.dirname(os.path.dirname(plugin_share)) + "/lib"

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = "test_304"
    
    time.sleep(1.5)
    
    sim_cmd = [
        "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
        "width:=1.20", f"custom_world:={WORLD_SDF}",
        f"custom_robot_model:={ROBOT_MODEL_SDF}",
        "obstacle:=human", "gui:=false", "rviz:=false",
        "robot_model:=mecanum", "x:=-13.0", "y:=0.0", "yaw:=0.0"
    ]
    gz_p = _processes.popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(4.5)
    
    nav_cmd = [
        "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
        "controller:=mppi", f"params_file:={NAV2_PARAMS_YAML}",
        "map:=corridor_120", "robot_model:=mecanum"
    ]
    nav_log = open("/tmp/nav2_test_304.log", "w")
    nav_p = _processes.popen(nav_cmd, env=env, stdout=nav_log, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
    
    t0 = time.time()
    while time.time() - t0 < 60.0:
        if os.path.exists("/tmp/nav2_test_304.log"):
            with open("/tmp/nav2_test_304.log") as f:
                if f.read().count("Managed nodes are active") >= 2:
                    break
        time.sleep(0.4)
        
    time.sleep(1.0)
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    os.environ["GZ_PARTITION"] = "test_304"
    rclpy.init()
    node = Seed304Logger()
    node.run()
    telemetry = node.telemetry
    node.destroy_node()
    rclpy.shutdown()
    
    _processes.stop()
    nav_log.close()
    
    out_path = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/mppi_ablation/step1_seed304_telemetry.json")
    with open(out_path, "w") as f:
        json.dump(telemetry, f, indent=2)
    print(f"Saved {len(telemetry)} frames to {out_path}")

if __name__ == '__main__':
    main()
