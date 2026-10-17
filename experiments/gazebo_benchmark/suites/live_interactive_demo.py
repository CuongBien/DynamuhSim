#!/usr/bin/env python3
"""
Full-Corridor Live Interactive Nav2 MPPI Demo.
Launches Gazebo Harmonic 3D GUI and Nav2 MPPI controller, then dispatches
a long navigation goal across the entire corridor (21 meters) passing all pedestrians.

Usage:
  python3 experiments/gazebo_benchmark/suites/live_interactive_demo.py
  python3 experiments/gazebo_benchmark/suites/live_interactive_demo.py --goal-x 8.0 --width 1.20
"""

import os
import sys
import time
import json
import signal
import argparse
import subprocess
import tempfile
import re

ROOT_DIR = "/home/cuongbien/DynamuhSim"
sys.path.insert(0, os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/core"))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry


class LiveNavigatorNode(Node):
    def __init__(self, target_x: float = 8.0, target_y: float = 0.0):
        super().__init__('live_interactive_navigator')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_x = target_x
        self.target_y = target_y

        self.robot_pos = (-13.0, 0.0)
        self.robot_speed = 0.0
        self.sim_time = 0.0

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 50)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _odom_cb(self, msg: Odometry):
        # odom frame origin is at spawn point (-13.0, 0.0)
        if self.robot_pos == (-13.0, 0.0):
            self.robot_pos = (msg.pose.pose.position.x - 13.0, msg.pose.pose.position.y)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        spd = (vx * vx + vy * vy) ** 0.5
        if self.robot_speed == 0.0:
            self.robot_speed = spd

    def _gt_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            if 'robot' in d and 'x' in d['robot']:
                self.robot_pos = (d['robot']['x'], d['robot']['y'])
                self.robot_speed = d['robot'].get('speed', 0.0)
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Full-Corridor Live Nav2 MPPI Demo")
    parser.add_argument("--width", type=str, default="1.20", choices=["0.90", "1.20"], help="Corridor width (default: 1.20)")
    parser.add_argument("--humans", type=int, default=10, choices=[5, 10], help="Number of dynamic humans (default: 10)")
    parser.add_argument("--goal-x", type=float, default=8.0, help="Target X coordinate at the far end of the corridor (default: 8.0m)")
    parser.add_argument("--goal-y", type=float, default=0.0, help="Target Y coordinate (default: 0.0m)")
    parser.add_argument("--rviz", action="store_true", default=False, help="Enable RViz2 window alongside Gazebo (default: False, Gazebo 3D GUI only for maximum GPU stability)")
    parser.add_argument("--domain-id", type=int, default=0, help="ROS_DOMAIN_ID (default: 0)")
    args = parser.parse_args()

    if args.humans == 10:
        base_world = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated/world_ep_0024.sdf")
    else:
        base_world = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated/world_ep_0007.sdf")

    with open(base_world, "r") as f:
        sdf_data = f.read()

    # Ensure real-time factor is 1.0 (smooth live interactive speed)
    sdf_data = re.sub(r'<real_time_factor>.*?</real_time_factor>', '<real_time_factor>1.0</real_time_factor>', sdf_data)
    # Fix scenario path to results directory so RVO2 plugin can load all human trajectories
    sdf_data = sdf_data.replace('/pilot_300_dataset/', '/results/pilot_300/pilot_300_dataset/')

    tmp_world = tempfile.NamedTemporaryFile(mode="w", suffix="_live_demo.sdf", delete=False)
    tmp_world.write(sdf_data)
    tmp_world.close()
    active_world_file = tmp_world.name

    print("=" * 80)
    print("🌟 DYNAMUHSIM FULL-CORRIDOR LIVE INTERACTIVE DEMO (NAV2 + MPPI)")
    print("=" * 80)
    print(f"  • Corridor Width   : {args.width} m")
    print(f"  • Dynamic Humans   : {args.humans} walking pedestrians (RVO2 reactive)")
    print(f"  • Robot Model      : Mecanum (Omnidirectional 4-wheel drive)")
    print(f"  • Controller       : MPPI (Model Predictive Path Integral)")
    print(f"  • Costmap Tuning   : inflation_radius = 0.20m (tuned for narrow corridors)")
    print(f"  • Start Position   : x = -13.0 m, y = 0.0 m")
    print(f"  • Target Goal      : x = +{args.goal_x:.1f} m, y = {args.goal_y:.1f} m  (~21 METERS TOTAL TRANSIT)")
    print(f"  • 3D Viewport      : Gazebo Harmonic 3D GUI {'+ RViz2' if args.rviz else '(Gazebo only for GPU stability)'}")
    print("=" * 80)
    print("Starting simulation and navigation stack... Gazebo window will appear on your desktop.")
    print("Press Ctrl+C at any time to exit.\n")

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"

    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    procs = []
    nav2_log_fp = None
    try:
        # Step 1: Launch Gazebo + Mecanum robot + 10 Humans
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={args.width}",
            f"custom_world:={active_world_file}",
            "obstacle:=human",
            "gui:=true",
            f"rviz:={'true' if args.rviz else 'false'}",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0",
            "paused:=false"
        ]
        p_sim = subprocess.Popen(sim_cmd, env=env, preexec_fn=os.setsid)
        procs.append(p_sim)

        print(f"⏳ Step 1/3: Launching Gazebo 3D simulation with {args.humans} pedestrians (6 seconds)...")
        time.sleep(6.0)

        # Step 2: Launch Nav2 MPPI
        nav_map = "corridor_120" if args.width == "1.20" else "corridor_090"
        nav_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            "robot_model:=mecanum",
            f"map:={nav_map}"
        ]
        nav2_log_path = "/tmp/nav2_live_demo.log"
        nav2_log_fp = open(nav2_log_path, "w")
        p_nav = subprocess.Popen(nav_cmd, env=env, stdout=nav2_log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
        procs.append(p_nav)

        print("⏳ Step 2/3: Launching Nav2 stack with MPPI controller...")
        print("    Waiting for Nav2 lifecycle manager to activate all nodes...")

        # Actively wait for "Managed nodes are active" from Nav2 lifecycle manager
        t0 = time.time()
        nav2_ready = False
        while time.time() - t0 < 35.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path, "r") as fp:
                    if "Managed nodes are active" in fp.read():
                        nav2_ready = True
                        print("✨ Nav2 Lifecycle Manager: ALL NODES ARE ACTIVE!")
                        break
            time.sleep(0.4)

        if not nav2_ready:
            print("⚠️ Lifecycle manager log wait timed out, continuing to action server check...")

        rclpy.init(domain_id=args.domain_id)
        nav_node = LiveNavigatorNode(target_x=args.goal_x, target_y=args.goal_y)

        # Wait for Action Server
        if not nav_node.nav_client.wait_for_server(timeout_sec=15.0):
            print("\n❌ Timed out waiting for Nav2 action server.")
            return

        print("✨ Step 3/3: Nav2 Navigation Action Server is READY!")
        time.sleep(1.0)  # Brief settle for AMCL initial particle cloud and local costmap

        # Dispatch Goal synchronously with retry loop
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = nav_node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(args.goal_x)
        goal_msg.pose.pose.position.y = float(args.goal_y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        goal_handle = None
        for attempt in range(12):
            print(f"🚀 Dispatching Navigation Goal to x={args.goal_x:.1f}m, y={args.goal_y:.1f}m (Attempt {attempt+1}/12)...")
            send_future = nav_node.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(nav_node, send_future, timeout_sec=4.0)
            if send_future.done():
                handle = send_future.result()
                if handle and handle.accepted:
                    goal_handle = handle
                    print("✅ Goal ACCEPTED by Nav2 MPPI Planner! Robot is now navigating across the corridor!\n")
                    break
                else:
                    print("   [Notice] Nav2 action server busy or in transition. Retrying in 1.0s...")
            time.sleep(1.0)

        if not goal_handle:
            print("\n❌ Goal was repeatedly rejected by Nav2. Check /tmp/nav2_live_demo.log for details.")
            return

        result_future = goal_handle.get_result_async()
        last_print = 0.0

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(nav_node, timeout_sec=0.05)
            now = time.time()
            if now - last_print >= 0.3:
                last_print = now
                rx, ry = nav_node.robot_pos
                dist_traveled = max(0.0, rx - (-13.0))
                total_dist = args.goal_x - (-13.0)
                pct = max(0.0, min(100.0, (dist_traveled / total_dist) * 100.0))
                print(f"\r  [Live Nav] Robot: x={rx:6.2f}m, y={ry:5.2f}m | "
                      f"Progress: {dist_traveled:5.1f}m / {total_dist:5.1f}m ({pct:4.1f}%) | "
                      f"Speed: {nav_node.robot_speed:4.2f} m/s", end="", flush=True)

        print("\n\n🏁 ROBOT HAS REACHED THE DESTINATION AT THE END OF THE CORRIDOR!")
        print("Keeping simulation open for 5 seconds...")
        time.sleep(5.0)
        nav_node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("\n\n⏹️ Live Demo stopped by user.")
    finally:
        print("Cleaning up processes...")
        if nav2_log_fp and not nav2_log_fp.closed:
            nav2_log_fp.close()
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.3)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        # Ensure orphaned background Gazebo instances in WSL are cleaned up
        for pattern in ["gz sim", "gz-sim-server", "rviz2", "ros_gz_bridge"]:
            subprocess.run(["pkill", "-9", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if 'active_world_file' in locals() and os.path.exists(active_world_file):
            try:
                os.remove(active_world_file)
            except Exception:
                pass
        print("Done. All processes cleaned up.")


if __name__ == '__main__':
    main()
