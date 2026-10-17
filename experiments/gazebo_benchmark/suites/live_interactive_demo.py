#!/usr/bin/env python3
"""
Full-Corridor Live Interactive Nav2 MPPI Demo.
Launches Gazebo Harmonic 3D GUI, RViz2, and Nav2 MPPI controller, then dispatches
a long navigation goal across the entire corridor (20+ meters) passing all pedestrians.

Usage:
  python3 experiments/gazebo_benchmark/suites/live_interactive_demo.py
  python3 experiments/gazebo_benchmark/suites/live_interactive_demo.py --goal-x 8.0 --width 1.20
"""

import os
import sys
import time
import signal
import argparse
import subprocess

ROOT_DIR = "/home/cuongbien/DynamuhSim"
sys.path.insert(0, os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/core"))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock


class LiveNavigatorNode(Node):
    def __init__(self, target_x: float = 8.0, target_y: float = 0.0):
        super().__init__('live_interactive_navigator')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_x = target_x
        self.target_y = target_y

        self.robot_pos = (-13.0, 0.0)
        self.robot_speed = 0.0
        self.sim_time = 0.0
        self.goal_sent = False
        self.goal_done = False

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _gt_cb(self, msg: String):
        try:
            import json
            d = json.loads(msg.data)
            if 'robot' in d and 'x' in d['robot']:
                self.robot_pos = (d['robot']['x'], d['robot']['y'])
                self.robot_speed = d['robot'].get('speed', 0.0)
        except Exception:
            pass

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target_x)
        goal_msg.pose.pose.position.y = float(self.target_y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        print(f"\n🚀 Sending Navigation Goal: target_x = {self.target_x:.1f}m, target_y = {self.target_y:.1f}m")
        self.goal_future = self.nav_client.send_goal_async(goal_msg)
        self.goal_future.add_done_callback(self._goal_response_cb)
        self.goal_sent = True

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ Goal was rejected by Nav2!")
            self.goal_done = True
            return
        print("✅ Goal accepted by Nav2 planner! Robot is now moving...\n")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        print("\n\n🏁 ROBOT HAS REACHED THE DESTINATION AT THE END OF THE CORRIDOR!")
        self.goal_done = True


def main():
    parser = argparse.ArgumentParser(description="Full-Corridor Live Nav2 MPPI Demo")
    parser.add_argument("--width", type=str, default="1.20", choices=["0.90", "1.20"], help="Corridor width (default: 1.20)")
    parser.add_argument("--humans", type=int, default=10, choices=[5, 10], help="Number of dynamic humans (default: 10)")
    parser.add_argument("--goal-x", type=float, default=8.0, help="Target X coordinate at the far end of the corridor (default: 8.0m)")
    parser.add_argument("--goal-y", type=float, default=0.0, help="Target Y coordinate (default: 0.0m)")
    parser.add_argument("--rviz", action="store_true", default=False, help="Enable RViz2 window alongside Gazebo (default: False, Gazebo 3D GUI only for maximum GPU stability)")
    parser.add_argument("--domain-id", type=int, default=0, help="ROS_DOMAIN_ID (default: 0)")
    args = parser.parse_args()

    import tempfile, re
    if args.humans == 10:
        base_world = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated/world_ep_0024.sdf")
    else:
        base_world = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated/world_ep_0007.sdf")

    with open(base_world, "r") as f:
        sdf_data = f.read()
    sdf_data = re.sub(r'<real_time_factor>.*?</real_time_factor>', '<real_time_factor>1.0</real_time_factor>', sdf_data)
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
    print("Starting simulation and navigation stack... Windows will appear on your desktop.")
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
        p_nav = subprocess.Popen(nav_cmd, env=env, preexec_fn=os.setsid)
        procs.append(p_nav)

        print("⏳ Step 2/3: Launching Nav2 stack with MPPI controller...")
        print("    Waiting for Nav2 lifecycle manager to become active (usually 10-15s)...")

        rclpy.init(domain_id=args.domain_id)
        nav_node = LiveNavigatorNode(target_x=args.goal_x, target_y=args.goal_y)

        # Wait for Nav2 action server to be available
        t_wait_start = time.time()
        nav_ready = False
        while time.time() - t_wait_start < 40.0:
            rclpy.spin_once(nav_node, timeout_sec=0.2)
            if nav_node.nav_client.wait_for_server(timeout_sec=0.5):
                nav_ready = True
                print("✨ Step 3/3: Nav2 Navigation Server is READY!")
                break
            print(".", end="", flush=True)

        if not nav_ready:
            print("\n❌ Timed out waiting for Nav2 action server.")
            return

        # Give AMCL a brief second to settle initial localization
        time.sleep(1.5)

        # Dispatch Goal
        nav_node.send_goal()

        start_time = time.time()
        last_print = 0.0

        while rclpy.ok() and not nav_node.goal_done:
            rclpy.spin_once(nav_node, timeout_sec=0.05)
            now = time.time()
            if now - last_print >= 0.3:
                last_print = now
                rx, ry = nav_node.robot_pos
                dist_traveled = rx - (-13.0)
                total_dist = args.goal_x - (-13.0)
                pct = max(0.0, min(100.0, (dist_traveled / total_dist) * 100.0))
                print(f"\r  [Live Nav] Robot: x={rx:6.2f}m, y={ry:5.2f}m | "
                      f"Progress: {dist_traveled:5.1f}m / {total_dist:5.1f}m ({pct:4.1f}%) | "
                      f"Speed: {nav_node.robot_speed:4.2f} m/s", end="", flush=True)

        print("\nDemo completed. Keeping visualization open for 5 seconds...")
        time.sleep(5.0)
        nav_node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("\n\n⏹️ Live Demo stopped by user.")
    finally:
        print("Cleaning up processes...")
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
