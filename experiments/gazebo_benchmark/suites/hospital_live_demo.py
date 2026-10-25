#!/usr/bin/env python3
"""
Hospital Arena Live Interactive Navigation Demo.
Launches Gazebo Harmonic with Hospital Arena (Easy/Medium/Hard) and Nav2 stack
(supporting either Mecanum or TurtleBot3 Burger with MPPI/DWB), then guides
the robot through the clinical ward route (North Room 5 -> South Room 5 -> Return).

Usage:
  python3 experiments/gazebo_benchmark/suites/hospital_live_demo.py --difficulty easy --robot-model mecanum
  python3 experiments/gazebo_benchmark/suites/hospital_live_demo.py --difficulty medium --robot-model mecanum
  python3 experiments/gazebo_benchmark/suites/hospital_live_demo.py --difficulty hard --robot-model burger
"""

import os
import sys
import time
import math
import argparse
import subprocess

ROOT_DIR = "/home/cuongbien/DynamuhSim"
sys.path.insert(0, os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/core"))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env

# Waypoint route in Hospital Arena:
# Robot spawns at (-18.0, 0.0)
HOSPITAL_ROUTE = [
    ("North Room 5", -2.8, 3.8, math.pi / 2),
    ("Exit North Room 5", -2.8, 0.0, -math.pi / 2),
    ("South Room 5", -2.8, -3.8, -math.pi / 2),
    ("Exit South Room 5", -2.8, 0.0, math.pi / 2),
    ("Return to Entrance", -18.0, 0.0, 0.0),
]


class HospitalNavigatorNode(Node):
    def __init__(self):
        super().__init__('hospital_live_navigator')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.robot_pos = (-18.0, 0.0)
        self.robot_speed = 0.0
        self.sim_time = 0.0

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _odom_cb(self, msg: Odometry):
        # odom origin in hospital world is (-18.0, 0.0)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.robot_pos = (px - 18.0, py)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_speed = (vx * vx + vy * vy) ** 0.5


@scoped_processes
def main(*, _processes=None):
    parser = argparse.ArgumentParser(description="Hospital Arena Live Interactive Navigation Demo")
    parser.add_argument("--difficulty", type=str, default="easy", choices=["easy", "medium", "hard"], help="Hospital difficulty level (default: easy)")
    parser.add_argument("--robot-model", type=str, default="mecanum", choices=["mecanum", "burger"], help="Robot drive model: mecanum or burger (default: mecanum)")
    parser.add_argument("--controller", type=str, default="mppi", choices=["mppi", "dwb"], help="Nav2 controller: mppi or dwb (default: mppi)")
    parser.add_argument("--rviz", action="store_true", default=True, help="Launch RViz2 window (default: True)")
    parser.add_argument("--no-rviz", dest="rviz", action="store_false", help="Disable RViz2 window (Gazebo only)")
    parser.add_argument("--interactive", action="store_true", default=False, help="Skip automated route and start in interactive 2D Goal Pose mode immediately")
    parser.add_argument("--domain-id", type=int, default=0, help="ROS_DOMAIN_ID (default: 0)")
    args = parser.parse_args()

    human_counts = {
        "easy": "3 dynamic walkers (H1-H5 social personas)",
        "medium": "7 walkers + 1 trolley + 3 static (yielding crossing, clinical patrols)",
        "hard": "14 walkers + 2 trolleys + 3 static (complex ward traffic, door exits, group walks)"
    }

    print("=" * 80)
    print("🏥 DYNAMUHSIM HOSPITAL ARENA LIVE INTERACTIVE DEMO")
    print("=" * 80)
    print(f"  • Level / Difficulty : {args.difficulty.upper()} ({human_counts[args.difficulty]})")
    print(f"  • Robot Model        : {args.robot_model.upper()} ({'Omnidirectional Mecanum' if args.robot_model == 'mecanum' else 'Differential Burger'})")
    print(f"  • Nav2 Controller    : {args.controller.upper()}")
    print(f"  • Hospital Size      : 60m x 17m (Corridor Width: 5.0m, 20 Clinical Rooms)")
    print(f"  • Start Pose         : x = -18.0 m, y = 0.0 m")
    print(f"  • Route Waypoints    : 5 clinical stops (North Room 5 -> South Room 5 -> Home)")
    print(f"  • 3D Viewport        : Gazebo Harmonic 3D GUI {'+ RViz2' if args.rviz else '(Gazebo only)'}")
    print("=" * 80)
    print("Starting simulation and navigation stack... Windows will appear on your desktop.")
    print("Press Ctrl+C at any time to exit.\n")

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    env = build_benchmark_env(ROOT_DIR)
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    package_share = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor")

    nav2_log_fp = None
    try:
        # Step 1: Launch Hospital Arena Simulation (rviz:=false to let Gazebo initialize OpenGL first)
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "hospital_arena.launch.py",
            f"difficulty:={args.difficulty}",
            f"robot_model:={args.robot_model}",
            "gui:=true",
            "rviz:=false"
        ]
        p_sim = _processes.popen(sim_cmd, env=env, preexec_fn=os.setsid)

        print(f"⏳ Step 1/3: Launching Gazebo 3D simulation ({args.difficulty} level, 6 seconds)...")
        time.sleep(6.0)

        # Step 2: Launch Nav2 Hospital Stack
        nav2_log_path = "/tmp/nav2_hospital_live.log"
        nav2_log_fp = open(nav2_log_path, "w")
        nav_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_hospital.launch.py",
            f"controller:={args.controller}",
            f"robot_model:={args.robot_model}",
            f"map:=hospital/hospital_{args.difficulty}",
            "auto_route:=false"
        ]
        p_nav = _processes.popen(nav_cmd, env=env, stdout=nav2_log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

        print("⏳ Step 2/3: Launching Nav2 stack...")
        print("    Waiting for Nav2 lifecycle managers to activate all nodes...")

        # Wait until "Managed nodes are active" in nav2 log
        t0 = time.time()
        nav2_ready = False
        while time.time() - t0 < 35.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path, "r") as fp:
                    if fp.read().count("Managed nodes are active") >= 2:
                        nav2_ready = True
                        print("✨ Nav2 Lifecycle Manager: ALL LOCALIZATION & NAVIGATION NODES ARE ACTIVE!")
                        break
            time.sleep(0.4)

        if not nav2_ready:
            print("⚠️ Lifecycle manager log wait timed out, continuing to action server check...")

        # Step 3: Launch RViz2 (Staggered after Gazebo is fully stabilized to avoid WSLg GPU context clash)
        if args.rviz:
            print("⏳ Step 3/3: Opening RViz2 Visualization Window...")
            rviz_cfg = os.path.join(package_share, "rviz", "hospital.rviz")
            if not os.path.exists(rviz_cfg):
                rviz_cfg = os.path.join(package_share, "rviz", "corridor.rviz")
            rviz_cmd = ["rviz2", "-d", rviz_cfg]
            p_rviz = _processes.popen(rviz_cmd, env=env, preexec_fn=os.setsid)
            time.sleep(2.0)

        rclpy.init(domain_id=args.domain_id)
        nav_node = HospitalNavigatorNode()

        # Wait for Action Server
        if not nav_node.nav_client.wait_for_server(timeout_sec=15.0):
            print("\n❌ Timed out waiting for Nav2 action server.")
            return

        print("✨ Nav2 Navigation Server is READY!")
        time.sleep(2.0)  # Brief settle for AMCL initial localization on 60m hospital map

        last_print = 0.0

        if not args.interactive:
            print("\n🚀 Beginning Clinical Waypoint Route:")
            for idx, (name, gx, gy, gyaw) in enumerate(HOSPITAL_ROUTE, 1):
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = PoseStamped()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.header.stamp = nav_node.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = float(gx)
                goal_msg.pose.pose.position.y = float(gy)
                goal_msg.pose.pose.orientation.z = math.sin(gyaw / 2.0)
                goal_msg.pose.pose.orientation.w = math.cos(gyaw / 2.0)

                print(f"\n📍 [{idx}/{len(HOSPITAL_ROUTE)}] Heading to '{name}': target_x = {gx:5.1f}m, target_y = {gy:5.1f}m")

                goal_handle = None
                for attempt in range(10):
                    send_future = nav_node.nav_client.send_goal_async(goal_msg)
                    rclpy.spin_until_future_complete(nav_node, send_future, timeout_sec=4.0)
                    if send_future.done():
                        handle = send_future.result()
                        if handle and handle.accepted:
                            goal_handle = handle
                            print(f"   ✅ Goal accepted by Nav2 planner. Robot is moving...")
                            break
                    print(f"   ⏳ Retrying goal dispatch (attempt {attempt+1}/10)...")
                    time.sleep(1.0)

                if not goal_handle:
                    print(f"   ❌ Goal rejected for {name}. Halting route.")
                    break

                result_future = goal_handle.get_result_async()

                while rclpy.ok() and not result_future.done():
                    rclpy.spin_once(nav_node, timeout_sec=0.05)
                    now = time.time()
                    if now - last_print >= 0.3:
                        last_print = now
                        rx, ry = nav_node.robot_pos
                        d_rem = math.hypot(gx - rx, gy - ry)
                        print(f"\r   [Transit] Robot Pos: x={rx:6.2f}m, y={ry:5.2f}m | Distance to '{name}': {d_rem:5.2f}m | Speed: {nav_node.robot_speed:4.2f} m/s", end="", flush=True)

                status = result_future.result().status if result_future.done() else GoalStatus.STATUS_UNKNOWN
                if status == GoalStatus.STATUS_SUCCEEDED:
                    print(f"\n   🎉 Reached '{name}' successfully!")
                else:
                    print(f"\n   ⚠️ Navigation completed with status: {status}")

                time.sleep(1.0)

            print("\n\n🏁 Clinical Waypoint Route completed! Entering Interactive Mode...")

        print("=" * 80)
        print("🎮 INTERACTIVE MODE ACTIVE:")
        print("  • In RViz2, click the '2D Goal Pose' (Nav2 Goal) button at the top toolbar.")
        print("  • Click & drag anywhere in the 60m hospital ward to send new destinations.")
        print("  • Watch the omnidirectional robot navigate reactively around hospital pedestrians!")
        print("  • Press Ctrl+C in this terminal when finished.")
        print("=" * 80)

        while rclpy.ok():
            rclpy.spin_once(nav_node, timeout_sec=0.1)
            now = time.time()
            if now - last_print >= 0.5:
                last_print = now
                rx, ry = nav_node.robot_pos
                print(f"\r  [Hospital Robot Pos] x={rx:6.2f}m, y={ry:5.2f}m | Speed: {nav_node.robot_speed:4.2f} m/s  (Click '2D Goal Pose' in RViz2 to move)   ", end="", flush=True)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\n\n⏹️ Hospital Demo stopped.")
    finally:
        print("Cleaning up processes...")
        if nav2_log_fp and not nav2_log_fp.closed:
            nav2_log_fp.close()
        _processes.stop()
        print("Done. All processes cleaned up.")


if __name__ == '__main__':
    main()
