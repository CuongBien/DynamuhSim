#!/usr/bin/env python3
"""
Full-Corridor Live Interactive Nav2 MPPI Demo with Dynamic Human Personas (H1-H5).
Features:
- Random sampling of human personas (40% H1, 10% H2, 25% H3, 15% H4, 10% H5).
- Staggered Gazebo 3D + RViz2 launch for 100% WSLg GPU stability.
- Automatic navigation to target goal + Interactive Mode (use RViz2 '2D Goal Pose' to move the robot).

Usage:
  python3 experiments/gazebo_benchmark/suites/live_interactive_demo.py
  python3 experiments/gazebo_benchmark/suites/live_interactive_demo.py --humans 5 --goal-x 8.0
  python3 experiments/gazebo_benchmark/suites/live_interactive_demo.py --humans 10 --no-rviz
"""

import os
import sys
import time
import json
import math
import random
import argparse
import subprocess
import tempfile
import numpy as np

ROOT_DIR = "/home/cuongbien/DynamuhSim"
sys.path.insert(0, os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/core"))

from benchmark_world_generator import BenchmarkWorldGenerator

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env

PERSONA_CONFIGS = {
    "H1": {"name": "Cooperative", "trait": "Reciprocal ORCA (alpha=0.50), yields laterally", "color": "\033[92m"},
    "H2": {"name": "Unyielding", "trait": "Low-responsibility ORCA (alpha=0.20), presses forward", "color": "\033[91m"},
    "H3": {"name": "Robot-Blind", "trait": "Ignores robot completely (phone distracted)", "color": "\033[93m"},
    "H4": {"name": "Static/Idle", "trait": "Stationary conversational group / bystander (v=0)", "color": "\033[94m"},
    "H5": {"name": "Social-Force", "trait": "Soft exponential repulsion cushion + smooth LPF", "color": "\033[95m"}
}
RESET_COLOR = "\033[0m"


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


def generate_random_scenario(num_humans: int = 5, seed: int = None):
    """
    Generates a realistic scenario JSON with personas sampled from the agreed
    natural proportion: 40% H1, 10% H2, 25% H3, 15% H4, 10% H5.
    """
    if seed is None:
        seed = int(time.time()) % 100000
    rng = np.random.RandomState(seed)

    persona_choices = ['H1', 'H2', 'H3', 'H4', 'H5']
    persona_probs = [0.40, 0.10, 0.25, 0.15, 0.10]

    # Human 1 meets the robot head-on in the encounter zone
    h1_persona = str(rng.choice(persona_choices, p=persona_probs))
    h1_speed = 0.0 if h1_persona == "H4" else float(rng.uniform(0.90, 1.15))
    h1_mode = "stationary" if h1_persona == "H4" else "autonomous"
    h1_start_x = float(rng.uniform(-5.5, -4.0))
    h1_start_y = float(rng.uniform(-0.25, 0.25)) if h1_persona != "H4" else float(rng.choice([-0.35, 0.35]))

    humans = [{
        "id": 1,
        "name": "human_01",
        "mode": h1_mode,
        "speed": round(h1_speed, 3),
        "max_speed": round(h1_speed * 1.2, 3),
        "persona": h1_persona,
        "loop": False,
        "wait_for_robot": True,
        "start": {"x": round(h1_start_x, 3), "y": round(h1_start_y, 3), "yaw": 3.14159},
        "waypoints": [[round(h1_start_x, 3), round(h1_start_y, 3)], [-15.0, round(h1_start_y, 3)]],
        "proxy_z": 0.875
    }]

    # Background humans spaced along corridor (-2.0m to 10.5m)
    if num_humans > 1:
        bg_xs = np.linspace(-2.0, 10.5, num_humans - 1)
        for i in range(2, num_humans + 1):
            p = str(rng.choice(persona_choices, p=persona_probs))
            bx = float(bg_xs[i - 2] + rng.uniform(-0.35, 0.35))
            if p == "H4":
                # Static bystander standing near the corridor wall
                by = float(rng.choice([-0.36, 0.36]))
                b_spd = 0.0
                b_mode = "stationary"
                b_yaw = float(rng.choice([0.0, 3.14159, 1.57, -1.57]))
                wps = [[round(bx, 3), round(by, 3)]]
            else:
                by = float(rng.uniform(-0.28, 0.28))
                b_spd = float(rng.uniform(0.75, 1.10))
                b_mode = "autonomous"
                # Alternate directions: even walking towards -X, odd walking towards +X
                b_dir = -1.0 if (i % 2 == 0) else 1.0
                b_yaw = 3.14159 if b_dir < 0 else 0.0
                b_target_x = -14.5 if b_dir < 0 else 14.5
                wps = [[round(bx, 3), round(by, 3)], [b_target_x, round(by, 3)]]

            humans.append({
                "id": i,
                "name": f"human_{i:02d}",
                "mode": b_mode,
                "speed": round(b_spd, 3),
                "max_speed": round(b_spd * 1.2 if b_spd > 0 else 0.0, 3),
                "persona": p,
                "loop": False,
                "wait_for_robot": False,
                "start": {"x": round(bx, 3), "y": round(by, 3), "yaw": b_yaw},
                "waypoints": wps,
                "proxy_z": 0.875
            })

    return {"seed": seed, "humans": humans}


@scoped_processes
def main(*, _processes=None):
    parser = argparse.ArgumentParser(description="Full-Corridor Live Interactive Nav2 MPPI Demo")
    parser.add_argument("--width", type=str, default="1.20", choices=["0.90", "1.20"], help="Corridor width (default: 1.20)")
    parser.add_argument("--humans", type=int, default=5, choices=[2, 5, 8, 10], help="Number of dynamic humans (default: 5)")
    parser.add_argument("--goal-x", type=float, default=8.0, help="Initial target X coordinate at end of corridor (default: 8.0m)")
    parser.add_argument("--goal-y", type=float, default=0.0, help="Initial target Y coordinate (default: 0.0m)")
    parser.add_argument("--rviz", action="store_true", default=True, help="Enable RViz2 window (default: True)")
    parser.add_argument("--no-rviz", dest="rviz", action="store_false", help="Disable RViz2 window (Gazebo only)")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for personas and positions")
    parser.add_argument("--domain-id", type=int, default=0, help="ROS_DOMAIN_ID (default: 0)")
    args = parser.parse_args()

    # Step A: Generate Scenario with Persona Distribution
    scen_data = generate_random_scenario(num_humans=args.humans, seed=args.seed)
    tmp_scen = tempfile.NamedTemporaryFile(mode="w", suffix="_scenario.json", delete=False)
    json.dump(scen_data, tmp_scen, indent=2)
    tmp_scen.close()
    scen_file = tmp_scen.name

    # Step B: Generate World SDF using BenchmarkWorldGenerator
    gen = BenchmarkWorldGenerator(ROOT_DIR)
    corr_type = "090" if args.width == "0.90" else "120"
    active_world_file = gen.generate_world_with_humans(
        num_humans=args.humans,
        corridor_type=corr_type,
        seed=scen_data["seed"],
        real_time_factor=1.0,
        out_filename=f"live_demo_{args.humans}h_{scen_data['seed']}.sdf",
        scenario_file=scen_file,
        scenario_humans=scen_data["humans"]
    )

    print("=" * 80)
    print("🌟 DYNAMUHSIM FULL-CORRIDOR LIVE INTERACTIVE DEMO (NAV2 MPPI + RVO2)")
    print("=" * 80)
    print(f"  • Corridor Width   : {args.width} m")
    print(f"  • Robot Model      : Mecanum (4-wheel omnidirectional drive)")
    print(f"  • Navigation Goal  : x = +{args.goal_x:.1f} m, y = {args.goal_y:.1f} m  (~21 METERS TOTAL TRANSIT)")
    print(f"  • Random Seed      : {scen_data['seed']}")
    print(f"  • 3D Viewport      : Gazebo Harmonic 3D GUI {'+ RViz2' if args.rviz else '(Gazebo only)'}")
    print("=" * 80)
    print(f"👥 Pedestrian Crowd Roster ({args.humans} Humans sampled from 40% H1 / 10% H2 / 25% H3 / 15% H4 / 10% H5):")
    print("  " + "-" * 76)
    for h in scen_data["humans"]:
        p = h["persona"]
        p_info = PERSONA_CONFIGS.get(p, {"name": p, "trait": "", "color": ""})
        c = p_info["color"]
        print(f"  • {h['name']}: {c}[{p} - {p_info['name']}]{RESET_COLOR} | "
              f"Speed: {h['speed']:4.2f} m/s | Pos: ({h['start']['x']:5.2f}, {h['start']['y']:5.2f}) | {p_info['trait']}")
    print("  " + "-" * 76)
    print("Starting simulation and navigation stack... Windows will appear on your desktop.")
    print("Press Ctrl+C at any time to exit.\n")

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    env = build_benchmark_env(ROOT_DIR)
    env["ROS_DOMAIN_ID"] = str(args.domain_id)

    nav2_log_fp = None
    try:
        # Step 1: Launch Gazebo Simulation (rviz:=false to let Gazebo initialize OpenGL first)
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={args.width}",
            f"custom_world:={active_world_file}",
            "obstacle:=human",
            "gui:=true",
            "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0",
            "paused:=false"
        ]
        p_sim = _processes.popen(sim_cmd, env=env, preexec_fn=os.setsid)

        print(f"⏳ Step 1/3: Launching Gazebo 3D simulation ({args.humans} pedestrians, 6 seconds)...")
        time.sleep(6.0)

        # Step 2: Launch Nav2 MPPI Stack
        nav_map = "corridor_120" if args.width == "1.20" else "corridor_090"
        nav_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            "robot_model:=mecanum",
            f"map:={nav_map}"
        ]
        nav2_log_path = "/tmp/nav2_live_demo.log"
        nav2_log_fp = open(nav2_log_path, "w")
        p_nav = _processes.popen(nav_cmd, env=env, stdout=nav2_log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

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

        # Step 3: Launch RViz2 (Staggered after Gazebo is fully stabilized to avoid WSLg GPU context clash)
        if args.rviz:
            print("⏳ Step 3/3: Opening RViz2 Visualization Window...")
            rviz_cfg = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/rviz/corridor.rviz")
            rviz_cmd = ["rviz2", "-d", rviz_cfg]
            p_rviz = _processes.popen(rviz_cmd, env=env, preexec_fn=os.setsid)
            time.sleep(2.0)

        rclpy.init(domain_id=args.domain_id)
        nav_node = LiveNavigatorNode(target_x=args.goal_x, target_y=args.goal_y)

        # Wait for Action Server
        if not nav_node.nav_client.wait_for_server(timeout_sec=15.0):
            print("\n❌ Timed out waiting for Nav2 action server.")
            return

        print("✨ Nav2 Navigation Action Server is READY!")
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

        # Phase 1: Track transit to target goal
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

        print("\n\n🏁 Initial Goal reached! Entering Interactive Navigation Mode...")
        print("=" * 80)
        print("🎮 INTERACTIVE MODE ACTIVE:")
        print("  • In RViz2, click the '2D Goal Pose' (Nav2 Goal) button at the top toolbar.")
        print("  • Click & drag anywhere on the corridor map to send a new destination to the robot.")
        print("  • Watch the robot reactively dodge walking H1-H5 pedestrians in real time!")
        print("  • Press Ctrl+C in this terminal when finished.")
        print("=" * 80)

        # Phase 2: Interactive loop - keeps running and listening to RViz2 goal clicks!
        while rclpy.ok():
            rclpy.spin_once(nav_node, timeout_sec=0.1)
            now = time.time()
            if now - last_print >= 0.5:
                last_print = now
                rx, ry = nav_node.robot_pos
                print(f"\r  [Interactive Robot Pos] x={rx:6.2f}m, y={ry:5.2f}m | Speed: {nav_node.robot_speed:4.2f} m/s  (Click '2D Goal Pose' in RViz2 to move)   ", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\n⏹️ Live Demo stopped by user.")
    finally:
        print("Cleaning up processes...")
        if nav2_log_fp and not nav2_log_fp.closed:
            nav2_log_fp.close()
        _processes.stop()
        # Ensure orphaned background Gazebo instances in WSL are cleaned up
        if 'scen_file' in locals() and os.path.exists(scen_file):
            try: os.remove(scen_file)
            except Exception: pass
        print("Done. All processes cleaned up.")


if __name__ == '__main__':
    main()
