#!/usr/bin/env python3
"""
Interactive Visual Replay Tool for DynamuhSim.
Replays any recorded Pilot Episode with full Gazebo Harmonic 3D GUI and RViz2 interface.

Usage:
  python3 experiments/gazebo_benchmark/suites/pilot_300/replay_with_gui.py --episode 30
  python3 experiments/gazebo_benchmark/suites/pilot_300/replay_with_gui.py --episode 45 --rtf 1.0
  python3 experiments/gazebo_benchmark/suites/pilot_300/replay_with_gui.py --episode 24 --no-rviz
"""

import os
import sys
import time
import json
import re
import argparse
import subprocess
import tempfile
from typing import Dict, List, Any

ROOT_DIR = "/home/cuongbien/DynamuhSim"
sys.path.insert(0, os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/core"))

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env


class VisualPilotReplayer(Node):
    def __init__(self, recorded_cmds: List[Dict], duration_s: float):
        super().__init__('visual_pilot_replayer')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.duration_s = duration_s

        self.cmds = [(float(c.get('t_rel', 0.0)), float(c.get('vx', 0.0)), float(c.get('vy', 0.0)), float(c.get('w', 0.0))) for c in recorded_cmds]
        self.cmds.sort(key=lambda x: x[0])

        self.sim_time = 0.0
        self.t_sim_start = None
        self.unpaused = False
        self.cmd_idx = 0
        self.last_pub_sim_time = -1.0
        self.robot_pos = (-13.0, 0.0)
        self.active = False
        self.completed = False

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 50)

    def _gt_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            if 'robot' in d and 'x' in d['robot']:
                self.robot_pos = (d['robot']['x'], d['robot']['y'])
        except Exception:
            pass

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if not self.unpaused:
            return

        if self.t_sim_start is None:
            self.t_sim_start = self.sim_time
            self.active = True
            self.last_pub_sim_time = self.sim_time

        t_rel = self.sim_time - self.t_sim_start
        if t_rel >= self.duration_s:
            self.completed = True
            # Stop command
            stop_tw = TwistStamped()
            stop_tw.header.stamp = msg.clock
            stop_tw.header.frame_id = 'base_footprint'
            self.cmd_pub.publish(stop_tw)
            return

        # Advance scheduled command
        prev_idx = self.cmd_idx
        while self.cmd_idx < len(self.cmds) - 1 and self.cmds[self.cmd_idx + 1][0] <= t_rel:
            self.cmd_idx += 1

        # Dispatch at 50 Hz sim-time or on index advance
        if (self.sim_time - self.last_pub_sim_time >= 0.02) or (self.cmd_idx != prev_idx):
            self.last_pub_sim_time = self.sim_time
            sched_t, vx, vy, w = self.cmds[self.cmd_idx]
            tw = TwistStamped()
            tw.header.stamp = msg.clock
            tw.header.frame_id = 'base_footprint'
            tw.twist.linear.x = float(vx)
            tw.twist.linear.y = float(vy)
            tw.twist.angular.z = float(w)
            self.cmd_pub.publish(tw)


@scoped_processes
def main(*, _processes=None):
    parser = argparse.ArgumentParser(description="Replay recorded Pilot Episode with 3D Gazebo GUI and RViz2")
    parser.add_argument("--episode", type=int, default=24, help="Episode ID to replay (1..300, default: 24 [10 humans, 6m transit])")
    parser.add_argument("--gui", action="store_true", default=True, help="Enable Gazebo Harmonic 3D GUI window (default: True)")
    parser.add_argument("--no-gui", dest="gui", action="store_false", help="Disable Gazebo GUI (run headless)")
    parser.add_argument("--rviz", action="store_true", default=True, help="Enable RViz2 visualization window (default: True)")
    parser.add_argument("--no-rviz", dest="rviz", action="store_false", help="Disable RViz2 window")
    parser.add_argument("--rtf", type=float, default=1.0, help="Simulation Real Time Factor (default: 1.0 for natural real-time)")
    parser.add_argument("--a3-4", dest="a3_4", action="store_true", help="Replay A3.4 real head-on interaction (Seed 101, Corridor 0.90m)")
    parser.add_argument("--domain-id", type=int, default=88, help="ROS_DOMAIN_ID (default: 88)")

    args = parser.parse_args()

    if args.a3_4:
        rec_file = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/correctness_a3/a3_4_recorded_cmdvel.json")
        if not os.path.exists(rec_file):
            print(f"[ERROR] A3.4 recorded file not found: {rec_file}")
            sys.exit(1)
        with open(rec_file, "r") as f:
            rec_data = json.load(f)
        manifest = {
            "corridor": "corridor_090",
            "persona": "H1",
            "num_humans": 1,
            "time_to_goal": None
        }
        raw_cmds = rec_data.get("recorded_cmds", [])
        t_first = rec_data.get("t_first_cmd", raw_cmds[0]["sim_time"] if raw_cmds else 0.0)
        recorded_cmds = [{"t_rel": round(c["sim_time"] - t_first, 4), "vx": c["vx"], "vy": c["vy"], "w": c["w"]} for c in raw_cmds]
        orig_gt = [{"timestamp": g["t_sim"], "sim_time_rel": round(g["t_sim"] - t_first, 4), "robot": {"x": g["rx"], "y": g["ry"]}} for g in rec_data.get("recorded_gt", [])]
        corr_type = "090"
        world_name = "corridor_0.90"
        base_world_file = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/generated/world_a3_4_record_101.sdf")
        robot_file = ""
    else:
        episodes_dir = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/pilot_300/pilot_300_dataset/episodes")
        ep_dir = os.path.join(episodes_dir, f"ep_{args.episode:04d}")

        if not os.path.exists(ep_dir):
            print(f"[ERROR] Episode directory not found: {ep_dir}")
            print("Available episodes: ep_0001 to ep_0300")
            sys.exit(1)

        with open(os.path.join(ep_dir, "manifest.json")) as f:
            manifest = json.load(f)
        with open(os.path.join(ep_dir, "cmd_vel.json")) as f:
            recorded_cmds = json.load(f)
        with open(os.path.join(ep_dir, "ground_truth.json")) as f:
            orig_gt = json.load(f)

        corr = manifest.get("corridor", "corridor_090")
        corr_type = "090" if corr == "corridor_090" else "120"
        world_name = "corridor_0.90" if corr_type == "090" else "corridor_1.20"
        base_world_file = os.path.join(ROOT_DIR, f"experiments/gazebo_benchmark/configs/generated/world_ep_{args.episode:04d}.sdf")
        robot_file = os.path.join(ep_dir, "robot.sdf")

    if not os.path.exists(base_world_file):
        print(f"[ERROR] World file not found: {base_world_file}")
        sys.exit(1)

    # Prepare world file with requested real_time_factor
    with open(base_world_file, "r") as f:
        sdf_content = f.read()

    sdf_content = re.sub(r'<real_time_factor>.*?</real_time_factor>', f'<real_time_factor>{args.rtf}</real_time_factor>', sdf_content)
    sdf_content = sdf_content.replace('/pilot_300_dataset/', '/results/pilot_300/pilot_300_dataset/')
    tmp_world = tempfile.NamedTemporaryFile(mode="w", suffix=f"_visual_ep_{args.episode:04d}.sdf", delete=False)
    tmp_world.write(sdf_content)
    tmp_world.close()
    active_world_file = tmp_world.name

    duration_s = min(orig_gt[-1].get("sim_time_rel", 20.0), 20.0)
    if manifest.get("time_to_goal") is not None:
        duration_s = min(duration_s, float(manifest["time_to_goal"]))

    print("=" * 80)
    print(f"🎬 DYNAMUHSIM VISUAL REPLAY: EPISODE #{args.episode:04d}")
    print("=" * 80)
    print(f"  • Corridor Width   : {'0.90 m' if corr_type == '090' else '1.20 m'}")
    print(f"  • Persona Profile  : {manifest.get('persona', 'H1')}")
    print(f"  • Pedestrians      : {manifest.get('num_humans', 1)} humans")
    print(f"  • Robot Model      : Mecanum (4 mecanum wheels, lidar, RGB-D camera)")
    print(f"  • Episode Duration : {duration_s:.2f} s")
    print(f"  • Real Time Factor : {args.rtf}x")
    print(f"  • Gazebo 3D GUI    : {'ENABLED' if args.gui else 'DISABLED'}")
    print(f"  • RViz2 Interface  : {'ENABLED' if args.rviz else 'DISABLED'}")
    print(f"  • ROS_DOMAIN_ID    : {args.domain_id}")
    print("=" * 80)
    print("Starting simulation... Windows will appear on your desktop.")
    print("Press Ctrl+C at any time to exit.\n")

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    env = build_benchmark_env(ROOT_DIR)
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    env["GZ_PARTITION"] = f"gz_vis_{args.domain_id}_ep_{args.episode:04d}"

    try:
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={'0.90' if corr_type == '090' else '1.20'}",
            f"custom_world:={active_world_file}",
            f"custom_robot_model:={robot_file}",
            "obstacle:=human",
            f"gui:={'true' if args.gui else 'false'}",
            f"rviz:={'true' if args.rviz else 'false'}",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0",
            "paused:=false"
        ]

        p_sim = _processes.popen(sim_cmd, env=env, preexec_fn=os.setsid)

        if args.rviz:
            # Publish map -> odom static transform to satisfy RViz2 corridor config without Nav2
            tf_cmd = [
                "ros2", "run", "tf2_ros", "static_transform_publisher",
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "map", "--child-frame-id", "odom"
            ]
            p_tf = _processes.popen(tf_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        # Allow Gazebo and RViz2 GUI to initialize and load 3D meshes
        print("⏳ Waiting for Gazebo and RViz2 to initialize (3 seconds)...")
        time.sleep(3.5)

        rclpy.init()
        replayer = VisualPilotReplayer(recorded_cmds, duration_s=duration_s)
        replayer.unpaused = True

        print("▶️ Simulation active! Playback started.")
        start_wall = time.time()
        last_print = 0.0

        while rclpy.ok() and not replayer.completed:
            rclpy.spin_once(replayer, timeout_sec=0.02)
            cur_wall = time.time() - start_wall
            if cur_wall - last_print >= 0.2:
                last_print = cur_wall
                if replayer.t_sim_start is not None:
                    t_rel = max(0.0, replayer.sim_time - replayer.t_sim_start)
                    pct = min(100.0, (t_rel / duration_s) * 100.0)
                    print(f"\r  [Playback] Sim-Time: {t_rel:5.1f}s / {duration_s:5.1f}s ({pct:4.1f}%) | "
                          f"Robot Pos: x={replayer.robot_pos[0]:6.2f}m, y={replayer.robot_pos[1]:5.2f}m", end="", flush=True)

        print("\n\n✅ Episode playback finished successfully!")

        # Keep windows open for 2 seconds so user can see final resting state
        time.sleep(2.0)

        replayer.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("\n\n⏹️ Replay interrupted by user.")
    finally:
        print("Cleaning up processes...")
        _processes.stop()
        if os.path.exists(active_world_file):
            try:
                os.remove(active_world_file)
            except Exception:
                pass
        print("Done. All processes cleaned up.")


if __name__ == '__main__':
    main()
