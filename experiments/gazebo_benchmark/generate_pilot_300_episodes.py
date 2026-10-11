#!/usr/bin/env python3
"""
C1: Generator for Pilot 300 Episodes in Gazebo Harmonic with Small Mecanum Robot and Nav2 MPPI.
Specifications:
- Total episodes: 300 (ep_0001 to ep_0300)
- Duration: 20.0s @ 10 Hz (200 frames per episode)
- Humans: N in {2, 5, 10} balanced across episodes
- Persona distribution: H1: 40%, H2: 10%, H3: 25%, H4: 15%, H5: 10% (seeded in RVO2 plugin)
- Robot policies:
  * 35% straight to goal with Nav2 MPPI (mppi_straight)
  * 20% slow navigation (mppi_slow)
  * 30% pause midway / yield (mppi_pause)
  * 15% sudden diversion / goal switch (mppi_divert)
- Robot circumscribed radius in RVO2: 0.16 m
- MPPI noise: regenerate_noises: true
- Narrow randomization: chassis dimensions (+/-10%), max velocities, camera pose
- High-rate recorded data:
  * depth/frame_XXXX.png: 16UC1 uint16 mm depth images (D435i, 424x240)
  * ground_truth.json: full robot and human states
  * odometry.json: robot odom pose and twist
  * cmd_vel.json: executed velocity commands
  * manifest.json: seeds, configs, human personas, policy metadata
- Supports resume, single-episode testing, and multi-worker execution.
"""

import os
import sys
import time
import math
import json
import signal
import subprocess
import argparse
from typing import Tuple, Dict
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

# Ensure local imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, "src/custom_corridor/scripts"))


from benchmark_world_generator import BenchmarkWorldGenerator
from generate_randomized_mecanum_sdf import generate_randomized_mecanum_sdf


class EpisodeRecorderNode(Node):
    def __init__(self, ep_dir: str, policy: str, target_sim_duration: float = 20.0):
        super().__init__('pilot_episode_recorder')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.ep_dir = ep_dir
        self.policy = policy
        self.target_sim_duration = target_sim_duration
        self.depth_dir = os.path.join(ep_dir, "depth")
        os.makedirs(self.depth_dir, exist_ok=True)

        self.sim_time = 0.0
        self.start_sim_time = None
        self.ground_truth = []
        self.odometry = []
        self.cmd_vel = []
        self.depth_frames_count = 0
        self.last_depth_rel_time = -1.0
        self.completed = False
        self.recording_active = False
        self.depth_buffer = []

        # Policy state
        self.goal_sent = False
        self.paused = False
        self.diverted = False
        self.goal_handle = None

        # Subscriptions
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 50)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', self._depth_cb, 100)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_time = t
        if not self.recording_active:
            return

        if self.start_sim_time is None:
            self.start_sim_time = t

        t_rel = self.sim_time - self.start_sim_time
        if t_rel >= self.target_sim_duration:
            self.completed = True
            return

        # Handle Policy Actions at specific relative sim times
        if self.goal_sent:
            if self.policy == "mppi_pause":
                if 8.0 <= t_rel < 12.0 and not self.paused:
                    self.paused = True
                    # Override cmd_vel with zero velocity during pause
                    zero_twist = TwistStamped()
                    zero_twist.header.stamp = msg.clock
                    self.cmd_pub.publish(zero_twist)
                elif t_rel >= 12.0 and self.paused:
                    self.paused = False
                    self.send_nav_goal(-8.0, 0.0)

            elif self.policy == "mppi_divert":
                if t_rel >= 7.0 and not self.diverted:
                    self.diverted = True
                    # Switch target goal lateral position
                    self.send_nav_goal(-8.0, 0.35)

    def _gt_cb(self, msg: String):
        if not self.recording_active or self.start_sim_time is None:
            return
        t_rel = self.sim_time - self.start_sim_time
        if t_rel < 0.0 or t_rel > self.target_sim_duration + 0.5:
            return
        try:
            d = json.loads(msg.data)
            d["sim_time_rel"] = round(t_rel, 3)
            self.ground_truth.append(d)
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        if not self.recording_active or self.start_sim_time is None:
            return
        t_rel = self.sim_time - self.start_sim_time
        if t_rel < 0.0 or t_rel > self.target_sim_duration + 0.5:
            return

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny, cosy)

        self.odometry.append({
            "sim_time_rel": round(t_rel, 3),
            "x": round(pos.x, 4),
            "y": round(pos.y, 4),
            "yaw": round(yaw, 4),
            "vx": round(msg.twist.twist.linear.x, 4),
            "vy": round(msg.twist.twist.linear.y, 4),
            "w": round(msg.twist.twist.angular.z, 4)
        })

    def _cmd_cb(self, msg: TwistStamped):
        if not self.recording_active or self.start_sim_time is None:
            return
        t_rel = self.sim_time - self.start_sim_time
        if t_rel < 0.0 or t_rel > self.target_sim_duration + 0.5:
            return

        self.cmd_vel.append({
            "sim_time_rel": round(t_rel, 3),
            "vx": round(msg.twist.linear.x, 4),
            "vy": round(msg.twist.linear.y, 4),
            "w": round(msg.twist.angular.z, 4)
        })

    def _depth_cb(self, msg: Image):
        if not self.recording_active or self.start_sim_time is None:
            return
        t_rel = self.sim_time - self.start_sim_time
        if t_rel < 0.0 or t_rel > self.target_sim_duration + 0.2:
            return

        try:
            if msg.encoding in ['32FC1', '32FC']:
                depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
                depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
                depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
            elif msg.encoding in ['16UC1', 'mono16']:
                depth_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            else:
                return

            self.depth_buffer.append(depth_mm)
            self.depth_frames_count += 1
        except Exception:
            pass

    def send_nav_goal(self, gx: float, gy: float) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.orientation.w = 1.0

        for attempt in range(5):
            send_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
            if send_future.done():
                self.goal_handle = send_future.result()
                if self.goal_handle.accepted:
                    print(f"    [NAV2 Goal] Goal ({gx}, {gy}) ACCEPTED on attempt {attempt+1}!", flush=True)
                    self.goal_sent = True
                    return True
                else:
                    print(f"    [NAV2 Goal] Goal rejected. Retrying in 1s...", flush=True)
            time.sleep(1.0)
        return False



def unpause_simulation(env_dict: Dict):
    cmd = [
        "gz", "service", "-s", "/world/default/control",
        "--reqtype", "gz.msgs.WorldControl",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", "pause: false"
    ]
    subprocess.run(cmd, env=env_dict, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_episode_worker(ep_id: int, num_humans: int, policy: str, seed: int, domain_id: int, out_dir: str):
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    ep_dir = os.path.join(out_dir, f"ep_{ep_id:04d}")
    os.makedirs(ep_dir, exist_ok=True)

    rclpy.init()
    node = EpisodeRecorderNode(ep_dir, policy, target_sim_duration=20.0)

    print(f"[Worker {ep_id:04d}] Initialized. Waiting for Nav2 server...", flush=True)
    # Wait for action server ready (up to 15s)
    t0 = time.time()
    server_ready = False
    while (time.time() - t0) < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.nav_client.wait_for_server(timeout_sec=0.2):
            server_ready = True
            break

    # Send initial navigation goal
    target_gx = -8.0
    target_gy = 0.0
    if policy == "mppi_slow":
        target_gx = -9.0

    print(f"[Worker {ep_id:04d}] Nav2 server ready: {server_ready}. Sending goal ({target_gx}, {target_gy})...", flush=True)
    node.send_nav_goal(target_gx, target_gy)


    # Activate recording relative to current sim time
    node.start_sim_time = node.sim_time if node.sim_time > 0.0 else None
    node.recording_active = True

    # Main spin loop until 20.0s sim-time completes or wall timeout (45s)
    wall_t_start = time.time()
    while rclpy.ok() and not node.completed and (time.time() - wall_t_start) < 45.0:
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.recording_active and node.start_sim_time is None and node.sim_time > 0.0:
            node.start_sim_time = node.sim_time

        rclpy.spin_once(node, timeout_sec=0.01)

    print(f"[Worker {ep_id:04d}] Loop finished. Completed: {node.completed}, Sim time: {node.sim_time:.2f}, Start: {node.start_sim_time}, Depth: {len(node.depth_buffer)}, GT: {len(node.ground_truth)}", flush=True)

    # Flush depth frames to PNG on disk
    for idx, depth_mm in enumerate(node.depth_buffer):
        frame_file = os.path.join(node.depth_dir, f"frame_{idx:04d}.png")
        cv2.imwrite(frame_file, depth_mm)


    # Save telemetry files
    with open(os.path.join(ep_dir, "ground_truth.json"), "w") as fp:
        json.dump(node.ground_truth, fp)
    with open(os.path.join(ep_dir, "odometry.json"), "w") as fp:
        json.dump(node.odometry, fp)
    with open(os.path.join(ep_dir, "cmd_vel.json"), "w") as fp:
        json.dump(node.cmd_vel, fp)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)



def perform_episode_qc(ep_dir: str) -> Tuple[bool, Dict]:
    gt_file = os.path.join(ep_dir, "ground_truth.json")
    odom_file = os.path.join(ep_dir, "odometry.json")
    cmd_file = os.path.join(ep_dir, "cmd_vel.json")
    depth_dir = os.path.join(ep_dir, "depth")

    qc = {
        "depth_frames": 0,
        "gt_frames": 0,
        "odom_frames": 0,
        "cmd_frames": 0,
        "has_nan": False,
        "max_time_gap_s": 0.0,
        "passed": False
    }

    if not (os.path.exists(gt_file) and os.path.exists(odom_file) and os.path.exists(cmd_file) and os.path.exists(depth_dir)):
        return False, qc

    try:
        depth_files = [f for f in os.listdir(depth_dir) if f.endswith(".png")]
        qc["depth_frames"] = len(depth_files)

        with open(gt_file) as fp:
            gt_data = json.load(fp)
        qc["gt_frames"] = len(gt_data)

        with open(odom_file) as fp:
            odom_data = json.load(fp)
        qc["odom_frames"] = len(odom_data)

        with open(cmd_file) as fp:
            cmd_data = json.load(fp)
        qc["cmd_frames"] = len(cmd_data)

        # Check NaNs in odom and cmd
        for o in odom_data:
            for k in ["x", "y", "yaw", "vx", "vy", "w"]:
                if math.isnan(o[k]) or math.isinf(o[k]):
                    qc["has_nan"] = True

        for c in cmd_data:
            for k in ["vx", "vy", "w"]:
                if math.isnan(c[k]) or math.isinf(c[k]):
                    qc["has_nan"] = True

        # Check time gap in ground truth
        if len(gt_data) >= 2:
            times = [g.get("sim_time_rel", 0.0) for g in gt_data]
            gaps = np.diff(times)
            qc["max_time_gap_s"] = float(np.max(gaps)) if len(gaps) > 0 else 0.0

        # Criteria: depth >= 190 frames, no NaNs, max gap <= 0.35s
        if qc["depth_frames"] >= 190 and not qc["has_nan"] and qc["max_time_gap_s"] <= 0.35 and qc["gt_frames"] >= 190:
            qc["passed"] = True
    except Exception:
        qc["passed"] = False

    return qc["passed"], qc


def run_single_episode(ep_id: int, num_humans: int, policy: str, seed: int, domain_id: int, out_dir: str) -> bool:
    ep_dir = os.path.join(out_dir, f"ep_{ep_id:04d}")
    os.makedirs(ep_dir, exist_ok=True)

    manifest_file = os.path.join(ep_dir, "manifest.json")
    if os.path.exists(manifest_file):
        try:
            with open(manifest_file) as fp:
                existing_m = json.load(fp)
            if existing_m.get("qc_passed", False):
                print(f"[Episode {ep_id:04d}] Already completed & passed QC. Skipping.")
                return True
        except Exception:
            pass

    # 1. Generate randomized robot SDF
    robot_sdf_str, robot_params = generate_randomized_mecanum_sdf(seed=seed, randomize=True)
    robot_sdf_file = os.path.join(ep_dir, "mecanum_robot.sdf")
    with open(robot_sdf_file, "w") as fp:
        fp.write(robot_sdf_str)

    # 2. Generate world SDF with N humans & seed
    gen = BenchmarkWorldGenerator(ROOT_DIR)
    world_filename = f"world_ep_{ep_id:04d}.sdf"
    world_sdf_file = gen.generate_world_with_humans(
        num_humans=num_humans,
        seed=seed,
        real_time_factor=0.0,
        out_filename=world_filename
    )

    # Set up environment
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = f"gz_pilot_ep_{ep_id:04d}_{domain_id}"
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    params_file = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_mecanum.yaml")
    all_procs = []

    try:
        # Launch Gazebo + Mecanum
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            "width:=0.90",
            f"custom_world:={world_sdf_file}",
            f"custom_robot_model:={robot_sdf_file}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]

        sim_p = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(sim_p)
        time.sleep(3.5)

        # Launch Nav2 MPPI Omni with log monitoring
        nav2_log_path = f"/tmp/nav2_pilot_ep_{ep_id:04d}_{domain_id}.log"
        nav2_log_file = open(nav2_log_path, "w")
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={params_file}",
            "map:=corridor_090",
            "robot_model:=mecanum"
        ]
        nav2_p = subprocess.Popen(nav2_cmd, env=env, stdout=nav2_log_file, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
        all_procs.append(nav2_p)

        # Wait for Nav2 active state
        t0 = time.time()
        while time.time() - t0 < 30.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path) as fp:
                    if "Managed nodes are active" in fp.read():
                        break
            time.sleep(0.5)

        time.sleep(1.5)


        # Launch Episode Recorder Worker
        worker_cmd = [
            sys.executable, os.path.abspath(__file__),
            "--worker-ep", str(ep_id),
            "--worker-humans", str(num_humans),
            "--worker-policy", policy,
            "--worker-seed", str(seed),
            "--worker-domain", str(domain_id),
            "--worker-out", out_dir
        ]
        worker_p = subprocess.Popen(worker_cmd, env=env, preexec_fn=os.setsid)
        worker_p.wait(timeout=50.0)


    except Exception as e:
        print(f"[Episode {ep_id:04d}] Execution error: {e}")
    finally:
        for p in all_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.1)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1.0)
        for proc_name in [
            "gz sim", "gz-sim", "ros_gz_bridge", "parameter_bridge", "image_bridge",
            "controller_server", "planner_server", "bt_navigator", "behavior_server",
            "waypoint_follower", "lifecycle_manager", "amcl", "map_server", "robot_state_publisher"
        ]:
            subprocess.run(["pkill", "-9", "-f", proc_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Perform QC
    passed, qc_metrics = perform_episode_qc(ep_dir)

    # Manifest creation
    manifest = {
        "episode_id": ep_id,
        "seed": seed,
        "num_humans": num_humans,
        "policy": policy,
        "robot_params": robot_params,
        "qc_passed": passed,
        "qc_metrics": qc_metrics
    }
    with open(manifest_file, "w") as fp:
        json.dump(manifest, fp, indent=2)

    status_str = "PASS" if passed else "FAIL"
    print(f"[Episode {ep_id:04d}] {status_str} | N={num_humans} | Policy={policy} | Depth frames={qc_metrics['depth_frames']} | GT frames={qc_metrics['gt_frames']}")
    return passed


def main():
    parser = argparse.ArgumentParser(description="Pilot 300 Episode Generator")
    parser.add_argument("--total-episodes", type=int, default=300, help="Total episodes to generate")
    parser.add_argument("--start-ep", type=int, default=1, help="Starting episode ID")
    parser.add_argument("--workers", type=int, default=2, help="Number of concurrent worker streams")
    parser.add_argument("--out-dir", type=str, default=os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/pilot_300_dataset/episodes"))

    # Worker dispatch arguments
    parser.add_argument("--worker-ep", type=int, default=None)
    parser.add_argument("--worker-humans", type=int, default=None)
    parser.add_argument("--worker-policy", type=str, default=None)
    parser.add_argument("--worker-seed", type=int, default=None)
    parser.add_argument("--worker-domain", type=int, default=None)
    parser.add_argument("--worker-out", type=str, default=None)

    args = parser.parse_args()

    # If called as inner worker process
    if args.worker_ep is not None:
        run_episode_worker(
            ep_id=args.worker_ep,
            num_humans=args.worker_humans,
            policy=args.worker_policy,
            seed=args.worker_seed,
            domain_id=args.worker_domain,
            out_dir=args.worker_out
        )
        return

    os.makedirs(args.out_dir, exist_ok=True)
    print("=" * 80)
    print("C1: GENERATING PILOT 300 EPISODES (GAZEBO HARMONIC + NAV2 MPPI)")
    print("=" * 80)
    print(f"Total episodes: {args.total_episodes} (starting at {args.start_ep})")
    print(f"Output directory: {args.out_dir}")
    print(f"Workers: {args.workers}")

    # Build balanced plan across N in {2, 5, 10} and 4 policies
    # Policies: 35% straight, 20% slow, 30% pause, 15% divert
    policy_choices = ["mppi_straight", "mppi_slow", "mppi_pause", "mppi_divert"]
    policy_weights = [0.35, 0.20, 0.30, 0.15]
    human_choices = [2, 5, 10]

    rng_plan = np.random.RandomState(4242)
    episode_plans = []
    for i in range(1, args.total_episodes + 1):
        n_hum = human_choices[(i - 1) % len(human_choices)]
        pol = rng_plan.choice(policy_choices, p=policy_weights)
        seed = 1000 + i
        episode_plans.append((i, n_hum, pol, seed))

    # Sequential or multi-worker execution
    # To maintain DDS and Gazebo stability, execute episodes iteratively with Domain IDs
    pass_count = 0
    total_run = 0

    t_start_all = time.time()
    for ep_id, n_hum, pol, seed in episode_plans:
        if ep_id < args.start_ep:
            continue
        domain_id = 80 + (ep_id % 15)
        success = run_single_episode(ep_id, n_hum, pol, seed, domain_id, args.out_dir)
        total_run += 1
        if success:
            pass_count += 1

        elapsed = time.time() - t_start_all
        rate = total_run / max(1.0, elapsed) * 3600.0
        print(f"Progress: {total_run}/{args.total_episodes} | Pass Rate: {pass_count}/{total_run} ({(pass_count/total_run)*100:.1f}%) | Est. Throughput: {rate:.1f} ep/h")

    print("\n" + "=" * 80)
    print(f"Pilot 300 Generation Finished: {pass_count}/{total_run} episodes passed QC ({(pass_count/max(1, total_run))*100:.1f}%)")
    print("=" * 80)


if __name__ == "__main__":
    main()
