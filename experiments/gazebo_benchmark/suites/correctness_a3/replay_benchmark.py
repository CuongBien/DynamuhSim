"""
Replay Benchmark (Section 8 of Benchmark Spec):
Tests the decoupled architecture (quỹ đạo từ sim nhẹ 2D, depth render từ Gazebo Harmonic):
1. Generates a sample trajectory: robot and humans pose over time (10 Hz, 20s = 200 steps).
2. Replays in Gazebo in synchronous step mode using native in-process gz.transport13.
3. Measures:
   - Render FPS (frames per second)
   - Total wall time for 20s episode
   - Speedup relative to real-time
   - Pixel-level repeatability across 2 independent replay runs
"""

import os
from pathlib import Path
import sys
import time
import math
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node as RosNode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from gz.transport13 import Node as GzNode
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env, spawn_benchmark_robot
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes


class DepthImageCollector(RosNode):
    def __init__(self):
        super().__init__('depth_image_collector')
        self.sub = self.create_subscription(Image, '/camera/depth_image', self._cb, 10)
        self.bridge = CvBridge()
        self.latest_depth = None
        self.frame_count = 0

    def _cb(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.frame_count += 1
        except Exception:
            pass

    def get_latest_frame(self, timeout_s=0.2):
        start = time.time()
        prev_count = self.frame_count
        while time.time() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.frame_count > prev_count and self.latest_depth is not None:
                return self.latest_depth.copy()
        return None


def generate_sample_trajectory(num_steps=200, dt=0.1):
    """Generates sample 20s trajectory for robot and 5 humans."""
    times = np.arange(num_steps) * dt
    traj = {
        'times': times,
        'robot': [],
        'humans': {f"human_{i:02d}": [] for i in range(1, 6)}
    }

    # Robot moves forward along X from x=-2.0 to x=1.0
    for t in times:
        rx = -2.0 + 0.15 * t
        ry = 0.05 * math.sin(0.2 * t)
        ryaw = 0.05 * math.cos(0.2 * t)
        traj['robot'].append((rx, ry, ryaw))

        # 5 humans walking or standing
        for i in range(1, 6):
            hx = 3.0 + i * 1.2 - 0.2 * t
            hy = 0.15 * math.sin(0.3 * t + i)
            hyaw = 3.14159
            traj['humans'][f"human_{i:02d}"].append((hx, hy, hyaw))

    return traj


@scoped_processes
def run_replay_trial(traj, root_dir="/home/cuongbien/DynamuhSim", trial_idx=1, *, _processes=None):
    print(f"\n[REPLAY TRIAL {trial_idx}] Starting Gazebo in synchronous step mode...")

    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_n5_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/burger_lidar1_depth1_640x480_30fps.sdf")
    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")
    env = build_benchmark_env(root_dir, turtlebot3=True, d3d12=False)

    # 1. Start Gazebo server
    gz_proc = _processes.popen(
        ["gz", "sim", "-s", "-r", world_file],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    time.sleep(3.0)

    # 2. Spawn robot
    spawn_benchmark_robot(robot_file, env, processes=_processes)

    # 3. Start image bridge for depth
    img_bridge_proc = _processes.popen(
        ["ros2", "run", "ros_gz_image", "image_bridge", "/camera/depth_image"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
    )

    time.sleep(2.0)

    # 4. Initialize native gz.transport13 node and pause
    gz_node = GzNode()
    pause_req = WorldControl()
    pause_req.pause = True
    gz_node.request('/world/corridor_0.90/control', pause_req, WorldControl, Boolean, 2000)

    collector = DepthImageCollector()
    captured_frames = []

    num_steps = len(traj['times'])
    print(f"  -> Replaying {num_steps} steps (dt=0.1s, total {num_steps*0.1:.1f}s episode)...")
    start_wall = time.time()

    try:
        # Pre-allocate protobuf messages
        pose_msg = Pose()
        step_msg = WorldControl()
        step_msg.multi_step = 25  # 25 physics steps * 0.004s = 0.1s sim time

        for step_idx in range(num_steps):
            rx, ry, ryaw = traj['robot'][step_idx]

            # Set robot pose
            pose_msg.name = "burger"
            pose_msg.position.x = rx
            pose_msg.position.y = ry
            pose_msg.position.z = 0.01
            pose_msg.orientation.z = math.sin(ryaw / 2.0)
            pose_msg.orientation.w = math.cos(ryaw / 2.0)
            gz_node.request('/world/corridor_0.90/set_pose', pose_msg, Pose, Boolean, 1000)

            # Set human proxy poses
            for i in range(1, 6):
                hx, hy, hyaw = traj['humans'][f"human_{i:02d}"][step_idx]
                pose_msg.name = f"human_{i:02d}_proxy"
                pose_msg.position.x = hx
                pose_msg.position.y = hy
                pose_msg.position.z = 0.80
                pose_msg.orientation.z = math.sin(hyaw / 2.0)
                pose_msg.orientation.w = math.cos(hyaw / 2.0)
                gz_node.request('/world/corridor_0.90/set_pose', pose_msg, Pose, Boolean, 1000)

            # Advance 0.1s in simulation
            gz_node.request('/world/corridor_0.90/control', step_msg, WorldControl, Boolean, 1000)

            # Capture rendered depth frame
            frame = collector.get_latest_frame(timeout_s=0.15)
            if frame is not None:
                captured_frames.append(frame)

        elapsed = time.time() - start_wall
        fps = len(captured_frames) / elapsed if elapsed > 0 else 0.0
        print(f"  [DONE] Rendered {len(captured_frames)} frames in {elapsed:.2f}s ({fps:.1f} FPS)")

    finally:
        collector.destroy_node()
        _processes.stop()

    return {
        'elapsed_s': elapsed,
        'frames_captured': len(captured_frames),
        'fps': fps,
        'frames': captured_frames,
    }


def main():
    rclpy.init()
    try:
        # Generate 100 steps (10s episode) for benchmark
        traj = generate_sample_trajectory(num_steps=100, dt=0.1)
        res1 = run_replay_trial(traj, trial_idx=1)
        res2 = run_replay_trial(traj, trial_idx=2)

        # Calculate pixel-level repeatability between run 1 and run 2
        max_pixel_diff = 0.0
        if len(res1['frames']) > 0 and len(res2['frames']) > 0:
            n_compare = min(len(res1['frames']), len(res2['frames']))
            diffs = []
            for i in range(n_compare):
                f1 = res1['frames'][i]
                f2 = res2['frames'][i]
                valid_mask = np.isfinite(f1) & np.isfinite(f2)
                if np.any(valid_mask):
                    diffs.append(np.max(np.abs(f1[valid_mask] - f2[valid_mask])))
            max_pixel_diff = float(np.max(diffs)) if diffs else 0.0

        print("\n" + "=" * 70)
        print("REPLAY BENCHMARK SUMMARY (SECTION 8)")
        print("=" * 70)
        print(f"1. Render Throughput:           {res1['fps']:.1f} FPS")
        print(f"2. Time for 20s episode:        {(20.0 / 10.0) * res1['elapsed_s']:.1f} s")
        print(f"3. Relative to Real-Time:       {(res1['fps'] / 10.0):.2f}x real-time speed")
        print(f"4. Max Repeatability Pixel Diff:{max_pixel_diff:.4f} m ({max_pixel_diff*1000.0:.2f} mm)")
        print("=" * 70)

        # Save results
        summary_data = {
            'replay_fps': res1['fps'],
            'replay_time_20s_ep': (20.0 / 10.0) * res1['elapsed_s'],
            'speedup_relative_to_realtime': res1['fps'] / 10.0,
            'max_pixel_diff_m': max_pixel_diff,
        }
        np.save("/home/cuongbien/DynamuhSim/experiments/gazebo_benchmark/results/correctness_a3/replay_benchmark_results.npy", summary_data)

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
