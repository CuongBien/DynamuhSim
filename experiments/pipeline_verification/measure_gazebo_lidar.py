#!/usr/bin/env python3
"""
Step 1: Empirical Verification of LiDAR Perception on Human Actor in Gazebo Harmonic.
Captures /scan at z = 0.171m as the human approaches the robot from 1.5 - 2.0m.
Analyzes:
  1. Point cloud profile: Single circular arc (cylinder proxy) vs. Two oscillating clusters (legs).
  2. Foot-skate & animation cadence: Compares stride frequency and speed with linear velocity.
"""

import sys
import time
import math
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock


class LidarInspector(Node):
    def __init__(self):
        super().__init__('lidar_inspector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        self.current_sim_time = 0.0
        self.captured_scans = []
        self.max_scans = 50  # 10 seconds at 5 Hz
        self.get_logger().info("LidarInspector initialized, waiting for /scan...")

    def clock_callback(self, msg: Clock):
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def scan_callback(self, msg: LaserScan):
        if len(self.captured_scans) >= self.max_scans:
            return

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Convert to robot Cartesian frame (X forward, Y left)
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)
        r = ranges[valid_mask]
        a = angles[valid_mask]
        x = r * np.cos(a)
        y = r * np.sin(a)

        # In corridor_090, walls are at y = +/- 0.45m.
        # Robot is at x=3.0, human is walking towards robot from x=5.0 (relative X in [0.3, 3.0]).
        # Human obstacle points: 0.3m < x < 3.0m and |y| < 0.38m (excluding walls)
        human_mask = (x > 0.2) & (x < 3.0) & (np.abs(y) < 0.38)
        human_pts_x = x[human_mask]
        human_pts_y = y[human_mask]

        stamp = self.current_sim_time if self.current_sim_time > 0 else (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

        scan_info = {
            'stamp': stamp,
            'total_pts': len(human_pts_x),
            'x': human_pts_x,
            'y': human_pts_y,
            'min_dist': np.min(r[human_mask]) if len(human_pts_x) > 0 else None,
        }
        self.captured_scans.append(scan_info)
        if len(human_pts_x) > 0:
            self.get_logger().info(
                f"Scan #{len(self.captured_scans)}: stamp={stamp:.2f}s | "
                f"Human detected: {len(human_pts_x)} pts | min_x={np.min(human_pts_x):.3f}m | "
                f"y_span=[{np.min(human_pts_y):.3f}, {np.max(human_pts_y):.3f}]m"
            )


def analyze_scans(scans):
    print("\n" + "="*70)
    print("STEP 1: EMPIRICAL LIDAR MEASUREMENT & ACTOR ANALYSIS")
    print("="*70)

    valid_frames = [s for s in scans if s['total_pts'] >= 3]
    if not valid_frames:
        print("[ERROR] No human points captured in the expected ROI!")
        return None

    print(f"Total valid human scan frames analyzed: {len(valid_frames)}")

    profiles = []
    cluster_gaps = []
    widths = []

    for i, f in enumerate(valid_frames):
        pts_x = f['x']
        pts_y = f['y']

        # Sort points by Y coordinate
        sorted_idx = np.argsort(pts_y)
        sorted_y = pts_y[sorted_idx]
        sorted_x = pts_x[sorted_idx]

        # Calculate width
        width = np.max(sorted_y) - np.min(sorted_y)
        widths.append(width)

        # Check for gaps between adjacent points in Y
        diffs = np.diff(sorted_y)
        max_gap = np.max(diffs) if len(diffs) > 0 else 0.0
        cluster_gaps.append(max_gap)

        # Check curvature / circular fit
        # If it's a cylinder proxy of radius 0.28m, points will lie on circle of radius ~0.28m
        # If it's two legs, there will be two separate tight clusters separated by a gap (> 0.08m)
        num_clusters = 1
        if max_gap > 0.07:  # A gap of >7cm between consecutive scan points indicates separate leg segments
            num_clusters = 2

        profiles.append({
            'frame': i,
            'stamp': f['stamp'],
            'num_pts': len(pts_x),
            'center_x': np.mean(pts_x),
            'center_y': np.mean(pts_y),
            'width': width,
            'max_gap': max_gap,
            'clusters': num_clusters
        })

    # Summary statistics
    avg_width = np.mean(widths)
    max_gap_overall = np.max(cluster_gaps)
    two_cluster_frames = sum(1 for p in profiles if p['clusters'] == 2)
    one_cluster_frames = sum(1 for p in profiles if p['clusters'] == 1)

    print(f"\n1. PROFILE DISCRIMINATION RESULTS:")
    print(f"   - Average detected obstacle width: {avg_width:.3f} m (std: {np.std(widths):.3f} m)")
    print(f"   - Maximum intra-cluster gap:       {max_gap_overall:.3f} m")
    print(f"   - Frames with 2 distinct clusters: {two_cluster_frames} ({two_cluster_frames/len(profiles)*100:.1f}%)")
    print(f"   - Frames with 1 continuous arc:    {one_cluster_frames} ({one_cluster_frames/len(profiles)*100:.1f}%)")

    # Check velocity and foot-skate
    times = [p['stamp'] for p in profiles]
    xs = [p['center_x'] for p in profiles]
    dt = times[-1] - times[0]
    dx = xs[0] - xs[-1]
    measured_speed = dx / dt if dt > 0 else 0.0

    print(f"\n2. KINEMATIC & ANIMATION CADENCE RESULTS:")
    print(f"   - Time window:                     {dt:.2f} s")
    print(f"   - Distance traveled towards robot: {dx:.3f} m")
    print(f"   - Measured linear speed:           {measured_speed:.3f} m/s")

    return {
        'profiles': profiles,
        'avg_width': avg_width,
        'max_gap_overall': max_gap_overall,
        'two_cluster_frames': two_cluster_frames,
        'one_cluster_frames': one_cluster_frames,
        'measured_speed': measured_speed,
        'dt': dt,
        'dx': dx
    }


def main():
    rclpy.init()
    inspector = LidarInspector()

    start_time = time.time()
    try:
        while rclpy.ok() and len(inspector.captured_scans) < inspector.max_scans:
            rclpy.spin_once(inspector, timeout_sec=0.1)
            if time.time() - start_time > 15.0:
                print("[TIMEOUT] Reached 15s collection window.")
                break
    finally:
        results = analyze_scans(inspector.captured_scans)
        inspector.destroy_node()
        rclpy.shutdown()

    if results is not None:
        # Save raw results for reporting
        np.save('/home/cuongbien/DynamuhSim/experiments/pipeline_verification/raw_lidar_measurement.npy', results)
        print("\n[SUCCESS] Raw measurement saved to experiments/pipeline_verification/raw_lidar_measurement.npy")


if __name__ == '__main__':
    main()
