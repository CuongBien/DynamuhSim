#!/usr/bin/env python3
"""
Measurement script for depth camera noise on a flat wall at 2.0m distance.
Captures 20 consecutive frames and evaluates statistics on a 20x20 center patch.
"""

import os
import sys
import time
import signal
import subprocess
import json
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


class DepthPatchCaptureNode(Node):
    def __init__(self, target_frames=20, patch_size=20, warmup_frames=5):
        super().__init__('depth_patch_capture_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_frames = target_frames
        self.patch_size = patch_size
        self.warmup_frames = warmup_frames

        self.frames_received = 0
        self.captured_patches = []
        self.captured_full_stats = []
        self.finished = False

        self.sub_depth = self.create_subscription(
            Image,
            '/camera/depth_image',
            self._depth_cb,
            10
        )
        self.get_logger().info("Subscribed to /camera/depth_image, awaiting frames...")

    def _depth_cb(self, msg: Image):
        if self.finished:
            return

        self.frames_received += 1
        if self.frames_received <= self.warmup_frames:
            self.get_logger().info(f"Warming up: frame {self.frames_received}/{self.warmup_frames}")
            return

        # Decode float32 depth image (meters)
        h, w = msg.height, msg.width
        raw_arr = np.frombuffer(msg.data, dtype=np.float32)
        if raw_arr.size != h * w:
            # Maybe 16UC1 (uint16 mm)?
            raw_u16 = np.frombuffer(msg.data, dtype=np.uint16)
            if raw_u16.size == h * w:
                depth_img = raw_u16.reshape((h, w)).astype(np.float32) / 1000.0
            else:
                self.get_logger().error(f"Unexpected image buffer size: {len(msg.data)} bytes for {w}x{h}")
                return
        else:
            depth_img = raw_arr.reshape((h, w))

        # Check full frame validity
        n_nan = int(np.isnan(depth_img).sum())
        n_inf = int(np.isinf(depth_img).sum())
        n_zero = int((depth_img == 0.0).sum())
        n_total = h * w
        n_valid = n_total - n_nan - n_inf - n_zero

        # Center patch (e.g., 20x20 in center)
        cy, cx = h // 2, w // 2
        half_p = self.patch_size // 2
        y1, y2 = cy - half_p, cy + half_p
        x1, x2 = cx - half_p, cx + half_p
        patch = depth_img[y1:y2, x1:x2].copy()

        patch_nan = int(np.isnan(patch).sum())
        patch_inf = int(np.isinf(patch).sum())
        patch_zero = int((patch == 0.0).sum())

        self.captured_patches.append(patch)
        self.captured_full_stats.append({
            "frame_idx": len(self.captured_patches),
            "full_shape": [h, w],
            "full_nan_count": n_nan,
            "full_inf_count": n_inf,
            "full_zero_count": n_zero,
            "full_valid_count": n_valid,
            "patch_bounds": [int(y1), int(y2), int(x1), int(x2)],
            "patch_nan_count": patch_nan,
            "patch_inf_count": patch_inf,
            "patch_zero_count": patch_zero,
            "patch_mean_m": float(np.mean(patch)),
            "patch_std_m": float(np.std(patch)),
            "patch_min_m": float(np.min(patch)),
            "patch_max_m": float(np.max(patch)),
        })

        self.get_logger().info(
            f"Captured frame {len(self.captured_patches)}/{self.target_frames}: "
            f"center patch mean={np.mean(patch):.4f}m, std={np.std(patch)*1000.0:.3f}mm"
        )

        if len(self.captured_patches) >= self.target_frames:
            self.finished = True


def run_measurement(model_sdf_path: str, label: str = "measurement") -> dict:
    root_dir = "/home/cuongbien/DynamuhSim"
    world_sdf = os.path.join(root_dir, "experiments/gazebo_benchmark/depth_test/flat_wall_test_world.sdf")

    corridor_models = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    domain_id = "88"
    env["ROS_DOMAIN_ID"] = domain_id
    partition = f"depth_test_{label}_{int(time.time())}"
    env["GZ_PARTITION"] = partition

    all_procs = []
    try:
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            "width:=0.90",
            f"custom_world:={world_sdf}",
            f"custom_robot_model:={model_sdf_path}",
            "obstacle:=none",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=0.0", "y:=0.0", "yaw:=0.0"
        ]
        print(f"\n[{label.upper()}] Starting Gazebo simulation with custom_robot_model: {model_sdf_path}...")
        gz_p = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(gz_p)

        # Wait for bridge and sensor to initialize
        time.sleep(5.0)

        os.environ["ROS_DOMAIN_ID"] = domain_id
        os.environ["GZ_PARTITION"] = partition
        rclpy.init()
        capture_node = DepthPatchCaptureNode(target_frames=20, patch_size=20, warmup_frames=5)

        t0 = time.time()
        while rclpy.ok() and not capture_node.finished and (time.time() - t0 < 25.0):
            rclpy.spin_once(capture_node, timeout_sec=0.1)

        if len(capture_node.captured_patches) < 20:
            print(f"[{label.upper()}] ERROR: Only received {len(capture_node.captured_patches)}/20 frames in 25s!")
            capture_node.destroy_node()
            rclpy.shutdown()
            return None

        patches = np.array(capture_node.captured_patches)  # Shape: (20, 20, 20)
        capture_node.destroy_node()
        rclpy.shutdown()

        # Compute comprehensive statistics
        # 1. Total pooled std dev (across all 20 frames x 20x20 pixels)
        pooled_std_m = float(np.std(patches))
        pooled_std_mm = pooled_std_m * 1000.0

        # 2. Spatial std dev: within each frame's 20x20 patch, then average over 20 frames
        spatial_std_per_frame_m = np.std(patches, axis=(1, 2))
        avg_spatial_std_m = float(np.mean(spatial_std_per_frame_m))
        avg_spatial_std_mm = avg_spatial_std_m * 1000.0

        # 3. Temporal std dev: for each pixel across the 20 frames, then average over the 400 pixels
        temporal_std_per_pixel_m = np.std(patches, axis=0)
        avg_temporal_std_m = float(np.mean(temporal_std_per_pixel_m))
        avg_temporal_std_mm = avg_temporal_std_m * 1000.0

        # 4. Global values
        mean_depth_m = float(np.mean(patches))
        min_depth_m = float(np.min(patches))
        max_depth_m = float(np.max(patches))

        # 5. Invalid values check (holes, 0, inf, nan)
        total_pixels = patches.size
        nan_count = int(np.isnan(patches).sum())
        inf_count = int(np.isinf(patches).sum())
        zero_count = int((patches == 0.0).sum())
        has_holes = bool(nan_count > 0 or inf_count > 0 or zero_count > 0)

        # Full frame stats check across all 20 frames
        full_nans = sum(s['full_nan_count'] for s in capture_node.captured_full_stats)
        full_infs = sum(s['full_inf_count'] for s in capture_node.captured_full_stats)
        full_zeros = sum(s['full_zero_count'] for s in capture_node.captured_full_stats)

        results = {
            "label": label,
            "model_sdf": model_sdf_path,
            "frames_analyzed": len(patches),
            "patch_dimensions": [20, 20],
            "total_patch_samples": total_pixels,
            "mean_depth_m": round(mean_depth_m, 6),
            "min_depth_m": round(min_depth_m, 6),
            "max_depth_m": round(max_depth_m, 6),
            "pooled_std_dev_mm": round(pooled_std_mm, 4),
            "spatial_std_dev_mm": round(avg_spatial_std_mm, 4),
            "temporal_std_dev_mm": round(avg_temporal_std_mm, 4),
            "has_holes_in_patch": has_holes,
            "patch_invalid_counts": {
                "nan": nan_count,
                "inf": inf_count,
                "zero": zero_count
            },
            "full_frame_invalid_counts_sum_20frames": {
                "nan": full_nans,
                "inf": full_infs,
                "zero": full_zeros
            },
            "frame_by_frame_stats": capture_node.captured_full_stats
        }

        print(f"\n--- [{label.upper()}] SUMMARY RESULTS ---")
        print(f"Mean Depth:         {mean_depth_m:.4f} m (Range: [{min_depth_m:.4f}, {max_depth_m:.4f}] m)")
        print(f"Pooled Std Dev:     {pooled_std_mm:.4f} mm ({pooled_std_m:.6f} m)")
        print(f"Spatial Std Dev:    {avg_spatial_std_mm:.4f} mm")
        print(f"Temporal Std Dev:   {avg_temporal_std_mm:.4f} mm")
        print(f"Holes / 0 / inf:    {'NONE (Clean image)' if not has_holes else f'DETECTED (nan={nan_count}, inf={inf_count}, zero={zero_count})'}")
        print("------------------------------------------\n")
        return results

    finally:
        for p in all_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.3)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1.5)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-sdf', required=True, help='Path to robot model.sdf')
    parser.add_argument('--label', default='measurement', help='Label for measurement')
    parser.add_argument('--out-json', default='/tmp/depth_meas.json', help='Output JSON path')
    args = parser.parse_args()

    res = run_measurement(args.model_sdf, args.label)
    if res:
        with open(args.out_json, 'w') as f:
            json.dump(res, f, indent=2)
        print(f"Saved results to {args.out_json}")
    else:
        sys.exit(1)
