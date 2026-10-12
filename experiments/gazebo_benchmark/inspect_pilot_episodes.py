#!/usr/bin/env python3
"""
Inspect Pilot 300 Episodes & Depth Noise Verification:
1. Automated QC across all 300 episodes.
2. In-depth inspection of 10 randomly selected episodes.
3. Clean depth verification: extracts 20x20 patch on flat wall in real episode, computes std dev (< 1 mm).
4. Outputs raw JSON: experiments/gazebo_benchmark/pilot_qc_inspection_results.json
"""

import os
import sys
import json
import math
import cv2
import numpy as np
from typing import Dict, List, Any

ROOT_DIR = "/home/cuongbien/DynamuhSim"
EPISODES_DIR = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/pilot_300_dataset/episodes")
OUTPUT_JSON = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/pilot_qc_inspection_results.json")


def inspect_all_episodes(episodes_dir: str = EPISODES_DIR) -> Dict[str, Any]:
    ep_dirs = sorted([d for d in os.listdir(episodes_dir) if d.startswith("ep_")])
    total_episodes = len(ep_dirs)

    qc_summary = {
        "total_episodes": total_episodes,
        "qc_passed_count": 0,
        "has_nan_count": 0,
        "max_time_gap_s": 0.0,
        "invariant_violations": 0,
        "episodes_with_corrupt_files": 0
    }

    all_max_gaps = []

    for ep_name in ep_dirs:
        ep_path = os.path.join(episodes_dir, ep_name)
        m_file = os.path.join(ep_path, "manifest.json")
        gt_file = os.path.join(ep_path, "ground_truth.json")
        odom_file = os.path.join(ep_path, "odometry.json")
        cmd_file = os.path.join(ep_path, "cmd_vel.json")
        depth_dir = os.path.join(ep_path, "depth")

        if not (os.path.exists(m_file) and os.path.exists(gt_file) and os.path.exists(odom_file) and os.path.exists(cmd_file) and os.path.exists(depth_dir)):
            qc_summary["episodes_with_corrupt_files"] += 1
            continue

        with open(m_file) as fp:
            m = json.load(fp)
        with open(gt_file) as fp:
            gt = json.load(fp)
        with open(odom_file) as fp:
            odom = json.load(fp)
        with open(cmd_file) as fp:
            cmd = json.load(fp)

        if m.get("qc_passed", False):
            qc_summary["qc_passed_count"] += 1
        qc_summary["invariant_violations"] += m.get("invariant_violations", 0)

        # Check NaNs
        has_nan = False
        for o in odom:
            for k in ["x", "y", "yaw", "vx", "vy", "w"]:
                v = o.get(k, 0.0)
                if math.isnan(v) or math.isinf(v):
                    has_nan = True
        for c in cmd:
            for k in ["vx", "vy", "w"]:
                v = c.get(k, 0.0)
                if math.isnan(v) or math.isinf(v):
                    has_nan = True
        if has_nan:
            qc_summary["has_nan_count"] += 1

        # Check time gap in ground truth
        if len(gt) >= 2:
            times = [g.get("sim_time_rel", 0.0) for g in gt]
            gaps = np.diff(times)
            max_gap = float(np.max(gaps)) if len(gaps) > 0 else 0.0
            all_max_gaps.append(max_gap)

    qc_summary["max_time_gap_s"] = float(np.max(all_max_gaps)) if all_max_gaps else 0.0
    qc_summary["qc_pass_rate_pct"] = round(qc_summary["qc_passed_count"] / max(1, total_episodes) * 100.0, 1)

    # 2. Detailed Inspection of 10 randomly selected episodes
    rng = np.random.RandomState(999)
    sample_indices = rng.choice(len(ep_dirs), size=min(10, len(ep_dirs)), replace=False)
    inspected_episodes = []

    for idx in sorted(sample_indices):
        ep_name = ep_dirs[idx]
        ep_path = os.path.join(episodes_dir, ep_name)
        with open(os.path.join(ep_path, "manifest.json")) as fp:
            m = json.load(fp)
        with open(os.path.join(ep_path, "ground_truth.json")) as fp:
            gt = json.load(fp)

        depth_dir = os.path.join(ep_path, "depth")
        depth_files = sorted([f for f in os.listdir(depth_dir) if f.endswith(".png")])

        # Validate first depth image properties
        first_img = cv2.imread(os.path.join(depth_dir, depth_files[0]), cv2.IMREAD_UNCHANGED) if depth_files else None
        dtype_str = str(first_img.dtype) if first_img is not None else "None"
        shape_str = list(first_img.shape) if first_img is not None else []

        inspected_episodes.append({
            "episode_name": ep_name,
            "persona": m.get("persona"),
            "corridor": m.get("corridor"),
            "num_humans": m.get("num_humans"),
            "use_nav2": m.get("use_nav2"),
            "is_interactive_target": m.get("is_interactive_target"),
            "collision_occurred": m.get("collision_occurred"),
            "goal_reached": m.get("goal_reached"),
            "gt_steps": len(gt),
            "depth_frames_count": len(depth_files),
            "depth_dtype": dtype_str,
            "depth_shape": shape_str,
            "delta_t_cmd0_s": m.get("delta_t_cmd0_s"),
            "status": "PASS" if m.get("qc_passed") else "FAIL"
        })

    # 3. Clean Depth Verification on Flat Wall Patch in Real Episode
    # Pick first episode with depth frames
    wall_patch_results = {"verified": False}
    for ep_name in ep_dirs:
        depth_dir = os.path.join(episodes_dir, ep_name, "depth")
        depth_files = sorted([f for f in os.listdir(depth_dir) if f.endswith(".png")])
        if len(depth_files) > 10:
            frame_path = os.path.join(depth_dir, depth_files[10])
            img_u16 = cv2.imread(frame_path, cv2.IMREAD_UNCHANGED)
            if img_u16 is not None:
                # In corridor, side pixels correspond to flat wall
                # Pick 20x20 patch in lower left region where corridor wall is seen
                # In corridor, side pixels correspond to flat wall
                # Pick 20x20 patch in lower left region where corridor wall is seen
                h, w = img_u16.shape
                patch = img_u16[h//2:h//2+20, 20:40].astype(np.float32)
                # Filter valid non-zero
                valid_mask = patch > 0
                if np.sum(valid_mask) > 300:
                    Y, X = np.indices(patch.shape)
                    valid_X = X[valid_mask].ravel()
                    valid_Y = Y[valid_mask].ravel()
                    valid_Z = patch[valid_mask].ravel()
                    A = np.c_[valid_X, valid_Y, np.ones_like(valid_X)]
                    coeff, _, _, _ = np.linalg.lstsq(A, valid_Z, rcond=None)
                    plane = (A @ coeff)
                    residuals_mm = valid_Z - plane
                    std_dev_mm = float(np.std(residuals_mm))
                    mean_dist_m = float(np.mean(valid_Z) / 1000.0)
                    zero_count = int(np.sum(patch == 0))
                    wall_patch_results = {
                        "verified": True,
                        "episode": ep_name,
                        "frame_file": depth_files[10],
                        "patch_size": "20x20 pixels",
                        "mean_distance_m": round(mean_dist_m, 4),
                        "std_dev_mm": round(std_dev_mm, 4),
                        "std_dev_under_1mm": bool(std_dev_mm < 1.0),
                        "zero_count": zero_count
                    }
                    break

    final_report = {
        "qc_summary": qc_summary,
        "inspected_sample_10_episodes": inspected_episodes,
        "depth_wall_patch_noise_verification": wall_patch_results
    }

    with open(OUTPUT_JSON, "w") as fp:
        json.dump(final_report, fp, indent=2)

    print("=" * 80)
    print("PILOT 300 QC & INSPECTION COMPLETE")
    print("=" * 80)
    print(f"Total Episodes: {qc_summary['total_episodes']}")
    print(f"QC Passed: {qc_summary['qc_passed_count']}/{qc_summary['total_episodes']} ({qc_summary['qc_pass_rate_pct']}%)")
    print(f"Max Time Gap: {qc_summary['max_time_gap_s']:.3f} s (Threshold <= 0.35 s)")
    print(f"Invariant Violations: {qc_summary['invariant_violations']}")
    print(f"NaNs/Infs: {qc_summary['has_nan_count']}")
    print(f"\nDepth Wall Patch Verification:")
    print(f"  * Episode: {wall_patch_results.get('episode')}")
    print(f"  * Std Dev: {wall_patch_results.get('std_dev_mm')} mm (Threshold < 1.0 mm: {wall_patch_results.get('std_dev_under_1mm')})")
    print(f"Exported to: {OUTPUT_JSON}")
    return final_report


if __name__ == '__main__':
    inspect_all_episodes()
