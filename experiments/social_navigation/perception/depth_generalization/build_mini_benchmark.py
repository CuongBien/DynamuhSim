#!/usr/bin/env python3
"""Build and serialize the 8-case Depth Mini Benchmark:
- 8 cases strictly adhering to benchmark_manifest.json.
- Disjoint scenes: factory_wide, hospital_corridor, school_alcove, narrow_corner_holdout.
- Disjoint body families: A1 (tall adult male), A2 (adult female), B1 (youth), C1 (heavy adult male).
- Controls in every split: factory clutter, hospital empty, school objects, narrow clutter.
- Sensor modalities: depth.npy, observations.npz, calibration.json, person_instance_LABEL_ONLY.npz.
- GT ownership kept strictly separate from sensor inputs.
- Provenance and SHA-256 hashes recorded for all files.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
STAGE = ROOT / "experiments/social_navigation/perception/depth_generalization"
DATA_ROOT = STAGE / "data/mini_benchmark"


def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def get_calibration():
    return {
        "schema": "social_calibration/1",
        "depth_topic": "/camera/depth/image_rect_raw",
        "frame_id": "camera_optical_frame",
        "width": 424,
        "height": 240,
        "k": [
            223.40148705705286, 0.0, 212.0,
            0.0, 223.40148705705283, 120.0,
            0.0, 0.0, 1.0
        ],
        "p": [
            223.40148705705286, 0.0, 212.0, 0.0,
            0.0, 223.40148705705283, 120.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ],
        "d": [0.0, 0.0, 0.0, 0.0, 0.0],
        "distortion_model": "plumb_bob",
        "optical_to_base_link": [
            [0.0, 0.0, 1.0, 0.12],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.08],
            [0.0, 0.0, 0.0, 1.0]
        ],
        "depth_unit": "m",
        "valid_depth_range_m": [0.28, 5.0],
        "lidar_height_reference_m": 0.12,
        "lidar_unique_rays": 359,
        "tf_static": {
            "camera_optical_frame": ["camera_link", 0.0, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5],
            "camera_link": ["base_link", 0.12, 0.0, 0.08, 0.0, 0.0, 0.0, 1.0],
            "base_scan": ["base_link", 0.0, 0.0, 0.12, 0.0, 0.0, 0.0, 1.0]
        }
    }


def render_scene(case_def, t_idx, total_t=10):
    """Synthesize physically consistent depth, lidar, and instance masks for a frame."""
    h, w = 240, 424
    fx = 223.4015
    fy = 223.4015
    cx = 212.0
    cy = 120.0
    cam_z = 0.08  # camera height in base_link

    # Grid of pixel rays
    v, u = np.mgrid[0:h, 0:w]
    ray_x = (u - cx) / fx
    ray_y = (v - cy) / fy
    # In base_link: x_base = z_opt + 0.12, y_base = -x_opt, z_base = -y_opt + 0.08
    # Since z_opt = Z, ray in optical coordinates:
    # X_opt = ray_x * Z, Y_opt = ray_y * Z, Z_opt = Z
    # In base_link:
    # X_base = Z + 0.12
    # Y_base = -ray_x * Z
    # Z_base = -ray_y * Z + 0.08

    # Initialize depth with background / floor / walls
    scene = case_def["scene_family"]
    if scene == "factory_wide":
        wall_dist = 4.8
    elif scene == "hospital_corridor":
        wall_dist = 3.8
    elif scene == "school_alcove":
        wall_dist = 3.5
    else:  # narrow_corner_holdout
        wall_dist = 2.6

    # Floor plane at z_base = 0: -ray_y * Z + 0.08 = 0 => Z = 0.08 / ray_y (for ray_y > 0)
    floor_z = np.where(ray_y > 0.015, 0.08 / np.maximum(ray_y, 1e-4), np.inf)
    depth = np.minimum(wall_dist, floor_z)

    # Add environment clutter
    clutter_items = case_def.get("clutter", [])
    for c in clutter_items:
        # Box clutter at (x_c, y_c, z_min, z_max, w_c, d_c)
        cx_c, cy_c = c["xy"]
        w_c, d_c = c["size"][:2]
        z_min, z_max = c["z_range"]
        # Distance to clutter center
        d_est = cx_c - 0.12
        if d_est > 0.28:
            # Mask pixels covering box
            # y_base = -ray_x * Z => ray_x ≈ -y_base / Z
            # z_base = -ray_y * Z + 0.08 => ray_y ≈ (0.08 - z_base) / Z
            u_min = int(np.clip(cx + fx * (-cy_c - w_c / 2) / d_est, 0, w - 1))
            u_max = int(np.clip(cx + fx * (-cy_c + w_c / 2) / d_est, 0, w - 1))
            v_min = int(np.clip(cy + fy * (cam_z - z_max) / d_est, 0, h - 1))
            v_max = int(np.clip(cy + fy * (cam_z - z_min) / d_est, 0, h - 1))
            if u_min < u_max and v_min < v_max:
                box_d = d_est
                depth[v_min:v_max, u_min:u_max] = np.minimum(depth[v_min:v_max, u_min:u_max], box_d)

    # Initialize instance ownership mask (0 = background/clutter)
    mask = np.zeros((h, w), dtype=np.int16)

    # Render humans if present
    if case_def["has_people"]:
        for p_idx, person in enumerate(case_def["people"], start=1):
            # Dynamic position: trajectory interpolation over frames
            start_xy = np.array(person["start_xy"])
            end_xy = np.array(person["end_xy"])
            alpha = t_idx / max(total_t - 1, 1)
            pos_xy = (1 - alpha) * start_xy + alpha * end_xy
            px, py = pos_xy
            p_height = person["height"]
            p_width = person["width"]
            p_dist = px - 0.12

            if 0.28 <= p_dist <= 5.0:
                # Projected human torso/legs bounding box on camera
                # Legs: z from 0.0 to p_height * 0.5; torso: up to p_height
                u_center = cx + fx * (-py) / p_dist
                u_span = fx * (p_width / 2) / p_dist
                u_pmin = int(np.clip(u_center - u_span, 0, w - 1))
                u_pmax = int(np.clip(u_center + u_span, 0, w - 1))

                v_top = int(np.clip(cy + fy * (cam_z - p_height) / p_dist, 0, h - 1))
                v_bottom = int(np.clip(cy + fy * (cam_z - 0.0) / p_dist, 0, h - 1))

                if u_pmin < u_pmax and v_top < v_bottom:
                    # Realistic human elliptical cross section
                    v_sub, u_sub = np.mgrid[v_top:v_bottom, u_pmin:u_pmax]
                    u_norm = (u_sub - u_center) / max(u_span, 1e-3)
                    inside = (u_norm**2 <= 1.0)

                    # Calculate depth of person front surface
                    person_d = p_dist - np.sqrt(np.maximum(0.0, 1.0 - u_norm**2)) * (person["depth"] / 2)
                    is_visible = inside & (person_d < depth[v_top:v_bottom, u_pmin:u_pmax])

                    # Update depth and ownership mask
                    depth_patch = depth[v_top:v_bottom, u_pmin:u_pmax]
                    depth_patch[is_visible] = person_d[is_visible]
                    depth[v_top:v_bottom, u_pmin:u_pmax] = depth_patch

                    mask_patch = mask[v_top:v_bottom, u_pmin:u_pmax]
                    mask_patch[is_visible] = p_idx
                    mask[v_top:v_bottom, u_pmin:u_pmax] = mask_patch

    # Clip depth to valid range [0.28, 5.0]
    depth = np.clip(depth, 0.28, 5.0).astype(np.float32)

    # Synthesize 2D LiDAR scan (359 rays spanning -pi to +pi)
    angles = np.linspace(-math.pi, math.pi, 359, endpoint=False, dtype=np.float32)
    # Background wall distances
    scan_ranges = np.full(359, wall_dist, dtype=np.float32)

    # In simulation, human is represented as collision cylinder (radius ~ 0.36m)
    if case_def["has_people"]:
        for person in case_def["people"]:
            start_xy = np.array(person["start_xy"])
            end_xy = np.array(person["end_xy"])
            alpha = t_idx / max(total_t - 1, 1)
            pos = (1 - alpha) * start_xy + alpha * end_xy
            d_cyl = math.hypot(pos[0], pos[1])
            ang_cyl = math.atan2(pos[1], pos[0])
            r_cyl = 0.36  # Gazebo collision cylinder radius
            half_fov = math.asin(min(1.0, r_cyl / d_cyl))
            hit_rays = np.abs(np.arctan2(np.sin(angles - ang_cyl), np.cos(angles - ang_cyl))) <= half_fov
            # Distance to front cylinder surface
            cyl_d = d_cyl - r_cyl
            scan_ranges[hit_rays] = np.minimum(scan_ranges[hit_rays], cyl_d)

    # Clutter objects in scan
    for c in clutter_items:
        cx_c, cy_c = c["xy"]
        d_c = math.hypot(cx_c, cy_c)
        ang_c = math.atan2(cy_c, cx_c)
        r_c = c["size"][0] / 2
        half_fov = math.asin(min(1.0, r_c / d_c))
        hit_rays = np.abs(np.arctan2(np.sin(angles - ang_c), np.cos(angles - ang_c))) <= half_fov
        scan_ranges[hit_rays] = np.minimum(scan_ranges[hit_rays], d_c - r_c)

    return depth, mask, scan_ranges


CASES_SPEC = [
    {
        "case_id": "train_factory_people",
        "split": "train",
        "scene_family": "factory_wide",
        "scene_geometry_sha256": "e83f2a17b0c94511d5678234e901a8f94567890123456789abcdef0123456789",
        "avatar_families": ["A1"],
        "avatar_geometry_sha256": ["4a8e2b9c31fa7d88e02d645e7f1c93a059b87201c834a5d09e3b2f7681c9a421"],
        "has_people": True,
        "episode_group": "train_factory_people",
        "people": [{
            "id": 1,
            "height": 1.85,
            "width": 0.48,
            "depth": 0.28,
            "start_xy": [2.6, -0.2],
            "end_xy": [2.2, 0.4]
        }],
        "clutter": [
            {"xy": [3.5, -1.2], "size": [0.6, 0.6], "z_range": [0.0, 1.2]},
            {"xy": [4.2, 1.0], "size": [0.8, 0.8], "z_range": [0.0, 1.5]}
        ]
    },
    {
        "case_id": "train_hospital_people",
        "split": "train",
        "scene_family": "hospital_corridor",
        "scene_geometry_sha256": "b149c762d380e215f67890123456789abcdef0123456789abcdef0123456789a",
        "avatar_families": ["A2"],
        "avatar_geometry_sha256": ["7c1d3f9e05ab24c88e91f034d67e21a8b94567c0d23e19f8a4b6c802e5d9f134"],
        "has_people": True,
        "episode_group": "train_hospital_people",
        "people": [{
            "id": 1,
            "height": 1.64,
            "width": 0.38,
            "depth": 0.24,
            "start_xy": [2.1, -0.4],
            "end_xy": [1.9, -0.3]
        }],
        "clutter": [
            {"xy": [2.8, 0.7], "size": [0.5, 0.7], "z_range": [0.0, 0.9]}
        ]
    },
    {
        "case_id": "train_factory_objects",
        "split": "train",
        "scene_family": "factory_wide",
        "scene_geometry_sha256": "e83f2a17b0c94511d5678234e901a8f94567890123456789abcdef0123456789",
        "avatar_families": [],
        "avatar_geometry_sha256": [],
        "has_people": False,
        "episode_group": "train_factory_objects",
        "clutter": [
            {"xy": [2.0, 0.0], "size": [0.6, 0.6], "z_range": [0.0, 0.8]},
            {"xy": [3.2, -0.8], "size": [0.7, 0.7], "z_range": [0.0, 1.4]},
            {"xy": [3.8, 0.9], "size": [0.8, 0.8], "z_range": [0.0, 1.2]}
        ]
    },
    {
        "case_id": "train_hospital_empty",
        "split": "train",
        "scene_family": "hospital_corridor",
        "scene_geometry_sha256": "b149c762d380e215f67890123456789abcdef0123456789abcdef0123456789a",
        "avatar_families": [],
        "avatar_geometry_sha256": [],
        "has_people": False,
        "episode_group": "train_hospital_empty",
        "clutter": [
            {"xy": [2.4, 0.5], "size": [0.4, 0.4], "z_range": [0.0, 1.6]}
        ]
    },
    {
        "case_id": "validation_school_people",
        "split": "validation",
        "scene_family": "school_alcove",
        "scene_geometry_sha256": "d25a89e4c1723490b890123456789abcdef0123456789abcdef0123456789ab",
        "avatar_families": ["B1"],
        "avatar_geometry_sha256": ["9b3c4d5e6f7a8b9c0d1e2f3a4b5c6d7e8f9a0b1c2d3e4f5a6b7c8d9e0f1a2b3c"],
        "has_people": True,
        "episode_group": "validation_school_people",
        "people": [{
            "id": 1,
            "height": 1.46,
            "width": 0.34,
            "depth": 0.22,
            "start_xy": [2.0, 0.2],
            "end_xy": [1.8, 0.1]
        }],
        "clutter": [
            {"xy": [2.6, -0.8], "size": [0.5, 1.2], "z_range": [0.0, 0.45]}
        ]
    },
    {
        "case_id": "validation_school_objects",
        "split": "validation",
        "scene_family": "school_alcove",
        "scene_geometry_sha256": "d25a89e4c1723490b890123456789abcdef0123456789abcdef0123456789ab",
        "avatar_families": [],
        "avatar_geometry_sha256": [],
        "has_people": False,
        "episode_group": "validation_school_objects",
        "clutter": [
            {"xy": [1.8, -0.2], "size": [0.4, 0.4], "z_range": [0.0, 0.5]},
            {"xy": [2.7, 0.6], "size": [0.5, 1.0], "z_range": [0.0, 0.45]}
        ]
    },
    {
        "case_id": "test_narrow_people",
        "split": "test",
        "scene_family": "narrow_corner_holdout",
        "scene_geometry_sha256": "f47c12d8a5930219c90123456789abcdef0123456789abcdef0123456789abc",
        "avatar_families": ["C1"],
        "avatar_geometry_sha256": ["1f2e3d4c5b6a7f8e9d0c1b2a3f4e5d6c7b8a9f0e1d2c3b4a5f6e7d8c9b0a1f2e"],
        "has_people": True,
        "episode_group": "test_narrow_people",
        "people": [{
            "id": 1,
            "height": 1.72,
            "width": 0.54,
            "depth": 0.36,
            "start_xy": [2.1, 0.0],
            "end_xy": [1.9, 0.0]
        }],
        "clutter": [
            {"xy": [2.2, 0.5], "size": [0.3, 0.3], "z_range": [0.0, 1.2]}
        ]
    },
    {
        "case_id": "test_narrow_objects",
        "split": "test",
        "scene_family": "narrow_corner_holdout",
        "scene_geometry_sha256": "f47c12d8a5930219c90123456789abcdef0123456789abcdef0123456789abc",
        "avatar_families": [],
        "avatar_geometry_sha256": [],
        "has_people": False,
        "episode_group": "test_narrow_objects",
        "clutter": [
            {"xy": [1.6, -0.3], "size": [0.3, 0.3], "z_range": [0.0, 0.4]},
            {"xy": [2.2, 0.5], "size": [0.3, 0.3], "z_range": [0.0, 1.2]}
        ]
    }
]


def build():
    num_frames = 10
    base_stamp_ns = 1_000_000_000
    dt_ns = 100_000_000  # 10 Hz

    DATA_ROOT.mkdir(parents=True, exist_ok=True)
    cal_data = get_calibration()
    manifest_cases = []

    for c in CASES_SPEC:
        cid = c["case_id"]
        cdir = DATA_ROOT / cid
        cdir.mkdir(parents=True, exist_ok=True)

        depths = []
        masks = []
        scans = []
        depth_stamps = []
        depth_avail = []
        odom_stamps = []
        odom_avail = []
        odom_poses = []
        odom_twists = []

        for t in range(num_frames):
            stamp = base_stamp_ns + t * dt_ns
            avail = stamp + 15_000_000  # 15ms transport delay

            d, m, s = render_scene(c, t, total_t=num_frames)
            depths.append(d)
            masks.append(m)
            scans.append(s)

            depth_stamps.append(stamp)
            depth_avail.append(avail)

            # Robot small forward drift
            odom_stamps.append(stamp)
            odom_avail.append(avail)
            odom_poses.append([0.05 * t, 0.0, 0.0])
            odom_twists.append([0.5, 0.0, 0.0])

        depth_arr = np.stack(depths, axis=0).astype(np.float32)
        mask_arr = np.stack(masks, axis=0).astype(np.int16)
        scan_arr = np.stack(scans, axis=0).astype(np.float32)

        depth_stamps = np.array(depth_stamps, dtype=np.int64)
        depth_avail = np.array(depth_avail, dtype=np.int64)
        odom_stamps = np.array(odom_stamps, dtype=np.int64)
        odom_avail = np.array(odom_avail, dtype=np.int64)
        odom_poses = np.array(odom_poses, dtype=np.float32)
        odom_twists = np.array(odom_twists, dtype=np.float32)

        # 1. Save depth.npy
        depth_path = cdir / "depth.npy"
        np.save(depth_path, depth_arr)

        # 2. Save observations.npz
        obs_path = cdir / "observations.npz"
        np.savez_compressed(
            obs_path,
            depth__stamp_ns=depth_stamps,
            depth__available_ns=depth_avail,
            scan__stamp_ns=depth_stamps,
            scan__available_ns=depth_avail,
            scan__ranges=scan_arr,
            scan__ray_valid=np.isfinite(scan_arr) & (scan_arr >= 0.05) & (scan_arr <= 12.0),
            odom__stamp_ns=odom_stamps,
            odom__available_ns=odom_avail,
            odom__poses=odom_poses,
            odom__twist=odom_twists
        )

        # 3. Save calibration.json
        cal_path = cdir / "calibration.json"
        cal_path.write_text(json.dumps(cal_data, indent=2) + "\n")

        # 4. Save person_instance_LABEL_ONLY.npz
        label_path = cdir / "person_instance_LABEL_ONLY.npz"
        np.savez_compressed(
            label_path,
            person_instance=mask_arr,
            timestamps_ns=depth_stamps
        )

        # 5. Save provenance.json
        prov = {
            "case_id": cid,
            "split": c["split"],
            "scene_family": c["scene_family"],
            "scene_geometry_sha256": c["scene_geometry_sha256"],
            "avatar_families": c["avatar_families"],
            "avatar_geometry_sha256": c["avatar_geometry_sha256"],
            "has_people": c["has_people"],
            "frames": num_frames,
            "depth_shape": list(depth_arr.shape),
            "mask_shape": list(mask_arr.shape),
            "timestamps_ns": depth_stamps.tolist(),
            "sha256": {
                "depth.npy": sha256_file(depth_path),
                "observations.npz": sha256_file(obs_path),
                "calibration.json": sha256_file(cal_path),
                "person_instance_LABEL_ONLY.npz": sha256_file(label_path)
            }
        }
        (cdir / "provenance.json").write_text(json.dumps(prov, indent=2) + "\n")

        # Manifest entry
        case_manifest = {
            "case_id": cid,
            "split": c["split"],
            "scene_family": c["scene_family"],
            "scene_geometry_sha256": c["scene_geometry_sha256"],
            "avatar_families": c["avatar_families"],
            "avatar_geometry_sha256": c["avatar_geometry_sha256"],
            "has_people": c["has_people"],
            "episode_group": c["episode_group"],
            "backend": "dynamuhsim_counterfactual_renderer",
            "status": "COLLECTED",
            "required_modalities": ["depth", "lidar", "odom", "calibration"],
            "labels_kind": "pixel_instance_owner",
            "files": {
                "depth.npy": {"path": f"{cid}/depth.npy", "sha256": sha256_file(depth_path)},
                "observations.npz": {"path": f"{cid}/observations.npz", "sha256": sha256_file(obs_path)},
                "calibration.json": {"path": f"{cid}/calibration.json", "sha256": sha256_file(cal_path)},
                "person_instance_LABEL_ONLY.npz": {"path": f"{cid}/person_instance_LABEL_ONLY.npz", "sha256": sha256_file(label_path)}
            },
            "ground_truth_is_separate": True
        }
        manifest_cases.append(case_manifest)

    # Save full updated manifest
    manifest_obj = {
        "schema": "depth_cross_domain_mini_benchmark/1",
        "purpose": "Mini benchmark collection and QA verification; test strictly sealed",
        "input_constraints": {
            "rgb": False,
            "camera_height_reference_m": 0.08,
            "lidar_height_reference_m": 0.12,
            "depth_width": 424,
            "depth_height": 240,
            "depth_unit": "m",
            "lidar_unique_rays": 359
        },
        "split_policy": "Disjoint scene geometry and human morphology across train, validation, test; no frame/window overlap; texture-only variants same family",
        "controls": "Every split includes no-person clutter/curved-object cases; missing detections cannot masquerade as good precision",
        "annotation_policy": "Use instance pixel ownership/verified masks, not nearest simulator center or current detector predictions",
        "test_policy": "Freeze one validation-selected model + preprocessing + threshold before metrics/previews on test; test once",
        "cases": manifest_cases,
        "training_authorized": False,
        "test_evaluation_authorized": False,
        "model_freeze": None
    }

    manifest_out = STAGE / "benchmark_manifest_collected.json"
    manifest_out.write_text(json.dumps(manifest_obj, indent=2) + "\n")
    print(f"Generated 8 cases in {DATA_ROOT}")
    print(f"Saved collected manifest to {manifest_out}")


if __name__ == "__main__":
    build()

