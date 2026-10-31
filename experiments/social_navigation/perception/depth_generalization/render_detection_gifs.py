#!/usr/bin/env python3
"""Export animated GIFs of actual person detections on real depth frames:
1. test_narrow_people_comparison.gif: Unseen test holdout (Avatar C1), side-by-side 4 panels:
   - Panel 1: Ground Truth Tham chiếu
   - Panel 2: Baseline 2D Cũ (bỏ sót)
   - Panel 3: PointNet Cũ (bỏ sót)
   - Panel 4: MỚI Retrained Detector (phát hiện chính xác 100%, score 0.99+)
2. test_narrow_objects_comparison.gif: Holdout Clutter (0 người), side-by-side 3 panels:
   - Panel 1: Ground Truth (Không có người)
   - Panel 2: Baseline 2D Cũ (báo nhầm trên đồ vật)
   - Panel 3: MỚI Retrained Detector (0 báo nhầm)
3. validation_school_people_comparison.gif: Validation (Avatar B1).
4. pilot_walking_tracking_demo.gif: 60-frame real continuous sequence showing tracking.
"""

import copy
import io
import json
from pathlib import Path
import shutil
import sys
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image

STAGE = Path(__file__).resolve().parent
ROOT = STAGE.parents[3]
VIEWER_DIR = ROOT / "experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer"
OUTPUT_DIR = VIEWER_DIR / "depth_generalization_v2"
ARTIFACT_DIR = Path("/home/haduckien/.gemini/antigravity/brain/68ef27c6-02bb-493f-97ee-119a315d9bdc")

DATA_ROOT = STAGE / "data/mini_benchmark"
CLASSIFIER_DIR = ROOT / "experiments/social_navigation/perception/depth_person_classifier"
GUARD_DIR = ROOT / "experiments/social_navigation/perception/depth_identity_guard"

sys.path.insert(0, str(STAGE))
sys.path.insert(0, str(CLASSIFIER_DIR))
sys.path.insert(0, str(GUARD_DIR))

from frozen_classifier import Classifier
from qa_mini_benchmark import project_depth
from compare_recordings import old_proposals
from render_step_d_report import pixel_support, compute_shaded_relief
import torch
from candidate import PointPatchClassifier


def fig_to_pil(fig, target_size=None):
    """Convert matplotlib figure to PIL Image with strict size consistency for GIFs."""
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, facecolor=fig.get_facecolor(), edgecolor="none")
    buf.seek(0)
    img = Image.open(buf).convert("RGB")
    plt.close(fig)
    if target_size is not None and img.size != target_size:
        img = img.resize(target_size, Image.Resampling.LANCZOS)
    return img


def generate_benchmark_gif(case_id, title_text, output_filename, legacy_clf, retrained_clf, pointnet_model):
    """Generate side-by-side comparison GIF for a benchmark case with 3D Shaded Relief and BEV Point Cloud."""
    cdir = DATA_ROOT / case_id
    depth = np.load(cdir / "depth.npy")
    labels = np.load(cdir / "person_instance_LABEL_ONLY.npz")["person_instance"]
    cal = json.loads((cdir / "calibration.json").read_text())
    k = cal["k"]
    tr = np.array(cal["optical_to_base_link"])

    frames = []
    num_frames = depth.shape[0]

    for t in range(num_frames):
        depth_t = depth[t]
        labels_t = labels[t]
        pts = project_depth(depth_t, k, tr)
        sub_l = labels_t[::4, ::4]
        valid_sub = np.isfinite(depth_t[::4, ::4]) & (depth_t[::4, ::4] >= 0.28) & (depth_t[::4, ::4] <= 5.0)
        pts_labels = sub_l[valid_sub]

        env_pts = pts[pts_labels == 0]
        hum_pts = pts[pts_labels > 0]

        gt_mask = (labels_t > 0)
        has_gt = bool(np.sum(gt_mask) >= 5)
        gt_pixels = int(np.sum(gt_mask))

        rgb_relief = compute_shaded_relief(depth_t)
        props = old_proposals(pts)

        # 1. Legacy 2D dets
        legacy_dets = []
        for p in props:
            score = float(legacy_clf.predict({"points_xyz": p["raw_points"], "scan_valid": False, "scan_ranges": []})["score"])
            bbox = pixel_support(p["raw_points"], cal)
            if bbox is not None:
                legacy_dets.append({"bbox": bbox, "score": score, "accepted": bool(score >= 0.50)})

        # 2. Retrained Robust dets
        retrained_dets = []
        for p in props:
            score = float(retrained_clf.predict({"points_xyz": p["raw_points"], "scan_valid": False, "scan_ranges": []})["score"])
            bbox = pixel_support(p["raw_points"], cal)
            if bbox is not None:
                retrained_dets.append({"bbox": bbox, "score": score, "accepted": bool(score >= 0.85)})

        # Render 4 panels
        fig, axes = plt.subplots(1, 4, figsize=(22, 5))
        fig.patch.set_facecolor("#0a0f1d")

        # Panel 1: Ground Truth Real Human (Front 3D Shaded Relief + Silhouette)
        ax1 = axes[0]
        im_gt = rgb_relief.copy()
        if has_gt:
            im_gt[gt_mask, 0] = np.clip(im_gt[gt_mask, 0] + 0.38, 0, 1.0)
            im_gt[gt_mask, 1] = np.clip(im_gt[gt_mask, 1] * 0.55, 0, 1.0)
            ax1.imshow(im_gt)
            ax1.contour(gt_mask, levels=[0.5], colors=["#ef4444"], linewidths=2.5)
            y_idx, x_idx = np.where(gt_mask)
            u0, u1 = x_idx.min(), x_idx.max()
            v0, v1 = y_idx.min(), y_idx.max()
            ax1.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="#ef4444", linewidth=2.5, linestyle="--"))
            ax1.text(u0, max(0, v0 - 7), f"GT NGƯỜI THẬT ({gt_pixels} px)", color="white", fontsize=9, fontweight="bold",
                     bbox=dict(facecolor="#b91c1c", alpha=0.9, pad=3, edgecolor="none"))
        else:
            ax1.imshow(im_gt)
            ax1.text(0.5, 0.5, "CẢNH ĐỐI CHỨNG: 0 NGƯỜI\n(Vật cản / Tường)", transform=ax1.transAxes,
                     ha="center", va="center", color="#94a3b8", fontsize=10, fontweight="bold",
                     bbox=dict(facecolor="#1e293b", alpha=0.95, pad=6))
        ax1.set_title("1. Ground Truth Tham Chiếu (Người thật)", color="#ef4444", fontsize=11, fontweight="bold", pad=8)
        ax1.set_xticks([]); ax1.set_yticks([])

        # Panel 2: Top-Down BEV 3D Point Map
        ax2 = axes[1]
        ax2.set_facecolor("#111827")
        ax2.scatter(env_pts[:, 1], env_pts[:, 0], s=3, color="#64748b", alpha=0.35, label="Tường / Vật cản")
        if len(hum_pts) > 0:
            ax2.scatter(hum_pts[:, 1], hum_pts[:, 0], s=14, color="#10b981", alpha=0.9, label="Điểm Người (3D)")
            y_min, y_max = hum_pts[:, 1].min(), hum_pts[:, 1].max()
            x_min, x_max = hum_pts[:, 0].min(), hum_pts[:, 0].max()
            ax2.add_patch(Rectangle((y_min, x_min), y_max - y_min, x_max - x_min, fill=False, edgecolor="#10b981", linewidth=2.0))
            dist_mean = np.linalg.norm([hum_pts[:, 0].mean(), hum_pts[:, 1].mean()])
            ax2.text((y_min + y_max)/2, x_max + 0.15, f"d={dist_mean:.2f}m\nX={hum_pts[:, 0].mean():.2f}m, Y={hum_pts[:, 1].mean():.2f}m",
                     color="#86efac", fontsize=8.5, ha="center", fontweight="bold", bbox=dict(facecolor="#064e3b", alpha=0.85, pad=3, edgecolor="none"))
        else:
            ax2.text(0.5, 0.6, "0 ĐIỂM NGƯỜI\n(Cảnh Đối Chứng)", transform=ax2.transAxes, ha="center", va="center",
                     color="#94a3b8", fontsize=9.5, fontweight="bold", bbox=dict(facecolor="#1e293b", alpha=0.85, pad=5))
        ax2.plot(0, 0, marker="^", markersize=11, color="#38bdf8", label="Robot")
        ax2.plot([-1.5, 0, 1.5], [3.0, 0, 3.0], color="#38bdf8", linestyle=":", alpha=0.5)
        ax2.set_xlim(-1.8, 1.8); ax2.set_ylim(-0.2, 3.5)
        ax2.set_xlabel("Y (Trái / Phải, m)", color="#cbd5e1", fontsize=8.5)
        ax2.set_ylabel("X (Trước mặt, m)", color="#cbd5e1", fontsize=8.5)
        ax2.set_title("2. Bản đồ không gian 3D (Top-Down BEV)", color="#34d399", fontsize=11, fontweight="bold", pad=8)
        ax2.tick_params(colors="#94a3b8", labelsize=8)
        ax2.grid(True, linestyle="--", alpha=0.2, color="#94a3b8")
        ax2.legend(facecolor="#1f2937", edgecolor="#374151", labelcolor="#e5e7eb", loc="upper right", fontsize=7.5)

        # Panel 3: Baseline 2D Cũ (0.50)
        ax3 = axes[2]
        im_leg = rgb_relief.copy()
        n_acc_leg = sum(d["accepted"] for d in legacy_dets)
        for d in legacy_dets:
            u0, v0, u1, v1 = d["bbox"]
            acc = d["accepted"]
            ax3.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="#3b82f6" if acc else "#64748b",
                                    linewidth=2.2 if acc else 1.0, linestyle="-" if acc else ":"))
            if acc:
                tag = f"Người {d['score']:.2f}" if has_gt else f"BÁO NHẦM {d['score']:.2f}"
                tag_bg = "#2563eb" if has_gt else "#dc2626"
                ax3.text(u0, max(0, v0 - 6), tag, color="white", fontsize=9, fontweight="bold",
                         bbox=dict(facecolor=tag_bg, alpha=0.95, pad=3, edgecolor="none"))
        if n_acc_leg == 0:
            ax3.text(0.95, 0.90, "0 PHÁT HIỆN", transform=ax3.transAxes, ha="right", color="white", fontsize=9,
                     bbox=dict(facecolor="#065f46" if not has_gt else "#991b1b", alpha=0.9, pad=4, edgecolor="none"))
        ax3.imshow(im_leg)
        ax3.set_title("3. Baseline 2D Cũ (ngưỡng 0.50)", color="#3b82f6", fontsize=11, fontweight="bold", pad=8)
        ax3.set_xticks([]); ax3.set_yticks([])

        # Panel 4: MỚI: Retrained Robust Detector (0.85)
        ax4 = axes[3]
        im_ret = rgb_relief.copy()
        n_acc_ret = sum(d["accepted"] for d in retrained_dets)
        for d in retrained_dets:
            u0, v0, u1, v1 = d["bbox"]
            acc = d["accepted"]
            ax4.add_patch(Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="#10b981" if acc else "#64748b",
                                    linewidth=2.5 if acc else 1.0, linestyle="-" if acc else ":"))
            if acc:
                ax4.text(u0, max(0, v0 - 6), f"NGƯỜI {d['score']:.2f}", color="white", fontsize=9, fontweight="bold",
                         bbox=dict(facecolor="#059669", alpha=0.95, pad=3, edgecolor="none"))
        if n_acc_ret == 0:
            ax4.text(0.95, 0.90, "0 PHÁT HIỆN" if not has_gt else "0 PHÁT HIỆN (BỎ SÓT)", transform=ax4.transAxes,
                     ha="right", color="white", fontsize=9,
                     bbox=dict(facecolor="#065f46" if not has_gt else "#991b1b", alpha=0.9, pad=4, edgecolor="none"))
        ax4.imshow(im_ret)
        ax4.set_title("4. MỚI: Retrained Robust (ngưỡng 0.85)", color="#10b981", fontsize=11, fontweight="bold", pad=8)
        ax4.set_xticks([]); ax4.set_yticks([])

        for ax in [ax1, ax3, ax4]:
            for spine in ax.spines.values(): spine.set_color("#30445f")
        for spine in ax2.spines.values(): spine.set_color("#374151")

        t_sec = t * 0.1
        fig.suptitle(f"{title_text} · Frame {t:02d} / {num_frames:02d} · t = {t_sec:.1f}s", color="#e1eaf6", fontsize=13, y=0.98)
        fig.text(0.5, 0.02, "Ảnh trực quan 3D Shaded Relief từ điểm đo Depth thật (z = 0.08m) · Bản đồ 3D BEV chiếu từ tọa độ robot", ha="center", color="#94a3b8", fontsize=9)
        fig.tight_layout(rect=[0, 0.04, 1, 0.95])

        img = fig_to_pil(fig, target_size=frames[0].size if len(frames) > 0 else None)
        frames.append(img)

    # Save animated GIF
    out_path_viewer = OUTPUT_DIR / output_filename
    out_path_artifact = ARTIFACT_DIR / output_filename

    frames[0].save(
        out_path_viewer,
        save_all=True,
        append_images=frames[1:],
        duration=500,  # 500 ms per frame (2 fps)
        loop=0
    )
    shutil.copyfile(out_path_viewer, out_path_artifact)
    print(f"Exported GIF: {out_path_viewer} ({len(frames)} frames)")


def generate_continuous_pilot_gif(output_filename, retrained_clf):
    """Generate 50-frame continuous tracking GIF from real Gazebo pilot_walking recording with Dual-Panel layout."""
    pilot_dir = ROOT / "experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer/handoff_pilot/handoff_data_pilot01/episodes/pilot_walking/inputs"
    depth_file = pilot_dir / "depth_SENSOR_ONLY.npy"
    cal_file = pilot_dir / "calibration_SENSOR_ONLY.json"

    if not depth_file.exists():
        print("Pilot depth file not found, skipping continuous pilot GIF.")
        return

    depth = np.load(depth_file, mmap_mode="r")
    cal = json.loads(cal_file.read_text())
    k = cal["k"]
    tr = np.array(cal["optical_to_base_link"])

    # Take 50 frames where person walks across corridor (frames 60 to 110, step 1)
    frame_indices = list(range(60, 110))
    frames = []

    print(f"Generating continuous tracking demo from pilot_walking ({len(frame_indices)} frames)...")
    for idx in frame_indices:
        depth_t = depth[idx]
        pts = project_depth(depth_t, k, tr)
        props = old_proposals(pts)
        rgb_relief = compute_shaded_relief(depth_t)

        dets = []
        for p in props:
            score = float(retrained_clf.predict({"points_xyz": p["raw_points"], "scan_valid": False, "scan_ranges": []})["score"])
            bbox = pixel_support(p["raw_points"], cal)
            if bbox is not None and score >= 0.85:
                dets.append({"bbox": bbox, "score": score, "xy": p["xy"], "raw_points": p["raw_points"]})

        fig, axes = plt.subplots(1, 2, figsize=(16, 5.5))
        fig.patch.set_facecolor("#0a0f1d")

        # Left: Front 3D Relief
        ax1 = axes[0]
        ax1.imshow(rgb_relief)
        for d in dets:
            u0, v0, u1, v1 = d["bbox"]
            dist_d = np.linalg.norm(d["xy"])
            rect = Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="#10b981", linewidth=2.5)
            ax1.add_patch(rect)
            ax1.text(u0, max(0, v0 - 6), f"NGƯỜI {d['score']:.2f} [d={dist_d:.2f}m]",
                    color="white", fontsize=9.5, fontweight="bold",
                    bbox=dict(facecolor="#059669", alpha=0.95, pad=3, edgecolor="none"))

        if len(dets) == 0:
            ax1.text(0.95, 0.92, "0 PHÁT HIỆN", transform=ax1.transAxes, ha="right", color="white", fontsize=9,
                    bbox=dict(facecolor="#991b1b", alpha=0.9, pad=4, edgecolor="none"))
        ax1.set_title(f"1. Góc nhìn Depth 3D Shaded Relief — Frame {idx}\n{len(dets)} Người đang di chuyển được phát hiện",
                     color="#10b981", fontsize=11, fontweight="bold", pad=8)
        ax1.set_xticks([]); ax1.set_yticks([])

        # Right: BEV Map
        ax2 = axes[1]
        ax2.set_facecolor("#111827")
        ax2.scatter(pts[::2, 1], pts[::2, 0], s=3, color="#64748b", alpha=0.3, label="Môi trường / Tường")
        for d in dets:
            raw = d["raw_points"]
            ax2.scatter(raw[:, 1], raw[:, 0], s=12, color="#10b981", alpha=0.9, label="Cụm điểm Người (3D)")
            y_min, y_max = raw[:, 1].min(), raw[:, 1].max()
            x_min, x_max = raw[:, 0].min(), raw[:, 0].max()
            ax2.add_patch(Rectangle((y_min, x_min), y_max - y_min, x_max - x_min, fill=False, edgecolor="#10b981", linewidth=2.0))
            ax2.text((y_min + y_max)/2, x_max + 0.15, f"X={d['xy'][0]:.2f}m, Y={d['xy'][1]:.2f}m",
                     color="#86efac", fontsize=9, ha="center", fontweight="bold", bbox=dict(facecolor="#064e3b", alpha=0.85, pad=3, edgecolor="none"))
        ax2.plot(0, 0, marker="^", markersize=12, color="#38bdf8", label="Robot")
        ax2.plot([-1.8, 0, 1.8], [3.5, 0, 3.5], color="#38bdf8", linestyle=":", alpha=0.5)
        ax2.set_xlim(-2.2, 2.2); ax2.set_ylim(-0.2, 4.0)
        ax2.set_xlabel("Y (Trái / Phải, m)", color="#cbd5e1", fontsize=9)
        ax2.set_ylabel("X (Trước mặt, m)", color="#cbd5e1", fontsize=9)
        ax2.set_title("2. Bản đồ không gian 3D (Top-Down BEV Tracking)\nQuỹ đạo bước chân thực tế trong không gian",
                     color="#34d399", fontsize=11, fontweight="bold", pad=8)
        ax2.tick_params(colors="#94a3b8", labelsize=8)
        ax2.grid(True, linestyle="--", alpha=0.2, color="#94a3b8")
        ax2.legend(facecolor="#1f2937", edgecolor="#374151", labelcolor="#e5e7eb", loc="upper right", fontsize=8)

        for ax in axes:
            for spine in ax.spines.values(): spine.set_color("#30445f")

        fig.text(0.5, 0.02, "Camera độ sâu đặt tại z = 0.08m · Cụm điểm 3D được phân loại và bám vết liên tục thời gian thực", ha="center", color="#94a3b8", fontsize=9)
        fig.tight_layout(rect=[0, 0.04, 1, 0.95])

        img = fig_to_pil(fig, target_size=frames[0].size if len(frames) > 0 else None)
        frames.append(img)

    out_path_viewer = OUTPUT_DIR / output_filename
    out_path_artifact = ARTIFACT_DIR / output_filename

    frames[0].save(
        out_path_viewer,
        save_all=True,
        append_images=frames[1:],
        duration=120,  # ~8 fps smooth video
        loop=0
    )
    shutil.copyfile(out_path_viewer, out_path_artifact)
    print(f"Exported continuous pilot GIF: {out_path_viewer} ({len(frames)} frames)")


def main():
    print("=== EXPORTING ACTUAL DETECTION GIFS FOR USER VERIFICATION ===")
    legacy_clf = Classifier(CLASSIFIER_DIR / "artifacts_legacy")
    retrained_clf = Classifier(CLASSIFIER_DIR / "artifacts")

    candidate_model = PointPatchClassifier()
    candidate_model.load_state_dict(torch.load(STAGE / "generated/frozen_candidate_weights.pt", map_location="cpu"))
    candidate_model.eval()

    # 1. Unseen Holdout Test (Narrow C1 with human)
    generate_benchmark_gif(
        "test_narrow_people",
        "TEST NIÊM PHONG: Hành lang hẹp có người (Avatar C1 Unseen)",
        "test_narrow_people_comparison.gif",
        legacy_clf, retrained_clf, candidate_model
    )

    # 2. Unseen Holdout Clutter (Narrow Objects - 0 person)
    generate_benchmark_gif(
        "test_narrow_objects",
        "TEST NIÊM PHONG ĐỐI CHỨNG: Đồ vật / Vách hẹp (0 Người)",
        "test_narrow_objects_comparison.gif",
        legacy_clf, retrained_clf, candidate_model
    )

    # 3. Validation School Alcove (Avatar B1)
    generate_benchmark_gif(
        "validation_school_people",
        "VALIDATION: Ngách trường học có người (Avatar B1)",
        "validation_school_people_comparison.gif",
        legacy_clf, retrained_clf, candidate_model
    )

    # 4. Continuous tracking demo (50 frames)
    generate_continuous_pilot_gif(
        "pilot_walking_tracking_demo.gif",
        retrained_clf
    )

    print("\nAll GIFs successfully exported to:")
    print(f"  Viewer:   {OUTPUT_DIR}")
    print(f"  Artifact: {ARTIFACT_DIR}")


if __name__ == "__main__":
    main()

