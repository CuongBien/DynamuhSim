#!/usr/bin/env python3
"""Render Step D Localhost Decision Report:
- Generates side-by-side comparative overlays of:
  1. Ground Truth Reference Mask (independent view, not used for detector boxes)
  2. Legacy 2D Classifier (0.50)
  3. 3D with Prior (0.45)
  4. 3D without Prior (0.45)
  5. Candidate PointNet (0.65 frozen)
- Generates summary comparison charts.
- Assembles interactive HTML report with frame viewer, metric tables, error breakdowns, and decision gates.
- Outputs into visual_demo_20261029T101053/viewer/depth_generalization_v2/
"""

import copy
import hashlib
import html
import json
from pathlib import Path
import shutil
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import torch

STAGE = Path(__file__).resolve().parent
ROOT = STAGE.parents[3]
VIEWER_DIR = ROOT / "experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer"
OUTPUT_DIR = VIEWER_DIR / "depth_generalization_v2"
DATA_ROOT = STAGE / "data/mini_benchmark"
CLASSIFIER_DIR = ROOT / "experiments/social_navigation/perception/depth_person_classifier"
GUARD_DIR = ROOT / "experiments/social_navigation/perception/depth_identity_guard"

sys.path.insert(0, str(CLASSIFIER_DIR))
sys.path.insert(0, str(GUARD_DIR))
sys.path.insert(0, str(STAGE))

from candidate import PointPatchClassifier
from generalized_detector import GeneralizedPersonDetector
from frozen_classifier import Classifier
from compare_recordings import old_proposals
from qa_mini_benchmark import project_depth, candidates_2d


def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def pixel_support(points, calibration, stride=4):
    """Project 3D points in base_link to depth pixel bounding box [u_min, v_min, u_max, v_max]."""
    tf = np.array(calibration["optical_to_base_link"])
    optical = (np.asarray(points) - tf[:3, 3]) @ tf[:3, :3]
    good = np.isfinite(optical).all(axis=1) & (optical[:, 2] > 0)
    optical = optical[good]
    k = calibration["k"]
    uv = optical[:, :2] / optical[:, 2, None] * [k[0], k[4]] + [k[2], k[5]]
    w, h = calibration["width"], calibration["height"]
    good = (uv[:, 0] >= -0.01) & (uv[:, 0] < w) & (uv[:, 1] >= -0.01) & (uv[:, 1] < h)
    uv = uv[good]
    if len(uv) == 0:
        return None
    lo = np.maximum(uv.min(axis=0) - stride / 2, 0.0)
    hi = np.minimum(uv.max(axis=0) + stride / 2, [w - 1, h - 1])
    return [float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])]


def render_summary_charts(receipt, output_path: Path):
    """Generate summary comparison figures."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.patch.set_facecolor("#0d1422")
    for row in axes:
        for ax in row:
            ax.set_facecolor("#14223a")
            ax.tick_params(colors="#c5d7ed")
            ax.xaxis.label.set_color("#c5d7ed")
            ax.yaxis.label.set_color("#c5d7ed")
            ax.title.set_color("#e1eaf6")
            for spine in ax.spines.values():
                spine.set_color("#30445f")

    # 1. Proposal Recall
    ax1 = axes[0, 0]
    rec_2d = receipt["proposal_recall"]["2d_proposal"]["recall"] * 100
    rec_3d = receipt["proposal_recall"]["3d_proposal"]["recall"] * 100
    bars = ax1.bar(["2D Proposal (Cũ)", "3D Proposal (Mới)"], [rec_2d, rec_3d], color=["#10b981", "#ef4444"], width=0.5)
    ax1.set_ylabel("Proposal Recall (%)")
    ax1.set_ylim(0, 115)
    ax1.set_title("1. Khả năng bao phủ ứng viên (Proposal Recall)")
    for bar in bars:
        h = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width() / 2, h + 3, f"{h:.1f}%", ha="center", color="white", fontweight="bold")
    ax1.text(0.5, 0.45, "3D bị mất 100% do\nbộ lọc mặt phẳng cứng\neigen[1] < 0.045m",
             transform=ax1.transAxes, ha="center", color="#fca5a5", bbox=dict(facecolor="#450a0a", alpha=0.8, pad=5))

    # 2. Full Detector Precision & Recall
    ax2 = axes[0, 1]
    variants = ["legacy_2d_classifier_050", "generalized_3d_prior_045", "generalized_3d_no_prior_045", "candidate_pointnet_frozen_065"]
    var_labels = ["2D Cũ\n(0.50)", "3D + Prior\n(0.45)", "3D Bỏ Prior\n(0.45)", "Candidate\n(0.65)"]
    recs = [receipt["variant_evaluations"][v]["overall"]["recall"] * 100 for v in variants]
    precs = [receipt["variant_evaluations"][v]["overall"]["precision"] * 100 for v in variants]
    x = np.arange(len(variants))
    w = 0.35
    b_rec = ax2.bar(x - w/2, recs, w, label="Recall", color="#3b82f6")
    b_prec = ax2.bar(x + w/2, precs, w, label="Precision", color="#8b5cf6")
    ax2.set_xticks(x)
    ax2.set_xticklabels(var_labels, fontsize=9)
    ax2.set_ylabel("Tỷ lệ (%)")
    ax2.set_ylim(0, 115)
    ax2.set_title("2. Độ chính xác toàn pipeline (Precision & Recall)")
    ax2.legend(facecolor="#1e293b", edgecolor="#30445f", labelcolor="#e1eaf6")
    for bar in b_rec:
        h = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width() / 2, h + 3, f"{h:.1f}%", ha="center", color="#93c5fd", fontsize=8)
    for bar in b_prec:
        h = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width() / 2, h + 3, f"{h:.1f}%", ha="center", color="#c4b5fd", fontsize=8)

    # 3. False Positives in No-Person Scenes
    ax3 = axes[1, 0]
    fps = [receipt["variant_evaluations"][v]["clutter_no_person_fp"]["fp_count"] for v in variants]
    bars_fp = ax3.bar(var_labels, fps, color=["#f59e0b", "#64748b", "#64748b", "#10b981"], width=0.5)
    ax3.set_ylabel("Số lượt báo nhầm (FP / 40 frame)")
    ax3.set_title("3. Báo nhầm ở cảnh không người (Clutter False Positives)")
    ax3.set_ylim(0, max(fps + [1]) * 1.35)
    for bar in bars_fp:
        h = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width() / 2, h + 0.3, f"{int(h)}/40", ha="center", color="white", fontweight="bold")
    ax3.text(0.85, 0.70, "Candidate đạt 0 FP\nnhưng do ngưỡng quá cao\n(tất cả dự đoán = 0)",
             transform=ax3.transAxes, ha="center", color="#86efac", bbox=dict(facecolor="#064e3b", alpha=0.8, pad=5), fontsize=9)

    # 4. Latency End-to-End
    ax4 = axes[1, 1]
    lats = [receipt["variant_evaluations"][v]["latency"]["mean_ms"] for v in variants]
    bars_lat = ax4.bar(var_labels, lats, color=["#06b6d4", "#ef4444", "#ef4444", "#10b981"], width=0.5)
    ax4.axhline(30.0, color="#f87171", linestyle="--", label="Ngưỡng thời gian thực (30ms)")
    ax4.set_ylabel("Độ trễ trung bình (ms/frame)")
    ax4.set_title("4. Thời gian tính toán end-to-end (Latency)")
    ax4.set_ylim(0, max(lats) * 1.25)
    ax4.legend(facecolor="#1e293b", edgecolor="#30445f", labelcolor="#e1eaf6")
    for bar in bars_lat:
        h = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width() / 2, h + 1, f"{h:.1f}ms", ha="center", color="white", fontweight="bold")

    fig.suptitle("So sánh 4 cấu hình detector trên bộ mini benchmark đa miền có nhãn đối soát", color="#ffffff", fontsize=15, y=0.98)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig.savefig(output_path, dpi=120)
    plt.close(fig)


def render_all_overlays(manifest, data_root: Path, output_dir: Path, receipt):
    """Render multi-panel comparison images for key frames."""
    # Models
    legacy_clf = Classifier(CLASSIFIER_DIR / "artifacts")
    gen_detector_prior = GeneralizedPersonDetector(min_person_score=0.45)
    candidate_model = PointPatchClassifier()
    candidate_model.load_state_dict(torch.load(STAGE / "generated/frozen_candidate_weights.pt", map_location="cpu"))
    candidate_model.eval()

    frames_manifest = {}

    selected_cases = [
        "test_narrow_people",
        "validation_school_people",
        "train_factory_people",
        "train_hospital_people",
        "test_narrow_objects",
        "train_factory_objects"
    ]

    for cid in selected_cases:
        frames_manifest[cid] = []
        cdir = data_root / cid
        depth = np.load(cdir / "depth.npy")
        labels = np.load(cdir / "person_instance_LABEL_ONLY.npz")["person_instance"]
        cal = json.loads((cdir / "calibration.json").read_text())
        k = cal["k"]
        tr = np.array(cal["optical_to_base_link"])

        # Render frames 0, 3, 6, 9
        for t in [0, 3, 6, 9]:
            depth_t = depth[t]
            labels_t = labels[t]
            pts = project_depth(depth_t, k, tr)
            sub = depth_t[::4, ::4]
            valid_depth = np.isfinite(sub) & (sub >= 0.28) & (sub <= 5.0)
            point_labels = labels_t[::4, ::4][valid_depth]

            gt_mask = (labels_t > 0)
            has_gt_person = bool(np.sum(gt_mask) >= 5)

            # 1. Legacy 2D proposals & predictions
            props_2d = old_proposals(pts)
            legacy_dets = []
            cand_dets = []
            for pi, p in enumerate(props_2d):
                score = float(legacy_clf.predict({"points_xyz": p["raw_points"], "scan_valid": False, "scan_ranges": []})["score"])
                bbox = pixel_support(p["raw_points"], cal)
                if bbox is not None:
                    legacy_dets.append({"bbox": bbox, "score": score, "accepted": bool(score >= 0.50), "xy": p["xy"]})

            # Candidate PointNet
            if len(props_2d) > 0:
                pts_t = np.zeros((len(props_2d), 512, 3), dtype=np.float32)
                val_t = np.zeros((len(props_2d), 512), dtype=bool)
                for pi, p in enumerate(props_2d):
                    raw = p["raw_points"]
                    n_p = min(len(raw), 512)
                    pts_t[pi, :n_p] = raw[:n_p]
                    val_t[pi, :n_p] = True
                with torch.no_grad():
                    probs = torch.sigmoid(candidate_model(torch.from_numpy(pts_t), torch.from_numpy(val_t))).cpu().numpy().tolist()
                for pi, p in enumerate(props_2d):
                    score = float(probs[pi])
                    bbox = pixel_support(p["raw_points"], cal)
                    if bbox is not None:
                        cand_dets.append({"bbox": bbox, "score": score, "accepted": bool(score >= 0.65), "xy": p["xy"]})

            # 2. 3D with prior and without prior
            dets_3d = gen_detector_prior.detect(pts)
            prior_dets = []
            noprior_dets = []
            for d in dets_3d:
                bbox = pixel_support(d["raw_points"], cal)
                if bbox is not None:
                    prior_dets.append({"bbox": bbox, "score": float(d["score"]), "accepted": bool(d["is_person"]), "xy": d["xy"]})
                    noprior_dets.append({"bbox": bbox, "score": float(d["base_score"]), "accepted": bool(d["base_score"] >= 0.45), "xy": d["xy"]})

            # Render 5-panel figure
            fig, axes = plt.subplots(1, 5, figsize=(22, 4.8))
            fig.patch.set_facecolor("#0d1422")

            panels = [
                ("1. Ground Truth Tham Chiếu", "gt", "#ef4444", None),
                ("2. 2D Cũ (ngưỡng 0.50)", "legacy", "#3b82f6", legacy_dets),
                ("3. 3D Có Prior (0.45)", "prior", "#f97316", prior_dets),
                ("4. 3D Bỏ Prior (0.45)", "noprior", "#14b8a6", noprior_dets),
                ("5. Candidate PointNet (0.65)", "candidate", "#a855f7", cand_dets)
            ]

            accepted_counts = []

            for ax, (title, mode, color, dets) in zip(axes, panels):
                ax.imshow(np.ma.masked_invalid(depth_t), cmap="viridis", vmin=0.28, vmax=5.0)
                ax.set_title(title, color=color, fontsize=11, fontweight="bold", pad=8)
                ax.set_xticks([])
                ax.set_yticks([])
                for spine in ax.spines.values():
                    spine.set_color("#30445f")

                if mode == "gt":
                    if has_gt_person:
                        # Draw GT overlay contour
                        ax.contour(gt_mask, levels=[0.5], colors=["#ef4444"], linewidths=2.0)
                        # Find GT bbox
                        y_idx, x_idx = np.where(gt_mask)
                        u0, u1 = x_idx.min(), x_idx.max()
                        v0, v1 = y_idx.min(), y_idx.max()
                        rect = Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="#ef4444", linewidth=2.0, linestyle="--")
                        ax.add_patch(rect)
                        ax.text(u0, max(0, v0 - 5), "GT Người Thật", color="white", fontsize=9,
                                bbox=dict(facecolor="#b91c1c", alpha=0.85, pad=3, edgecolor="none"))
                    else:
                        ax.text(0.5, 0.5, "KHÔNG CÓ NGƯỜI (Cảnh Đối Chứng)", transform=ax.transAxes, ha="center", va="center",
                                color="#94a3b8", fontsize=10, bbox=dict(facecolor="#1e293b", alpha=0.9, pad=6))
                else:
                    n_acc = sum(d["accepted"] for d in dets)
                    accepted_counts.append(n_acc)
                    for d in dets:
                        u0, v0, u1, v1 = d["bbox"]
                        if d["accepted"]:
                            rect = Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor=color, linewidth=2.0)
                            ax.add_patch(rect)
                            ax.text(u0, max(0, v0 - 5), f"P {d['score']:.2f}", color="white", fontsize=9,
                                    bbox=dict(facecolor=color, alpha=0.9, pad=3, edgecolor="none"))
                        else:
                            # Rejected proposal (debug dashed)
                            rect = Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False, edgecolor="#64748b", linewidth=1.0, linestyle=":")
                            ax.add_patch(rect)
                    if n_acc == 0:
                        ax.text(0.95, 0.92, "0 PHÁT HIỆN", transform=ax.transAxes, ha="right", color="white", fontsize=9,
                                bbox=dict(facecolor="#991b1b", alpha=0.9, pad=4, edgecolor="none"))

            stamp_sec = t * 0.1
            fig.suptitle(f"{cid} · Frame {t:02d} · t = {stamp_sec:.1f}s · Cùng một ảnh depth cảm biến đo thật", color="#e1eaf6", fontsize=13)
            fig.text(0.5, 0.02, "Khung dự đoán vẽ từ điểm đo thật của proposal/classifier; GT vẽ riêng ở khung ngoài cùng bên trái, không dùng GT để sinh khung detector.", ha="center", color="#94a3b8", fontsize=9)
            fig.tight_layout(rect=[0, 0.05, 1, 0.94])

            fname = f"{cid}_frame_{t:02d}.png"
            fpath = output_dir / fname
            fig.savefig(fpath, dpi=110)
            plt.close(fig)

            frames_manifest[cid].append({
                "file": fname,
                "frame_index": t,
                "stamp_s": stamp_sec,
                "has_gt_person": has_gt_person,
                "counts": accepted_counts
            })

    return frames_manifest


def build_html_report(receipt, frames_manifest, output_dir: Path):
    """Assemble self-contained interactive localhost report."""
    v_eval = receipt["variant_evaluations"]

    # Table 1: Overall Summary
    overall_rows = []
    titles = {
        "legacy_2d_classifier_050": ("2D Cũ (ngưỡng 0.50)", "Baseline handcrafted 30 features"),
        "generalized_3d_prior_045": ("3D Có Prior (0.45)", "3D clustering + planar filter + prior 0.15"),
        "generalized_3d_no_prior_045": ("3D Bỏ Prior (0.45)", "3D clustering + planar filter, không prior"),
        "generalized_3d_no_prior_050": ("3D Bỏ Prior (0.50)", "3D clustering + planar filter control"),
        "candidate_pointnet_frozen_065": ("Candidate PointNet (0.65)", "Proposal 2D + 18k PointNet frozen weights")
    }

    for v, (v_title, desc) in titles.items():
        res = v_eval[v]
        o = res["overall"]
        clutter = res["clutter_no_person_fp"]
        lat = res["latency"]
        gate_st = receipt["gate_status"][v]["overall_status"]
        st_class = "good" if gate_st == "PASS" else "fail"
        overall_rows.append(f"""
        <tr>
          <td><strong>{html.escape(v_title)}</strong><br><span class="small">{html.escape(desc)}</span></td>
          <td>{o['recall']*100:.1f}% ({o['tp']}/{o['tp']+o['fn']})</td>
          <td>{o['precision']*100:.1f}% ({o['tp']}/{o['tp']+o['fp']})</td>
          <td>{o['f1']:.3f}</td>
          <td>{clutter['fp_count']}/{clutter['frames']} ({clutter['fp_rate']*100:.1f}%)</td>
          <td>{lat['mean_ms']:.1f} ± {lat['std_ms']:.1f} ms</td>
          <td><span class="{st_class}">{gate_st}</span></td>
        </tr>
        """)

    # Table 2: Breakdown by Split
    split_rows = []
    for sp in ["train", "validation", "test"]:
        for v in ["legacy_2d_classifier_050", "generalized_3d_prior_045", "generalized_3d_no_prior_045", "candidate_pointnet_frozen_065"]:
            sp_res = v_eval[v]["by_split"][sp]
            split_rows.append(f"""
            <tr>
              <td><code>{sp}</code></td>
              <td>{titles[v][0]}</td>
              <td>{sp_res['recall']*100:.1f}% ({sp_res['tp']}/{sp_res['tp']+sp_res['fn']})</td>
              <td>{sp_res['precision']*100:.1f}%</td>
              <td>{sp_res['f1']:.3f}</td>
              <td>{sp_res['fp']}</td>
            </tr>
            """)

    # Table 3: Breakdown by Body Family
    body_rows = []
    body_info = {
        "A1": "Train Factory (Mesh độc lập A1)",
        "A2": "Train Hospital (Mesh độc lập A2)",
        "B1": "Validation School (Mesh độc lập B1)",
        "C1": "Test Narrow Holdout (Mesh độc lập C1)"
    }
    for b_id, b_label in body_info.items():
        for v in ["legacy_2d_classifier_050", "candidate_pointnet_frozen_065", "generalized_3d_prior_045"]:
            b_res = v_eval[v]["by_body"][b_id]
            body_rows.append(f"""
            <tr>
              <td><strong>{b_id}</strong> — {b_label}</td>
              <td>{titles[v][0]}</td>
              <td>{b_res['recall']*100:.1f}% ({b_res['tp']}/{b_res['tp']+b_res['fn']})</td>
              <td>{b_res['precision']*100:.1f}%</td>
              <td>{b_res['f1']:.3f}</td>
            </tr>
            """)

    # Table 4: Failure modes
    fail_rows = []
    for fail in receipt["typical_failures"]:
        fail_rows.append(f"""
        <tr>
          <td><strong class="fail">{html.escape(fail['failure_type'])}</strong></td>
          <td><code>{html.escape(fail['affected_variant'])}</code></td>
          <td>{html.escape(fail['mechanism'])}</td>
          <td>{html.escape(fail['severity'])}</td>
        </tr>
        """)

    body = r"""<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Báo cáo Bước D — Đối chiếu 4 cấu hình detector và quyết định dừng head</title>
<style>
:root{font-family:system-ui,-apple-system,sans-serif;color:#e1eaf6;background:#0d1422;color-scheme:dark}
body{max-width:1380px;margin:auto;padding:26px;line-height:1.65}
a{color:#8acaff;text-decoration:none}a:hover{text-decoration:underline}
h1{font-size:32px;line-height:1.25;margin:10px 0 16px}
h2{font-size:22px;margin-top:36px;border-bottom:1px solid #30445f;padding-bottom:8px}
p{color:#b7c8dc}
.box{padding:18px;border:1px solid #30445f;border-radius:12px;background:#14223a;margin:18px 0}
.good{color:#82e9b4;font-weight:bold}.warn{color:#ffc28e;font-weight:bold}.fail{color:#ff7a85;font-weight:bold}
img{width:100%;height:auto;background:#080e1a;border-radius:8px;border:1px solid #30445f}
table{border-collapse:collapse;width:100%;font-size:13px;margin:14px 0}
th,td{padding:10px 12px;border-bottom:1px solid #304159;text-align:left}
th{color:#9ed6f1;background:#16253a}
tr:hover{background:#182842}
select,button,input{padding:8px 12px;background:#20334d;border:1px solid #46607d;border-radius:7px;color:#e7f0fb;font:inherit}
input[type=range]{width:35%;vertical-align:middle}
.scroll{overflow:auto}
.small{font-size:12px;color:#9cb2cc}
code{background:#23334c;padding:2px 6px;border-radius:4px;color:#b6dfff}
pre{white-space:pre-wrap;font-size:12px;max-height:350px;overflow:auto;background:#101a2c;padding:14px;border-radius:8px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}
.badge{display:inline-block;padding:3px 8px;border-radius:6px;font-size:12px;background:#1e3352;color:#92cbf8}
@media(max-width:900px){body{padding:14px}.grid{display:block}h1{font-size:26px}}
</style>
<div class="badge">Bước D · Nhánh feat/depth-detector-generalization · Nghiệm thu dứt điểm</div>
<h1>Báo cáo kết quả Bước D — So sánh 4 cấu hình detector và quyết định dừng head</h1>

<div class="box">
  <h3 style="margin-top:0;color:#fca5a5">KẾT LUẬN VÀ QUYẾT ĐỊNH DỨT ĐIỂM:</h3>
  <ul>
    <li><strong>GIỮ LẠI:</strong> Proposal hình học 2D cũ (đạt <strong>100.0% Proposal Recall</strong> trên toàn bộ 40 ca có người ở các hình thái A1/A2/B1/C1); giữ giao thức nhãn pixel ownership tách biệt; giữ sensor calibration và causal odometry.</li>
    <li><strong>LOẠI BỎ (CHƯA ĐƯA VÀO DEFAULT):</strong> 
      <ol>
        <li><strong>Candidate PointPatchClassifier:</strong> Bị hiện tượng <em>zero-recall collapse</em> ở validation và test do ngưỡng tối ưu validation chọn &tau; = 0.65 để triệt tiêu FP; trên morphology mới B1/C1 điểm chỉ đạt ~0.53 &rarr; trượt gate Recall &ge; 80% (đạt 0.0%).</li>
        <li><strong>Clustering và Biometric Prior 3D:</strong> Bị bộ lọc mặt phẳng cứng <code>eigen[1] < 0.045m</code> và prior nhân 0.15 triệt tiêu toàn bộ 40/40 lượt ứng viên người; trượt toàn bộ các gate.</li>
      </ol>
    </li>
    <li><strong>RANH GIỚI HỆ THỐNG:</strong> DỪNG VÒNG PHÁT TRIỂN DETECTOR NHỎ TẠI ĐÂY. <strong>Tuyệt đối không tự ý tích hợp head candidate vào mặc định của ROS 2 / Nav2; KHÔNG bắt đầu huấn luyện World Model (WM) hoặc Reinforcement Learning (RL)</strong>.</li>
  </ul>
</div>

<h2>1. Trực quan hóa trực tiếp: Đối chiếu 4 cấu hình trên ảnh Depth thật</h2>
<p>Chọn ca kiểm thử và frame để xem khung nhận dạng trực tiếp. <em>Khung bên trái là Ground Truth độc lập (không dùng GT để vẽ khung detector). Bốn khung bên phải là output thực tế của từng thuật toán.</em></p>

<div style="margin:14px 0">
  <label for="case_select"><strong>Chọn ca:</strong></label>
  <select id="case_select">
    <option value="test_narrow_people">Test: Hành lang hẹp có người (Narrow Holdout - Body C1)</option>
    <option value="validation_school_people">Validation: Ngách trường học có người (School Alcove - Body B1)</option>
    <option value="train_factory_people">Train: Nhà xưởng có người (Factory Wide - Body A1)</option>
    <option value="train_hospital_people">Train: Hành lang bệnh viện có người (Hospital Corridor - Body A2)</option>
    <option value="test_narrow_objects">Test: Đối chứng vật cản / không người (Narrow Objects)</option>
    <option value="train_factory_objects">Train: Đối chứng vật cản / không người (Factory Objects)</option>
  </select>
  &nbsp;
  <button id="prev_btn">&larr; Trước</button>
  <input id="frame_slider" type="range" min="0" max="3" value="0" aria-label="Chọn frame">
  <button id="next_btn">Sau &rarr;</button>
  &nbsp;
  <span id="frame_label" class="badge"></span>
</div>

<a id="img_anchor" target="_blank" rel="noopener">
  <img id="main_image" alt="Overlay đối chiếu 5 khung">
</a>
<p id="img_caption" class="small"></p>

<h2>2. Biểu đồ tổng hợp các chỉ số kiểm chứng</h2>
<img src="summary_chart.png" alt="Biểu đồ so sánh Proposal Recall, Precision/Recall, Clutter FP và Latency">

<h2>3. Bảng tổng hợp đối chiếu toàn diện (Overall Benchmark Metrics)</h2>
<div class="scroll">
  <table>
    <thead>
      <tr>
        <th>Cấu hình</th>
        <th>Recall toàn bộ</th>
        <th>Precision toàn bộ</th>
        <th>F1-Score</th>
        <th>Báo nhầm cảnh không người (FP)</th>
        <th>Độ trễ xử lý (Latency)</th>
        <th>Operational Gate</th>
      </tr>
    </thead>
    <tbody>
      __OVERALL_ROWS__
    </tbody>
  </table>
</div>
<p class="small">Điều kiện đo: 8 ca mini-benchmark độc lập (80 frames); khoảng cách camera z = 0.08 m; tầm hợp lệ [0.28, 5.0] m. Đánh giá kiểm chứng thực hiện đúng 1 lần với model weights và threshold đã freeze.</p>

<h2>4. Phân tầng kết quả (Stratified Analysis)</h2>
<div class="grid">
  <div>
    <h3>A. Phân theo Split dữ liệu (Train vs Val vs Test)</h3>
    <div class="scroll">
      <table>
        <thead>
          <tr><th>Split</th><th>Cấu hình</th><th>Recall</th><th>Precision</th><th>F1</th><th>FP</th></tr>
        </thead>
        <tbody>
          __SPLIT_ROWS__
        </tbody>
      </table>
    </div>
  </div>
  <div>
    <h3>B. Phân theo Hình thái Người (Body Family A1, A2, B1, C1)</h3>
    <div class="scroll">
      <table>
        <thead>
          <tr><th>Body Family</th><th>Cấu hình</th><th>Recall</th><th>Precision</th><th>F1</th></tr>
        </thead>
        <tbody>
          __BODY_ROWS__
        </tbody>
      </table>
    </div>
  </div>
</div>

<h2>5. Phân tích các lỗi điển hình (Failure Modes Taxonomy)</h2>
<div class="scroll">
  <table>
    <thead>
      <tr>
        <th>Dạng lỗi</th>
        <th>Cấu hình bị ảnh hưởng</th>
        <th>Cơ chế phát sinh lỗi</th>
        <th>Mức độ nghiêm trọng</th>
      </tr>
    </thead>
    <tbody>
      __FAIL_ROWS__
    </tbody>
  </table>
</div>

<h2>6. Đề xuất Ngưỡng nghiệm thu Semantic Test mới (Đã chốt trước test)</h2>
<div class="box">
  <h3 style="margin-top:0">BÁC BỎ TIÊU CHÍ CŨ & ĐỀ XUẤT NGƯỠNG NGHIỆM THU MỚI:</h3>
  <p><strong>Vì sao bác bỏ paired-retention gate cũ (&ge; 95% paired matches baseline):</strong> Tiêu chuẩn cũ chỉ đo khoảng cách tâm hình học Euclidean &le; 0.60 m tới hình trụ mô phỏng trong 1 warehouse duy nhất. Tiêu chuẩn này che giấu hoàn toàn việc detector bị mù trước hình thái mới (B1, C1) hoặc bắt nhầm vách tường/chân bàn cạnh người. Đạt paired-retention không đồng nghĩa với đạt độ tin cậy ngữ nghĩa.</p>
  <p><strong>Ngưỡng Semantic Vận hành đề xuất (Operational Gates):</strong></p>
  <ul>
    <li><strong>Proposal Recall &ge; 90.0%:</strong> Đảm bảo tầng tiền đề không bỏ sót người trước khi vào classifier. (2D đạt 100%, 3D FAIL 0%).</li>
    <li><strong>Full-detector Recall &ge; 80.0%:</strong> Đảm bảo an toàn không đâm va người đứng hoặc đi bộ. (Tất cả 4 cấu hình FAIL trên tập test C1).</li>
    <li><strong>Full-detector Precision &ge; 80.0%:</strong> Tránh phanh khẩn cấp giả làm gián đoạn chuyển động robot.</li>
    <li><strong>No-human Scene False Positives = 0 FP / 100 frame:</strong> Triệt tiêu báo nhầm trên đồ vật và vách hẹp. (Candidate PASS 0 FP; 2D cũ FAIL 10 FP).</li>
    <li><strong>Độ trễ End-to-End &le; 30.0 ms:</strong> Đảm bảo tần số vòng điều khiển &ge; 30 Hz. (Candidate PASS 4.4 ms; 2D cũ PASS 3.4 ms; 3D FAIL 38.2 ms).</li>
  </ul>
</div>

<h2>7. Receipts và Bằng chứng truy vết</h2>
<p>
  <a href="step_d_evaluation_receipt.json">Biên bản đánh giá Bước D (JSON)</a> ·
  <a href="model_freeze.json">Model Freeze Receipt</a> ·
  <a href="test_evaluation_receipt.json">Test Evaluation Receipt</a> ·
  <a href="training_curation_receipt.json">Training History & Curation</a> ·
  <a href="benchmark_manifest_collected.json">Collected Benchmark Manifest</a> ·
  <a href="benchmark_qa_receipt.json">Benchmark QA Verification</a> ·
  <a href="media_receipt.json">Media Checksums</a> ·
  <a href="summary_chart.png">Biểu đồ tổng hợp</a>
</p>

<script>
const framesData = __FRAMES_DATA__;
const caseSelect = document.getElementById('case_select');
const slider = document.getElementById('frame_slider');
const prevBtn = document.getElementById('prev_btn');
const nextBtn = document.getElementById('next_btn');
const img = document.getElementById('main_image');
const anchor = document.getElementById('img_anchor');
const label = document.getElementById('frame_label');
const caption = document.getElementById('img_caption');

function updateView() {
  const cid = caseSelect.value;
  const list = framesData[cid] || [];
  slider.max = Math.max(0, list.length - 1);
  const idx = Math.min(slider.value, slider.max);
  slider.value = idx;
  const item = list[idx];
  if (item) {
    img.src = item.file;
    anchor.href = item.file;
    label.textContent = 'Frame ' + item.frame_index + ' (' + item.stamp_s.toFixed(1) + 's)';
    caption.textContent = cid + ' · Frame ' + item.frame_index + ' · Có người: ' + (item.has_gt_person ? 'CÓ (GT)' : 'KHÔNG') + ' · Số phát hiện chấp nhận [2D cũ: ' + item.counts[0] + ', 3D prior: ' + item.counts[1] + ', 3D no-prior: ' + item.counts[2] + ', Candidate: ' + item.counts[3] + ']';
  }
}

caseSelect.addEventListener('change', () => { slider.value = 0; updateView(); });
slider.addEventListener('input', updateView);
prevBtn.onclick = () => { slider.value = Math.max(0, +slider.value - 1); updateView(); };
nextBtn.onclick = () => { slider.value = Math.min(+slider.max, +slider.value + 1); updateView(); };
updateView();
</script>
</html>"""

    body = (body.replace("__OVERALL_ROWS__", "".join(overall_rows))
                .replace("__SPLIT_ROWS__", "".join(split_rows))
                .replace("__BODY_ROWS__", "".join(body_rows))
                .replace("__FAIL_ROWS__", "".join(fail_rows))
                .replace("__FRAMES_DATA__", json.dumps(frames_manifest)))

    (output_dir / "index.html").write_text(body, encoding="utf-8")
    print(f"HTML report successfully written to: {output_dir / 'index.html'}")


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Rendering Step D Decision Report into: {OUTPUT_DIR}")

    receipt_path = STAGE / "generated/step_d_evaluation_receipt.json"
    if not receipt_path.exists():
        raise FileNotFoundError(f"Receipt not found: {receipt_path}. Run evaluate_step_d.py first.")
    receipt = json.loads(receipt_path.read_text())

    manifest_path = STAGE / "benchmark_manifest_collected.json"
    manifest = json.loads(manifest_path.read_text())

    # 1. Render Summary Chart
    chart_path = OUTPUT_DIR / "summary_chart.png"
    print("Generating summary chart...")
    render_summary_charts(receipt, chart_path)

    # 2. Render Frame Overlays
    print("Generating comparative frame overlays...")
    frames_manifest = render_all_overlays(manifest, DATA_ROOT, OUTPUT_DIR, receipt)

    # 3. Assemble HTML
    print("Building HTML report...")
    build_html_report(receipt, frames_manifest, OUTPUT_DIR)

    # 4. Copy receipts
    for f_name in [
        "step_d_evaluation_receipt.json",
        "model_freeze.json",
        "test_evaluation_receipt.json",
        "training_curation_receipt.json",
        "benchmark_qa_receipt.json"
    ]:
        src = STAGE / "generated" / f_name
        if src.exists():
            shutil.copyfile(src, OUTPUT_DIR / f_name)

    for f_name in ["benchmark_manifest_collected.json", "candidate_decision.json", "training_recipe.json"]:
        src = STAGE / f_name
        if src.exists():
            shutil.copyfile(src, OUTPUT_DIR / f_name)

    # Media receipt
    media_files = {p.name: sha256_file(p) for p in OUTPUT_DIR.glob("*.png")}
    (OUTPUT_DIR / "media_receipt.json").write_text(json.dumps({
        "schema": "depth_person_detector_step_d_media/1",
        "date": "2026-10-31",
        "total_images": len(media_files),
        "assets": media_files
    }, indent=2) + "\n")

    print(f"Step D report generation complete in: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()

