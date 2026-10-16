#!/usr/bin/env python3
"""
Plots publication-quality diagnostic charts for Parallel Scaling, Throughput, and Sensor Performance:
1. parallel_throughput_scaling.png:
   - Aggregate RTF & Throughput (episodes/hour) across 1 to 4 instances (5 vs 10 humans, with/without Nav2)
   - Hardware resource usage (CPU %, GPU %, VRAM MB)
2. nav2_controller_frequency_ablation.png:
   - Stage 1 Baseline vs Stage 2 Parameter Configurations vs Stage 3 Parallel (4 instances)
   - Effective controller frequency (Hz) and real-time factor
3. depth_camera_noise_calibration.png:
   - Depth noise variance before vs. after calibration (spatial, temporal, pooled std dev in mm)
"""

import os
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RESULTS_DIR = os.path.join(BENCH_DIR, "results/scaling_perf")
FIG_DIR = os.path.join(RESULTS_DIR, "figures")
os.makedirs(FIG_DIR, exist_ok=True)


def plot_throughput_scaling():
    path = os.path.join(RESULTS_DIR, "a4_throughput_results.json")
    with open(path) as f:
        data = json.load(f)

    # 5 humans with nav2 (1, 2, 3, 4 instances)
    inst_x = [1, 2, 3, 4]
    rtf_5h_nav2 = [data[f"inst_{i}_humans_5_with_nav2"]["mean_total_rtf"] for i in inst_x]
    ep_5h_nav2 = [data[f"inst_{i}_humans_5_with_nav2"]["episodes_per_hour"] for i in inst_x]
    cpu_5h_nav2 = [data[f"inst_{i}_humans_5_with_nav2"]["mean_cpu_percent"] for i in inst_x]
    vram_5h_nav2 = [data[f"inst_{i}_humans_5_with_nav2"]["mean_vram_mb"] for i in inst_x]

    # Reference points
    rtf_10h_1 = data["inst_1_humans_10_with_nav2"]["mean_total_rtf"]
    rtf_10h_4 = data["inst_4_humans_10_with_nav2"]["mean_total_rtf"]
    ep_10h_1 = data["inst_1_humans_10_with_nav2"]["episodes_per_hour"]
    ep_10h_4 = data["inst_4_humans_10_with_nav2"]["episodes_per_hour"]

    rtf_nonav_1 = data["inst_1_humans_5_no_nav2"]["mean_total_rtf"]
    rtf_nonav_4 = data["inst_4_humans_5_no_nav2"]["mean_total_rtf"]
    ep_nonav_1 = data["inst_1_humans_5_no_nav2"]["episodes_per_hour"]
    ep_nonav_4 = data["inst_4_humans_5_no_nav2"]["episodes_per_hour"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5.5), dpi=200)

    # Panel 1: Throughput & Aggregate RTF
    ax1_twin = ax1.twinx()
    l1 = ax1.plot(inst_x, rtf_5h_nav2, "o-", color="#1d3557", linewidth=2.5, markersize=8, label="Total RTF (5 Humans + Nav2)")
    l2 = ax1.plot([1, 4], [rtf_10h_1, rtf_10h_4], "s--", color="#e76f51", linewidth=2.0, markersize=7, label="Total RTF (10 Humans + Nav2)")
    l3 = ax1.plot([1, 4], [rtf_nonav_1, rtf_nonav_4], "^-.", color="#2a9d8f", linewidth=2.0, markersize=7, label="Total RTF (5 Humans, Sim-Only)")

    b1 = ax1_twin.bar(np.array(inst_x) + 0.15, ep_5h_nav2, 0.3, color="#457b9d", alpha=0.35, label="Throughput (Episodes/Hour, 5H+Nav2)")

    ax1.set_title("A. Parallel Scaling & Throughput (1 to 4 Instances)", fontsize=11, fontweight="bold")
    ax1.set_xlabel("Number of Concurrent Instances", fontsize=10, fontweight="bold")
    ax1.set_ylabel("Total Aggregate Real-Time Factor (RTF)", fontsize=10, color="#1d3557")
    ax1_twin.set_ylabel("Throughput (Episodes / Hour)", fontsize=10, color="#457b9d")
    ax1.set_xticks(inst_x)
    ax1.set_ylim(0, 9.0)
    ax1_twin.set_ylim(0, 750)
    ax1.grid(True, linestyle="--", alpha=0.4)

    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_twin.get_legend_handles_labels()
    ax1.legend(lines + lines2, labels + labels2, loc="upper left", fontsize=8.5)

    # Panel 2: Resource Utilization (CPU & VRAM)
    ax2_twin = ax2.twinx()
    l_cpu = ax2.plot(inst_x, cpu_5h_nav2, "D-", color="#e63946", linewidth=2.5, markersize=8, label="Host CPU Total (%)")
    l_vram = ax2_twin.plot(inst_x, vram_5h_nav2, "o-", color="#7209b7", linewidth=2.5, markersize=8, label="GPU VRAM Usage (MB)")

    ax2.set_title("B. Hardware Resource Scaling (Ryzen 7 7735H + RTX 4060)", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Number of Concurrent Instances", fontsize=10, fontweight="bold")
    ax2.set_ylabel("Total CPU Load (%)", fontsize=10, color="#e63946")
    ax2_twin.set_ylabel("GPU VRAM Usage (MB)", fontsize=10, color="#7209b7")
    ax2.set_xticks(inst_x)
    ax2.set_ylim(0, 100)
    ax2_twin.set_ylim(1000, 4500)
    ax2.grid(True, linestyle="--", alpha=0.4)

    lines_a, labels_a = ax2.get_legend_handles_labels()
    lines_b, labels_b = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines_a + lines_b, labels_a + labels_b, loc="center left", fontsize=8.5)

    plt.tight_layout()
    out_file = os.path.join(FIG_DIR, "parallel_throughput_scaling.png")
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


def plot_controller_frequency():
    path = os.path.join(RESULTS_DIR, "nav2_controller_frequency_benchmark_results.json")
    with open(path) as f:
        data = json.load(f)

    # Stages:
    # 1. Baseline 1 Inst (RTF 1.0)
    # 2. CFG1 (Circular radius)
    # 3. CFG2 (Costmap 3m)
    # 4. CFG3 (5 critics)
    # 5. CFG4 (Batch 150)
    # 6. CFG5 (Freq 50Hz)
    # 7. Parallel 4 Inst Baseline
    items = [
        ("Base 1-Inst\n(Footprint)", data["stage1_baseline_1instance"]["rtf_1.0"]["sim_hz"], data["stage1_baseline_1instance"]["rtf_1.0"]["wall_hz"], data["stage1_baseline_1instance"]["rtf_1.0"]["measured_rtf"]),
        ("CFG1\n(Circ Radius)", data["stage2_ablations_1instance"]["cfg1_circular_radius"]["sim_hz"], data["stage2_ablations_1instance"]["cfg1_circular_radius"]["wall_hz"], data["stage2_ablations_1instance"]["cfg1_circular_radius"]["measured_rtf"]),
        ("CFG2\n(Costmap 3m)", data["stage2_ablations_1instance"]["cfg2_costmap_3m"]["sim_hz"], data["stage2_ablations_1instance"]["cfg2_costmap_3m"]["wall_hz"], data["stage2_ablations_1instance"]["cfg2_costmap_3m"]["measured_rtf"]),
        ("CFG3\n(5 Critics)", data["stage2_ablations_1instance"]["cfg3_critics_5"]["sim_hz"], data["stage2_ablations_1instance"]["cfg3_critics_5"]["wall_hz"], data["stage2_ablations_1instance"]["cfg3_critics_5"]["measured_rtf"]),
        ("CFG4\n(Batch 150)", data["stage2_ablations_1instance"]["cfg4_batch150"]["sim_hz"], data["stage2_ablations_1instance"]["cfg4_batch150"]["wall_hz"], data["stage2_ablations_1instance"]["cfg4_batch150"]["measured_rtf"]),
        ("CFG5\n(Target 50Hz)", data["stage2_ablations_1instance"]["cfg5_freq50hz"]["sim_hz"], data["stage2_ablations_1instance"]["cfg5_freq50hz"]["wall_hz"], data["stage2_ablations_1instance"]["cfg5_freq50hz"]["measured_rtf"]),
        ("Parallel 4-Inst\n(Base per inst)", data["stage3_parallel_4instances"]["rtf_1.0_baseline_params"]["mean_sim_hz"], data["stage3_parallel_4instances"]["rtf_1.0_baseline_params"]["mean_wall_hz"], data["stage3_parallel_4instances"]["rtf_1.0_baseline_params"]["mean_measured_rtf"]),
    ]

    labels = [it[0] for it in items]
    sim_hz = [it[1] for it in items]
    wall_hz = [it[2] for it in items]
    rtfs = [it[3] for it in items]

    fig, ax = plt.subplots(figsize=(14, 5.5), dpi=200)

    x = np.arange(len(items))
    w = 0.32

    ax.bar(x - w/2, sim_hz, w, label="Sim-Time Loop Frequency (Hz)", color="#3a86ff", alpha=0.85)
    ax.bar(x + w/2, wall_hz, w, label="Wall-Clock Published Frequency (Hz)", color="#ff006e", alpha=0.85)

    ax.axhline(20.0, color="green", linestyle="--", linewidth=1.8, label="Nav2 MPPI Nominal Target (20 Hz)")
    ax.axhline(10.0, color="red", linestyle=":", linewidth=1.5, label="Minimum Real-Time Threshold (10 Hz)")

    # Secondary twin axis for measured RTF
    ax_twin = ax.twinx()
    ax_twin.plot(x, rtfs, "D-", color="#ffbe0b", linewidth=2.5, markersize=8, label="Measured Real-Time Factor (RTF)")
    ax_twin.set_ylabel("Measured RTF", fontsize=10, color="#b58300")
    ax_twin.set_ylim(0.7, 1.15)

    ax.set_title("Nav2 MPPI Controller Frequency & Parameter Ablation Benchmark", fontsize=12, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9.5)
    ax.set_ylabel("Controller Frequency (Hz)", fontsize=10, fontweight="bold")
    ax.set_ylim(0, 30)
    ax.grid(True, linestyle="--", alpha=0.4, axis="y")

    # Combine legends
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax_twin.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8.5)

    plt.tight_layout()
    out_file = os.path.join(FIG_DIR, "nav2_controller_frequency_ablation.png")
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


def plot_depth_noise():
    path = os.path.join(RESULTS_DIR, "depth_noise_benchmark_results.json")
    with open(path) as f:
        data = json.load(f)["comparison"]

    before = data["before_disabling_noise"]
    after = data["after_disabling_noise"]

    categories = ["Spatial Std Dev", "Temporal Std Dev", "Pooled Std Dev"]
    vals_before = [before["spatial_std_dev_mm"], before["temporal_std_dev_mm"], before["pooled_std_dev_mm"]]
    vals_after = [after["spatial_std_dev_mm"], after["temporal_std_dev_mm"], after["pooled_std_dev_mm"]]

    fig, ax = plt.subplots(figsize=(9, 5.0), dpi=200)

    x = np.arange(len(categories))
    w = 0.35

    b1 = ax.bar(x - w/2, vals_before, w, label="Before: Uncalibrated Gaussian Noise (σ = 7 mm)", color="#e63946", alpha=0.88)
    b2 = ax.bar(x + w/2, vals_after, w, label="After: Clean Ground-Truth Pass (σ < 0.001 mm)", color="#2a9d8f", alpha=0.88)

    ax.set_title("Depth Camera Sensor Noise Removal Verification (Flat Wall @ 2.0m)", fontsize=11, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(categories, fontsize=10, fontweight="bold")
    ax.set_ylabel("Measured Standard Deviation (mm)", fontsize=10, fontweight="bold")
    ax.set_ylim(0, 8.5)
    ax.grid(True, linestyle="--", alpha=0.4, axis="y")

    for i in range(len(categories)):
        ax.text(x[i] - w/2, vals_before[i] + 0.2, f"{vals_before[i]:.3f} mm", ha="center", va="bottom", fontsize=9, fontweight="bold")
        ax.text(x[i] + w/2, vals_after[i] + 0.2, f"{vals_after[i]:.4f} mm", ha="center", va="bottom", fontsize=9, fontweight="bold")

    ax.legend(loc="upper right", fontsize=9)

    plt.tight_layout()
    out_file = os.path.join(FIG_DIR, "depth_camera_noise_calibration.png")
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


if __name__ == "__main__":
    plot_throughput_scaling()
    plot_controller_frequency()
    plot_depth_noise()
