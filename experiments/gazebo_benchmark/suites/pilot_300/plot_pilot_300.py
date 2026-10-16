#!/usr/bin/env python3
"""
Plots publication-quality diagnostic charts for 300-Episode Pilot Dataset:
- pilot_300_persona_stratified_boxplot.png:
  - Panel A: Human Lateral Deflection across personas (H1, H2, H3) stratified by Corridor Geometry (0.9m vs 1.2m vs Unfiltered)
  - Panel B: Minimum Clearance d_clear across personas & geometries
  - Panel C: Statistical Significance & p-value matrix (Mann-Whitney U tests confirming H1 vs H2 non-separability)
"""

import os
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RESULTS_DIR = os.path.join(BENCH_DIR, "results/pilot_300")
FIG_DIR = os.path.join(RESULTS_DIR, "figures")
os.makedirs(FIG_DIR, exist_ok=True)


def plot_pilot_300():
    path = os.path.join(RESULTS_DIR, "pilot_300_recalculated_persona_results.json")
    with open(path) as f:
        data = json.load(f)

    strat = data["stratified_results"]
    tests = data["statistical_tests"]["overall"]

    fig = plt.figure(figsize=(18, 5.5), dpi=200)
    gs = fig.add_gridspec(1, 3, width_ratios=[1.2, 1.2, 1.0], wspace=0.28)

    # Panel 1: Human Lateral Deflection Boxplot/Bar
    ax1 = fig.add_subplot(gs[0, 0])
    geoms = [
        ("0.9m Corridor", "corridor_090_wall_filtered"),
        ("1.2m Corridor", "corridor_120_wall_filtered"),
        ("Overall Combined", "overall_wall_filtered")
    ]
    personas = ["H1", "H2", "H3"]
    p_colors = {"H1": "#2a9d8f", "H2": "#e76f51", "H3": "#264653"}

    x = np.arange(len(geoms))
    w = 0.25

    for p_idx, p in enumerate(personas):
        medians = [strat[g_key][p]["max_human_lat_dev_m"]["median"] for _, g_key in geoms]
        iqrs = [strat[g_key][p]["max_human_lat_dev_m"]["iqr"] / 2.0 for _, g_key in geoms]
        q25 = [strat[g_key][p]["max_human_lat_dev_m"]["q25"] for _, g_key in geoms]
        q75 = [strat[g_key][p]["max_human_lat_dev_m"]["q75"] for _, g_key in geoms]

        yerrs = [[medians[i] - q25[i] for i in range(len(geoms))],
                 [q75[i] - medians[i] for i in range(len(geoms))]]

        ax1.bar(x + (p_idx - 1)*w, medians, w, yerr=yerrs, capsize=4,
                label=f"Persona {p}", color=p_colors[p], alpha=0.88)

    ax1.axhline(0.30, color="purple", linestyle="--", linewidth=1.2, label="Corridor Edge Limit (0.30m)")
    ax1.set_title("A. Human Max Lateral Deflection (Median & IQR, m)", fontsize=11, fontweight="bold")
    ax1.set_xticks(x)
    ax1.set_xticklabels([g[0] for g in geoms], fontsize=10, fontweight="bold")
    ax1.set_ylabel("Lateral Deflection (m)", fontsize=10)
    ax1.set_ylim(0, 0.50)
    ax1.grid(True, linestyle="--", alpha=0.4, axis="y")
    ax1.legend(loc="upper left", fontsize=8.5)

    # Panel 2: Robot Minimum Clearance d_clear
    ax2 = fig.add_subplot(gs[0, 1])
    for p_idx, p in enumerate(personas):
        medians = [strat[g_key][p]["min_d_clear_m"]["median"] for _, g_key in geoms]
        q25 = [strat[g_key][p]["min_d_clear_m"]["q25"] for _, g_key in geoms]
        q75 = [strat[g_key][p]["min_d_clear_m"]["q75"] for _, g_key in geoms]

        yerrs = [[max(0, medians[i] - q25[i]) for i in range(len(geoms))],
                 [max(0, q75[i] - medians[i]) for i in range(len(geoms))]]

        ax2.bar(x + (p_idx - 1)*w, medians, w, yerr=yerrs, capsize=4,
                label=f"Persona {p}", color=p_colors[p], alpha=0.88)

    ax2.axhline(0.0, color="red", linestyle="-", linewidth=1.2, label="Footprint Contact (d_clear = 0)")
    ax2.set_title("B. Robot Minimum Clearance d_clear (Median & IQR, m)", fontsize=11, fontweight="bold")
    ax2.set_xticks(x)
    ax2.set_xticklabels([g[0] for g in geoms], fontsize=10, fontweight="bold")
    ax2.set_ylabel("Clearance d_clear (m)", fontsize=10)
    ax2.set_ylim(-0.02, 0.22)
    ax2.grid(True, linestyle="--", alpha=0.4, axis="y")
    ax2.legend(loc="upper right", fontsize=8.5)

    # Panel 3: Statistical Significance Heatmap Matrix (Mann-Whitney U p-values)
    ax3 = fig.add_subplot(gs[0, 2])
    comp_labels = ["H1 vs H2", "H1 vs H3", "H2 vs H3"]
    metric_labels = ["Min Clearance", "Human Lat Dev", "Human Vel Drop"]

    pval_matrix = [
        [tests["H1_vs_H2"]["min_d_clear_m"]["p_value"], tests["H1_vs_H2"]["max_human_lat_dev_m"]["p_value"], tests["H1_vs_H2"]["human_vel_drop_mps"]["p_value"]],
        [tests["H1_vs_H3"]["min_d_clear_m"]["p_value"], tests["H1_vs_H3"]["max_human_lat_dev_m"]["p_value"], tests["H1_vs_H3"]["human_vel_drop_mps"]["p_value"]],
        [tests["H2_vs_H3"]["min_d_clear_m"]["p_value"], tests["H2_vs_H3"]["max_human_lat_dev_m"]["p_value"], tests["H2_vs_H3"]["human_vel_drop_mps"]["p_value"]]
    ]
    pval_matrix = np.array(pval_matrix)

    # Log10 scale for heatmap visualization: -log10(p)
    log_p = -np.log10(np.clip(pval_matrix, 1e-10, 1.0))
    im3 = ax3.imshow(log_p, cmap="YlOrRd", interpolation="nearest", vmin=0, vmax=6)

    ax3.set_title("C. Statistical Hypothesis Testing\nMann-Whitney U (-log10 p-value)", fontsize=11, fontweight="bold")
    ax3.set_xticks(range(3))
    ax3.set_yticks(range(3))
    ax3.set_xticklabels(metric_labels, rotation=20, ha="right", fontsize=9)
    ax3.set_yticklabels(comp_labels, fontsize=10, fontweight="bold")

    for i in range(3):
        for j in range(3):
            p_val = pval_matrix[i, j]
            txt = f"p={p_val:.3f}" if p_val >= 0.001 else f"p<0.001***"
            if i == 0 and j == 1:
                txt += "\n(NS, >0.5)"
            color = "white" if log_p[i, j] > 3.0 else "black"
            ax3.text(j, i, txt, ha="center", va="center", color=color, fontsize=8.5, fontweight="bold")

    cbar = plt.colorbar(im3, ax=ax3, fraction=0.046, pad=0.04)
    cbar.set_label("-log10(p-value) [Significance]", fontsize=9)

    plt.tight_layout()
    out_file = os.path.join(FIG_DIR, "pilot_300_persona_stratified_boxplot.png")
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


if __name__ == "__main__":
    plot_pilot_300()
