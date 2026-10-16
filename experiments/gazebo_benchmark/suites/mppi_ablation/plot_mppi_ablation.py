#!/usr/bin/env python3
"""
Plots publication-quality diagnostic charts for MPPI Controller Ablation and Wall Clearance:
1. mppi_ablation_progression.png:
   - Step 0 (Baseline) -> Step 1 (8m Vision) -> Step 2a/2b (Inflation/Cost) -> Step 3a/3b/3c (Horizon)
   - Goal reach rate, collision rate, and controller frequency showing why Step 3b is optimal.
2. wall_clearance_025_vs_030.png:
   - Impact of pedestrian wall clearance margin (0.25m vs 0.30m) in 0.9m and 1.2m corridors.
"""

import os
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RESULTS_DIR = os.path.join(BENCH_DIR, "results/mppi_ablation")
FIG_DIR = os.path.join(RESULTS_DIR, "figures")
os.makedirs(FIG_DIR, exist_ok=True)


def plot_progression():
    steps = [
        ("Step 0\n(Base)", os.path.join(RESULTS_DIR, "a3_3_step1_vision_results.json"), "baseline_summary_step0", 50, 2.5),
        ("Step 1\n(Vision 8m)", os.path.join(RESULTS_DIR, "a3_3_step1_vision_results.json"), "summary", 50, 2.5),
        ("Step 2a\n(Inf 0.30)", os.path.join(RESULTS_DIR, "a3_3_step2a_results.json"), "summary", 50, 2.5),
        ("Step 2b\n(Inf 0.20)", os.path.join(RESULTS_DIR, "a3_3_step2b_results.json"), "summary", 50, 2.5),
        ("Step 3a\n(N=70, 3.5s)", os.path.join(RESULTS_DIR, "a3_3_step3a_results.json"), "summary", 70, 3.5),
        ("Step 3b\n(N=80, 4.0s)*", os.path.join(RESULTS_DIR, "a3_3_step3b_results.json"), "summary", 80, 4.0),
        ("Step 3c\n(N=100, 5.0s)", os.path.join(RESULTS_DIR, "a3_3_step3c_results.json"), "summary", 100, 5.0),
    ]

    labels = [s[0] for s in steps]
    c09_h1_goals = []
    c09_h1_colls = []
    c12_h1_goals = []
    c12_h3_colls = []
    ctrl_hz = []
    horizons = [s[3] for s in steps]

    for label, path, key, n_steps, horiz in steps:
        with open(path) as f:
            d = json.load(f)
        s = d[key]
        c09_h1_goals.append(s["corridor_090_H1"]["goal_reach_rate_pct"])
        c09_h1_colls.append(s["corridor_090_H1"]["contact_collision_rate_pct"])
        c12_h1_goals.append(s["corridor_120_H1"]["goal_reach_rate_pct"])
        c12_h3_colls.append(s["corridor_120_H3"]["contact_collision_rate_pct"])
        ctrl_hz.append(s["corridor_120_H1"]["cmd_vel_sim_hz_mean"])

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5), dpi=200)

    # Panel A: 0.9m Narrow Corridor (The Stress Test)
    x = np.arange(len(steps))
    w = 0.35
    axes[0].bar(x - w/2, c09_h1_goals, w, label="Goal Reach Rate (%)", color="#2a9d8f", alpha=0.9)
    axes[0].bar(x + w/2, c09_h1_colls, w, label="Collision Rate (%)", color="#e76f51", alpha=0.9)

    # Highlight Step 3b as optimal
    axes[0].axvspan(4.5, 5.5, color="#52b788", alpha=0.2, label="Optimal Sweetspot (Step 3b)")
    axes[0].set_title("A. Narrow Corridor (0.9m, H1 Encounter)\nResolution of Blind Deadlock & Collision", fontsize=11, fontweight="bold")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels, fontsize=8.5)
    axes[0].set_ylabel("Percentage (%)", fontsize=10)
    axes[0].set_ylim(0, 115)
    axes[0].grid(True, linestyle="--", alpha=0.4, axis="y")
    axes[0].legend(loc="upper left", fontsize=8.5)

    # Panel B: 1.2m Corridor H1 vs H3
    axes[1].plot(x, c12_h1_goals, "o-", color="#1d3557", linewidth=2.5, markersize=8, label="1.2m H1 Goal Reach Rate (%)")
    axes[1].plot(x, c12_h3_colls, "s--", color="#d62828", linewidth=2.0, markersize=7, label="1.2m H3 Collision Rate (%) (Blind Human)")
    axes[1].axvspan(4.5, 5.5, color="#52b788", alpha=0.2, label="Optimal Sweetspot (Step 3b)")
    axes[1].set_title("B. Standard Corridor (1.2m)\nHigh Throughput & Yielding Behavior", fontsize=11, fontweight="bold")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, fontsize=8.5)
    axes[1].set_ylabel("Rate (%)", fontsize=10)
    axes[1].set_ylim(0, 115)
    axes[1].grid(True, linestyle="--", alpha=0.4)
    axes[1].legend(loc="center left", fontsize=8.5)

    # Panel C: Controller Frequency vs Horizon
    ax2_twin = axes[2].twinx()
    b1 = axes[2].bar(x, horizons, 0.4, label="Time Steps N (Planning Horizon)", color="#457b9d", alpha=0.5)
    l1 = ax2_twin.plot(x, ctrl_hz, "D-", color="#d00000", linewidth=2.5, markersize=8, label="Effective Controller Freq (Hz)")
    ax2_twin.axhline(20.0, color="green", linestyle="--", linewidth=1.5, label="Target Nav2 MPPI 20 Hz")

    axes[2].set_title("C. MPPI Real-Time Computation Budget\nControl Rate vs. Prediction Horizon", fontsize=11, fontweight="bold")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, fontsize=8.5)
    axes[2].set_ylabel("Planning Steps N (dt = 0.05s)", fontsize=10, color="#457b9d")
    ax2_twin.set_ylabel("Controller Frequency (Hz)", fontsize=10, color="#d00000")
    axes[2].set_ylim(0, 120)
    ax2_twin.set_ylim(15, 26)
    axes[2].grid(True, linestyle="--", alpha=0.4, axis="y")

    lines1, labels1 = axes[2].get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    axes[2].legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    plt.tight_layout()
    out_file = os.path.join(FIG_DIR, "mppi_ablation_progression.png")
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


def plot_wall_clearance():
    path = os.path.join(RESULTS_DIR, "a3_3_wall_clearance_results.json")
    with open(path) as f:
        data = json.load(f)["summary"]

    scenarios = ["corridor_090_H1", "corridor_090_H3", "corridor_120_H1", "corridor_120_H3"]
    scen_labels = [
        "Corridor 0.9m\n(H1 Coop)", "Corridor 0.9m\n(H3 Blind)",
        "Corridor 1.2m\n(H1 Coop)", "Corridor 1.2m\n(H3 Blind)"
    ]

    colls = [data[s]["contact_collision_rate_pct"] for s in scenarios]
    near_miss = [data[s]["near_miss_under_2cm_rate_pct"] for s in scenarios]
    goals = [data[s]["goal_reach_rate_pct"] for s in scenarios]
    h_clamp = [data[s]["human_clamped_step_pct_mean"] for s in scenarios]
    geom_margins = [data[s]["corridor_clearance_geom_m"] * 100.0 for s in scenarios]  # in cm

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5.5), dpi=200)

    # Panel 1: Safety & Outcome Rates
    x = np.arange(len(scenarios))
    w = 0.25
    ax1.bar(x - w, goals, w, label="Goal Reach Rate (%)", color="#2a9d8f", alpha=0.88)
    ax1.bar(x, colls, w, label="Collision Rate (%)", color="#e63946", alpha=0.88)
    ax1.bar(x + w, near_miss, w, label="Near-Miss <2cm (%)", color="#f4a261", alpha=0.88)

    ax1.set_title("A. Outcome Comparison with 0.30m Wall Clearance", fontsize=11, fontweight="bold")
    ax1.set_xticks(x)
    ax1.set_xticklabels(scen_labels, fontsize=10, fontweight="bold")
    ax1.set_ylabel("Percentage (%)", fontsize=10)
    ax1.set_ylim(0, 115)
    ax1.grid(True, linestyle="--", alpha=0.4, axis="y")
    ax1.legend(loc="upper right", fontsize=8.5)

    # Panel 2: Physical Clearance Margin vs Human Wall Pinching
    ax2_twin = ax2.twinx()
    b1 = ax2.bar(x - 0.18, geom_margins, 0.35, label="Geometric Passing Margin (cm)", color="#457b9d", alpha=0.85)
    l1 = ax2_twin.plot(x + 0.18, h_clamp, "s-", color="#7209b7", linewidth=2.5, markersize=8, label="Human Pinched against Wall (% steps)")

    ax2.set_title("B. Geometric Passage Room vs. Human Wall Clamping", fontsize=11, fontweight="bold")
    ax2.set_xticks(x)
    ax2.set_xticklabels(scen_labels, fontsize=10, fontweight="bold")
    ax2.set_ylabel("Geometric Clearance Margin (cm)", fontsize=10, color="#457b9d")
    ax2_twin.set_ylabel("Human Steps Clamped against Wall (%)", fontsize=10, color="#7209b7")
    ax2.set_ylim(0, 55)
    ax2_twin.set_ylim(-2, 50)
    ax2.grid(True, linestyle="--", alpha=0.4, axis="y")

    # Annotate 0.9m tight pinch
    ax2.annotate("Tight Pinch in 0.9m:\nOnly 14cm clear margin;\nHuman pinched 40.6% of time",
                 xy=(0, 14), xytext=(0.2, 35),
                 arrowprops=dict(facecolor="black", shrink=0.08, width=1.2, headwidth=5),
                 fontsize=9, fontweight="bold", bbox=dict(boxstyle="round,pad=0.3", fc="#fff3cd", ec="#ffeeba"))

    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8.5)

    plt.tight_layout()
    out_file = os.path.join(FIG_DIR, "wall_clearance_025_vs_030.png")
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


if __name__ == "__main__":
    plot_progression()
    plot_wall_clearance()
