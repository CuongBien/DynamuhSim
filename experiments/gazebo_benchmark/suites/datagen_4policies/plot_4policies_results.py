#!/usr/bin/env python3
"""
Plots publication-quality diagnostic charts for 4 Robot Policies (7 Sub-behaviors) Benchmark:
1. fig1_policy_action_distributions.png:
   - vx, vy, wz distributions (Intended vs. Applied vs. Noise) across P1, P2, P3a-d, P4 for H1 and H3
2. fig2_safety_and_behavior_rates.png:
   - Goal reach (%), Human collision (%), Wall contact (%), Freezing (%), Stop >=1s (%), |vy| > 0.1 m/s (%)
3. fig3_clearance_and_human_reaction.png:
   - Robot min d_clear & max lateral deviation
   - Human reaction in fixed d < 4m window (lateral deviation, % wall clamped, speed decel)
"""

import os
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RESULTS_DIR = os.path.join(BENCH_DIR, "results/datagen_4policies")
FIG_DIR = os.path.join(RESULTS_DIR, "figures")
os.makedirs(FIG_DIR, exist_ok=True)

RESULTS_JSON = os.path.join(RESULTS_DIR, "a3_3_4policies_results.json")


def load_data():
    with open(RESULTS_JSON, "r") as fp:
        return json.load(fp)


def plot_action_distributions(data):
    policies = ["P1", "P2", "P3a", "P3b", "P3c", "P3d", "P4"]
    p_labels = [
        "P1\n(MPPI)", "P2\n(OU Noise)", "P3a\n(Stop/Wait)",
        "P3b\n(Nudge)", "P3c\n(Slow)", "P3d\n(Control)", "P4\n(Random)"
    ]

    fig, axes = plt.subplots(3, 2, figsize=(18, 12), dpi=200, sharex=True)
    # Row 0: vx (H1, H3)
    # Row 1: vy (H1, H3)
    # Row 2: wz (H1, H3)

    channels = [("vx", "$v_x$ (Forward Velocity, m/s)", (-0.15, 0.75)),
                ("vy", "$v_y$ (Lateral Velocity, m/s)", (-0.45, 0.45)),
                ("wz", "$\omega$ (Angular Velocity, rad/s)", (-1.3, 1.3))]

    personas = [("H1", "Persona H1 (Cooperative Pedestrian)", 0),
                ("H3", "Persona H3 (Unyielding Pedestrian)", 1)]

    for row_idx, (ch_key, ch_title, ch_ylim) in enumerate(channels):
        for col_idx, (persona, persona_title, col_num) in enumerate(personas):
            ax = axes[row_idx, col_num]
            summary = data["summary_by_persona"][persona]

            x = np.arange(len(policies))
            w = 0.28

            # Extract Intended and Applied
            int_means = [summary[p]["actions_intended"][ch_key]["mean"] for p in policies]
            int_stds = [summary[p]["actions_intended"][ch_key]["std"] for p in policies]
            int_p5 = [summary[p]["actions_intended"][ch_key]["p5"] for p in policies]
            int_p95 = [summary[p]["actions_intended"][ch_key]["p95"] for p in policies]

            app_means = [summary[p]["actions_applied"][ch_key]["mean"] for p in policies]
            app_stds = [summary[p]["actions_applied"][ch_key]["std"] for p in policies]
            app_p5 = [summary[p]["actions_applied"][ch_key]["p5"] for p in policies]
            app_p95 = [summary[p]["actions_applied"][ch_key]["p95"] for p in policies]

            # Errorbars: std as dark bar, p5-p95 as thin line
            ax.bar(x - w/2, int_means, w, yerr=int_stds, capsize=4,
                   label="Intended Command (MPPI / RNG)", color="#3a86ff", alpha=0.85)
            ax.bar(x + w/2, app_means, w, yerr=app_stds, capsize=4,
                   label="Applied Command (Filtered & Executed)", color="#ff006e", alpha=0.85)

            # Mark 5th-95th percentile ranges
            for i in range(len(policies)):
                ax.plot([x[i] - w/2, x[i] - w/2], [int_p5[i], int_p95[i]], color="#001219", linewidth=1.2, alpha=0.6)
                ax.plot([x[i] + w/2, x[i] + w/2], [app_p5[i], app_p95[i]], color="#001219", linewidth=1.2, alpha=0.6)

            # In row 0 (vx), show forward progress threshold for P2 & P4
            if ch_key == "vx":
                ax.axhline(0.25, color="green", linestyle="--", linewidth=1.2, alpha=0.8, label="Forward Progress Target (0.25 m/s)")
            elif ch_key == "vy":
                ax.axhline(0.10, color="gray", linestyle=":", linewidth=1.0)
                ax.axhline(-0.10, color="gray", linestyle=":", linewidth=1.0)

            ax.set_ylabel(ch_title, fontsize=10, fontweight="bold")
            ax.set_ylim(ch_ylim)
            ax.grid(True, linestyle="--", alpha=0.4, axis="y")

            if row_idx == 0:
                ax.set_title(f"{persona_title}\n{ch_title}", fontsize=11, fontweight="bold")
            else:
                ax.set_title(ch_title, fontsize=11, fontweight="bold")

            if row_idx == 2:
                ax.set_xticks(x)
                ax.set_xticklabels(p_labels, fontsize=10, fontweight="bold")

            if row_idx == 0 and col_idx == 0:
                ax.legend(loc="upper right", fontsize=8.5, framealpha=0.9)

    plt.tight_layout()
    out_file = os.path.join(FIG_DIR, "fig1_policy_action_distributions.png")
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


def plot_safety_and_behavior_rates(data):
    policies = ["P1", "P2", "P3a", "P3b", "P3c", "P3d", "P4"]
    p_labels = ["P1 (MPPI)", "P2 (OU Noise)", "P3a (Stop/Wait)", "P3b (Nudge)", "P3c (Slow)", "P3d (Control)", "P4 (Random)"]

    fig, axes = plt.subplots(2, 3, figsize=(18, 9), dpi=200)

    metrics = [
        ("goal_reach_rate_pct", "Goal Reach Rate (%)", [0, 105], "#2a9d8f", True),
        ("collision_human_rate_pct", "Human Footprint Collision (%)", [0, 105], "#e63946", False),
        ("wall_contact_rate_pct", "Wall Contact Rate (%)", [0, 105], "#f4a261", False),
        ("freezing_or_stuck_3s_rate_pct", "Freezing / Stuck Rate (>=3s) (%)", [0, 105], "#9d0208", False),
        ("stopped_fully_1s_rate_pct", "Full Stop Rate (>=1s) (%)", [0, 105], "#457b9d", None),
        ("ratio_time_vy_gt_01_pct", "Time with |vy| > 0.1 m/s (%)", [0, 105], "#6a0572", None),
    ]

    for idx, (m_key, m_title, m_ylim, bar_color, target_high) in enumerate(metrics):
        r, c = idx // 3, idx % 3
        ax = axes[r, c]

        h1_vals = [data["summary_by_persona"]["H1"][p][m_key] for p in policies]
        h3_vals = [data["summary_by_persona"]["H3"][p][m_key] for p in policies]

        x = np.arange(len(policies))
        w = 0.35

        ax.bar(x - w/2, h1_vals, w, label="Persona H1 (Cooperative)", color="#1d3557", alpha=0.88)
        ax.bar(x + w/2, h3_vals, w, label="Persona H3 (Unyielding Blind)", color="#e76f51", alpha=0.88)

        # Draw guideline / spec threshold
        if m_key == "wall_contact_rate_pct":
            ax.axhline(5.0, color="red", linestyle="--", linewidth=1.8, label="Wall Safety Spec (<= 5%)")
        elif m_key == "freezing_or_stuck_3s_rate_pct":
            ax.axhline(20.0, color="orange", linestyle="--", linewidth=1.5, label="Freeze Limit (<= 20%)")
        elif m_key == "goal_reach_rate_pct":
            ax.axhline(60.0, color="green", linestyle="--", linewidth=1.5, label="H1 Progress Spec (>= 60%)")

        ax.set_title(m_title, fontsize=11, fontweight="bold")
        ax.set_xticks(x)
        ax.set_xticklabels(p_labels, rotation=25, ha="right", fontsize=9)
        ax.set_ylabel("Rate (%)", fontsize=10)
        ax.set_ylim(m_ylim)
        ax.grid(True, linestyle="--", alpha=0.4, axis="y")

        # Value annotations
        for i in range(len(policies)):
            if h1_vals[i] > 2:
                ax.text(x[i] - w/2, h1_vals[i] + 2, f"{h1_vals[i]:.0f}", ha="center", va="bottom", fontsize=7.5, fontweight="bold")
            if h3_vals[i] > 2:
                ax.text(x[i] + w/2, h3_vals[i] + 2, f"{h3_vals[i]:.0f}", ha="center", va="bottom", fontsize=7.5, fontweight="bold")

        if idx == 0:
            ax.legend(loc="lower left", fontsize=8.5)
        elif m_key in ["wall_contact_rate_pct", "freezing_or_stuck_3s_rate_pct"]:
            ax.legend(loc="upper right", fontsize=8.5)

    plt.tight_layout()
    out_file = os.path.join(FIG_DIR, "fig2_safety_and_behavior_rates.png")
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


def plot_clearance_and_human_reaction(data):
    policies = ["P1", "P2", "P3a", "P3b", "P3c", "P3d", "P4"]
    p_labels = ["P1 (MPPI)", "P2 (OU Noise)", "P3a (Stop/Wait)", "P3b (Nudge)", "P3c (Slow)", "P3d (Control)", "P4 (Random)"]

    fig, axes = plt.subplots(2, 2, figsize=(16, 9), dpi=200)

    # Panel (0,0): Robot Footprint Clearance dmin_clear (m)
    ax0 = axes[0, 0]
    h1_dmin = [data["summary_by_persona"]["H1"][p]["dmin_clear_m_med"] for p in policies]
    h3_dmin = [data["summary_by_persona"]["H3"][p]["dmin_clear_m_med"] for p in policies]
    # Replace None with 0.0 for collision
    h1_dmin_clean = [v if v is not None else 0.0 for v in h1_dmin]
    h3_dmin_clean = [v if v is not None else 0.0 for v in h3_dmin]

    x = np.arange(len(policies))
    w = 0.35
    ax0.bar(x - w/2, h1_dmin_clean, w, label="Persona H1", color="#2a9d8f", alpha=0.85)
    ax0.bar(x + w/2, h3_dmin_clean, w, label="Persona H3", color="#e76f51", alpha=0.85)
    ax0.axhline(0.0, color="red", linestyle="-", linewidth=1.5, label="Contact Collision (d_clear <= 0)")
    ax0.axhline(0.02, color="orange", linestyle=":", linewidth=1.5, label="Near-miss Boundary (2 cm)")
    ax0.set_title("A. Robot Minimum Clearance d_clear (Median, m)", fontsize=11, fontweight="bold")
    ax0.set_xticks(x)
    ax0.set_xticklabels(p_labels, rotation=20, ha="right", fontsize=9)
    ax0.set_ylabel("Clearance d_clear (m)", fontsize=10)
    ax0.set_ylim(-0.01, 0.12)
    ax0.grid(True, linestyle="--", alpha=0.4, axis="y")
    ax0.legend(loc="upper right", fontsize=8.5)

    # Panel (0,1): Robot Lateral Deviation max |y|
    ax1 = axes[0, 1]
    h1_lat_med = [data["summary_by_persona"]["H1"][p]["max_lateral_dev_m_med"] for p in policies]
    h1_lat_max = [data["summary_by_persona"]["H1"][p]["max_lateral_dev_m_max"] for p in policies]
    h3_lat_med = [data["summary_by_persona"]["H3"][p]["max_lateral_dev_m_med"] for p in policies]
    h3_lat_max = [data["summary_by_persona"]["H3"][p]["max_lateral_dev_m_max"] for p in policies]

    ax1.bar(x - w/2, h1_lat_max, w, label="H1 Max Lateral Dev", color="#457b9d", alpha=0.4, hatch="//")
    ax1.bar(x - w/2, h1_lat_med, w*0.7, label="H1 Median Lateral Dev", color="#1d3557", alpha=0.9)
    ax1.bar(x + w/2, h3_lat_max, w, label="H3 Max Lateral Dev", color="#f4a261", alpha=0.4, hatch="//")
    ax1.bar(x + w/2, h3_lat_med, w*0.7, label="H3 Median Lateral Dev", color="#e63946", alpha=0.9)
    ax1.axhline(0.60 - 0.10, color="black", linestyle="--", linewidth=1.2, label="Corridor Boundary limit (0.50m)")
    ax1.set_title("B. Robot Lateral Deviation |y| (Median / Max, m)", fontsize=11, fontweight="bold")
    ax1.set_xticks(x)
    ax1.set_xticklabels(p_labels, rotation=20, ha="right", fontsize=9)
    ax1.set_ylabel("Lateral Deviation (m)", fontsize=10)
    ax1.set_ylim(0, 0.55)
    ax1.grid(True, linestyle="--", alpha=0.4, axis="y")
    ax1.legend(loc="upper left", fontsize=8, ncol=2)

    # Panel (1,0): Human Reaction - Lateral Deviation in Window
    ax2 = axes[1, 0]
    h1_hlat = [data["summary_by_persona"]["H1"][p]["human_max_lat_dev_m_mean"] for p in policies]
    h3_hlat = [data["summary_by_persona"]["H3"][p]["human_max_lat_dev_m_mean"] for p in policies]
    ax2.bar(x - w/2, h1_hlat, w, label="H1 Human Max Lateral Dev", color="#2a9d8f", alpha=0.85)
    ax2.bar(x + w/2, h3_hlat, w, label="H3 Human Max Lateral Dev (Blind = 0)", color="#e76f51", alpha=0.85)
    ax2.axhline(0.30, color="purple", linestyle="--", linewidth=1.5, label="Human Wall Hugging Limit (0.30m)")
    ax2.set_title("C. Human Lateral Deviation in Window (d < 4m to CPA, m)", fontsize=11, fontweight="bold")
    ax2.set_xticks(x)
    ax2.set_xticklabels(p_labels, rotation=20, ha="right", fontsize=9)
    ax2.set_ylabel("Human Deflection (m)", fontsize=10)
    ax2.set_ylim(0, 0.38)
    ax2.grid(True, linestyle="--", alpha=0.4, axis="y")
    ax2.legend(loc="upper right", fontsize=8.5)

    # Panel (1,1): Human Deceleration & Wall Clamping
    ax3 = axes[1, 1]
    h1_clamp = [data["summary_by_persona"]["H1"][p]["human_clamped_pct_mean"] for p in policies]
    h1_decel = [data["summary_by_persona"]["H1"][p]["human_decel_mps_mean"] for p in policies]
    h3_decel = [data["summary_by_persona"]["H3"][p]["human_decel_mps_mean"] for p in policies]

    ax3_twin = ax3.twinx()
    b1 = ax3.bar(x - w/2, h1_clamp, w, label="H1 Clamped Steps (% time against wall)", color="#9b5de5", alpha=0.7)
    l1 = ax3_twin.plot(x, h1_decel, "o-", color="#00f5d4", linewidth=2.5, markersize=8, label="H1 Human Decel (m/s)")
    l2 = ax3_twin.plot(x, h3_decel, "s--", color="#f15bb5", linewidth=2.0, markersize=6, label="H3 Human Decel (m/s)")

    ax3.set_title("D. Human Wall Clamping & Speed Deceleration", fontsize=11, fontweight="bold")
    ax3.set_xticks(x)
    ax3.set_xticklabels(p_labels, rotation=20, ha="right", fontsize=9)
    ax3.set_ylabel("Wall Clamped Ratio (%)", fontsize=10, color="#9b5de5")
    ax3_twin.set_ylabel("Speed Deceleration (m/s)", fontsize=10, color="#00f5d4")
    ax3.set_ylim(0, 15)
    ax3_twin.set_ylim(-0.02, 0.20)
    ax3.grid(True, linestyle="--", alpha=0.4, axis="y")

    # Combine legends
    lines, labels = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3_twin.get_legend_handles_labels()
    ax3.legend(lines + lines2, labels + labels2, loc="upper right", fontsize=8.5)

    plt.tight_layout()
    out_file = os.path.join(FIG_DIR, "fig3_clearance_and_human_reaction.png")
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


if __name__ == "__main__":
    data = load_data()
    plot_action_distributions(data)
    plot_safety_and_behavior_rates(data)
    plot_clearance_and_human_reaction(data)
