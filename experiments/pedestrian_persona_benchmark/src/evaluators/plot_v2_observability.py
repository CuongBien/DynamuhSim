#!/usr/bin/env python3
"""
Plots publication-quality diagnostic charts for M3 Persona Observability Benchmark V2:
1. v2_geometry_and_noise_observability.png:
   - 5x5 Confusion Matrix Heatmaps (Open Space vs Corridor All)
   - Macro-F1 breakdown by geometry (Open 6x6 vs Corridor 1.2m vs Corridor 0.9m)
   - Performance & H1-H2 error rate across depth noise levels (sigma in {0.02, 0.05, 0.10}m) with Bootstrap CI95
2. v2_temporal_causality_curve.png:
   - Macro-F1 and Binary Avoidance AUC relative to Closest Point of Approach (t_closest)
"""

import os
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RESULTS_DIR = os.path.join(ROOT_DIR, "results")
METRICS_DIR = os.path.join(RESULTS_DIR, "metrics")
FIG_DIR = os.path.join(RESULTS_DIR, "figures")
os.makedirs(FIG_DIR, exist_ok=True)


def load_cm_csv(filepath):
    matrix = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Header
        for row in reader:
            matrix.append([int(x) for x in row[1:]])
    return np.array(matrix)


def plot_geometry_and_noise():
    v2_json_path = os.path.join(METRICS_DIR, "h1_h5_separability_results_v2.json")
    with open(v2_json_path, 'r') as f:
        data = v2_json_path = json.load(f)

    cm_open = load_cm_csv(os.path.join(METRICS_DIR, "confusion_matrix_5x5_open_sigma005.csv"))
    cm_corr = load_cm_csv(os.path.join(METRICS_DIR, "confusion_matrix_5x5_corridor_all_sigma005.csv"))

    fig = plt.figure(figsize=(18, 9), dpi=200)
    gs = fig.add_gridspec(2, 4, width_ratios=[1.1, 1.1, 1.2, 1.2], hspace=0.35, wspace=0.32)

    # Subplot 1: CM Open Space
    ax1 = fig.add_subplot(gs[0, 0])
    labels = ["H1", "H2", "H3", "H4", "H5"]
    im1 = ax1.imshow(cm_open, cmap="Blues", interpolation="nearest")
    ax1.set_title("Open Space 6x6m (σ = 0.05m)\nMacro-F1 = 44.4%", fontsize=11, fontweight="bold")
    ax1.set_xticks(range(5))
    ax1.set_yticks(range(5))
    ax1.set_xticklabels(labels)
    ax1.set_yticklabels(labels)
    ax1.set_xlabel("Predicted Persona", fontsize=10)
    ax1.set_ylabel("True Persona", fontsize=10)
    for i in range(5):
        for j in range(5):
            val = cm_open[i, j]
            color = "white" if val > cm_open.max() * 0.55 else "black"
            ax1.text(j, i, str(val), ha="center", va="center", color=color, fontsize=9, fontweight="bold")

    # Subplot 2: CM Corridor All
    ax2 = fig.add_subplot(gs[0, 1])
    im2 = ax2.imshow(cm_corr, cmap="Oranges", interpolation="nearest")
    ax2.set_title("Corridor All (0.9m & 1.2m)\nMacro-F1 = 26.5% (Homogenized)", fontsize=11, fontweight="bold")
    ax2.set_xticks(range(5))
    ax2.set_yticks(range(5))
    ax2.set_xticklabels(labels)
    ax2.set_yticklabels(labels)
    ax2.set_xlabel("Predicted Persona", fontsize=10)
    for i in range(5):
        for j in range(5):
            val = cm_corr[i, j]
            color = "white" if val > cm_corr.max() * 0.55 else "black"
            ax2.text(j, i, str(val), ha="center", va="center", color=color, fontsize=9, fontweight="bold")

    # Subplot 3: Geometry breakdown (Open vs Corridors)
    ax3 = fig.add_subplot(gs[0, 2:])
    geoms = ["open_6x6", "corridor_12", "corridor_09", "corridor_all"]
    geom_labels = ["Open Space (6x6m)", "Corridor 1.2m", "Corridor 0.9m", "Corridor Combined"]

    gru_geom = data["noise_experiments"]["sigma_0.05"]["pytorch_gru"]["breakdown_geometry"]
    gbdt_geom = data["noise_experiments"]["sigma_0.05"]["gbdt_handcrafted"]["breakdown_geometry"]

    gru_f1 = [gru_geom[g]["macro_f1"] * 100.0 for g in geoms]
    gbdt_f1 = [gbdt_geom[g]["macro_f1"] * 100.0 for g in geoms]
    gru_acc = [gru_geom[g]["accuracy"] * 100.0 for g in geoms]
    gbdt_acc = [gbdt_geom[g]["accuracy"] * 100.0 for g in geoms]

    x = np.arange(len(geoms))
    w = 0.20
    ax3.bar(x - 1.5*w, gru_f1, w, label="PyTorch GRU (Macro-F1)", color="#2b5c8f", alpha=0.9)
    ax3.bar(x - 0.5*w, gbdt_f1, w, label="GBDT Handcrafted (Macro-F1)", color="#e07a5f", alpha=0.9)
    ax3.bar(x + 0.5*w, gru_acc, w, label="PyTorch GRU (Accuracy)", color="#6495ed", alpha=0.6, hatch="//")
    ax3.bar(x + 1.5*w, gbdt_acc, w, label="GBDT Handcrafted (Accuracy)", color="#f4a261", alpha=0.6, hatch="\\\\")

    ax3.axhline(40.0, color="#2a9d8f", linestyle="--", linewidth=1.5, label="Prior Mode Baseline (H1 = 40%)")
    ax3.axhline(20.0, color="gray", linestyle=":", linewidth=1.2, label="Random Guess (20%)")
    ax3.set_xticks(x)
    ax3.set_xticklabels(geom_labels, fontsize=10, fontweight="bold")
    ax3.set_ylabel("Score (%)", fontsize=11)
    ax3.set_ylim(0, 60)
    ax3.set_title("Performance Collapse: Open Space vs. Corridors (σ = 0.05m)", fontsize=12, fontweight="bold")
    ax3.grid(True, linestyle="--", alpha=0.4, axis="y")
    ax3.legend(loc="upper right", fontsize=8.5, ncol=2)

    # Subplot 4: Performance vs Noise σ
    ax4 = fig.add_subplot(gs[1, :2])
    sigmas = [0.02, 0.05, 0.10]
    sig_labels = ["σ = 0.02m (Low)", "σ = 0.05m (Nominal)", "σ = 0.10m (High)"]
    gru_noise_f1 = [data["noise_experiments"][f"sigma_{s:.2f}"]["pytorch_gru"]["overall_macro_f1"] * 100.0 for s in sigmas]
    gbdt_noise_f1 = [data["noise_experiments"][f"sigma_{s:.2f}"]["gbdt_handcrafted"]["overall_macro_f1"] * 100.0 for s in sigmas]
    gru_noise_acc = [data["noise_experiments"][f"sigma_{s:.2f}"]["pytorch_gru"]["overall_accuracy"] * 100.0 for s in sigmas]
    gbdt_noise_acc = [data["noise_experiments"][f"sigma_{s:.2f}"]["gbdt_handcrafted"]["overall_accuracy"] * 100.0 for s in sigmas]

    x_sig = np.arange(len(sigmas))
    ax4.plot(x_sig, gru_noise_f1, "o-", color="#2b5c8f", linewidth=2.5, markersize=8, label="GRU Macro-F1")
    ax4.plot(x_sig, gbdt_noise_f1, "s-", color="#e07a5f", linewidth=2.5, markersize=8, label="GBDT Macro-F1")
    ax4.plot(x_sig, gru_noise_acc, "o--", color="#6495ed", linewidth=1.8, markersize=6, label="GRU Accuracy")
    ax4.plot(x_sig, gbdt_noise_acc, "s--", color="#f4a261", linewidth=1.8, markersize=6, label="GBDT Accuracy")

    # Errorbar for nominal condition from Bootstrap CI95
    ci_f1 = data["bootstrap_ci95_sigma005_gru"]["overall_macro_f1_ci95"]
    ax4.errorbar(1, gru_noise_f1[1], yerr=[[gru_noise_f1[1] - ci_f1[0]*100], [ci_f1[1]*100 - gru_noise_f1[1]]],
                 fmt="none", ecolor="#1d3557", elinewidth=2.5, capsize=5, capthick=2, label="GRU 95% Bootstrap CI")

    ax4.axhline(40.0, color="#2a9d8f", linestyle="--", linewidth=1.5, label="Natural Prior (40%)")
    ax4.set_xticks(x_sig)
    ax4.set_xticklabels(sig_labels, fontsize=10, fontweight="bold")
    ax4.set_ylabel("Score (%)", fontsize=11)
    ax4.set_ylim(15, 45)
    ax4.set_title("Robustness to Depth Noise σ (0.02m to 0.10m)", fontsize=12, fontweight="bold")
    ax4.grid(True, linestyle="--", alpha=0.5)
    ax4.legend(loc="lower left", fontsize=8.5, ncol=2)

    # Subplot 5: H1 vs H2 Misclassification & Decision Rule
    ax5 = fig.add_subplot(gs[1, 2:])
    h2_as_h1 = [
        data["noise_experiments"][f"sigma_{s:.2f}"]["pytorch_gru"]["interactive_tau15"]["h2_misclassified_as_h1_rate"] * 100.0
        for s in sigmas
    ]
    h1_as_h2 = [
        data["noise_experiments"][f"sigma_{s:.2f}"]["pytorch_gru"]["interactive_tau15"]["h1_misclassified_as_h2_rate"] * 100.0
        for s in sigmas
    ]
    gbdt_h2_as_h1 = [
        data["noise_experiments"][f"sigma_{s:.2f}"]["gbdt_handcrafted"]["interactive_tau15"]["h2_misclassified_as_h1_rate"] * 100.0
        for s in sigmas
    ]

    w_bar = 0.25
    ax5.bar(x_sig - w_bar, h2_as_h1, w_bar, label="H2 misclassified as H1 (GRU)", color="#d62828", alpha=0.85)
    ax5.bar(x_sig, h1_as_h2, w_bar, label="H1 misclassified as H2 (GRU)", color="#f77f00", alpha=0.85)
    ax5.bar(x_sig + w_bar, gbdt_h2_as_h1, w_bar, label="H2 misclassified as H1 (GBDT)", color="#9d0208", alpha=0.6, hatch="//")

    ax5.axhline(30.0, color="black", linestyle="--", linewidth=2.0, label="Merge Decision Threshold (30%)")
    ax5.set_xticks(x_sig)
    ax5.set_xticklabels(sig_labels, fontsize=10, fontweight="bold")
    ax5.set_ylabel("Confusion Rate (%)", fontsize=11)
    ax5.set_ylim(0, 65)
    ax5.set_title("Physical Non-Separability of H1 & H2 (Threshold > 30% -> MERGE)", fontsize=12, fontweight="bold")
    ax5.grid(True, linestyle="--", alpha=0.4, axis="y")
    ax5.legend(loc="upper left", fontsize=8.5)

    out_file = os.path.join(FIG_DIR, "v2_geometry_and_noise_observability.png")
    plt.tight_layout()
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


def plot_temporal_causality():
    v2_json_path = os.path.join(METRICS_DIR, "h1_h5_separability_results_v2.json")
    with open(v2_json_path, 'r') as f:
        data = json.load(f)

    gru_time = data["noise_experiments"]["sigma_0.05"]["pytorch_gru"]["time_progression_relative_to_closest_point"]
    gbdt_time = data["noise_experiments"]["sigma_0.05"]["gbdt_handcrafted"]["time_progression_relative_to_closest_point"]

    bins = list(gru_time.keys())
    bin_labels = [
        "-3s to -2s\n(Early Approach)",
        "-2s to -1s\n(Impending CPA)",
        "-1s to 0s\n(Immediate Encounter)",
        "0s to +1s\n(Crossing Point)",
        "+1s to +2s\n(Departure / Post-CPA)"
    ]

    gru_f1 = [gru_time[b]["macro_f1"] * 100.0 for b in bins]
    gbdt_f1 = [gbdt_time[b]["macro_f1"] * 100.0 for b in bins]
    gru_auc = [gru_time[b]["binary_avoid_auc"] for b in bins]
    gbdt_auc = [gbdt_time[b]["binary_avoid_auc"] for b in bins]
    counts = [gru_time[b]["count"] for b in bins]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5.5), dpi=200)

    # Panel 1: Macro-F1 Progression
    x = np.arange(len(bins))
    ax1.plot(x, gru_f1, "o-", color="#2b5c8f", linewidth=2.5, markersize=8, label="PyTorch GRU (Macro-F1)")
    ax1.plot(x, gbdt_f1, "s-", color="#e07a5f", linewidth=2.5, markersize=8, label="GBDT Handcrafted (Macro-F1)")
    ax1.axvline(2.5, color="red", linestyle=":", linewidth=2.0, label="Closest Point of Approach (CPA)")
    ax1.axhline(20.0, color="gray", linestyle="--", linewidth=1.2, label="Random Guess F1 (20%)")

    # Annotate Post-CPA jump
    ax1.annotate("Post-CPA Recovery:\nGBDT F1 jumps to 39.7%\nafter human returns to lane",
                 xy=(4, gbdt_f1[-1]), xytext=(2.6, 36.0),
                 arrowprops=dict(facecolor="black", shrink=0.08, width=1.5, headwidth=6),
                 fontsize=9, fontweight="bold", bbox=dict(boxstyle="round,pad=0.3", fc="#fff3cd", ec="#ffeeba"))

    ax1.set_xticks(x)
    ax1.set_xticklabels(bin_labels, fontsize=9.5)
    ax1.set_ylabel("Macro-F1 Score (%)", fontsize=11)
    ax1.set_ylim(15, 45)
    ax1.set_title("A. Macro-F1 Score Relative to Interaction CPA", fontsize=12, fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper left", fontsize=9)

    # Panel 2: Binary Avoidance AUC (Yield vs Non-Yield)
    ax2.plot(x, gru_auc, "o-", color="#2b5c8f", linewidth=2.5, markersize=8, label="PyTorch GRU (AUC)")
    ax2.plot(x, gbdt_auc, "s-", color="#e07a5f", linewidth=2.5, markersize=8, label="GBDT Handcrafted (AUC)")
    ax2.axvline(2.5, color="red", linestyle=":", linewidth=2.0, label="Closest Point of Approach (CPA)")
    ax2.axhline(0.50, color="black", linestyle="--", linewidth=1.5, label="Random Guess Baseline (AUC = 0.50)")

    # Shaded unobservable zone
    ax2.axvspan(-0.5, 2.5, color="gray", alpha=0.15, label="Physically Unobservable Zone (t < t_CPA)")

    ax2.annotate("Before CPA: AUC ≈ 0.51 - 0.53\n(Pure chance guessing)",
                 xy=(1, 0.52), xytext=(0.3, 0.56),
                 arrowprops=dict(facecolor="black", shrink=0.08, width=1.2, headwidth=5),
                 fontsize=9, fontweight="bold", bbox=dict(boxstyle="round,pad=0.3", fc="#f8d7da", ec="#f5c6cb"))

    ax2.set_xticks(x)
    ax2.set_xticklabels(bin_labels, fontsize=9.5)
    ax2.set_ylabel("ROC Area Under Curve (AUC)", fontsize=11)
    ax2.set_ylim(0.40, 0.65)
    ax2.set_title("B. Binary Avoidance AUC (Yield vs. Unyielding)", fontsize=12, fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper left", fontsize=9)

    out_file = os.path.join(FIG_DIR, "v2_temporal_causality_curve.png")
    plt.tight_layout()
    plt.savefig(out_file, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved {out_file}")


if __name__ == "__main__":
    plot_geometry_and_noise()
    plot_temporal_causality()
