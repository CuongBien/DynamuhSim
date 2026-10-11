import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sklearn.model_selection import StratifiedShuffleSplit
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score, f1_score
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline

def main():
    print("=" * 70)
    print("EVALUATING GROUND-TRUTH SEPARABILITY & ORACLE CLASSIFIER (H1 - H5)")
    print("=" * 70)

    # 1. Load ground truth dataset
    with open('experiments/pedestrian_persona_benchmark/ground_truth_dataset.json') as f:
        data = json.load(f)

    print(f"Loaded {len(data)} interaction episodes.")

    # 2. Extract engineered kinematic features
    X = []
    y = []
    alphas = []
    scenarios = []
    max_lat_devs = []

    persona_map = {"H1": 0, "H2": 1, "H3": 2, "H4": 3, "H5": 4}
    rev_persona_map = {v: k for k, v in persona_map.items()}

    for ep in data:
        p_name = ep["persona"]
        sc = ep["scenario"]
        alpha = ep["alpha"]
        traj = ep["trajectory"]

        # Kinematic time series arrays
        ts = np.array([pt["t"] for pt in traj])
        hxs = np.array([pt["hx"] for pt in traj])
        hys = np.array([pt["hy"] for pt in traj])
        hvxs = np.array([pt["hvx"] for pt in traj])
        hvys = np.array([pt["hvy"] for pt in traj])
        rxs = np.array([pt["rx"] for pt in traj])
        rys = np.array([pt["ry"] for pt in traj])
        d_robots = np.array([pt["d_robot"] for pt in traj])
        lat_devs = np.array([pt["lat_dev"] for pt in traj])

        speeds = np.hypot(hvxs, hvys)
        accels = np.diff(speeds) / 0.1

        # Calculate curvature (change in velocity angle)
        headings = np.arctan2(hvys, hvxs)
        heading_diffs = np.abs(np.diff(np.unwrap(headings)))

        # Feature vector
        feat = [
            # 1. Lateral deviation features
            np.max(lat_devs),
            np.mean(lat_devs),
            lat_devs[-1],
            np.std(lat_devs),
            # 2. Robot interaction distance
            np.min(d_robots),
            np.mean(d_robots),
            d_robots[np.argmax(lat_devs)], # Distance to robot when maximum deviation occurs
            # 3. Speed & Acceleration dynamics
            np.mean(speeds),
            np.std(speeds),
            np.min(speeds),
            np.max(speeds),
            np.max(np.abs(accels)) if len(accels) > 0 else 0.0,
            # 4. Curvature & Heading deviation
            np.sum(heading_diffs) if len(heading_diffs) > 0 else 0.0,
            np.max(heading_diffs) if len(heading_diffs) > 0 else 0.0,
            # 5. Trajectory endpoints displacement
            np.hypot(hxs[-1] - hxs[0], hys[-1] - hys[0]),
            # 6. Early vs Late reaction (first half vs second half lateral deviation ratio)
            np.mean(lat_devs[:len(lat_devs)//2]) / (np.mean(lat_devs[len(lat_devs)//2:]) + 1e-4)
        ]

        X.append(feat)
        y.append(persona_map[p_name])
        alphas.append(alpha)
        scenarios.append(sc)
        max_lat_devs.append(np.max(lat_devs))

    X = np.array(X)
    y = np.array(y)
    alphas = np.array(alphas)
    max_lat_devs = np.array(max_lat_devs)

    # 3. Analyze Lateral Deviation Distribution & H1 vs H2 overlap
    print("\n--- STATISTICAL DISTRIBUTION OF LATERAL DEVIATION |Y| ---")
    stats = {}
    for p_name, p_idx in persona_map.items():
        mask = (y == p_idx)
        vals = max_lat_devs[mask]
        stats[p_name] = {
            "mean": float(np.mean(vals)),
            "std": float(np.std(vals)),
            "median": float(np.median(vals)),
            "p25": float(np.percentile(vals, 25)),
            "p75": float(np.percentile(vals, 75)),
            "p95": float(np.percentile(vals, 95)),
            "min": float(np.min(vals)),
            "max": float(np.max(vals))
        }
        print(f"[{p_name:2s}] Mean: {stats[p_name]['mean']:.4f} m | Median: {stats[p_name]['median']:.4f} m | "
              f"IQR: [{stats[p_name]['p25']:.4f}, {stats[p_name]['p75']:.4f}] | "
              f"Max: {stats[p_name]['max']:.4f} m")

    # Specifically analyze H2 across different alpha sub-ranges
    print("\n--- H2 SUB-RANGE BREAKDOWN vs H1 (alpha = 0.50) ---")
    h2_mask = (y == persona_map["H2"])
    h2_alphas = alphas[h2_mask]
    h2_devs = max_lat_devs[h2_mask]

    bins = [
        ("H2 (0.10 <= alpha < 0.20)", (h2_alphas >= 0.10) & (h2_alphas < 0.20)),
        ("H2 (0.20 <= alpha < 0.30)", (h2_alphas >= 0.20) & (h2_alphas < 0.30)),
        ("H2 (0.30 <= alpha <= 0.35) [UPPER BOUND]", (h2_alphas >= 0.30) & (h2_alphas <= 0.35))
    ]
    h2_sub_stats = {}
    for b_name, b_mask in bins:
        b_vals = h2_devs[b_mask]
        h2_sub_stats[b_name] = {
            "count": int(np.sum(b_mask)),
            "mean": float(np.mean(b_vals)),
            "median": float(np.median(b_vals)),
            "p95": float(np.percentile(b_vals, 95))
        }
        print(f"  {b_name:42s} (N={np.sum(b_mask):3d}): Mean = {np.mean(b_vals):.4f} m, Median = {np.median(b_vals):.4f} m, p95 = {np.percentile(b_vals, 95):.4f} m")

    # 4. Train Oracle Upper-Bound Classifier (Ground Truth)
    print("\n--- TRAINING ORACLE GROUND-TRUTH CLASSIFIER ---")
    sss = StratifiedShuffleSplit(n_splits=1, test_size=0.25, random_state=42)
    train_idx, test_idx = next(sss.split(X, y))

    X_train, X_test = X[train_idx], X[test_idx]
    y_train, y_test = y[train_idx], y[test_idx]

    # Evaluate Random Forest & Gradient Boosting
    rf = RandomForestClassifier(n_estimators=150, max_depth=12, random_state=42, class_weight='balanced')
    rf.fit(X_train, y_train)
    y_pred = rf.predict(X_test)

    acc_overall = accuracy_score(y_test, y_pred)
    f1_macro_overall = f1_score(y_test, y_pred, average='macro')

    # DYNAMIC SUBSET METRICS (Excluding H4 Static)
    dynamic_mask = (y_test != persona_map["H4"])
    acc_dynamic = accuracy_score(y_test[dynamic_mask], y_pred[dynamic_mask])
    f1_macro_dynamic = f1_score(y_test[dynamic_mask], y_pred[dynamic_mask], average='macro')

    cm = confusion_matrix(y_test, y_pred)
    target_names = ["H1 (Coop)", "H2 (Unyield)", "H3 (Blind)", "H4 (Static)", "H5 (SFM)"]
    report_dict = classification_report(y_test, y_pred, target_names=target_names, output_dict=True)
    report_text = classification_report(y_test, y_pred, target_names=target_names)

    print("\n[ORACLE CLASSIFIER RESULTS ON GROUND TRUTH]")
    print(f"  Overall Accuracy (H1-H5)        : {acc_overall * 100:.2f}%")
    print(f"  Overall Macro-F1 (H1-H5)        : {f1_macro_overall * 100:.2f}%")
    print(f"  Dynamic-Only Accuracy (H1,H2,H3,H5 - No H4): {acc_dynamic * 100:.2f}%")
    print(f"  Dynamic-Only Macro-F1 (H1,H2,H3,H5 - No H4): {f1_macro_dynamic * 100:.2f}%")
    print("\nDetailed Classification Report:")
    print(report_text)

    print("Confusion Matrix:")
    print("      " + " ".join([f"{k[:4]:>6s}" for k in ["H1", "H2", "H3", "H4", "H5"]]))
    for idx, row in enumerate(cm):
        print(f"{target_names[idx][:8]:8s} " + " ".join([f"{val:6d}" for val in row]))

    # Feature Importance
    importances = rf.feature_importances_
    feature_names = [
        "max_lat_dev", "mean_lat_dev", "final_lat_dev", "std_lat_dev",
        "min_dist_robot", "mean_dist_robot", "dist_at_max_dev",
        "mean_speed", "std_speed", "min_speed", "max_speed", "max_accel",
        "sum_heading_diff", "max_heading_diff", "displacement", "early_vs_late_ratio"
    ]
    sorted_idx = np.argsort(importances)[::-1]
    print("\nTop Features Contributing to Separability:")
    for i in range(min(8, len(feature_names))):
        print(f"  {i+1}. {feature_names[sorted_idx[i]]:22s}: {importances[sorted_idx[i]]*100:.2f}%")

    # 5. Plotting Distributions & Confusion Matrix
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Plot 1: Lateral deviation boxplot
    data_to_plot = [max_lat_devs[y == persona_map[p]] for p in ["H1", "H2", "H3", "H4", "H5"]]
    bp = axes[0].boxplot(data_to_plot, labels=["H1\n(Coop)", "H2\n(Unyield)", "H3\n(Blind)", "H4\n(Static)", "H5\n(SFM)"], patch_artist=True)
    colors = ['#4daf4a', '#377eb8', '#ff7f00', '#984ea3', '#e41a1c']
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)
    axes[0].set_title("Ground-Truth Lateral Deviation Distribution |Y| (m)")
    axes[0].set_ylabel("Lateral Deviation (m)")
    axes[0].grid(True, linestyle='--', alpha=0.5)

    # Plot 2: Confusion Matrix Heatmap
    im = axes[1].imshow(cm, cmap='Blues')
    axes[1].set_title(f"Oracle Confusion Matrix (Dynamic Acc: {acc_dynamic*100:.1f}%)")
    axes[1].set_xticks(np.arange(5))
    axes[1].set_yticks(np.arange(5))
    axes[1].set_xticklabels(["H1", "H2", "H3", "H4", "H5"])
    axes[1].set_yticklabels(["H1", "H2", "H3", "H4", "H5"])
    axes[1].set_xlabel("Predicted Persona")
    axes[1].set_ylabel("True Persona")
    for i in range(5):
        for j in range(5):
            axes[1].text(j, i, cm[i, j], ha="center", va="center",
                         color="white" if cm[i, j] > cm.max()/2 else "black")
    plt.colorbar(im, ax=axes[1])
    plt.tight_layout()
    plt.savefig('experiments/pedestrian_persona_benchmark/separability_evaluation.png', dpi=300)
    plt.close()

    # Save detailed JSON summary
    results_summary = {
        "n_samples": len(data),
        "overall_accuracy": float(acc_overall),
        "overall_macro_f1": float(f1_macro_overall),
        "dynamic_accuracy_no_h4": float(acc_dynamic),
        "dynamic_macro_f1_no_h4": float(f1_macro_dynamic),
        "persona_stats": stats,
        "h2_sub_range_stats": h2_sub_stats,
        "classification_report": report_dict,
        "confusion_matrix": cm.tolist(),
        "top_features": [(feature_names[sorted_idx[i]], float(importances[sorted_idx[i]])) for i in range(len(feature_names))]
    }
    with open('experiments/pedestrian_persona_benchmark/gt_separability_results.json', 'w') as f:
        json.dump(results_summary, f, indent=2)

    print("\nSaved evaluation plots to experiments/pedestrian_persona_benchmark/separability_evaluation.png")
    print("Saved results JSON to experiments/pedestrian_persona_benchmark/gt_separability_results.json")

if __name__ == '__main__':
    main()
