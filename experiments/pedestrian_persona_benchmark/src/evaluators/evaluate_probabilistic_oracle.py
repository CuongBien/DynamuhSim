import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sklearn.model_selection import StratifiedShuffleSplit
from sklearn.ensemble import RandomForestClassifier
from sklearn.calibration import CalibratedClassifierCV
from sklearn.metrics import log_loss, confusion_matrix, accuracy_score, f1_score

def compute_nominal_cpa(ep):
    traj = ep["trajectory"]
    p0 = np.array([traj[0]["hx"] - traj[0]["rx"], traj[0]["hy"] - traj[0]["ry"]])
    # Nominal initial velocities
    v0 = np.array([traj[0]["hvx"] - traj[0]["rvx"], traj[0]["hvy"] - traj[0]["rvy"]])
    v_norm_sq = np.dot(v0, v0)
    if v_norm_sq < 1e-6:
        return np.linalg.norm(p0), 0.0
    t_cpa = -np.dot(p0, v0) / v_norm_sq
    if t_cpa <= 0.0:
        d_cpa = np.linalg.norm(p0)
    else:
        p_cpa = p0 + v0 * t_cpa
        d_cpa = np.linalg.norm(p_cpa)
    return float(d_cpa), float(t_cpa)

def extract_window_features(traj, max_t=2.0):
    # Slice strictly within t <= max_t (e.g. 2.0s observation window = 20 steps at 10Hz)
    sub = [pt for pt in traj if pt["t"] <= max_t + 1e-4]
    if len(sub) == 0:
        sub = traj[:1]

    hxs = np.array([pt["hx"] for pt in sub])
    hys = np.array([pt["hy"] for pt in sub])
    hvxs = np.array([pt["hvx"] for pt in sub])
    hvys = np.array([pt["hvy"] for pt in sub])
    rxs = np.array([pt["rx"] for pt in sub])
    rys = np.array([pt["ry"] for pt in sub])
    d_robots = np.array([pt["d_robot"] for pt in sub])
    lat_devs = np.array([pt["lat_dev"] for pt in sub])

    speeds = np.hypot(hvxs, hvys)
    accels = np.diff(speeds) / 0.1 if len(speeds) > 1 else np.array([0.0])
    headings = np.arctan2(hvys, hvxs)
    heading_diffs = np.abs(np.diff(np.unwrap(headings))) if len(headings) > 1 else np.array([0.0])

    features = [
        # Lateral deviation within window
        np.max(lat_devs),
        np.mean(lat_devs),
        lat_devs[-1],
        np.std(lat_devs),
        # Robot distance within window
        np.min(d_robots),
        d_robots[-1],
        d_robots[np.argmax(lat_devs)],
        # Speed dynamics
        np.mean(speeds),
        np.std(speeds),
        np.min(speeds),
        np.max(speeds),
        np.max(np.abs(accels)) if len(accels) > 0 else 0.0,
        # Heading & curvature
        np.sum(heading_diffs) if len(heading_diffs) > 0 else 0.0,
        np.max(heading_diffs) if len(heading_diffs) > 0 else 0.0,
        # Total displacement in window
        np.hypot(hxs[-1] - hxs[0], hys[-1] - hys[0]),
        # Rate of lateral deviation growth
        (lat_devs[-1] - lat_devs[0]) / max(0.1, sub[-1]["t"] - sub[0]["t"])
    ]
    return features

def calculate_ece(y_true, y_prob, n_bins=10):
    confidences = np.max(y_prob, axis=1)
    predictions = np.argmax(y_prob, axis=1)
    accuracies = (predictions == y_true)

    bin_boundaries = np.linspace(0, 1, n_bins + 1)
    ece = 0.0
    bin_details = []

    for i in range(n_bins):
        in_bin = (confidences > bin_boundaries[i]) & (confidences <= bin_boundaries[i + 1])
        prop_in_bin = np.mean(in_bin)
        if prop_in_bin > 0:
            accuracy_in_bin = np.mean(accuracies[in_bin])
            avg_confidence_in_bin = np.mean(confidences[in_bin])
            ece += np.abs(avg_confidence_in_bin - accuracy_in_bin) * prop_in_bin
            bin_details.append({
                "bin": f"[{bin_boundaries[i]:.2f}, {bin_boundaries[i+1]:.2f}]",
                "count": int(np.sum(in_bin)),
                "accuracy": float(accuracy_in_bin),
                "confidence": float(avg_confidence_in_bin)
            })
    return float(ece), bin_details

def main():
    print("=" * 80)
    print("PROBABILISTIC M3 ORACLE EVALUATION: 2.0s WINDOW, NOMINAL CPA & CALIBRATION")
    print("=" * 80)

    import os
    base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    dataset_path = os.path.join(base_dir, 'results', 'datasets', 'ground_truth_dataset.json')
    with open(dataset_path) as f:
        data = json.load(f)

    persona_map = {"H1": 0, "H2": 1, "H3": 2, "H4": 3, "H5": 4}
    rev_persona_map = {v: k for k, v in persona_map.items()}

    # Compute Nominal CPA for every episode (Purely geometric at t=0)
    for ep in data:
        d_cpa, t_cpa = compute_nominal_cpa(ep)
        ep["nominal_d_cpa"] = d_cpa
        ep["nominal_t_cpa"] = t_cpa

    # Build 2.0s observation window feature matrix
    X_2s = []
    y = []
    nominal_cpas = []
    scenarios = []
    alphas = []

    for ep in data:
        feat = extract_window_features(ep["trajectory"], max_t=2.0)
        X_2s.append(feat)
        y.append(persona_map[ep["persona"]])
        nominal_cpas.append(ep["nominal_d_cpa"])
        scenarios.append(ep["scenario"])
        alphas.append(ep["alpha"])

    X_2s = np.array(X_2s)
    y = np.array(y)
    nominal_cpas = np.array(nominal_cpas)
    alphas = np.array(alphas)

    # 1. Stratified train/test split on FULL DATASET (No filtering during training!)
    sss = StratifiedShuffleSplit(n_splits=1, test_size=0.25, random_state=42)
    train_idx, test_idx = next(sss.split(X_2s, y))

    X_train, X_test = X_2s[train_idx], X_2s[test_idx]
    y_train, y_test = y[train_idx], y[test_idx]
    cpa_test = nominal_cpas[test_idx]
    alphas_test = alphas[test_idx]

    # Train calibrated random forest model
    base_rf = RandomForestClassifier(n_estimators=150, max_depth=10, random_state=42, class_weight='balanced')
    calibrated_clf = CalibratedClassifierCV(base_rf, method='sigmoid', cv=3)
    calibrated_clf.fit(X_train, y_train)

    # Probabilistic prediction on test set P(H | o_{1:2s})
    probs_test = calibrated_clf.predict_proba(X_test)
    preds_test = np.argmax(probs_test, axis=1)

    # 2. Evaluation Metrics: Full Test Set
    nll_overall = log_loss(y_test, probs_test)
    # Multi-class Brier score
    y_test_onehot = np.eye(5)[y_test]
    brier_overall = np.mean(np.sum((probs_test - y_test_onehot)**2, axis=1))
    ece_overall, bin_details = calculate_ece(y_test, probs_test)
    acc_overall = accuracy_score(y_test, preds_test)
    f1_macro_overall = f1_score(y_test, preds_test, average='macro')

    # Dynamic subset (excluding H4 Static)
    dyn_mask = (y_test != persona_map["H4"])
    acc_dynamic = accuracy_score(y_test[dyn_mask], preds_test[dyn_mask])
    f1_macro_dynamic = f1_score(y_test[dyn_mask], preds_test[dyn_mask], average='macro')

    print(f"\n[A. FULL TEST SET EVALUATION (2.0s Window, N={len(y_test)})]")
    print(f"  Overall Accuracy (H1-H5)        : {acc_overall * 100:.2f}%")
    print(f"  Overall Macro-F1 (H1-H5)        : {f1_macro_overall * 100:.2f}%")
    print(f"  Dynamic-Only Accuracy (No H4)   : {acc_dynamic * 100:.2f}%")
    print(f"  Dynamic-Only Macro-F1 (No H4)   : {f1_macro_dynamic * 100:.2f}%")
    print(f"  Negative Log-Likelihood (NLL)   : {nll_overall:.4f}")
    print(f"  Multi-class Brier Score         : {brier_overall:.4f}")
    print(f"  Expected Calibration Error (ECE): {ece_overall * 100:.2f}%")

    # 3. Geometric Interaction Subset: Nominal CPA <= 1.2m
    # (Unbiased, defined purely by t=0 straight-line trajectory before any dodge!)
    conflict_mask = (cpa_test <= 1.2)
    n_conflict = np.sum(conflict_mask)
    y_conf = y_test[conflict_mask]
    probs_conf = probs_test[conflict_mask]
    preds_conf = preds_test[conflict_mask]

    acc_conf = accuracy_score(y_conf, preds_conf)
    f1_conf = f1_score(y_conf, preds_conf, average='macro')
    nll_conf = log_loss(y_conf, probs_conf)
    brier_conf = np.mean(np.sum((probs_conf - np.eye(5)[y_conf])**2, axis=1))

    dyn_conf_mask = (y_conf != persona_map["H4"])
    acc_dyn_conf = accuracy_score(y_conf[dyn_conf_mask], preds_conf[dyn_conf_mask])
    f1_dyn_conf = f1_score(y_conf[dyn_conf_mask], preds_conf[dyn_conf_mask], average='macro')

    print(f"\n[B. GEOMETRIC CONFLICT TEST SET (Nominal CPA <= 1.2m, N={n_conflict} / {len(y_test)})]")
    print(f"  Conflict Accuracy (H1-H5)       : {acc_conf * 100:.2f}%")
    print(f"  Conflict Macro-F1 (H1-H5)       : {f1_conf * 100:.2f}%")
    print(f"  Dynamic Conflict Accuracy       : {acc_dyn_conf * 100:.2f}%")
    print(f"  Dynamic Conflict Macro-F1       : {f1_dyn_conf * 100:.2f}%")
    print(f"  Conflict NLL                    : {nll_conf:.4f}")
    print(f"  Conflict Brier Score            : {brier_conf:.4f}")

    cm_conf = confusion_matrix(y_conf, preds_conf)
    target_names = ["H1", "H2", "H3", "H4", "H5"]
    print("\n  Conflict Confusion Matrix (2.0s window):")
    print("        " + " ".join([f"{k:>5s}" for k in target_names]))
    for idx, row in enumerate(cm_conf):
        print(f"  {target_names[idx]:5s} " + " ".join([f"{val:5d}" for val in row]))

    # 4. Non-Conflicting Episodes (Nominal CPA > 2.0m): Prior Fallback Analysis
    free_mask = (cpa_test > 2.0)
    if np.sum(free_mask) > 0:
        probs_free = probs_test[free_mask]
        mean_p_free = np.mean(probs_free, axis=0)
        mean_entropy = -np.mean(np.sum(probs_free * np.log(probs_free + 1e-8), axis=1))
        max_entropy = np.log(5.0)
        print(f"\n[C. NON-CONFLICTING EPISODES (Nominal CPA > 2.0m, N={np.sum(free_mask)})]")
        print(f"  Mean Output Probabilities       : H1={mean_p_free[0]:.3f}, H2={mean_p_free[1]:.3f}, H3={mean_p_free[2]:.3f}, H4={mean_p_free[3]:.3f}, H5={mean_p_free[4]:.3f}")
        print(f"  Normalized Predictive Entropy   : {mean_entropy / max_entropy:.3f} (1.0 = completely diffuse / uniform)")

    # 5. Detection Latency / Early Recognition Analysis (Expanding Window 0.5s -> 2.0s)
    print("\n--- DETECTION LATENCY: TIME-TO-CONFIDENCE ACROSS WINDOW LENGTHS ---")
    window_lengths = [0.5, 1.0, 1.5, 2.0]
    latency_results = []

    for w_t in window_lengths:
        X_w = np.array([extract_window_features(ep["trajectory"], max_t=w_t) for ep in data])
        X_w_tr, X_w_te = X_w[train_idx], X_w[test_idx]
        clf_w = CalibratedClassifierCV(RandomForestClassifier(n_estimators=100, max_depth=10, random_state=42, class_weight='balanced'), cv=3)
        clf_w.fit(X_w_tr, y_train)
        p_w = clf_w.predict_proba(X_w_te)
        pred_w = np.argmax(p_w, axis=1)

        w_acc = accuracy_score(y_test[conflict_mask], pred_w[conflict_mask])
        w_f1 = f1_score(y_test[conflict_mask], pred_w[conflict_mask], average='macro')
        w_nll = log_loss(y_test[conflict_mask], p_w[conflict_mask])
        w_conf = np.mean(np.max(p_w[conflict_mask], axis=1))

        latency_results.append({
            "window_sec": w_t,
            "accuracy": float(w_acc),
            "macro_f1": float(w_f1),
            "nll": float(w_nll),
            "mean_confidence": float(w_conf)
        })
        print(f"  Window {w_t:.1f}s | Conflict Acc: {w_acc*100:.1f}% | Macro-F1: {w_f1:.3f} | Mean Confidence: {w_conf:.3f} | NLL: {w_nll:.4f}")

    # 6. Plot Calibration Curve & Latency Curve
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    # Plot 1: Reliability Diagram / Calibration Curve
    bin_confs = [b["confidence"] for b in bin_details]
    bin_accs = [b["accuracy"] for b in bin_details]
    axes[0].plot([0, 1], [0, 1], 'k--', label="Perfect Calibration")
    axes[0].plot(bin_confs, bin_accs, 's-', color='#1f77b4', linewidth=2.0, label=f"M3 Calibrated Model (ECE = {ece_overall*100:.2f}%)")
    axes[0].set_title("Reliability Diagram (Model Calibration on 2.0s Window)", fontsize=12, fontweight='bold')
    axes[0].set_xlabel("Predicted Confidence", fontsize=11)
    axes[0].set_ylabel("Empirical Accuracy", fontsize=11)
    axes[0].set_xlim([0, 1])
    axes[0].set_ylim([0, 1])
    axes[0].grid(True, linestyle='--', alpha=0.5)
    axes[0].legend(loc="upper left")

    # Plot 2: Accuracy & Confidence vs Window Duration (Early Recognition)
    w_secs = [r["window_sec"] for r in latency_results]
    w_accs = [r["accuracy"] * 100 for r in latency_results]
    w_confs = [r["mean_confidence"] * 100 for r in latency_results]

    axes[1].plot(w_secs, w_accs, 'o-', color='#2ca02c', linewidth=2.2, label="Conflict Accuracy (%)")
    axes[1].plot(w_secs, w_confs, 'd--', color='#ff7f0e', linewidth=2.2, label="Mean Confidence (%)")
    axes[1].set_title("Early Recognition Profile: Accuracy & Confidence vs Observation Window", fontsize=12, fontweight='bold')
    axes[1].set_xlabel("Observation Window Length (seconds)", fontsize=11)
    axes[1].set_ylabel("Percentage (%)", fontsize=11)
    axes[1].set_ylim([40, 100])
    axes[1].grid(True, linestyle='--', alpha=0.5)
    axes[1].legend(loc="lower right")

    fig_path = os.path.join(base_dir, 'results', 'figures', 'probabilistic_calibration_latency.png')
    plt.tight_layout()
    plt.savefig(fig_path, dpi=300)
    plt.close()

    # Save summary report JSON
    prob_summary = {
        "observation_window_seconds": 2.0,
        "full_test_metrics": {
            "overall_accuracy": float(acc_overall),
            "overall_macro_f1": float(f1_macro_overall),
            "dynamic_accuracy": float(acc_dynamic),
            "dynamic_macro_f1": float(f1_macro_dynamic),
            "nll": float(nll_overall),
            "brier_score": float(brier_overall),
            "ece": float(ece_overall)
        },
        "nominal_conflict_metrics": {
            "threshold_cpa_m": 1.2,
            "n_samples": int(n_conflict),
            "accuracy": float(acc_conf),
            "macro_f1": float(f1_conf),
            "dynamic_accuracy": float(acc_dyn_conf),
            "dynamic_macro_f1": float(f1_dyn_conf),
            "nll": float(nll_conf),
            "brier_score": float(brier_conf),
            "confusion_matrix": cm_conf.tolist()
        },
        "early_detection_latency": latency_results
    }
    json_path = os.path.join(base_dir, 'results', 'metrics', 'probabilistic_evaluation_results.json')
    with open(json_path, 'w') as f:
        json.dump(prob_summary, f, indent=2)

    print(f"\nSaved plots to {fig_path}")
    print(f"Saved results to {json_path}")

if __name__ == '__main__':
    main()
