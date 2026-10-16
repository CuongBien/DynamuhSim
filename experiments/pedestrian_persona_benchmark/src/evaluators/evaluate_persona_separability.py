#!/usr/bin/env python3
"""
Comprehensive H1-H5 Persona Separability & Observability Benchmark (M3 Real Constraints).
Features:
- Observation window: 2.0s (20 steps @ 10 Hz), sliding stride: 0.5s.
- 3 noise levels: sigma = 0.00m (clean), sigma = 0.05m, sigma = 0.10m.
- Finite-difference velocity calculation (no ground-truth velocities).
- Camera M3 Field-of-View constraints: 87 deg horizontal FOV (|theta| <= 43.5 deg), range [0.28m, 5.0m].
- Geometric Nominal CPA (tau = 1.5m and tau = 2.0m) evaluated strictly before window observation.
- 2 Classifiers: (a) Handcrafted + GBDT, (b) Sequence Temporal Model (MLP on raw temporal frames), plus Prior & Uniform baselines.
- 7 Mandated Report Sections:
  1. 5x5 and 4x4 Confusion Matrices, Precision, Recall, F1, Macro-F1 (saved to CSV).
  2. Subsets: Interactive (tau=1.5m, tau=2.0m), Non-interactive, All windows.
  3. H1 vs H2 confusion rate broken down by alpha bins: [0.10, 0.20), [0.20, 0.30), [0.30, 0.35].
  4. NLL, ECE (10 bins), Brier score, reliability diagram data.
  5. Early Detection Latency (P(true) >= 0.7 for >= 3 consecutive windows).
  6. Multi-dimensional slices: approach type, geometry (open, 0.9m, 1.2m), noise level.
  7. 95% Confidence Intervals via 1,000 Scenario-level Bootstrap iterations.
- Rule-based decision for H1/H2 merging threshold (30%).
"""

import os
import sys
import json
import time
import math
import csv
from collections import defaultdict
from typing import Dict, List, Tuple
import numpy as np

from sklearn.ensemble import HistGradientBoostingClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import confusion_matrix, precision_recall_fscore_support, f1_score, accuracy_score, log_loss


# ----------------------------------------------------------------------
# 1. Feature Extraction & Windowing Helpers
# ----------------------------------------------------------------------

HALF_FOV_RAD = math.radians(43.5) # 87 deg total horizontal FOV
MIN_CAMERA_RANGE = 0.28
MAX_CAMERA_RANGE = 5.0

def apply_sensor_noise_and_fov(window_frames: List[dict], sigma: float, rng: np.random.RandomState):
    """
    Applies Gaussian position noise and FOV occlusion masking.
    Computes finite-difference velocities.
    """
    T = len(window_frames)
    noisy_x = np.zeros(T)
    noisy_y = np.zeros(T)
    is_observed = np.zeros(T, dtype=np.float32)
    
    # 1. Add noise and check camera frustum
    for t in range(T):
        f = window_frames[t]
        raw_x = f["x_rel"]
        raw_y = f["y_rel"]
        
        if sigma > 0.0:
            nx = raw_x + rng.normal(0.0, sigma)
            ny = raw_y + rng.normal(0.0, sigma)
        else:
            nx = raw_x
            ny = raw_y
            
        dist = math.hypot(nx, ny)
        angle = math.atan2(ny, nx)
        
        # Check camera M3 horizontal FOV (|theta| <= 43.5 deg) and range [0.28, 5.0m]
        if (abs(angle) <= HALF_FOV_RAD) and (dist >= MIN_CAMERA_RANGE) and (dist <= MAX_CAMERA_RANGE):
            is_observed[t] = 1.0
            noisy_x[t] = nx
            noisy_y[t] = ny
        else:
            is_observed[t] = 0.0
            noisy_x[t] = nx # still track coordinates for smooth difference, but masked out in model
            noisy_y[t] = ny

    # 2. Recompute velocity by finite difference (dt = 0.1s)
    dt = 0.10
    noisy_vx = np.zeros(T)
    noisy_vy = np.zeros(T)
    for t in range(T):
        if t == 0:
            noisy_vx[t] = (noisy_x[1] - noisy_x[0]) / dt if T > 1 else 0.0
            noisy_vy[t] = (noisy_y[1] - noisy_y[0]) / dt if T > 1 else 0.0
        else:
            noisy_vx[t] = (noisy_x[t] - noisy_x[t-1]) / dt
            noisy_vy[t] = (noisy_y[t] - noisy_y[t-1]) / dt
            
    return noisy_x, noisy_y, noisy_vx, noisy_vy, is_observed


def extract_handcrafted_features(noisy_x, noisy_y, noisy_vx, noisy_vy, is_observed, window_frames) -> np.ndarray:
    """
    Extracts physically motivated features from 20-step observation window.
    """
    T = len(noisy_x)
    dists = np.hypot(noisy_x, noisy_y)
    angles = np.arctan2(noisy_y, noisy_x)
    speeds = np.hypot(noisy_vx, noisy_vy)
    
    # Valid observation mask
    obs_ratio = np.mean(is_observed)
    
    # Distance statistics
    d_min = np.min(dists)
    d_mean = np.mean(dists)
    d_start = dists[0]
    d_end = dists[-1]
    delta_d = d_end - d_start
    
    # Lateral displacement relative to initial bearing
    init_bearing = angles[0]
    lat_displacements = dists * np.sin(angles - init_bearing)
    lat_max = np.max(np.abs(lat_displacements))
    lat_end = lat_displacements[-1]
    
    # Angle rate
    delta_angle = angles[-1] - angles[0]
    abs_angle_rate = np.mean(np.abs(np.diff(angles))) / 0.10 if T > 1 else 0.0
    
    # Speed statistics
    speed_mean = np.mean(speeds)
    speed_std = np.std(speeds)
    speed_min = np.min(speeds)
    speed_max = np.max(speeds)
    speed_change = speeds[-1] - speeds[0]
    
    # Finite-difference acceleration
    if T > 1:
        accels = np.diff(speeds) / 0.10
        accel_mean = np.mean(np.abs(accels))
    else:
        accel_mean = 0.0
        
    # Robot actions
    r_vxs = [f["r_vx"] for f in window_frames]
    r_vys = [f["r_vy"] for f in window_frames]
    r_ws = [f["r_w"] for f in window_frames]
    r_vx_mean = np.mean(r_vxs)
    r_vy_mean = np.mean(r_vys)
    r_w_mean = np.mean(np.abs(r_ws))
    
    feats = np.array([
        d_min, d_mean, d_start, d_end, delta_d,
        lat_max, lat_end,
        delta_angle, abs_angle_rate,
        speed_mean, speed_std, speed_min, speed_max, speed_change,
        accel_mean, obs_ratio,
        r_vx_mean, r_vy_mean, r_w_mean,
        noisy_vx[-1], noisy_vy[-1]
    ], dtype=np.float32)
    
    return feats


def extract_sequence_features(noisy_x, noisy_y, noisy_vx, noisy_vy, is_observed, window_frames) -> np.ndarray:
    """
    Extracts raw 20-step temporal tensor: shape [20, 10]
    Channels: [x_rel, y_rel, vx_rel, vy_rel, dist, angle, r_vx, r_vy, r_w, is_observed]
    """
    T = len(noisy_x)
    seq = np.zeros((T, 10), dtype=np.float32)
    for t in range(T):
        f = window_frames[t]
        d = math.hypot(noisy_x[t], noisy_y[t])
        ang = math.atan2(noisy_y[t], noisy_x[t])
        obs = is_observed[t]
        seq[t, 0] = noisy_x[t] * obs
        seq[t, 1] = noisy_y[t] * obs
        seq[t, 2] = noisy_vx[t] * obs
        seq[t, 3] = noisy_vy[t] * obs
        seq[t, 4] = d * obs
        seq[t, 5] = ang * obs
        seq[t, 6] = f["r_vx"]
        seq[t, 7] = f["r_vy"]
        seq[t, 8] = f["r_w"]
        seq[t, 9] = obs
    return seq.flatten() # 200-dim temporal sequence vector for neural sequence classifier


# ----------------------------------------------------------------------
# 2. Calibration & Metric Functions
# ----------------------------------------------------------------------

def compute_ece(probs: np.ndarray, labels: np.ndarray, n_bins: int = 10) -> Tuple[float, List[dict]]:
    """
    Computes Expected Calibration Error (ECE) with 10 equal-width bins.
    """
    confidences = np.max(probs, axis=1)
    predictions = np.argmax(probs, axis=1)
    accuracies = (predictions == labels).astype(float)
    
    bin_boundaries = np.linspace(0.0, 1.0, n_bins + 1)
    ece = 0.0
    bin_details = []
    
    for i in range(n_bins):
        bin_lower = bin_boundaries[i]
        bin_upper = bin_boundaries[i + 1]
        
        in_bin = (confidences > bin_lower) & (confidences <= bin_upper)
        prop_in_bin = np.mean(in_bin)
        
        if prop_in_bin > 0:
            accuracy_in_bin = np.mean(accuracies[in_bin])
            avg_confidence_in_bin = np.mean(confidences[in_bin])
            ece += np.abs(avg_confidence_in_bin - accuracy_in_bin) * prop_in_bin
            bin_details.append({
                "bin": i,
                "range": [float(bin_lower), float(bin_upper)],
                "accuracy": float(accuracy_in_bin),
                "confidence": float(avg_confidence_in_bin),
                "count": int(np.sum(in_bin))
            })
        else:
            bin_details.append({
                "bin": i,
                "range": [float(bin_lower), float(bin_upper)],
                "accuracy": 0.0,
                "confidence": float((bin_lower + bin_upper) / 2.0),
                "count": 0
            })
            
    return float(ece), bin_details


def compute_brier_score(probs: np.ndarray, labels: np.ndarray) -> float:
    """
    Multi-class Brier score: mean squared error between probability distribution and one-hot true class.
    """
    N, K = probs.shape
    one_hot = np.zeros((N, K), dtype=np.float32)
    for i in range(N):
        one_hot[i, labels[i]] = 1.0
    return float(np.mean(np.sum((probs - one_hot) ** 2, axis=1)))


def compute_early_detection_latency(test_records: List[dict], probs: np.ndarray) -> Dict:
    """
    Calculates Early Detection Latency per scenario on interactive test set:
    First window where P(true_class) >= 0.70 sustained for >= 3 consecutive windows.
    Returns median detection distance and time to CPA.
    """
    sc_groups = defaultdict(list)
    for i, r in enumerate(test_records):
        prob_true = probs[i, r["persona_idx"]]
        sc_groups[r["scenario_id"]].append((
            r["window_idx"],
            prob_true,
            r["start_dist"],
            r["t_cpa"],
            r["is_interactive_15"]
        ))
    
    detection_dists = []
    time_to_cpas = []
    total_interactive_scenarios = 0
    detected_scenarios = 0
    
    for sc_id, items in sc_groups.items():
        is_interactive = any(item[4] for item in items)
        if not is_interactive:
            continue
            
        total_interactive_scenarios += 1
        items.sort(key=lambda x: x[0]) # sort by window_idx
        
        found = False
        for w in range(len(items) - 2):
            if items[w][1] >= 0.70 and items[w+1][1] >= 0.70 and items[w+2][1] >= 0.70:
                detection_dists.append(items[w][2])
                time_to_cpas.append(items[w][3])
                detected_scenarios += 1
                found = True
                break
                
    success_rate = (detected_scenarios / total_interactive_scenarios) if total_interactive_scenarios > 0 else 0.0
    median_dist = float(np.median(detection_dists)) if detection_dists else 0.0
    median_tt_cpa = float(np.median(time_to_cpas)) if time_to_cpas else 0.0
    
    return {
        "total_interactive_scenarios": total_interactive_scenarios,
        "detected_scenarios": detected_scenarios,
        "detection_success_rate": success_rate,
        "median_detection_dist_m": median_dist,
        "median_time_to_cpa_s": median_tt_cpa
    }


def save_confusion_matrix_csv(cm: np.ndarray, labels: List[str], filepath: str):
    """
    Saves confusion matrix as a formatted CSV table.
    """
    with open(filepath, "w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(["Actual \\ Predicted"] + [f"Pred_{l}" for l in labels])
        for i, l in enumerate(labels):
            writer.writerow([l] + list(cm[i]))


# ----------------------------------------------------------------------
# 3. Main Benchmark Execution
# ----------------------------------------------------------------------

def main():
    base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    dataset_path = os.path.join(base_dir, "results/datasets/h1_h5_scenarios_dataset.json")
    metrics_dir = os.path.join(base_dir, "results/metrics")
    
    print("=" * 80)
    print("M3 PERSONA OBSERVABILITY BENCHMARK (H1-H5 SEPARABILITY UNDER REAL CONSTRAINTS)")
    print("=" * 80)
    print(f"Loading 5,000 scenarios dataset from {dataset_path}...")
    
    t0 = time.time()
    with open(dataset_path, "r") as fp:
        raw_dataset = json.load(fp)
    print(f"Loaded {len(raw_dataset)} scenarios in {time.time() - t0:.2f} s")
    
    # ------------------------------------------------------------------
    # Step 2 & 3: Sliding Windows (2.0s, stride 0.5s) across all scenarios
    # ------------------------------------------------------------------
    print("\n[Step 2 & 3] Generating 2.0s sliding windows and geometric Nominal CPA flags...")
    
    records = []
    # window_len = 20 steps (2.0s @ 10Hz), stride = 5 steps (0.5s)
    W_LEN = 20
    STRIDE = 5
    
    for ep in raw_dataset:
        sc_id = ep["scenario_id"]
        split = ep["split"]
        p_name = ep["persona"]
        p_idx = ep["persona_idx"]
        geom = ep["geometry"]
        app_type = ep["approach_type"]
        alpha = ep["alpha"]
        frames = ep["frames"]
        n_frames = len(frames)
        
        n_windows = (n_frames - W_LEN) // STRIDE + 1
        for w in range(n_windows):
            start_f = w * STRIDE
            end_f = start_f + W_LEN
            sub_frames = frames[start_f:end_f]
            
            # Step 2: Nominal CPA at the BEGINNING of window (t_0)
            d_cpa = sub_frames[0]["d_cpa"]
            t_cpa = sub_frames[0]["t_cpa"]
            start_dist = sub_frames[0]["dist"]
            
            is_interactive_15 = (d_cpa <= 1.5) and (0.0 <= t_cpa <= 3.0)
            is_interactive_20 = (d_cpa <= 2.0) and (0.0 <= t_cpa <= 3.0)
            
            records.append({
                "scenario_id": sc_id,
                "window_idx": w,
                "split": split,
                "persona": p_name,
                "persona_idx": p_idx,
                "geometry": geom,
                "approach_type": app_type,
                "alpha": alpha,
                "d_cpa": d_cpa,
                "t_cpa": t_cpa,
                "start_dist": start_dist,
                "is_interactive_15": is_interactive_15,
                "is_interactive_20": is_interactive_20,
                "frames": sub_frames
            })
            
    total_windows = len(records)
    splits = np.array([r["split"] for r in records])
    train_mask = (splits == "train")
    val_mask = (splits == "val")
    test_mask = (splits == "test")
    
    train_scenarios = len(set(r["scenario_id"] for r in records if r["split"] == "train"))
    val_scenarios = len(set(r["scenario_id"] for r in records if r["split"] == "val"))
    test_scenarios = len(set(r["scenario_id"] for r in records if r["split"] == "test"))
    
    test_records = [records[i] for i in range(total_windows) if test_mask[i]]
    test_scenarios_list = [r["scenario_id"] for r in test_records]
    test_unique_scenarios = list(dict.fromkeys(test_scenarios_list))
    
    y_all = np.array([r["persona_idx"] for r in records], dtype=int)
    y_train = y_all[train_mask]
    y_test = y_all[test_mask]
    
    test_is_interactive_15 = np.array([r["is_interactive_15"] for r in test_records], dtype=bool)
    test_is_interactive_20 = np.array([r["is_interactive_20"] for r in test_records], dtype=bool)
    test_alpha = np.array([r["alpha"] for r in test_records], dtype=float)
    test_approach = np.array([r["approach_type"] for r in test_records])
    test_geometry = np.array([r["geometry"] for r in test_records])
    
    print(f"Total sliding windows generated: {total_windows}")
    print(f"  Train windows : {np.sum(train_mask)} ({train_scenarios} scenarios, 60%)")
    print(f"  Val windows   : {np.sum(val_mask)} ({val_scenarios} scenarios, 20%)")
    print(f"  Test windows  : {np.sum(test_mask)} ({test_scenarios} scenarios, 20%, SEALED)")
    print(f"  Interactive windows (tau=1.5m): {np.sum(test_is_interactive_15)} in test ({np.mean(test_is_interactive_15)*100:.1f}%)")
    print(f"  Interactive windows (tau=2.0m): {np.sum(test_is_interactive_20)} in test ({np.mean(test_is_interactive_20)*100:.1f}%)")

    # ------------------------------------------------------------------
    # Step 4, 5, 6: Evaluate across 3 Noise Levels
    # ------------------------------------------------------------------
    noise_levels = [0.00, 0.05, 0.10]
    
    benchmark_output = {
        "total_scenarios": len(raw_dataset),
        "total_windows": total_windows,
        "split_summary": {
            "train_scenarios": train_scenarios,
            "val_scenarios": val_scenarios,
            "test_scenarios": test_scenarios,
            "test_windows": int(np.sum(test_mask))
        },
        "noise_experiments": {}
    }
    
    cached_probs_005 = None
    
    for sigma in noise_levels:
        noise_key = f"sigma_{sigma:.2f}"
        print(f"\n========================================================================")
        print(f"[Step 4, 5, 6] RUNNING BENCHMARK AT NOISE LEVEL: sigma = {sigma:.2f} m")
        print(f"========================================================================")
        
        # Apply noise & extract features
        rng = np.random.RandomState(42 + int(sigma * 100))
        
        X_handcrafted = []
        X_sequence = []
        
        for r in records:
            nx, ny, nvx, nvy, is_obs = apply_sensor_noise_and_fov(r["frames"], sigma, rng)
            f_hand = extract_handcrafted_features(nx, ny, nvx, nvy, is_obs, r["frames"])
            f_seq = extract_sequence_features(nx, ny, nvx, nvy, is_obs, r["frames"])
            X_handcrafted.append(f_hand)
            X_sequence.append(f_seq)
            
        X_hand = np.array(X_handcrafted, dtype=np.float32)
        X_seq = np.array(X_sequence, dtype=np.float32)
        
        X_train_hand = X_hand[train_mask]
        X_test_hand = X_hand[test_mask]
        
        X_train_seq = X_seq[train_mask]
        X_test_seq = X_seq[test_mask]
        
        # --------------------------------------------------------------
        # Model (a): Handcrafted Features + HistGradientBoosting
        # --------------------------------------------------------------
        print(f"  Training Model (a): Handcrafted Features + GBDT (HistGradientBoosting)...")
        gbdt = HistGradientBoostingClassifier(
            max_iter=120, max_leaf_nodes=31, learning_rate=0.08, min_samples_leaf=20, random_state=42
        )
        gbdt.fit(X_train_hand, y_train)
        probs_gbdt = gbdt.predict_proba(X_test_hand)
        
        # --------------------------------------------------------------
        # Model (b): Sequence Model (MLP on raw temporal frames)
        # --------------------------------------------------------------
        print(f"  Training Model (b): Sequence Temporal Model (MLP on raw temporal frames)...")
        seq_model = MLPClassifier(
            hidden_layer_sizes=(128, 64), max_iter=40, learning_rate_init=0.003, batch_size=256, random_state=42
        )
        seq_model.fit(X_train_seq, y_train)
        probs_seq = seq_model.predict_proba(X_test_seq)
        
        # --------------------------------------------------------------
        # Baselines: Prior (40/10/25/15/10) and Uniform (20% each)
        # --------------------------------------------------------------
        prior_dist = np.array([0.40, 0.10, 0.25, 0.15, 0.10])
        probs_prior = np.tile(prior_dist, (len(y_test), 1))
        probs_uniform = np.full((len(y_test), 5), 0.20)
        
        models_dict = {
            "gbdt_handcrafted": probs_gbdt,
            "sequence_model": probs_seq,
            "prior_baseline": probs_prior,
            "uniform_baseline": probs_uniform
        }
        
        noise_results = {}
        for m_name, m_probs in models_dict.items():
            preds = np.argmax(m_probs, axis=1)
            
            # 1. Overall Metrics
            acc = accuracy_score(y_test, preds)
            macro_f1 = f1_score(y_test, preds, average="macro")
            nll = log_loss(y_test, m_probs)
            brier = compute_brier_score(m_probs, y_test)
            ece, ece_bins = compute_ece(m_probs, y_test, n_bins=10)
            
            # Precision, Recall, F1 per class (5 classes)
            prec_5, rec_5, f1_5, supp_5 = precision_recall_fscore_support(y_test, preds, labels=[0, 1, 2, 3, 4], zero_division=0)
            per_class_5 = {
                f"H{i+1}": {
                    "precision": float(prec_5[i]),
                    "recall": float(rec_5[i]),
                    "f1": float(f1_5[i]),
                    "support": int(supp_5[i])
                }
                for i in range(5)
            }
            
            # 2. Interactive tau = 1.5m
            m15 = test_is_interactive_15
            acc_15 = accuracy_score(y_test[m15], preds[m15]) if np.sum(m15) > 0 else 0.0
            macro_f1_15 = f1_score(y_test[m15], preds[m15], average="macro") if np.sum(m15) > 0 else 0.0
            
            # 3. Interactive tau = 2.0m
            m20 = test_is_interactive_20
            acc_20 = accuracy_score(y_test[m20], preds[m20]) if np.sum(m20) > 0 else 0.0
            macro_f1_20 = f1_score(y_test[m20], preds[m20], average="macro") if np.sum(m20) > 0 else 0.0
            
            # 4. Non-interactive
            m_non = ~m20
            acc_non = accuracy_score(y_test[m_non], preds[m_non]) if np.sum(m_non) > 0 else 0.0
            macro_f1_non = f1_score(y_test[m_non], preds[m_non], average="macro") if np.sum(m_non) > 0 else 0.0
            
            # 5. Confusion matrix 5x5 and 4x4 (no H4)
            cm_5x5 = confusion_matrix(y_test, preds, labels=[0, 1, 2, 3, 4])
            
            # 4x4 excluding H4 (H4 is idx 3: H1=0, H2=1, H3=2, H5=4)
            no_h4_mask = (y_test != 3)
            cm_4x4 = confusion_matrix(y_test[no_h4_mask], preds[no_h4_mask], labels=[0, 1, 2, 4])
            macro_f1_no_h4 = f1_score(y_test[no_h4_mask], preds[no_h4_mask], labels=[0, 1, 2, 4], average="macro")
            
            # Interactive 5x5 and 4x4
            cm_5x5_15 = confusion_matrix(y_test[m15], preds[m15], labels=[0, 1, 2, 3, 4]) if np.sum(m15) > 0 else np.zeros((5,5), dtype=int)
            no_h4_15 = m15 & (y_test != 3)
            cm_4x4_15 = confusion_matrix(y_test[no_h4_15], preds[no_h4_15], labels=[0, 1, 2, 4]) if np.sum(no_h4_15) > 0 else np.zeros((4,4), dtype=int)
            
            # H1 vs H2 confusion rate on interactive set (tau=1.5m and tau=2.0m)
            h2_mask_15 = m15 & (y_test == 1)
            h2_as_h1_rate_15 = np.mean(preds[h2_mask_15] == 0) if np.sum(h2_mask_15) > 0 else 0.0
            
            h1_mask_15 = m15 & (y_test == 0)
            h1_as_h2_rate_15 = np.mean(preds[h1_mask_15] == 1) if np.sum(h1_mask_15) > 0 else 0.0
            
            h2_mask_20 = m20 & (y_test == 1)
            h2_as_h1_rate_20 = np.mean(preds[h2_mask_20] == 0) if np.sum(h2_mask_20) > 0 else 0.0
            
            # Breakdown of H2 as H1 across alpha bins (tau=1.5m)
            bin1 = h2_mask_15 & (test_alpha >= 0.10) & (test_alpha < 0.20)
            bin2 = h2_mask_15 & (test_alpha >= 0.20) & (test_alpha < 0.30)
            bin3 = h2_mask_15 & (test_alpha >= 0.30) & (test_alpha <= 0.35)
            
            h2_h1_alpha_breakdown = {
                "bin_010_020": float(np.mean(preds[bin1] == 0)) if np.sum(bin1) > 0 else 0.0,
                "bin_020_030": float(np.mean(preds[bin2] == 0)) if np.sum(bin2) > 0 else 0.0,
                "bin_030_035": float(np.mean(preds[bin3] == 0)) if np.sum(bin3) > 0 else 0.0,
                "counts": [int(np.sum(bin1)), int(np.sum(bin2)), int(np.sum(bin3))]
            }
            
            # Early detection latency
            early_det = compute_early_detection_latency(test_records, m_probs) if "baseline" not in m_name else {}
            
            # Breakdown by approach & geometry
            breakdown_approach = {}
            for app in ["head_on", "crossing", "oblique", "same_direction"]:
                app_mask = (test_approach == app)
                breakdown_approach[app] = {
                    "accuracy": float(accuracy_score(y_test[app_mask], preds[app_mask])),
                    "macro_f1": float(f1_score(y_test[app_mask], preds[app_mask], average="macro")),
                    "count": int(np.sum(app_mask))
                }
                
            breakdown_geometry = {}
            for geom in ["open_6x6", "corridor_09", "corridor_12"]:
                geom_mask = (test_geometry == geom)
                breakdown_geometry[geom] = {
                    "accuracy": float(accuracy_score(y_test[geom_mask], preds[geom_mask])),
                    "macro_f1": float(f1_score(y_test[geom_mask], preds[geom_mask], average="macro")),
                    "count": int(np.sum(geom_mask))
                }

            noise_results[m_name] = {
                "overall_accuracy": float(acc),
                "overall_macro_f1": float(macro_f1),
                "macro_f1_no_h4": float(macro_f1_no_h4),
                "per_class_5": per_class_5,
                "nll": float(nll),
                "brier_score": float(brier),
                "ece": float(ece),
                "ece_bins": ece_bins,
                "interactive_tau15": {
                    "accuracy": float(acc_15),
                    "macro_f1": float(macro_f1_15),
                    "count": int(np.sum(m15)),
                    "h2_misclassified_as_h1_rate": float(h2_as_h1_rate_15),
                    "h1_misclassified_as_h2_rate": float(h1_as_h2_rate_15),
                    "h2_h1_alpha_breakdown": h2_h1_alpha_breakdown
                },
                "interactive_tau20": {
                    "accuracy": float(acc_20),
                    "macro_f1": float(macro_f1_20),
                    "count": int(np.sum(m20)),
                    "h2_misclassified_as_h1_rate": float(h2_as_h1_rate_20)
                },
                "non_interactive": {
                    "accuracy": float(acc_non),
                    "macro_f1": float(macro_f1_non),
                    "count": int(np.sum(m_non))
                },
                "early_detection": early_det,
                "breakdown_approach": breakdown_approach,
                "breakdown_geometry": breakdown_geometry,
                "confusion_matrix_5x5": cm_5x5.tolist(),
                "confusion_matrix_4x4_no_h4": cm_4x4.tolist(),
                "confusion_matrix_5x5_interactive15": cm_5x5_15.tolist(),
                "confusion_matrix_4x4_interactive15": cm_4x4_15.tolist()
            }
            
        benchmark_output["noise_experiments"][noise_key] = noise_results
        
        # Save confusion matrices as CSV for noise 0.05 (primary M3 condition)
        if abs(sigma - 0.05) < 1e-4:
            cached_probs_005 = probs_gbdt.copy()
            cm5 = np.array(noise_results["gbdt_handcrafted"]["confusion_matrix_5x5"])
            save_confusion_matrix_csv(cm5, ["H1", "H2", "H3", "H4", "H5"],
                                      os.path.join(metrics_dir, "confusion_matrix_5x5_sigma005.csv"))
            
            cm4 = np.array(noise_results["gbdt_handcrafted"]["confusion_matrix_4x4_no_h4"])
            save_confusion_matrix_csv(cm4, ["H1", "H2", "H3", "H5"],
                                      os.path.join(metrics_dir, "confusion_matrix_4x4_noH4_sigma005.csv"))

            cm5_15 = np.array(noise_results["gbdt_handcrafted"]["confusion_matrix_5x5_interactive15"])
            save_confusion_matrix_csv(cm5_15, ["H1", "H2", "H3", "H4", "H5"],
                                      os.path.join(metrics_dir, "confusion_matrix_5x5_interactive15_sigma005.csv"))

            cm4_15 = np.array(noise_results["gbdt_handcrafted"]["confusion_matrix_4x4_interactive15"])
            save_confusion_matrix_csv(cm4_15, ["H1", "H2", "H3", "H5"],
                                      os.path.join(metrics_dir, "confusion_matrix_4x4_interactive15_sigma005.csv"))

    # ------------------------------------------------------------------
    # Step 6.7: 95% Confidence Intervals via Scenario-level Bootstrap
    # (Evaluated on primary condition: sigma = 0.05 m, GBDT)
    # ------------------------------------------------------------------
    print("\n[Step 6.7] Performing 1,000 Scenario-level Bootstrap resamplings for 95% CI...")
    
    n_test_sc = len(test_unique_scenarios)
    
    boot_accs = []
    boot_macro_f1s = []
    boot_f1_15s = []
    boot_h2_h1_errs = []
    boot_eces = []
    
    b_rng = np.random.RandomState(2026)
    
    # Pre-map scenario_id to row indices in test set
    sc_to_indices = defaultdict(list)
    for idx, sc in enumerate(test_scenarios_list):
        sc_to_indices[sc].append(idx)
        
    cached_probs = cached_probs_005
    cached_preds = np.argmax(cached_probs, axis=1)
    
    for b in range(1000):
        sampled_scs = b_rng.choice(test_unique_scenarios, size=n_test_sc, replace=True)
        boot_idx = []
        for s in sampled_scs:
            boot_idx.extend(sc_to_indices[s])
        boot_idx = np.array(boot_idx)
        
        y_b = y_test[boot_idx]
        p_b = cached_preds[boot_idx]
        pr_b = cached_probs[boot_idx]
        
        # Overall
        boot_accs.append(accuracy_score(y_b, p_b))
        boot_macro_f1s.append(f1_score(y_b, p_b, average="macro"))
        
        # Interactive tau=1.5
        m15_b = test_is_interactive_15[boot_idx]
        if np.sum(m15_b) > 0:
            boot_f1_15s.append(f1_score(y_b[m15_b], p_b[m15_b], average="macro"))
            h2_mask_b = m15_b & (y_b == 1)
            if np.sum(h2_mask_b) > 0:
                boot_h2_h1_errs.append(np.mean(p_b[h2_mask_b] == 0))
                
        # ECE
        b_ece, _ = compute_ece(pr_b, y_b, n_bins=10)
        boot_eces.append(b_ece)
        
    ci_results = {
        "overall_accuracy_ci95": [float(np.percentile(boot_accs, 2.5)), float(np.percentile(boot_accs, 97.5))],
        "overall_macro_f1_ci95": [float(np.percentile(boot_macro_f1s, 2.5)), float(np.percentile(boot_macro_f1s, 97.5))],
        "interactive_15_macro_f1_ci95": [float(np.percentile(boot_f1_15s, 2.5)), float(np.percentile(boot_f1_15s, 97.5))],
        "h2_misclassified_as_h1_rate_ci95": [float(np.percentile(boot_h2_h1_errs, 2.5)), float(np.percentile(boot_h2_h1_errs, 97.5))],
        "ece_ci95": [float(np.percentile(boot_eces, 2.5)), float(np.percentile(boot_eces, 97.5))]
    }
    
    benchmark_output["bootstrap_ci95_sigma005"] = ci_results
    
    # ------------------------------------------------------------------
    # Step 7: Rule-based Decision Evaluation
    # ------------------------------------------------------------------
    h2_h1_rate_mean = benchmark_output["noise_experiments"]["sigma_0.05"]["gbdt_handcrafted"]["interactive_tau15"]["h2_misclassified_as_h1_rate"]
    ci_upper = ci_results["h2_misclassified_as_h1_rate_ci95"][1]
    
    should_merge_h1_h2 = (h2_h1_rate_mean > 0.30) or (ci_upper > 0.30)
    decision = {
        "rule_threshold": 0.30,
        "measured_h2_as_h1_rate": h2_h1_rate_mean,
        "ci95_range": ci_results["h2_misclassified_as_h1_rate_ci95"],
        "exceeds_threshold": bool(should_merge_h1_h2),
        "recommendation": (
            "ĐỀ XUẤT GỘP H1 VÀ H2 (hoặc chuyển sang hồi quy alpha liên tục cho cụm này)"
            if should_merge_h1_h2 else
            "GIỮ NGUYÊN H1–H5 (tách biệt thỏa mãn quy tắc, tỷ lệ nhầm lẫn <= 30% và CI95 không chạm ngưỡng)"
        )
    }
    benchmark_output["decision_rule"] = decision

    # Save complete JSON
    out_json_path = os.path.join(metrics_dir, "h1_h5_separability_results.json")
    with open(out_json_path, "w") as fp:
        json.dump(benchmark_output, fp, indent=2)
        
    print(f"\nSaved benchmark results to {out_json_path}")
    
    # Print Executive Summary Table
    print("\n" + "=" * 80)
    print("KẾT QUẢ ĐO LƯỜNG TỔNG HỢP (M3 OBSERVABILITY BENCHMARK):")
    print("=" * 80)
    print(f"1. Tổng số kịch bản: {len(raw_dataset)} (1,000 kịch bản/persona)")
    print(f"2. Tổng số cửa sổ test niêm phong: {len(test_records)}")
    print(f"\n[KẾT QUẢ TẠI NHIỄU CHUẨN M3: sigma = 0.05 m]:")
    res_005 = benchmark_output["noise_experiments"]["sigma_0.05"]["gbdt_handcrafted"]
    print(f"  - Macro-F1 Toàn bộ      : {res_005['overall_macro_f1']*100:.2f}% (95% CI: [{ci_results['overall_macro_f1_ci95'][0]*100:.2f}%, {ci_results['overall_macro_f1_ci95'][1]*100:.2f}%])")
    print(f"  - Macro-F1 Bỏ H4 (4x4)  : {res_005['macro_f1_no_h4']*100:.2f}%")
    print(f"  - Macro-F1 Có tương tác (tau=1.5m): {res_005['interactive_tau15']['macro_f1']*100:.2f}% (95% CI: [{ci_results['interactive_15_macro_f1_ci95'][0]*100:.2f}%, {ci_results['interactive_15_macro_f1_ci95'][1]*100:.2f}%])")
    print(f"  - Macro-F1 Có tương tác (tau=2.0m): {res_005['interactive_tau20']['macro_f1']*100:.2f}%")
    print(f"  - Macro-F1 Không tương tác        : {res_005['non_interactive']['macro_f1']*100:.2f}%")
    print(f"  - Tỷ lệ H2 bị nhầm thành H1 (tau=1.5m): {res_005['interactive_tau15']['h2_misclassified_as_h1_rate']*100:.2f}% (95% CI: [{ci_results['h2_misclassified_as_h1_rate_ci95'][0]*100:.2f}%, {ci_results['h2_misclassified_as_h1_rate_ci95'][1]*100:.2f}%])")
    print(f"  - Chi tiết theo dải alpha:")
    ab = res_005['interactive_tau15']['h2_h1_alpha_breakdown']
    print(f"      [0.10, 0.20): {ab['bin_010_020']*100:.2f}% nhầm H1 (N={ab['counts'][0]})")
    print(f"      [0.20, 0.30): {ab['bin_020_030']*100:.2f}% nhầm H1 (N={ab['counts'][1]})")
    print(f"      [0.30, 0.35]: {ab['bin_030_035']*100:.2f}% nhầm H1 (N={ab['counts'][2]})")
    print(f"  - Hiệu chỉnh ECE (10 bins): {res_005['ece']*100:.2f}% (95% CI: [{ci_results['ece_ci95'][0]*100:.2f}%, {ci_results['ece_ci95'][1]*100:.2f}%]), NLL = {res_005['nll']:.4f}, Brier = {res_005['brier_score']:.4f}")
    
    ed = res_005['early_detection']
    print(f"  - Độ sớm nhận ra (Early Detection):")
    print(f"      Tỷ lệ nhận ra thành công: {ed['detection_success_rate']*100:.2f}%")
    print(f"      Trung vị khoảng cách khi nhận ra: {ed['median_detection_dist_m']:.2f} m")
    print(f"      Trung vị thời gian tới CPA      : {ed['median_time_to_cpa_s']:.2f} s")
    
    print("\n" + "=" * 80)
    print(f"KẾT LUẬN QUY TẮC (BƯỚC 7):")
    print(f"  Ngưỡng quy định: 30.0% nhầm lẫn")
    print(f"  Tỷ lệ H2 nhầm H1 đo được: {h2_h1_rate_mean*100:.2f}% (CI95: [{ci_results['h2_misclassified_as_h1_rate_ci95'][0]*100:.2f}%, {ci_results['h2_misclassified_as_h1_rate_ci95'][1]*100:.2f}%])")
    print(f"  QUYẾT ĐỊNH: {decision['recommendation']}")
    print("=" * 80)


if __name__ == "__main__":
    main()
