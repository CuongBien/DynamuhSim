#!/usr/bin/env python3
"""
Comprehensive H1-H5 Persona Observability Benchmark (Version 2 - Refined State Estimation & PyTorch Sequence Modeling).
Key Improvements:
1. Replaced finite difference with Degree-2 Polynomial State Estimation over 20-step window (Kalman/Least-Squares smoothing).
2. Noise levels evaluated: sigma = 0.02 m, 0.05 m, and 0.10 m.
3. PyTorch sequence temporal model (PersonaGRU with early stopping) replaces sklearn MLP.
4. Accuracy and Macro-F1 tracked over time relative to closest point (from -3.0s to +2.0s).
5. Binary classification task: "Does pedestrian avoid robot?" (H3 vs H1+H2+H5), reporting ROC-AUC.
6. Prior baseline evaluated on natural class distribution (40/10/25/15/10) with importance weights.
7. Corridor-specific confusion matrices (Corridor 0.9m, Corridor 1.2m, Combined corridor, Open 6x6m).
8. 95% Bootstrap Confidence Intervals (1,000 scenario-level resamplings).
9. Updated Step 7 decision rule with proper state estimation.
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

import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader

from sklearn.ensemble import HistGradientBoostingClassifier
from sklearn.metrics import (
    confusion_matrix, precision_recall_fscore_support, f1_score,
    accuracy_score, log_loss, roc_auc_score
)


# ----------------------------------------------------------------------
# 1. State Estimation (Degree-2 Polynomial Regression / Smooth Filter)
# ----------------------------------------------------------------------

HALF_FOV_RAD = math.radians(43.5) # 87 deg total horizontal FOV
MIN_CAMERA_RANGE = 0.28
MAX_CAMERA_RANGE = 5.0

def fit_polynomial_state(noisy_x: np.ndarray, noisy_y: np.ndarray, is_observed: np.ndarray, dt: float = 0.10):
    """
    Fits degree-2 polynomial over observed points in 20-step window.
    Yields smoothed coordinates, analytical velocities (2*a2*t + a1), and accelerations (2*a2).
    """
    T = len(noisy_x)
    t = np.arange(T) * dt
    obs_idx = np.where(is_observed > 0.5)[0]
    
    smooth_x = np.copy(noisy_x)
    smooth_y = np.copy(noisy_y)
    vx = np.zeros(T)
    vy = np.zeros(T)
    ax = np.zeros(T)
    ay = np.zeros(T)
    
    if len(obs_idx) >= 3:
        t_obs = t[obs_idx]
        px = np.polyfit(t_obs, noisy_x[obs_idx], 2)
        py = np.polyfit(t_obs, noisy_y[obs_idx], 2)
        
        smooth_x = px[0] * t**2 + px[1] * t + px[2]
        smooth_y = py[0] * t**2 + py[1] * t + py[2]
        vx = 2 * px[0] * t + px[1]
        vy = 2 * py[0] * t + py[1]
        ax = np.full(T, 2 * px[0])
        ay = np.full(T, 2 * py[0])
    elif len(obs_idx) == 2:
        t_obs = t[obs_idx]
        px = np.polyfit(t_obs, noisy_x[obs_idx], 1)
        py = np.polyfit(t_obs, noisy_y[obs_idx], 1)
        smooth_x = px[0] * t + px[1]
        smooth_y = py[0] * t + py[1]
        vx = np.full(T, px[0])
        vy = np.full(T, py[0])
    elif len(obs_idx) == 1:
        smooth_x = np.full(T, noisy_x[obs_idx[0]])
        smooth_y = np.full(T, noisy_y[obs_idx[0]])
        
    return smooth_x, smooth_y, vx, vy, ax, ay


def apply_sensor_noise_and_poly_estimation(window_frames: List[dict], sigma: float, rng: np.random.RandomState):
    """
    Applies Gaussian position noise, FOV truncation, and degree-2 polynomial state estimation.
    """
    T = len(window_frames)
    noisy_x = np.zeros(T)
    noisy_y = np.zeros(T)
    is_observed = np.zeros(T, dtype=np.float32)
    
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
        
        if (abs(angle) <= HALF_FOV_RAD) and (dist >= MIN_CAMERA_RANGE) and (dist <= MAX_CAMERA_RANGE):
            is_observed[t] = 1.0
            noisy_x[t] = nx
            noisy_y[t] = ny
        else:
            is_observed[t] = 0.0
            noisy_x[t] = nx
            noisy_y[t] = ny
            
    smooth_x, smooth_y, vx, vy, ax, ay = fit_polynomial_state(noisy_x, noisy_y, is_observed, dt=0.10)
    return smooth_x, smooth_y, vx, vy, ax, ay, is_observed


def extract_handcrafted_features_poly(smooth_x, smooth_y, vx, vy, ax, ay, is_observed, window_frames) -> np.ndarray:
    """
    Extracts physically motivated features from smoothed polynomial states.
    """
    T = len(smooth_x)
    dists = np.hypot(smooth_x, smooth_y)
    angles = np.arctan2(smooth_y, smooth_x)
    speeds = np.hypot(vx, vy)
    
    obs_ratio = np.mean(is_observed)
    
    d_min = np.min(dists)
    d_mean = np.mean(dists)
    d_start = dists[0]
    d_end = dists[-1]
    delta_d = d_end - d_start
    
    init_bearing = angles[0]
    lat_displacements = dists * np.sin(angles - init_bearing)
    lat_max = np.max(np.abs(lat_displacements))
    lat_end = lat_displacements[-1]
    
    delta_angle = angles[-1] - angles[0]
    abs_angle_rate = np.mean(np.abs(np.diff(angles))) / 0.10 if T > 1 else 0.0
    
    speed_mean = np.mean(speeds)
    speed_std = np.std(speeds)
    speed_min = np.min(speeds)
    speed_max = np.max(speeds)
    speed_change = speeds[-1] - speeds[0]
    
    accel_mag = np.hypot(ax, ay)
    accel_mean = np.mean(accel_mag)
    
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
        vx[-1], vy[-1], ax[-1], ay[-1]
    ], dtype=np.float32)
    
    return feats


def extract_sequence_tensor(smooth_x, smooth_y, vx, vy, is_observed, window_frames) -> np.ndarray:
    """
    Extracts raw 20-step temporal tensor: shape [20, 10]
    Channels: [smooth_x, smooth_y, vx, vy, dist, angle, r_vx, r_vy, r_w, is_observed]
    """
    T = len(smooth_x)
    seq = np.zeros((T, 10), dtype=np.float32)
    for t in range(T):
        f = window_frames[t]
        d = math.hypot(smooth_x[t], smooth_y[t])
        ang = math.atan2(smooth_y[t], smooth_x[t])
        obs = is_observed[t]
        seq[t, 0] = smooth_x[t] * obs
        seq[t, 1] = smooth_y[t] * obs
        seq[t, 2] = vx[t] * obs
        seq[t, 3] = vy[t] * obs
        seq[t, 4] = d * obs
        seq[t, 5] = ang * obs
        seq[t, 6] = f["r_vx"]
        seq[t, 7] = f["r_vy"]
        seq[t, 8] = f["r_w"]
        seq[t, 9] = obs
    return seq


# ----------------------------------------------------------------------
# 2. PyTorch Sequence Model (PersonaGRU with Early Stopping)
# ----------------------------------------------------------------------

class PersonaGRU(nn.Module):
    def __init__(self, input_dim: int = 10, hidden_dim: int = 64, num_layers: int = 2, num_classes: int = 5, dropout: float = 0.2):
        super().__init__()
        self.gru = nn.GRU(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0,
            bidirectional=False
        )
        self.classifier = nn.Sequential(
            nn.Linear(hidden_dim, 32),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(32, num_classes)
        )
        
    def forward(self, x):
        # x: [B, T=20, C=10]
        out, _ = self.gru(x)
        last_hidden = out[:, -1, :] # final step representation
        logits = self.classifier(last_hidden)
        return logits


def train_pytorch_gru(X_tr: np.ndarray, y_tr: np.ndarray, X_va: np.ndarray, y_va: np.ndarray,
                      epochs: int = 30, batch_size: int = 256, patience: int = 5, seed: int = 42):
    """
    Trains PersonaGRU with Early Stopping on validation Macro-F1.
    """
    torch.manual_seed(seed)
    np.random.seed(seed)
    
    device = torch.device("cpu")
    model = PersonaGRU().to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.003, weight_decay=1e-4)
    
    tr_dataset = TensorDataset(torch.tensor(X_tr, dtype=torch.float32), torch.tensor(y_tr, dtype=torch.long))
    va_dataset = TensorDataset(torch.tensor(X_va, dtype=torch.float32), torch.tensor(y_va, dtype=torch.long))
    
    tr_loader = DataLoader(tr_dataset, batch_size=batch_size, shuffle=True)
    va_loader = DataLoader(va_dataset, batch_size=batch_size, shuffle=False)
    
    best_val_f1 = -1.0
    best_weights = None
    epochs_no_improve = 0
    
    for epoch in range(epochs):
        model.train()
        for batch_x, batch_y in tr_loader:
            optimizer.zero_grad()
            logits = model(batch_x)
            loss = criterion(logits, batch_y)
            loss.backward()
            optimizer.step()
            
        # Validation
        model.eval()
        val_preds = []
        with torch.no_grad():
            for batch_x, _ in va_loader:
                logits = model(batch_x)
                preds = torch.argmax(logits, dim=1).cpu().numpy()
                val_preds.extend(preds)
                
        val_f1 = f1_score(y_va, val_preds, average="macro")
        
        if val_f1 > best_val_f1:
            best_val_f1 = val_f1
            best_weights = {k: v.cpu().clone() for k, v in model.state_dict().items()}
            epochs_no_improve = 0
        else:
            epochs_no_improve += 1
            if epochs_no_improve >= patience:
                # Early stop
                break
                
    if best_weights is not None:
        model.load_state_dict(best_weights)
        
    return model


def predict_pytorch_gru(model: PersonaGRU, X_te: np.ndarray, batch_size: int = 256) -> np.ndarray:
    model.eval()
    dataset = TensorDataset(torch.tensor(X_te, dtype=torch.float32))
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=False)
    all_probs = []
    softmax = nn.Softmax(dim=1)
    with torch.no_grad():
        for (bx,) in loader:
            logits = model(bx)
            probs = softmax(logits).cpu().numpy()
            all_probs.append(probs)
    return np.vstack(all_probs)


# ----------------------------------------------------------------------
# 3. Metric & Evaluation Helpers
# ----------------------------------------------------------------------

def compute_ece(probs: np.ndarray, labels: np.ndarray, n_bins: int = 10) -> Tuple[float, List[dict]]:
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
            acc_in_bin = np.mean(accuracies[in_bin])
            conf_in_bin = np.mean(confidences[in_bin])
            ece += np.abs(conf_in_bin - acc_in_bin) * prop_in_bin
            bin_details.append({
                "bin": i,
                "range": [float(bin_lower), float(bin_upper)],
                "accuracy": float(acc_in_bin),
                "confidence": float(conf_in_bin),
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


def compute_brier_score(probs: np.ndarray, labels: np.ndarray, sample_weight: np.ndarray = None) -> float:
    N, K = probs.shape
    one_hot = np.zeros((N, K), dtype=np.float32)
    for i in range(N):
        one_hot[i, labels[i]] = 1.0
    sq_err = np.sum((probs - one_hot) ** 2, axis=1)
    if sample_weight is not None:
        return float(np.average(sq_err, weights=sample_weight))
    return float(np.mean(sq_err))


def save_confusion_matrix_csv(cm: np.ndarray, labels: List[str], filepath: str):
    with open(filepath, "w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(["Actual \\ Predicted"] + [f"Pred_{l}" for l in labels])
        for i, l in enumerate(labels):
            writer.writerow([l] + list(cm[i]))


# ----------------------------------------------------------------------
# 4. Main Benchmark Execution
# ----------------------------------------------------------------------

def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    dataset_path = os.path.join(root_dir, "experiments/pedestrian_persona_benchmark/h1_h5_scenarios_dataset.json")
    
    print("=" * 80)
    print("M3 PERSONA OBSERVABILITY BENCHMARK V2 (POLYNOMIAL STATE ESTIMATION & PYTORCH GRU)")
    print("=" * 80)
    print(f"Loading 5,000 scenarios dataset from {dataset_path}...")
    
    t0 = time.time()
    with open(dataset_path, "r") as fp:
        raw_dataset = json.load(fp)
    print(f"Loaded {len(raw_dataset)} scenarios in {time.time() - t0:.2f} s")
    
    # ------------------------------------------------------------------
    # Step 2 & 3: Sliding Windows (2.0s, stride 0.5s) across all scenarios
    # ------------------------------------------------------------------
    print("\n[Step 2 & 3] Precomputing sliding windows, Nominal CPA, and Closest-Point relative time...")
    
    records = []
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
        
        # Scenario minimum distance (closest point)
        dists_all = [f["dist"] for f in frames]
        min_idx = int(np.argmin(dists_all))
        t_closest = min_idx * 0.10
        
        n_windows = (n_frames - W_LEN) // STRIDE + 1
        for w in range(n_windows):
            start_f = w * STRIDE
            end_f = start_f + W_LEN
            sub_frames = frames[start_f:end_f]
            
            d_cpa = sub_frames[0]["d_cpa"]
            t_cpa = sub_frames[0]["t_cpa"]
            start_dist = sub_frames[0]["dist"]
            
            # Nominal CPA interaction
            is_interactive_15 = (d_cpa <= 1.5) and (0.0 <= t_cpa <= 3.0)
            is_interactive_20 = (d_cpa <= 2.0) and (0.0 <= t_cpa <= 3.0)
            
            # Time relative to closest point at end of window
            t_window_end = end_f * 0.10
            dt_closest = t_window_end - t_closest
            
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
                "t_closest": t_closest,
                "dt_closest": dt_closest,
                "frames": sub_frames
            })
            
    total_windows = len(records)
    splits = np.array([r["split"] for r in records])
    train_mask = (splits == "train")
    val_mask = (splits == "val")
    test_mask = (splits == "test")
    
    test_records = [records[i] for i in range(total_windows) if test_mask[i]]
    test_scenarios_list = [r["scenario_id"] for r in test_records]
    test_unique_scenarios = list(dict.fromkeys(test_scenarios_list))
    
    y_all = np.array([r["persona_idx"] for r in records], dtype=int)
    y_train = y_all[train_mask]
    y_val = y_all[val_mask]
    y_test = y_all[test_mask]
    
    test_is_interactive_15 = np.array([r["is_interactive_15"] for r in test_records], dtype=bool)
    test_is_interactive_20 = np.array([r["is_interactive_20"] for r in test_records], dtype=bool)
    test_alpha = np.array([r["alpha"] for r in test_records], dtype=float)
    test_approach = np.array([r["approach_type"] for r in test_records])
    test_geometry = np.array([r["geometry"] for r in test_records])
    test_dt_closest = np.array([r["dt_closest"] for r in test_records])
    
    # Natural prevalence importance weights (40/10/25/15/10)
    # Uniform prevalence in test set is 0.20 per class. Weight = prior[c] / 0.20
    natural_prior = np.array([0.40, 0.10, 0.25, 0.15, 0.10])
    test_natural_weights = natural_prior[y_test] / 0.20
    
    print(f"Total sliding windows: {total_windows} (Train: {np.sum(train_mask)}, Val: {np.sum(val_mask)}, Test: {np.sum(test_mask)})")
    
    # ------------------------------------------------------------------
    # Step 4, 5, 6: Benchmark Across 3 Noise Levels: sigma = 0.02, 0.05, 0.10
    # ------------------------------------------------------------------
    noise_levels = [0.02, 0.05, 0.10]
    
    benchmark_output = {
        "version": "v2_polynomial_state_estimation_and_pytorch_gru",
        "total_scenarios": len(raw_dataset),
        "total_windows": total_windows,
        "noise_experiments": {}
    }
    
    cached_probs_005_gru = None
    cached_probs_005_gbdt = None
    
    for sigma in noise_levels:
        noise_key = f"sigma_{sigma:.2f}"
        print(f"\n========================================================================")
        print(f"[EVALUATION] NOISE LEVEL: sigma = {sigma:.2f} m (Polynomial Degree-2 Smoothing)")
        print(f"========================================================================")
        
        rng = np.random.RandomState(42 + int(sigma * 100))
        
        X_handcrafted = []
        X_seq_tensors = []
        
        for r in records:
            sx, sy, vx, vy, ax, ay, is_obs = apply_sensor_noise_and_poly_estimation(r["frames"], sigma, rng)
            f_hand = extract_handcrafted_features_poly(sx, sy, vx, vy, ax, ay, is_obs, r["frames"])
            f_seq = extract_sequence_tensor(sx, sy, vx, vy, is_obs, r["frames"])
            X_handcrafted.append(f_hand)
            X_seq_tensors.append(f_seq)
            
        X_hand = np.array(X_handcrafted, dtype=np.float32)
        X_seq = np.array(X_seq_tensors, dtype=np.float32) # [N, 20, 10]
        
        X_train_hand, X_val_hand, X_test_hand = X_hand[train_mask], X_hand[val_mask], X_hand[test_mask]
        X_train_seq, X_val_seq, X_test_seq = X_seq[train_mask], X_seq[val_mask], X_seq[test_mask]
        
        # 1. PyTorch GRU Model
        print(f"  Training PyTorch PersonaGRU (2-layer GRU with Early Stopping)...")
        gru_model = train_pytorch_gru(
            X_train_seq, y_train, X_val_seq, y_val,
            epochs=25, batch_size=256, patience=5, seed=42
        )
        probs_gru = predict_pytorch_gru(gru_model, X_test_seq)
        
        # 2. Handcrafted Features + GBDT (HistGradientBoosting)
        print(f"  Training Handcrafted GBDT (HistGradientBoosting with smooth polynomial features)...")
        gbdt = HistGradientBoostingClassifier(
            max_iter=120, max_leaf_nodes=31, learning_rate=0.08, min_samples_leaf=20, random_state=42
        )
        gbdt.fit(X_train_hand, y_train)
        probs_gbdt = gbdt.predict_proba(X_test_hand)
        
        # 3. Baselines: Natural Prior & Uniform
        probs_prior = np.tile(natural_prior, (len(y_test), 1))
        probs_uniform = np.full((len(y_test), 5), 0.20)
        
        models_dict = {
            "pytorch_gru": probs_gru,
            "gbdt_handcrafted": probs_gbdt,
            "prior_baseline": probs_prior,
            "uniform_baseline": probs_uniform
        }
        
        if abs(sigma - 0.05) < 1e-4:
            cached_probs_005_gru = probs_gru.copy()
            cached_probs_005_gbdt = probs_gbdt.copy()
            
        noise_results = {}
        for m_name, m_probs in models_dict.items():
            preds = np.argmax(m_probs, axis=1)
            
            # Overall metrics
            acc = accuracy_score(y_test, preds)
            acc_weighted = accuracy_score(y_test, preds, sample_weight=test_natural_weights)
            macro_f1 = f1_score(y_test, preds, average="macro")
            nll = log_loss(y_test, m_probs)
            nll_weighted = log_loss(y_test, m_probs, sample_weight=test_natural_weights)
            brier = compute_brier_score(m_probs, y_test)
            brier_weighted = compute_brier_score(m_probs, y_test, sample_weight=test_natural_weights)
            ece, ece_bins = compute_ece(m_probs, y_test, n_bins=10)
            
            # Per-class 5
            prec_5, rec_5, f1_5, supp_5 = precision_recall_fscore_support(y_test, preds, labels=[0, 1, 2, 3, 4], zero_division=0)
            per_class_5 = {
                f"H{i+1}": {
                    "precision": float(prec_5[i]), "recall": float(rec_5[i]),
                    "f1": float(f1_5[i]), "support": int(supp_5[i])
                }
                for i in range(5)
            }
            
            # 4x4 (No H4)
            no_h4_mask = (y_test != 3)
            cm_4x4 = confusion_matrix(y_test[no_h4_mask], preds[no_h4_mask], labels=[0, 1, 2, 4])
            macro_f1_no_h4 = f1_score(y_test[no_h4_mask], preds[no_h4_mask], labels=[0, 1, 2, 4], average="macro")
            
            # Interactive tau = 1.5m
            m15 = test_is_interactive_15
            acc_15 = accuracy_score(y_test[m15], preds[m15]) if np.sum(m15) > 0 else 0.0
            macro_f1_15 = f1_score(y_test[m15], preds[m15], average="macro") if np.sum(m15) > 0 else 0.0
            no_h4_15 = m15 & (y_test != 3)
            macro_f1_15_no_h4 = f1_score(y_test[no_h4_15], preds[no_h4_15], labels=[0, 1, 2, 4], average="macro") if np.sum(no_h4_15) > 0 else 0.0
            cm_5x5_15 = confusion_matrix(y_test[m15], preds[m15], labels=[0, 1, 2, 3, 4]) if np.sum(m15) > 0 else np.zeros((5,5), dtype=int)
            
            # Interactive tau = 2.0m
            m20 = test_is_interactive_20
            acc_20 = accuracy_score(y_test[m20], preds[m20]) if np.sum(m20) > 0 else 0.0
            macro_f1_20 = f1_score(y_test[m20], preds[m20], average="macro") if np.sum(m20) > 0 else 0.0
            
            # Non-interactive
            m_non = ~m20
            macro_f1_non = f1_score(y_test[m_non], preds[m_non], average="macro") if np.sum(m_non) > 0 else 0.0
            
            # H1 vs H2 confusion rate (interactive 1.5m)
            h2_mask_15 = m15 & (y_test == 1)
            h2_as_h1_rate_15 = np.mean(preds[h2_mask_15] == 0) if np.sum(h2_mask_15) > 0 else 0.0
            h1_mask_15 = m15 & (y_test == 0)
            h1_as_h2_rate_15 = np.mean(preds[h1_mask_15] == 1) if np.sum(h1_mask_15) > 0 else 0.0
            
            # Breakdown across alpha bins
            bin1 = h2_mask_15 & (test_alpha >= 0.10) & (test_alpha < 0.20)
            bin2 = h2_mask_15 & (test_alpha >= 0.20) & (test_alpha < 0.30)
            bin3 = h2_mask_15 & (test_alpha >= 0.30) & (test_alpha <= 0.35)
            h2_h1_alpha_breakdown = {
                "bin_010_020": float(np.mean(preds[bin1] == 0)) if np.sum(bin1) > 0 else 0.0,
                "bin_020_030": float(np.mean(preds[bin2] == 0)) if np.sum(bin2) > 0 else 0.0,
                "bin_030_035": float(np.mean(preds[bin3] == 0)) if np.sum(bin3) > 0 else 0.0,
                "counts": [int(np.sum(bin1)), int(np.sum(bin2)), int(np.sum(bin3))]
            }
            
            # ----------------------------------------------------------
            # Binary Classification: "Does Pedestrian Avoid Robot?"
            # Moving humans only: H3 (0 = Non-avoiding) vs {H1, H2, H5} (1 = Avoiding)
            # ----------------------------------------------------------
            moving_mask = (y_test != 3) # Exclude static H4
            y_binary = np.where(y_test[moving_mask] == 2, 0, 1) # H3=2 -> 0, H1,H2,H5 -> 1
            # Avoidance probability = P(H1) + P(H2) + P(H5) / sum(moving)
            moving_probs = m_probs[moving_mask]
            p_avoid = (moving_probs[:, 0] + moving_probs[:, 1] + moving_probs[:, 4])
            p_sum = p_avoid + moving_probs[:, 2]
            p_avoid_norm = np.where(p_sum > 1e-6, p_avoid / p_sum, 0.5)
            
            binary_auc_overall = float(roc_auc_score(y_binary, p_avoid_norm)) if len(np.unique(y_binary)) > 1 else 0.5
            
            # Binary AUC on interactive subset (tau=1.5m)
            moving_15_mask = m15 & (y_test != 3)
            if np.sum(moving_15_mask) > 0 and len(np.unique(y_test[moving_15_mask] == 2)) > 1:
                y_bin_15 = np.where(y_test[moving_15_mask] == 2, 0, 1)
                p_bin_15 = m_probs[moving_15_mask]
                p_av_15 = (p_bin_15[:, 0] + p_bin_15[:, 1] + p_bin_15[:, 4]) / (p_bin_15[:, 0] + p_bin_15[:, 1] + p_bin_15[:, 4] + p_bin_15[:, 2] + 1e-6)
                binary_auc_15 = float(roc_auc_score(y_bin_15, p_av_15))
            else:
                binary_auc_15 = 0.5
                
            # ----------------------------------------------------------
            # Accuracy & Macro-F1 Over Time Relative to Closest Point
            # Bins in [-3.0s, +2.0s]
            # ----------------------------------------------------------
            time_progression = {}
            time_bins_def = [
                ("-3.0s to -2.0s", -3.0, -2.0),
                ("-2.0s to -1.0s", -2.0, -1.0),
                ("-1.0s to +0.0s", -1.0, 0.0),
                ("+0.0s to +1.0s", 0.0, 1.0),
                ("+1.0s to +2.0s", 1.0, 2.0)
            ]
            for label_t, t_low, t_high in time_bins_def:
                t_bin_mask = (test_dt_closest >= t_low) & (test_dt_closest < t_high)
                cnt_t = int(np.sum(t_bin_mask))
                if cnt_t > 0:
                    acc_t = float(accuracy_score(y_test[t_bin_mask], preds[t_bin_mask]))
                    f1_t = float(f1_score(y_test[t_bin_mask], preds[t_bin_mask], average="macro"))
                    # Binary AUC in this time slice
                    mov_t = t_bin_mask & (y_test != 3)
                    if np.sum(mov_t) > 0 and len(np.unique(y_test[mov_t] == 2)) > 1:
                        y_b_t = np.where(y_test[mov_t] == 2, 0, 1)
                        p_b_t = m_probs[mov_t]
                        p_av_t = (p_b_t[:, 0] + p_b_t[:, 1] + p_b_t[:, 4]) / (p_b_t[:, 0] + p_b_t[:, 1] + p_b_t[:, 4] + p_b_t[:, 2] + 1e-6)
                        auc_t = float(roc_auc_score(y_b_t, p_av_t))
                    else:
                        auc_t = 0.5
                else:
                    acc_t, f1_t, auc_t = 0.0, 0.0, 0.5
                time_progression[label_t] = {
                    "count": cnt_t,
                    "accuracy": acc_t,
                    "macro_f1": f1_t,
                    "binary_avoid_auc": auc_t
                }
                
            # Corridor vs Open breakdown
            breakdown_geometry = {}
            for geom in ["open_6x6", "corridor_09", "corridor_12"]:
                geom_mask = (test_geometry == geom)
                breakdown_geometry[geom] = {
                    "accuracy": float(accuracy_score(y_test[geom_mask], preds[geom_mask])),
                    "macro_f1": float(f1_score(y_test[geom_mask], preds[geom_mask], average="macro")),
                    "count": int(np.sum(geom_mask))
                }
            # Combined corridors
            corr_all_mask = (test_geometry == "corridor_09") | (test_geometry == "corridor_12")
            breakdown_geometry["corridor_all"] = {
                "accuracy": float(accuracy_score(y_test[corr_all_mask], preds[corr_all_mask])),
                "macro_f1": float(f1_score(y_test[corr_all_mask], preds[corr_all_mask], average="macro")),
                "count": int(np.sum(corr_all_mask))
            }

            noise_results[m_name] = {
                "overall_accuracy": float(acc),
                "overall_accuracy_natural_weighted": float(acc_weighted),
                "overall_macro_f1": float(macro_f1),
                "macro_f1_no_h4": float(macro_f1_no_h4),
                "per_class_5": per_class_5,
                "nll": float(nll),
                "nll_natural_weighted": float(nll_weighted),
                "brier_score": float(brier),
                "brier_natural_weighted": float(brier_weighted),
                "ece": float(ece),
                "ece_bins": ece_bins,
                "binary_avoidance_task": {
                    "auc_overall": binary_auc_overall,
                    "auc_interactive_15": binary_auc_15
                },
                "time_progression_relative_to_closest_point": time_progression,
                "interactive_tau15": {
                    "accuracy": float(acc_15),
                    "macro_f1": float(macro_f1_15),
                    "macro_f1_no_h4": float(macro_f1_15_no_h4),
                    "count": int(np.sum(m15)),
                    "h2_misclassified_as_h1_rate": float(h2_as_h1_rate_15),
                    "h1_misclassified_as_h2_rate": float(h1_as_h2_rate_15),
                    "h2_h1_alpha_breakdown": h2_h1_alpha_breakdown
                },
                "interactive_tau20": {
                    "accuracy": float(acc_20),
                    "macro_f1": float(macro_f1_20),
                    "count": int(np.sum(m20))
                },
                "non_interactive": {
                    "macro_f1": float(macro_f1_non),
                    "count": int(np.sum(m_non))
                },
                "breakdown_geometry": breakdown_geometry,
                "confusion_matrix_5x5": confusion_matrix(y_test, preds, labels=[0, 1, 2, 3, 4]).tolist(),
                "confusion_matrix_5x5_interactive15": cm_5x5_15.tolist()
            }
            
        benchmark_output["noise_experiments"][noise_key] = noise_results
        
        # --------------------------------------------------------------
        # Save Corridor & Open Specific Confusion Matrices at sigma = 0.05
        # --------------------------------------------------------------
        if abs(sigma - 0.05) < 1e-4:
            # We save CSVs for both PyTorch GRU and GBDT
            preds_gru = np.argmax(probs_gru, axis=1)
            
            # Corridor All
            cm_corr = confusion_matrix(y_test[corr_all_mask], preds_gru[corr_all_mask], labels=[0, 1, 2, 3, 4])
            save_confusion_matrix_csv(cm_corr, ["H1", "H2", "H3", "H4", "H5"],
                                      os.path.join(root_dir, "experiments/pedestrian_persona_benchmark/confusion_matrix_5x5_corridor_all_sigma005.csv"))
            
            # Corridor 0.9m
            m09 = (test_geometry == "corridor_09")
            cm_09 = confusion_matrix(y_test[m09], preds_gru[m09], labels=[0, 1, 2, 3, 4])
            save_confusion_matrix_csv(cm_09, ["H1", "H2", "H3", "H4", "H5"],
                                      os.path.join(root_dir, "experiments/pedestrian_persona_benchmark/confusion_matrix_5x5_corridor09_sigma005.csv"))
            
            # Corridor 1.2m
            m12 = (test_geometry == "corridor_12")
            cm_12 = confusion_matrix(y_test[m12], preds_gru[m12], labels=[0, 1, 2, 3, 4])
            save_confusion_matrix_csv(cm_12, ["H1", "H2", "H3", "H4", "H5"],
                                      os.path.join(root_dir, "experiments/pedestrian_persona_benchmark/confusion_matrix_5x5_corridor12_sigma005.csv"))
            
            # Open 6x6m
            m_op = (test_geometry == "open_6x6")
            cm_op = confusion_matrix(y_test[m_op], preds_gru[m_op], labels=[0, 1, 2, 3, 4])
            save_confusion_matrix_csv(cm_op, ["H1", "H2", "H3", "H4", "H5"],
                                      os.path.join(root_dir, "experiments/pedestrian_persona_benchmark/confusion_matrix_5x5_open_sigma005.csv"))

    # ------------------------------------------------------------------
    # Step 6.7: 1,000 Bootstrap Resamplings for 95% Confidence Intervals
    # (Evaluated on primary condition: sigma = 0.05 m, PyTorch GRU)
    # ------------------------------------------------------------------
    print("\n[Step 6.7] Performing 1,000 Scenario-level Bootstrap resamplings for PyTorch GRU...")
    n_test_sc = len(test_unique_scenarios)
    b_rng = np.random.RandomState(2026)
    
    sc_to_indices = defaultdict(list)
    for idx, sc in enumerate(test_scenarios_list):
        sc_to_indices[sc].append(idx)
        
    cached_probs = cached_probs_005_gru
    cached_preds = np.argmax(cached_probs, axis=1)
    
    boot_accs, boot_macro_f1s, boot_f1_15s, boot_h2_h1_errs, boot_aucs = [], [], [], [], []
    
    for b in range(1000):
        sampled_scs = b_rng.choice(test_unique_scenarios, size=n_test_sc, replace=True)
        boot_idx = []
        for s in sampled_scs:
            boot_idx.extend(sc_to_indices[s])
        boot_idx = np.array(boot_idx)
        
        y_b = y_test[boot_idx]
        p_b = cached_preds[boot_idx]
        pr_b = cached_probs[boot_idx]
        
        boot_accs.append(accuracy_score(y_b, p_b))
        boot_macro_f1s.append(f1_score(y_b, p_b, average="macro"))
        
        m15_b = test_is_interactive_15[boot_idx]
        if np.sum(m15_b) > 0:
            boot_f1_15s.append(f1_score(y_b[m15_b], p_b[m15_b], average="macro"))
            h2_mask_b = m15_b & (y_b == 1)
            if np.sum(h2_mask_b) > 0:
                boot_h2_h1_errs.append(np.mean(p_b[h2_mask_b] == 0))
                
        # Binary AUC
        mov_b = (y_b != 3)
        if np.sum(mov_b) > 0 and len(np.unique(y_b[mov_b] == 2)) > 1:
            y_bin_b = np.where(y_b[mov_b] == 2, 0, 1)
            p_bin_b = pr_b[mov_b]
            p_av_b = (p_bin_b[:, 0] + p_bin_b[:, 1] + p_bin_b[:, 4]) / (p_bin_b[:, 0] + p_bin_b[:, 1] + p_bin_b[:, 4] + p_bin_b[:, 2] + 1e-6)
            boot_aucs.append(roc_auc_score(y_bin_b, p_av_b))
            
    ci_results_gru = {
        "overall_accuracy_ci95": [float(np.percentile(boot_accs, 2.5)), float(np.percentile(boot_accs, 97.5))],
        "overall_macro_f1_ci95": [float(np.percentile(boot_macro_f1s, 2.5)), float(np.percentile(boot_macro_f1s, 97.5))],
        "interactive_15_macro_f1_ci95": [float(np.percentile(boot_f1_15s, 2.5)), float(np.percentile(boot_f1_15s, 97.5))],
        "h2_misclassified_as_h1_rate_ci95": [float(np.percentile(boot_h2_h1_errs, 2.5)), float(np.percentile(boot_h2_h1_errs, 97.5))],
        "binary_avoidance_auc_ci95": [float(np.percentile(boot_aucs, 2.5)), float(np.percentile(boot_aucs, 97.5))]
    }
    benchmark_output["bootstrap_ci95_sigma005_gru"] = ci_results_gru
    
    # ------------------------------------------------------------------
    # Step 7: Rule-based Decision Evaluation (Updated)
    # ------------------------------------------------------------------
    res_gru_005 = benchmark_output["noise_experiments"]["sigma_0.05"]["pytorch_gru"]["interactive_tau15"]
    h2_h1_rate_mean = res_gru_005["h2_misclassified_as_h1_rate"]
    ci_upper = ci_results_gru["h2_misclassified_as_h1_rate_ci95"][1]
    
    should_merge_h1_h2 = (h2_h1_rate_mean > 0.30) or (ci_upper > 0.30)
    decision = {
        "rule_threshold": 0.30,
        "measured_h2_as_h1_rate_gru": h2_h1_rate_mean,
        "ci95_range_gru": ci_results_gru["h2_misclassified_as_h1_rate_ci95"],
        "exceeds_threshold": bool(should_merge_h1_h2),
        "recommendation": (
            "ĐỀ XUẤT GỘP H1 VÀ H2 (ngay cả khi đã dùng ước lượng bậc 2 và PyTorch GRU, tỷ lệ nhầm lẫn vẫn vượt hoặc chạm 30%)"
            if should_merge_h1_h2 else
            "GIỮ NGUYÊN H1–H5 (tách biệt thành công sau khi dùng ước lượng đúng)"
        )
    }
    benchmark_output["decision_rule"] = decision

    # Save output JSON
    out_json_path = os.path.join(root_dir, "experiments/pedestrian_persona_benchmark/h1_h5_separability_results_v2.json")
    with open(out_json_path, "w") as fp:
        json.dump(benchmark_output, fp, indent=2)
    print(f"\nSaved v2 benchmark results to {out_json_path}")
    
    # Summary Print
    print("\n" + "=" * 80)
    print("KẾT QUẢ ĐO LƯỜNG V2 TỔNG HỢP (PYTORCH GRU & POLYNOMIAL SMOOTHING):")
    print("=" * 80)
    for sig in noise_levels:
        sk = f"sigma_{sig:.2f}"
        mg = benchmark_output["noise_experiments"][sk]["pytorch_gru"]
        mb = benchmark_output["noise_experiments"][sk]["gbdt_handcrafted"]
        print(f"\n[Nhiễu sigma = {sig:.2f} m]:")
        print(f"  PyTorch GRU : Acc={mg['overall_accuracy']*100:.1f}%, F1={mg['overall_macro_f1']*100:.1f}%, F1(tau=1.5m)={mg['interactive_tau15']['macro_f1']*100:.1f}%, H2->H1={mg['interactive_tau15']['h2_misclassified_as_h1_rate']*100:.1f}%, Avoid-AUC={mg['binary_avoidance_task']['auc_overall']:.3f}")
        print(f"  Handcraft GBDT: Acc={mb['overall_accuracy']*100:.1f}%, F1={mb['overall_macro_f1']*100:.1f}%, F1(tau=1.5m)={mb['interactive_tau15']['macro_f1']*100:.1f}%, H2->H1={mb['interactive_tau15']['h2_misclassified_as_h1_rate']*100:.1f}%, Avoid-AUC={mb['binary_avoidance_task']['auc_overall']:.3f}")
        
    print("\n--- Tiến trình chính xác theo thời gian so với CPA (PyTorch GRU, sigma=0.05m) ---")
    tp_005 = benchmark_output["noise_experiments"]["sigma_0.05"]["pytorch_gru"]["time_progression_relative_to_closest_point"]
    for t_lbl, t_res in tp_005.items():
        print(f"  {t_lbl:16s} (N={t_res['count']:4d}): Accuracy={t_res['accuracy']*100:.1f}%, Macro-F1={t_res['macro_f1']*100:.1f}%, Avoid-AUC={t_res['binary_avoid_auc']:.3f}")
        
    print("\n--- Đánh giá theo tỷ lệ tự nhiên 40/10/25/15/10 (sigma=0.05m) ---")
    print(f"  Prior Baseline (Natural Weighted): Acc={benchmark_output['noise_experiments']['sigma_0.05']['prior_baseline']['overall_accuracy_natural_weighted']*100:.1f}%, NLL={benchmark_output['noise_experiments']['sigma_0.05']['prior_baseline']['nll_natural_weighted']:.4f}")
    print(f"  PyTorch GRU    (Natural Weighted): Acc={benchmark_output['noise_experiments']['sigma_0.05']['pytorch_gru']['overall_accuracy_natural_weighted']*100:.1f}%, NLL={benchmark_output['noise_experiments']['sigma_0.05']['pytorch_gru']['nll_natural_weighted']:.4f}")
    
    print("\n" + "=" * 80)
    print(f"QUY TẮC QUYẾT ĐỊNH MỚI (BƯỚC 7):")
    print(f"  Ngưỡng: 30.0%")
    print(f"  Tỷ lệ H2 nhầm H1 (PyTorch GRU, sigma=0.05m): {h2_h1_rate_mean*100:.2f}% (95% CI: [{ci_results_gru['h2_misclassified_as_h1_rate_ci95'][0]*100:.2f}%, {ci_results_gru['h2_misclassified_as_h1_rate_ci95'][1]*100:.2f}%])")
    print(f"  QUYẾT ĐỊNH: {decision['recommendation']}")
    print("=" * 80)


if __name__ == "__main__":
    main()
