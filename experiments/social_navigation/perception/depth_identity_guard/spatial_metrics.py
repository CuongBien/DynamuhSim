"""Evaluator-only spatial matching. Never imported by inference/ROS node."""
import numpy as np
from scipy.optimize import linear_sum_assignment


def match_positions(predicted, target, gate_m=.60):
    predicted = np.asarray(predicted, float).reshape(-1, 2)
    target = np.asarray(target, float).reshape(-1, 2)
    if not np.isfinite(predicted).all() or not np.isfinite(target).all():
        raise ValueError('Nonfinite position in spatial evaluator')
    pairs = []
    if len(predicted) and len(target):
        distance = np.linalg.norm(predicted[:, None] - target[None], axis=-1)
        rows, cols = linear_sum_assignment(np.where(distance <= gate_m, distance, 1e6))
        pairs = [dict(prediction=int(i), target=int(j), distance_m=float(distance[i, j]))
                 for i, j in zip(rows, cols) if distance[i, j] <= gate_m]
    return dict(tp=len(pairs), fp=len(predicted)-len(pairs), fn=len(target)-len(pairs), pairs=pairs)
