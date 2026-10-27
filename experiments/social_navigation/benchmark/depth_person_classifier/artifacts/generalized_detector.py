"""
Generalized Person Detector from Depth + LiDAR Geometry (Artifact version).
Portable, pure NumPy + SciPy.
"""
from pathlib import Path
import sys
import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from predict import Classifier


def adaptive_3d_clustering(
    points,
    cluster_radius=0.18,
    min_cluster_points=18,
    max_cluster_points=2000,
    min_x=0.25,
    max_x=5.2,
    max_abs_y=2.8,
    min_z=0.06,
    max_z=1.95,
):
    if len(points) == 0:
        return []

    mask = (
        (points[:, 0] >= min_x)
        & (points[:, 0] <= max_x)
        & (np.abs(points[:, 1]) <= max_abs_y)
        & (points[:, 2] >= min_z)
        & (points[:, 2] <= max_z)
    )
    p = points[mask]
    if len(p) < min_cluster_points:
        return []

    tree = cKDTree(p)
    visited = np.zeros(len(p), dtype=bool)
    clusters = []

    for i in range(len(p)):
        if visited[i]:
            continue

        neighbors = tree.query_ball_point(p[i], r=cluster_radius)
        if len(neighbors) < min_cluster_points:
            continue

        cluster_indices = []
        queue = list(neighbors)

        for idx in queue:
            if not visited[idx]:
                visited[idx] = True
                cluster_indices.append(idx)
                sub_neighbors = tree.query_ball_point(p[idx], r=cluster_radius)
                if len(sub_neighbors) >= min_cluster_points:
                    for sn in sub_neighbors:
                        if not visited[sn]:
                            queue.append(sn)

        if min_cluster_points <= len(cluster_indices) <= max_cluster_points:
            q = p[cluster_indices]
            extent = np.ptp(q, axis=0)

            # Decompose cluster if person is touching furniture/trolley
            if extent[0] > 0.95 or extent[1] > 0.95:
                upper = q[q[:, 2] >= 0.95]
                if len(upper) >= 25:
                    upper_center = np.median(upper[:, :2], axis=0)
                    dist = np.linalg.norm(q[:, :2] - upper_center, axis=1)
                    human_part = q[dist <= 0.45]
                    if len(human_part) >= min_cluster_points:
                        h_ext = np.ptp(human_part, axis=0)
                        if h_ext[0] <= 0.95 and h_ext[1] <= 0.95 and h_ext[2] >= 0.40:
                            clusters.append(human_part)
                continue

            clusters.append(q)

    return clusters


class GeneralizedPersonDetector:
    def __init__(self, artifacts_dir=None, min_person_score=0.45):
        self.artifacts_dir = Path(artifacts_dir) if artifacts_dir else HERE
        self.classifier = Classifier(folder=self.artifacts_dir)
        self.min_person_score = min_person_score

    def detect(self, points_3d, scan_sample=None, scan_geometry=None):
        clusters = adaptive_3d_clustering(points_3d)
        detections = []

        for q in clusters:
            extent = np.ptp(q, axis=0)
            center_xy = np.median(q[:, :2], axis=0)

            if len(q) >= 3:
                cov_xy = np.cov(q[:, :2].T, bias=True)
                eigen = np.sqrt(np.maximum(0, np.linalg.eigvalsh(cov_xy)))[::-1]
            else:
                eigen = np.zeros(2)

            shape = np.array([extent[2], eigen[0], eigen[1]])

            # Reject flat planar surfaces (e.g. wall fragments)
            if eigen[1] < 0.045 and extent[2] > 0.3:
                continue

            sample = {
                "points_xyz": q,
                "scan_valid": False if scan_sample is None else bool(scan_sample.get("scan_valid", False)),
                "scan_ranges": [] if scan_sample is None else scan_sample.get("scan_ranges", []),
            }

            try:
                res = self.classifier.predict(sample, scan_geometry)
                base_score = res["score"]
            except Exception:
                base_score = 0.5

            height = extent[2]
            width_max = max(extent[0], extent[1])
            aspect_ratio = height / (width_max + 1e-3)

            is_plausible_human_height = 0.40 <= height <= 2.0
            is_plausible_width = 0.15 <= width_max <= 0.90

            horizontal_area = max(extent[0] * extent[1], 0.04)
            slenderness = (height ** 2) / horizontal_area

            is_bulky_low_object = (height < 1.30 and width_max > 0.45)

            prior_factor = 1.0
            if (slenderness < 2.5 and height < 1.15) or is_bulky_low_object:
                prior_factor = 0.15
            elif is_plausible_human_height and is_plausible_width:
                if aspect_ratio >= 1.2 and slenderness >= 4.0:
                    prior_factor = 1.15
                elif aspect_ratio >= 0.8:
                    prior_factor = 1.05
            else:
                prior_factor = 0.65

            calibrated_score = float(np.clip(base_score * prior_factor, 0.0, 1.0))
            is_person = calibrated_score >= self.min_person_score

            detections.append(
                {
                    "xy": center_xy,
                    "points": len(q),
                    "extent": extent,
                    "shape": shape,
                    "score": calibrated_score,
                    "is_person": is_person,
                }
            )

        return sorted(detections, key=lambda c: tuple(c["xy"]))
