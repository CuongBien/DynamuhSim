"""
Unit tests for End-to-End Perception & Tracking Pipeline (Direction 2).
Verifies:
  1. 3D Euclidean clustering decouples person from adjacent wall/furniture.
  2. Ground plane is filtered out.
  3. Classifier predicts human vs environment geometry.
  4. Identity continuity is maintained over multiple frames.
  5. Small synthetic latency smoke (15 ms mean budget; not a sensor throughput guarantee).
"""
import time
import unittest
import numpy as np

from integrated_pipeline import EndToEndPerceptionPipeline
from generalized_detector import adaptive_3d_clustering, GeneralizedPersonDetector


def make_cylinder_points(center_xyz, radius=0.22, height=1.65, num_points=300):
    """Generate synthetic 3D points representing a human body."""
    rng = np.random.default_rng(42)
    theta = rng.uniform(0, 2 * np.pi, num_points)
    r = radius * np.sqrt(rng.uniform(0.1, 1.0, num_points))
    z = rng.uniform(0.0, height, num_points)
    x = center_xyz[0] + r * np.cos(theta)
    y = center_xyz[1] + r * np.sin(theta)
    z = center_xyz[2] + z
    return np.column_stack([x, y, z])


def make_wall_points(x_center, y_span, z_span, num_points=400):
    """Generate synthetic flat vertical wall points."""
    rng = np.random.default_rng(123)
    y = rng.uniform(y_span[0], y_span[1], num_points)
    z = rng.uniform(z_span[0], z_span[1], num_points)
    x = x_center + rng.normal(0, 0.02, num_points)
    return np.column_stack([x, y, z])


class TestIntegratedPerception(unittest.TestCase):
    def setUp(self):
        self.pipeline = EndToEndPerceptionPipeline()
        self.detector = GeneralizedPersonDetector()

    def test_clustering_separates_person_from_nearby_wall(self):
        """A person standing 25cm away from a corridor wall must NOT be merged into the wall."""
        person = make_cylinder_points(center_xyz=[2.0, 0.2, 0.08], radius=0.20, height=1.70, num_points=300)
        wall = make_wall_points(x_center=2.0, y_span=[0.65, 2.5], z_span=[0.05, 2.0], num_points=400)
        scene = np.vstack([person, wall])

        clusters = adaptive_3d_clustering(scene, cluster_radius=0.22)
        person_clusters = [c for c in clusters if np.abs(np.median(c[:, 1]) - 0.2) < 0.15]
        self.assertGreaterEqual(len(person_clusters), 1, "Person cluster was not cleanly separated from wall")

        p_cluster = person_clusters[0]
        extent = np.ptp(p_cluster, axis=0)
        self.assertLess(extent[1], 0.70, "Person cluster incorrectly merged into wall width")
        self.assertGreater(extent[2], 1.20, "Person cluster lost height")

    def test_floor_noise_is_filtered(self):
        """Ground plane points (z < 0.05m) must be suppressed."""
        floor_points = np.column_stack([
            np.random.uniform(1.0, 3.0, 200),
            np.random.uniform(-1.0, 1.0, 200),
            np.random.uniform(0.0, 0.04, 200) # Floor noise
        ])
        clusters = adaptive_3d_clustering(floor_points)
        self.assertEqual(len(clusters), 0, "Floor noise was incorrectly clustered as obstacle")

    def test_detector_identifies_human_vs_wall(self):
        """GeneralizedPersonDetector must output high score for person and reject flat wall."""
        person = make_cylinder_points([2.2, 0.0, 0.08], height=1.70, num_points=300)
        wall = make_wall_points(x_center=2.2, y_span=[-1.8, -0.6], z_span=[0.05, 1.8], num_points=400)
        scene = np.vstack([person, wall])

        detections = self.detector.detect(scene)
        self.assertGreater(len(detections), 0)

        person_det = [d for d in detections if np.abs(d['xy'][1]) < 0.25]
        self.assertEqual(len(person_det), 1)
        self.assertTrue(person_det[0]['is_person'])
        self.assertGreaterEqual(person_det[0]['score'], 0.45)

    def test_tracking_continuity_across_frames(self):
        """Continuous frames of a moving pedestrian must retain consistent track_id."""
        pipeline = EndToEndPerceptionPipeline()
        initial_pose = np.array([0.0, 0.0, 0.0]) # Robot at origin

        assigned_ids = []
        # Simulate pedestrian walking towards robot from x=3.5 to x=3.1 at 1.0 m/s
        for step in range(5):
            t_ns = step * 100_000_000 # 100ms per frame
            x_ped = 3.5 - step * 0.10
            pts = make_cylinder_points([x_ped, 0.0, 0.08], num_points=300)
            tracks = pipeline.process_point_cloud(
                stamp_ns=t_ns,
                points_3d=pts,
                robot_pose_odom=initial_pose,
                filter_non_person=True
            )
            self.assertEqual(len(tracks), 1)
            assigned_ids.append(tracks[0]['track_id'])

        # Track ID must remain the same across all frames
        self.assertEqual(len(set(assigned_ids)), 1, f"Track ID switched unexpectedly: {assigned_ids}")
        self.assertGreater(assigned_ids[0], 0, "Track ID must be positive assigned track")

    def test_inference_latency(self):
        """Small synthetic point-cloud loop has a 15ms mean budget on this host."""
        pts = make_cylinder_points([2.5, 0.1, 0.08], num_points=250)
        pose = np.zeros(3)

        # Warm up
        self.pipeline.process_point_cloud(0, pts, pose)

        start = time.perf_counter()
        iterations = 20
        for i in range(iterations):
            self.pipeline.process_point_cloud((i + 1) * 100_000_000, pts, pose)
        elapsed_per_frame_ms = (time.perf_counter() - start) / iterations * 1000.0

        print(f"\n[BENCHMARK] Average End-to-End Latency: {elapsed_per_frame_ms:.2f} ms / frame")
        self.assertLess(elapsed_per_frame_ms, 15.0, "Latency exceeded real-time budget")


if __name__ == '__main__':
    unittest.main()
