"""Behavior regressions for the optional depth detector integration."""
from pathlib import Path
import sys
import types
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PERCEPTION = ROOT / 'experiments/social_navigation/perception'
sys.path.insert(0, str(PERCEPTION / 'depth_identity_guard'))
sys.path.insert(0, str(PERCEPTION / 'depth_person_classifier'))
from ros_support import CausalBuffer, decode_depth, validate_calibration, transform_matrix
from generalized_detector import GeneralizedPersonDetector, adaptive_3d_clustering
from integrated_pipeline import EndToEndPerceptionPipeline
from spatial_metrics import match_positions
from frozen_classifier import Classifier


def image(array, encoding, padding=0):
    h, w = array.shape
    rows = [array[i].tobytes() + bytes(padding) for i in range(h)]
    return types.SimpleNamespace(encoding=encoding, height=h, width=w,
        step=w*array.dtype.itemsize+padding, is_bigendian=array.dtype.byteorder == '>', data=b''.join(rows))


def person():
    rng = np.random.default_rng(42)
    return np.c_[rng.uniform(2.3, 2.7, 300), rng.uniform(-.18, .18, 300), rng.uniform(.08, 1.78, 300)]


class DepthIntegrationTests(unittest.TestCase):
    def test_millimeters_convert_with_padded_rows_and_endian(self):
        for dtype in ('<u2', '>u2'):
            d = decode_depth(image(np.array([[2000, 0], [1000, 3000]], dtype=dtype), '16UC1', padding=4))
            np.testing.assert_allclose(d, [[2, 0], [1, 3]])
            self.assertEqual(d.dtype, np.float32)

    def test_float_depth_retains_invalid_evidence(self):
        a = np.array([[2., np.nan, np.inf]], dtype='>f4')
        d = decode_depth(image(a, '32FC1', padding=4))
        self.assertEqual(float(d[0, 0]), 2.)
        self.assertTrue(np.isnan(d[0, 1]))
        self.assertTrue(np.isinf(d[0, 2]))

    def test_invalid_image_layout_rejected(self):
        msg = image(np.ones((2, 3), np.float32), '32FC1')
        msg.step = 2
        with self.assertRaises(ValueError): decode_depth(msg)
        msg = image(np.ones((2, 3), np.float32), '32FC1')
        msg.data = msg.data[:-1]
        with self.assertRaises(ValueError): decode_depth(msg)
        msg.encoding = 'rgb8'
        with self.assertRaises(ValueError): decode_depth(msg)

    def test_causal_odom_never_uses_future_or_late_or_stale(self):
        b = CausalBuffer()
        b.add(100, 100, 'old')
        b.add(110, 110, 'future')
        b.add(104, 109, 'not_available_yet')
        self.assertEqual(b.select(105, 10)[2], 'old')
        self.assertIsNone(b.select(105, 4))
        self.assertEqual(b.select(111, 10)[2], 'future')
        b.clear()
        self.assertIsNone(b.select(111))

    def test_real_calibration_and_bad_matrices(self):
        k = [223.4, 0, 212, 0, 223.4, 120, 0, 0, 1]
        tf = transform_matrix([.12, 0, .08], [-.5, .5, -.5, .5])
        validate_calibration((240, 424), k, tf, 4)
        optical = np.array([0., 0., 2., 1.])
        np.testing.assert_allclose((tf @ optical)[:3], [2.12, 0, .08])
        for bad_k, bad_tf, stride in [(np.zeros(9), tf, 4), (k, np.zeros((4, 4)), 4), (k, tf, 0)]:
            with self.assertRaises(ValueError): validate_calibration((240, 424), bad_k, bad_tf, stride)

    def test_inference_exception_never_promotes_person(self):
        d = GeneralizedPersonDetector()
        def broken(*args): raise RuntimeError('injected')
        d.classifier.predict = broken
        rows = d.detect(person())
        self.assertGreater(len(rows), 0)
        self.assertTrue(all(not r['is_person'] and not r['inference_valid'] and r['score'] == 0 for r in rows))
        self.assertEqual(rows[0]['inference_error'], 'RuntimeError')

    def test_nonfinite_inference_rejected(self):
        d = GeneralizedPersonDetector()
        d.classifier.predict = lambda *args: dict(score=float('nan'))
        self.assertTrue(all(not r['is_person'] and not r['inference_valid'] for r in d.detect(person())))

    def test_clustering_handles_invalid_and_empty_points(self):
        self.assertEqual(adaptive_3d_clustering(np.empty((0, 3))), [])
        self.assertEqual(adaptive_3d_clustering(np.full((20, 3), np.nan)), [])
        with self.assertRaises(ValueError): adaptive_3d_clustering(np.ones((20, 2)))

    def test_tracking_reset_and_duplicate_timestamp(self):
        p = EndToEndPerceptionPipeline()
        rows = p.process_point_cloud(100, person(), np.zeros(3))
        self.assertTrue(rows)
        self.assertFalse(rows[0]['training_authorized'])
        with self.assertRaises(ValueError): p.process_point_cloud(100, person(), np.zeros(3))
        p.reset()
        self.assertTrue(p.process_point_cloud(10, person(), np.zeros(3)))

    def test_classifier_preserves_frozen_probability(self):
        # Reproduce the frozen mathematical path from unchanged feature/inference files.
        c = Classifier(PERCEPTION/'depth_person_classifier/artifacts')
        sample = dict(points_xyz=person(), scan_valid=False, scan_ranges=[])
        depth, lidar, mask = c.features.extract(sample, None)
        data = dict(depth=depth[None], lidar=lidar[None], lidar_mask=np.array([mask]))
        logit = c.inference.numpy_logits(data, c.arrays, c.config['hidden'], c.config['modality'])[0]
        expected = float(1/(1+np.exp(-np.clip(logit, -80, 80))))
        self.assertEqual(c.predict(sample)['score'], expected)

    def test_correct_count_wrong_location_is_not_true_positive(self):
        m = match_positions([[4., 2.]], [[1., 0.]])
        self.assertEqual((m['tp'], m['fp'], m['fn']), (0, 1, 1))

    def test_matching_is_one_to_one(self):
        m = match_positions([[1., 0.], [1.01, 0.]], [[1., 0.]])
        self.assertEqual((m['tp'], m['fp'], m['fn']), (1, 1, 0))

    def test_low_score_is_not_tracked_as_person(self):
        p = EndToEndPerceptionPipeline()
        p.detector.classifier.predict = lambda *args: dict(score=0.01)
        self.assertEqual(p.process_point_cloud(100, person(), np.zeros(3)), [])


if __name__ == '__main__': unittest.main()
