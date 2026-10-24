import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.analysis.evidence import sha256_file, jsonl_rows, dump_json, relative_name, resolve_sources
from dynamuhsim.analysis.geometry import pose_errors, swept_disc_gap
from dynamuhsim.analysis.timeline import nearest_index, interpolate_se2, ordered


class EvidenceTests(unittest.TestCase):
    def test_streamed_hash_and_gz_jsonl_preserve_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rows.jsonl.gz"
            content = b'{"value":1}\n\n{"value":2}\n'
            path.write_bytes(gzip.compress(content))
            before = path.read_bytes()
            self.assertEqual(list(jsonl_rows(path)), [dict(value=1), dict(value=2)])
            self.assertEqual(sha256_file(path), hashlib.sha256(before).hexdigest())
            self.assertEqual(path.read_bytes(), before)
            path = path.with_suffix("")
            path.write_text('{"value":NaN}\n')
            with self.assertRaisesRegex(ValueError, ":1:"): list(jsonl_rows(path))
            with self.assertRaises(ValueError): dump_json(path, dict(value=float("nan")))

    def test_relative_paths_reject_traversal_and_windows_escapes(self):
        for name in ["../secret", "/absolute", "a/../b", "C:/secret", "a\\b", "a//b", ""]:
            with self.subTest(name=name), self.assertRaises(ValueError): relative_name(name)
        self.assertEqual(str(relative_name("source/native.py")), "source/native.py")

    def test_sources_fall_back_only_to_the_recorded_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            subprocess.run(["git", "init", "-q", str(root)], check=True)
            path = root / "producer.py"
            path.write_bytes(b"original")
            subprocess.run(["git", "-C", str(root), "add", "producer.py"], check=True)
            subprocess.run(["git", "-C", str(root), "-c", "user.name=Audit Test", "-c", "user.email=audit@example.invalid", "commit", "-qm", "fixture"], check=True)
            head = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
            expected = {"producer.py": hashlib.sha256(b"original").hexdigest()}
            path.write_bytes(b"changed")
            found, missing = resolve_sources(expected, [root], git_repo=root, commit=head)
            self.assertFalse(missing)
            self.assertEqual(found["producer.py"]["data"], b"original")
            self.assertEqual(found["producer.py"]["origin"], "recorded_git")
            self.assertEqual(path.read_bytes(), b"changed")
            found, missing = resolve_sources(expected, [root])
            self.assertEqual(missing, ["producer.py"])
            with self.assertRaises(ValueError): resolve_sources(expected, git_repo=root, commit="HEAD")


class MeasurementTests(unittest.TestCase):
    def test_timestamp_ties_tolerance_and_large_integer_ns(self):
        base = 1_700_000_000_000_000_000
        stamps = [base, base + 200]
        self.assertEqual(nearest_index(stamps, base + 100, 100), 0)
        self.assertIsNone(nearest_index(stamps, base + 100, 99))
        self.assertIsNone(nearest_index([], 0, 1))
        with self.assertRaises(ValueError): nearest_index(stamps, base, float("nan"))
        with self.assertRaises(ValueError): ordered([1, 0])

    def test_pose_interpolation_wraps_yaw_and_does_not_extrapolate(self):
        poses = [(0, 0, math.radians(179)), (2, 4, math.radians(-179))]
        middle = interpolate_se2([0, 100], poses, 50, 100)
        self.assertEqual(middle[:2], (1, 2))
        self.assertAlmostEqual(abs(middle[2]), math.pi)
        self.assertIsNone(interpolate_se2([0, 100], poses, 50, 99))
        self.assertIsNone(interpolate_se2([0, 100], poses, -1, 100))
        self.assertEqual(interpolate_se2([0, 100], poses, 100, 0), poses[1])
        with self.assertRaises(ValueError): interpolate_se2([0, 0], poses, 0, 100)

    def test_goal_and_linear_swept_gap_cover_false_success_and_between_sample_overlap(self):
        xy, yaw = pose_errors(dict(x=-19.764, y=0, yaw=2*math.pi + .1), (-20, 0, 0))
        self.assertAlmostEqual(xy, .236)
        self.assertAlmostEqual(yaw, .1)
        self.assertAlmostEqual(swept_disc_gap((-1, 0), (1, 0), .2, (0, -1), (0, 1), .3), -.5)
        self.assertAlmostEqual(swept_disc_gap((0, 0), (1, 0), .2, (0, 1), (1, 1), .3), .5)
        self.assertAlmostEqual(swept_disc_gap((0, 0), (0, 0), .2, (.5, 0), (.5, 0), .3), 0.)
        with self.assertRaises(ValueError): swept_disc_gap((0, 0), (1, 1), -1, (0, 0), (0, 0), 1)


if __name__ == "__main__": unittest.main()
