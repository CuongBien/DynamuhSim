import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

DEMO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(DEMO))

from dataset_runner.episode_types import Pose2D
from dataset_runner.readiness_checker import (
    ReadinessChecker, ReadinessConfig, ReadinessError, ReadinessSnapshot,
    RosReadinessBackend,
)


class FakeBackend:
    def __init__(self, samples, clock):
        self.samples = samples
        self.clock = clock
        self.index = 0
        self.closed = False

    def snapshot(self):
        return self.samples[min(self.index, len(self.samples) - 1)]

    def tick(self, seconds):
        self.clock[0] += seconds
        self.index += 1

    def close(self):
        self.closed = True


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.config = ReadinessConfig(timeout_sec=0.03, poll_interval_sec=0.01)
        self.context = SimpleNamespace(start_pose=Pose2D(1.0, 2.0, math.pi - 0.02))
        self.good = ReadinessSnapshot(odom=True, scan=True, rgb=True, tf=True,
                                      robot_pose=Pose2D(1.01, 2.01, -math.pi + 0.02))
        self.clock = [0.0]

    def checker(self, *samples, config=None):
        backend = FakeBackend(samples, self.clock)
        return (ReadinessChecker(config=config or self.config,
                                 backend_factory=lambda _: backend,
                                 clock=lambda: self.clock[0]), backend)

    def assert_failure(self, sample, code):
        checker, backend = self.checker(sample)
        with self.assertRaises(ReadinessError) as cm:
            checker.wait_ready(self.context)
        self.assertEqual(cm.exception.code, code)
        checker.finalize()
        self.assertTrue(backend.closed)

    def test_all_fresh_signals_and_wrapped_yaw_are_ready(self):
        checker, backend = self.checker(ReadinessSnapshot(), self.good)
        result = checker.wait_ready(self.context)
        self.assertEqual(result, self.good)
        checker.finalize()
        self.assertTrue(backend.closed)

    def test_each_missing_dependency_blocks(self):
        for changed, code in (({"odom": False}, "odom_unavailable"),
                              ({"scan": False}, "scan_unavailable"),
                              ({"rgb": False}, "rgb_unavailable"),
                              ({"tf": False}, "tf_unavailable"),
                              ({"robot_pose": None}, "robot_pose_unavailable")):
            with self.subTest(code=code):
                self.clock[0] = 0.0
                sample = ReadinessSnapshot(**{**vars(self.good), **changed})
                self.assert_failure(sample, code)

    def test_invalid_start_position_yaw_and_nonfinite_pose_block(self):
        for pose in (Pose2D(1.3, 2.0, self.context.start_pose.heading),
                     Pose2D(1.0, 2.0, 0.0), Pose2D(float("nan"), 2.0, 0.0)):
            with self.subTest(pose=pose):
                self.clock[0] = 0.0
                self.assert_failure(ReadinessSnapshot(**{
                    **vars(self.good), "robot_pose": pose}), "robot_pose_invalid")

    def test_depth_only_when_configured(self):
        checker, _ = self.checker(self.good)
        self.assertEqual(checker.wait_ready(self.context), self.good)
        checker.finalize()
        self.clock[0] = 0.0
        config = ReadinessConfig(timeout_sec=0.03, poll_interval_sec=0.01,
                                 depth_required=True, depth_topic="/depth")
        checker, _ = self.checker(self.good, config=config)
        with self.assertRaises(ReadinessError) as cm:
            checker.wait_ready(self.context)
        self.assertEqual(cm.exception.code, "depth_unavailable")
        checker.finalize()

    def test_config_matches_existing_bridge_and_has_no_depth(self):
        config = ReadinessConfig.from_yaml()
        self.assertEqual((config.odom_topic, config.scan_topic, config.rgb_topic),
                         ("/odom", "/scan", "/robot/camera/image_raw"))
        self.assertFalse(config.depth_required)
        self.assertIsNone(config.depth_topic)
        self.assertEqual((config.tf_parent, config.tf_child, config.pose_frame),
                         ("odom", "base_footprint", "map"))

    def test_sensor_callbacks_require_valid_new_messages(self):
        backend = RosReadinessBackend.__new__(RosReadinessBackend)
        backend.config = self.config
        backend.received = {"odom": False, "scan": False, "rgb": False, "depth": False}
        backend.robot_pose = None
        stamp = SimpleNamespace(sec=10, nanosec=0)
        header = SimpleNamespace(stamp=stamp, frame_id="odom")
        quaternion = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        position = SimpleNamespace(x=1.0, y=2.0, z=0.0)
        odom = SimpleNamespace(header=header, child_frame_id="base_footprint",
                               pose=SimpleNamespace(pose=SimpleNamespace(position=position,
                                                                         orientation=quaternion)))
        backend._odom(odom)
        self.assertTrue(backend.received["odom"])
        backend._scan(SimpleNamespace(header=header, ranges=[]))
        self.assertFalse(backend.received["scan"])
        backend._scan(SimpleNamespace(header=header, ranges=[1.0]))
        self.assertTrue(backend.received["scan"])
        backend._rgb(SimpleNamespace(header=header, width=1, height=1, data=b""))
        self.assertFalse(backend.received["rgb"])
        backend._rgb(SimpleNamespace(header=header, width=1, height=1, data=b"x"))
        self.assertTrue(backend.received["rgb"])
        pose_header = SimpleNamespace(stamp=stamp, frame_id="map")
        backend._robot_pose(SimpleNamespace(header=pose_header, pose=SimpleNamespace(
            position=position, orientation=quaternion)))
        self.assertEqual(backend.robot_pose, Pose2D(1.0, 2.0, 0.0))
        wrong_frame = SimpleNamespace(stamp=stamp, frame_id="odom")
        backend._robot_pose(SimpleNamespace(header=wrong_frame, pose=SimpleNamespace(
            position=SimpleNamespace(x=9.0, y=9.0, z=0.0), orientation=quaternion)))
        self.assertEqual(backend.robot_pose, Pose2D(1.0, 2.0, 0.0))


if __name__ == "__main__":
    unittest.main()
