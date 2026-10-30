"""
End-to-End Perception & Identity-Preserving Tracking Pipeline (Direction 2).
Connects:
  1. Depth back-projection (Camera optical -> Robot base_link)
  2. GeneralizedPersonDetector (3D Euclidean Clustering + Biometrics + Depth Person Classifier)
  3. IdentityGuard (Causal, identity-preserving tracking in odom frame)
CPU NumPy/SciPy baseline. Timing must be measured on actual sensor frames.
"""
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
BENCHMARK = HERE.parent
CLASSIFIER_ROOT = BENCHMARK / "depth_person_classifier"
CLASSIFIER_PATH = CLASSIFIER_ROOT / "artifacts"

if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(CLASSIFIER_ROOT) not in sys.path:
    sys.path.insert(0, str(CLASSIFIER_ROOT))

from guard_tracker import IdentityGuard
from generalized_detector import GeneralizedPersonDetector
from snapshots.sensor import project


class EndToEndPerceptionPipeline:
    """
    Unified Perception Node running on Depth Camera + Robot Odometry.
    Outputs experimental person candidates; identity ambiguity is retained.
    """

    def __init__(self, artifacts_dir=None, min_person_score=0.45):
        self.detector = GeneralizedPersonDetector(
            artifacts_dir=artifacts_dir or CLASSIFIER_PATH,
            min_person_score=min_person_score,
        )
        self.tracker = IdentityGuard()

    def reset(self):
        """Start a fresh tracking episode; never reuse IDs across resets."""
        self.tracker = IdentityGuard()

    def process_point_cloud(
        self,
        stamp_ns,
        points_3d,
        robot_pose_odom,
        scan_sample=None,
        scan_geometry=None,
        filter_non_person=True,
    ):
        """
        Process pre-projected 3D point cloud in base_link.

        Parameters:
            stamp_ns: int, sensor timestamp in nanoseconds.
            points_3d: (N, 3) ndarray [x, y, z] in base_link.
            robot_pose_odom: (3,) ndarray [x, y, yaw] in odom frame.
            scan_sample: optional LiDAR scan dict.
            scan_geometry: optional LiDAR metadata.
            filter_non_person: whether to filter out low-confidence environment clusters.

        Returns:
            list of active tracks from IdentityGuard.
        """
        # 1. Detect candidate person proposals
        detections = self.detector.detect(points_3d, scan_sample, scan_geometry)
        self.last_diagnostics = dict(candidate_count=len(detections),
                                     inference_failures=sum(not d['inference_valid'] for d in detections))

        # 2. Filter environment objects if requested
        if filter_non_person:
            # Keep proposals that are predicted as person
            valid_detections = [d for d in detections if d["is_person"]]
        else:
            valid_detections = detections

        # 3. Update Identity Guard tracker
        pose = np.asarray(robot_pose_odom, dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError("Invalid odometry pose")
        tracks = self.tracker.update(int(stamp_ns), valid_detections, pose)
        for track, detection in zip(tracks, valid_detections):
            track.update(person_score=detection["score"],
                         inference_valid=detection["inference_valid"],
                         semantic_class="experimental_person_candidate",
                         semantic_validated=False, training_authorized=False)
        return tracks

    def process_depth_frame(
        self,
        stamp_ns,
        depth_array,
        camera_k,
        optical_to_base_tf,
        robot_pose_odom,
        stride=4,
        scan_sample=None,
        scan_geometry=None,
        filter_non_person=True,
    ):
        """
        Process raw 2D depth image from RealSense D435i.

        Parameters:
            stamp_ns: int, timestamp in nanoseconds.
            depth_array: (H, W) float32 ndarray in meters.
            camera_k: (9,) list/array intrinsic matrix K [fx, 0, cx, 0, fy, cy, 0, 0, 1].
            optical_to_base_tf: (4, 4) transformation matrix from optical to base_link.
            robot_pose_odom: (3,) array [x, y, yaw] in odom frame.
            stride: subsampling stride (default 4 for high-speed real-time processing).

        Returns:
            list of active tracks from IdentityGuard.
        """
        # 1. Back-project depth into base_link 3D point cloud
        from ros_support import validate_calibration
        validate_calibration(depth_array.shape, camera_k, optical_to_base_tf, stride)
        points_3d = project(depth_array, camera_k, optical_to_base_tf, stride=stride)

        # 2. Process points through detector and tracker
        return self.process_point_cloud(
            stamp_ns=stamp_ns,
            points_3d=points_3d,
            robot_pose_odom=robot_pose_odom,
            scan_sample=scan_sample,
            scan_geometry=scan_geometry,
            filter_non_person=filter_non_person,
        )

