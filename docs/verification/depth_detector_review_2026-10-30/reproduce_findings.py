"""Reproduce two findings from the reviewed commit; no ROS runtime or source edits."""
import argparse
import ast
import json
import sys
import types
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--snapshot', type=Path, required=True)
args = parser.parse_args()
root = args.snapshot.resolve()
sys.path.insert(0, str(root / 'experiments/social_navigation/benchmark/depth_identity_guard'))
from integrated_pipeline import EndToEndPerceptionPipeline
from generalized_detector import GeneralizedPersonDetector
from benchmark_scenarios import generate_person_points

class BrokenClassifier:
    def predict(self, *args):
        raise RuntimeError('deliberately injected inference failure')

detector = GeneralizedPersonDetector()
detector.classifier = BrokenClassifier()
detections = detector.detect(generate_person_points([2.5, 0., .08], num_points=350))
result = {'inference_exception': [dict(score=d['score'], is_person=d['is_person']) for d in detections]}

node_path = root / 'src/custom_corridor/scripts/depth_person_tracker_node.py'
tree = ast.parse(node_path.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'on_depth')
namespace = {'np': np, 'Image': object}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(node_path), 'exec'), namespace)

class StubPipeline:
    def process_depth_frame(self, **kwargs):
        self.depth = kwargs['depth_array']
        return []

class StubBridge:
    def imgmsg_to_cv2(self, msg, desired_encoding):
        return np.full((2, 2), 2000, np.uint16)

msg = types.SimpleNamespace(
    header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=0)),
    encoding='16UC1', height=2, width=2,
    data=np.full((2, 2), 2000, np.uint16).tobytes())
result['depth_units'] = []
for bridge in [StubBridge(), None]:
    node = types.SimpleNamespace(
        odom_received=True, bridge=bridge, pipeline=StubPipeline(),
        camera_k=np.eye(3).ravel(), optical_to_base=np.eye(4),
        robot_pose_odom=np.zeros(3), stride=4, publish_tracks=lambda *a: None)
    namespace['on_depth'](node, msg)
    result['depth_units'].append(dict(
        cv_bridge_enabled=bridge is not None,
        value_passed_to_pipeline=float(node.pipeline.depth[0, 0]), expected_meters=2.0))
print(json.dumps(result, indent=2))
