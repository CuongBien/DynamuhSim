"""ROS-independent image decoding, calibration validation and causal sample selection."""
from collections import deque
import numpy as np


def decode_depth(msg):
    if msg.encoding not in ('16UC1', '32FC1'):
        raise ValueError('Unsupported depth encoding: ' + msg.encoding)
    size = 2 if msg.encoding == '16UC1' else 4
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * size:
        raise ValueError('Invalid depth dimensions/step')
    if len(msg.data) != msg.step * msg.height:
        raise ValueError('Invalid depth buffer length')
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('u2' if size == 2 else 'f4'))
    image = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data), strides=(msg.step, size))
    depth = image.astype(np.float32)
    if size == 2:
        depth *= .001
    # Invalid values remain invalid; projection excludes them explicitly.
    return depth


def validate_calibration(shape, k, transform, stride):
    k = np.asarray(k, float).reshape(-1)
    tf = np.asarray(transform, float)
    if len(shape) != 2 or min(shape) <= 0 or not isinstance(stride, (int, np.integer)) or stride < 1:
        raise ValueError('Invalid depth shape/stride')
    if k.shape != (9,) or not np.isfinite(k).all() or min(k[0], k[4]) <= 0:
        raise ValueError('Invalid CameraInfo K')
    if not np.allclose(k[[1, 3, 6, 7, 8]], [0, 0, 0, 0, 1]):
        raise ValueError('Expected rectified pinhole CameraInfo')
    if tf.shape != (4, 4) or not np.isfinite(tf).all() or not np.allclose(tf[3], [0, 0, 0, 1]):
        raise ValueError('Invalid optical-to-base transform')
    if not np.allclose(tf[:3, :3].T @ tf[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(tf[:3, :3]), 1., atol=1e-5):
        raise ValueError('Transform rotation is not rigid')


def transform_matrix(translation, quaternion):
    q = np.asarray(quaternion, float)
    xyz = np.asarray(translation, float)
    if q.shape != (4,) or xyz.shape != (3,) or not np.isfinite(q).all() or not np.isfinite(xyz).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid TF quaternion/translation')
    x, y, z, w = q / np.linalg.norm(q)
    tf = np.eye(4)
    tf[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    tf[:3, 3] = xyz
    return tf


class CausalBuffer:
    def __init__(self, maxlen=300):
        self.rows = deque(maxlen=maxlen)

    def add(self, stamp_ns, available_ns, value):
        self.rows.append((int(stamp_ns), max(int(stamp_ns), int(available_ns)), value))

    def select(self, query_ns, max_age_ns=150_000_000):
        rows = [r for r in self.rows if r[0] <= query_ns and r[1] <= query_ns
                and (max_age_ns is None or query_ns-r[0] <= max_age_ns)]
        return max(rows, key=lambda r: (r[0], r[1])) if rows else None

    def clear(self):
        self.rows.clear()
