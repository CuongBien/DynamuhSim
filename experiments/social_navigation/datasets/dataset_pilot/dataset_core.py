"""Compatibility imports for the frozen, portable WM normalizer math.

The implementation is shared with wm_episode_batch; do not fork these functions.
"""
from pathlib import Path
import hashlib
import sys

_ROOT = Path(__file__).resolve().parents[4]
# Downstream analyzers pin this shim's hash; also pin its algorithm dependency.
_SOURCE = _ROOT / "experiments/social_navigation/datasets/wm_episode_batch/tools/legacy/dataset_core.py"
if hashlib.sha256(_SOURCE.read_bytes()).hexdigest() != "1bee12f29d587a09e08d0a44388bc8767131b90e0bee7ddb101e9a03b347ef3e":
    raise RuntimeError("Frozen normalizer math changed")
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from experiments.social_navigation.datasets.wm_episode_batch.tools.legacy.dataset_core import (
    causal_indices, interpolate_labels, rotation, depth_points, observed_bev,
    validate_clock, group_membership,
)

__all__ = [
    "causal_indices", "interpolate_labels", "rotation", "depth_points", "observed_bev",
    "validate_clock", "group_membership",
]
