"""Compatibility shim: canonical implementation is one directory above artifacts."""
import importlib.util
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
_spec = importlib.util.spec_from_file_location('_depth_person_canonical_detector', ROOT / 'generalized_detector.py')
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
GeneralizedPersonDetector = _module.GeneralizedPersonDetector
adaptive_3d_clustering = _module.adaptive_3d_clustering
