"""Compatibility import for the canonical NumPy-only episode reader."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'python'))
from dynamuhsim.datasets.portable import Episode
