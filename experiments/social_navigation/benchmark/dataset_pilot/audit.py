#!/usr/bin/env python3
"""Compatibility entrypoint for the frozen WM pilot audit engine."""
import runpy
import sys
from pathlib import Path

_ENGINE = Path(__file__).resolve().parents[1] / "wm_episode_batch/tools/legacy"
sys.path.insert(0, str(_ENGINE))
try:
    # Preserve CLI arguments and exported functions, including engine provenance.
    globals().update(runpy.run_path(str(_ENGINE / "audit.py"), run_name=__name__))
finally:
    sys.path.remove(str(_ENGINE))
