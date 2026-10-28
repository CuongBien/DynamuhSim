#!/usr/bin/env python3
"""Compatibility entry point; implementation lives in dynamuhsim.runtime.nav2_trial."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "python"))
from dynamuhsim.runtime.nav2_trial import main

if __name__ == "__main__":
    raise SystemExit(main("socnavgym_group_nav2", Path(__file__)))
