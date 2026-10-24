#!/usr/bin/env python3
"""Single entry point for the three shared-runtime warehouse Nav2 pilots."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.runtime.nav2_trial import main
from dynamuhsim.runtime.profiles import PROFILES


def cli(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    parser.add_argument("--experiment", choices=sorted(PROFILES), required=True)
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv in (["--help"], ["-h"]):
        parser.print_help()
        return 0
    selected, remaining = parser.parse_known_args(argv)
    return main(selected.experiment, Path(__file__), remaining)


if __name__ == "__main__":
    raise SystemExit(cli())
