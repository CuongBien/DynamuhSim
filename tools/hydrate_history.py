#!/usr/bin/env python3
"""Restore a historical group into its original checkout path, or check it."""
import argparse
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.analysis.evidence import load_json
from dynamuhsim.artifacts.hydrate import hydrate_history


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkout", type=Path, default=ROOT)
    parser.add_argument("--index", type=Path, default=ROOT / "docs/artifacts/history_2026-10-25.json")
    parser.add_argument("--group", required=True)
    parser.add_argument("--source", choices=["archive", "git"], default="archive")
    parser.add_argument("--store", type=Path)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.report.exists() or args.report.is_symlink(): raise FileExistsError("Preserve existing report")
    report = hydrate_history(args.checkout, load_json(args.index), args.group, source=args.source,
                             store=args.store, check=args.check)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open("x") as stream: stream.write(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(report["state"], args.group, "changed=" + str(report["changed"]))
    return 1 if args.check and report["state"] != "present_verified" else 0


if __name__ == "__main__": raise SystemExit(main())
