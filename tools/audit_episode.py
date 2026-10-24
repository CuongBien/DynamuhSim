#!/usr/bin/env python3
"""Audit an existing capture; never rerun simulation or modify original evidence."""
import argparse
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.analysis.capture import audit_capture
from dynamuhsim.analysis.evidence import load_json, sha256_file


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--policy", type=Path, default=ROOT / "configs/audit/capture_integrity.json")
    parser.add_argument("--source-root", type=Path, action="append", default=[])
    parser.add_argument("--snapshot-root", type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError("Preserve existing audit: " + str(args.output))
    roots = ([args.snapshot_root] if args.snapshot_root else []) + args.source_root
    try:
        report = audit_capture(args.run_dir, load_json(args.policy), source_roots=roots, git_repo=ROOT)
        report["policy_sha256"] = sha256_file(args.policy)
    except Exception as error:
        report = dict(schema="dynamuhsim_capture_audit/1", status="ERROR", error=str(error), training_ready=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        import json
        stream.write(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(report["status"], args.output)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
