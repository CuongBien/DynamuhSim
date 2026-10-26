#!/usr/bin/env python3
"""Catalog committed payloads or verify a fresh runtime/full resource install."""
import argparse
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.artifacts.catalog import create_catalog, verify_catalog, verify_install
from dynamuhsim.analysis.evidence import load_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ["create", "verify", "verify-install"]:
        command = commands.add_parser(name)
        command.add_argument("--output", type=Path, required=True)
        if name in ["create", "verify"]: command.add_argument("--store", type=Path, required=True)
        if name in ["create", "verify-install"]: command.add_argument("--ref", default="HEAD")
        if name == "verify": command.add_argument("--catalog", type=Path, required=True)
        if name == "verify-install":
            command.add_argument("--share-dir", type=Path, required=True)
            command.add_argument("--legacy", action="store_true")
            command.add_argument("--fresh", action="store_true", help="Also require a fresh prefix without Python caches")
            command.add_argument("--history-index", type=Path, help="Pinned history index for externalized legacy payloads")
    args = parser.parse_args(argv)
    if args.output.exists(): raise FileExistsError("Preserve existing receipt: " + str(args.output))
    if args.command == "create": report = create_catalog(ROOT, args.store, args.ref)
    elif args.command == "verify": report = verify_catalog(ROOT, args.store, load_json(args.catalog))
    else: report = verify_install(ROOT, args.share_dir, legacy=args.legacy, ref=args.ref,
                                  history_index=load_json(args.history_index) if args.history_index else None, fresh=args.fresh)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    import json
    with args.output.open("x") as stream: stream.write(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(report.get("status", "CATALOG_CREATED"), args.output)
    return 0 if report.get("status", "PASS") == "PASS" else 1


if __name__ == "__main__": raise SystemExit(main())
