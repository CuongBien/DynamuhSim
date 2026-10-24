#!/usr/bin/env python3
"""Pack/verify/restore captures; evict only raw payload after a verified round trip."""
import argparse
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.artifacts.store import pack_capture, verify, restore, evict_raw


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    pack = commands.add_parser("pack")
    pack.add_argument("--run-dir", type=Path, required=True)
    pack.add_argument("--source-root", type=Path, action="append", default=[])
    for name in ["verify", "restore", "evict-raw"]:
        command = commands.add_parser(name)
        command.add_argument("--object-id", required=True)
        if name == "restore": command.add_argument("--output", type=Path, required=True)
        if name == "evict-raw": command.add_argument("--run-dir", type=Path, required=True)
    for command in [pack, *[commands.choices[name] for name in ["verify", "restore", "evict-raw"]]]:
        command.add_argument("--store", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == "pack":
        result = pack_capture(args.run_dir, ROOT, args.store, source_roots=args.source_root)
    elif args.command == "verify": result = verify(args.store, args.object_id)
    elif args.command == "restore": result = restore(args.store, args.object_id, args.output)
    else: result = evict_raw(args.store, args.object_id, args.run_dir, ROOT)
    print(result.get("object_id"), args.command, "PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
