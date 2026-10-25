#!/usr/bin/env python3
"""Back up cataloged Git payloads; verify or restore independently of Git/ROS."""
import argparse
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.analysis.evidence import load_json
from dynamuhsim.artifacts.history import pack_history, verify_history_index, restore_history


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ["pack", "verify", "restore"]:
        command = commands.add_parser(name)
        command.add_argument("--store", type=Path, required=True)
        command.add_argument("--output", type=Path, required=True)
        if name == "pack": command.add_argument("--catalog", type=Path, required=True)
        if name == "verify": command.add_argument("--index", type=Path, required=True)
        if name == "restore": command.add_argument("--object-id", required=True)
    args = parser.parse_args(argv)
    if args.output.exists() or args.output.is_symlink():
        raise FileExistsError("Preserve existing output: " + str(args.output))
    if args.command == "restore":
        result = restore_history(args.store, args.object_id, args.output)
    else:
        result = (pack_history(ROOT, args.store, load_json(args.catalog)) if args.command == "pack"
                  else verify_history_index(args.store, load_json(args.index)))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(result, sort_keys=True, indent=2, allow_nan=False) + "\n")
    print(result.get("status", "ARCHIVED"), args.command, args.output)
    return 0


if __name__ == "__main__": raise SystemExit(main())
