#!/usr/bin/env python3
"""Full recursive file/directory inventory. Standard-library only; never edits source."""
from __future__ import annotations
import argparse
import collections
import csv
import hashlib
import json
import os
from pathlib import Path
import stat
from datetime import datetime, timezone


def iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat(timespec="seconds")


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while block := f.read(8 * 1024 * 1024):
            h.update(block)
    return h.hexdigest()


def human(num: int) -> str:
    return f"{num / 1024**3:.3f} GiB"


def main():
    ap = argparse.ArgumentParser(description="Record ALL paths inside one dataset/repository root")
    ap.add_argument("--root", required=True, type=Path, help="Dataset/repository root to scan")
    ap.add_argument("--out", required=True, type=Path, help="Folder for generated reports (must be outside --root)")
    ap.add_argument("--hash-max-mb", type=float, default=0,
                    help="Compute SHA256 for regular files <= N MiB; 0 disables hashing")
    ap.add_argument("--hash-all", action="store_true", help="Hash ALL files (slow with multi-GB datasets)")
    args = ap.parse_args()
    root = args.root.expanduser().resolve(strict=True)
    out = args.out.expanduser().resolve()
    if not root.is_dir():
        ap.error(f"Root is not a directory: {root}")
    if out == root or root in out.parents:
        ap.error("--out must be OUTSIDE --root, to avoid scanning generated reports recursively")
    if args.hash_max_mb < 0:
        ap.error("--hash-max-mb must be nonnegative")
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / "ALL_PATHS.csv"
    jsonl_path = out / "ALL_PATHS.jsonl"
    tree_path = out / "DIRECTORY_TREE.txt"
    exts = collections.Counter()
    topdirs = collections.Counter()
    filecount = dircount = symlinks = total = hashcount = 0
    errors = []
    samples = []
    rows = []
    # os.walk does not follow symlinked directories. scandir-based walk is efficient for large corpora.
    def walk(dir_path: Path):
        try:
            entries = sorted(os.scandir(dir_path), key=lambda e: e.name.lower())
        except OSError as e:
            errors.append({"path": str(dir_path), "error": str(e)})
            return
        for entry in entries:
            p = Path(entry.path)
            try:
                s = entry.stat(follow_symlinks=False)
                link = stat.S_ISLNK(s.st_mode)
                isdir = stat.S_ISDIR(s.st_mode)
                isfile = stat.S_ISREG(s.st_mode)
                kind = "symlink" if link else "directory" if isdir else "file" if isfile else "other"
                rel = p.relative_to(root).as_posix()
                size = s.st_size if isfile else 0
                sha = ""
                if isfile and (args.hash_all or (args.hash_max_mb and size <= args.hash_max_mb * 1024**2)):
                    sha = sha256_file(p)
                yield dict(relative_path=rel, absolute_path=str(p), kind=kind,
                           extension=p.suffix.lower() if isfile else "",
                           size_bytes=size, modified_utc=iso(s.st_mtime),
                           sha256=sha, symlink_target=os.readlink(p) if link else "")
                if isdir:
                    yield from walk(p)
            except OSError as e:
                errors.append({"path": str(p), "error": str(e)})

    headers = ["relative_path", "absolute_path", "kind", "extension", "size_bytes", "modified_utc", "sha256", "symlink_target"]
    with csv_path.open("w", newline="", encoding="utf-8") as fc, jsonl_path.open("w", encoding="utf-8") as fj, tree_path.open("w", encoding="utf-8") as ft:
        writer = csv.DictWriter(fc, fieldnames=headers)
        writer.writeheader()
        ft.write(root.name + "/\n")
        for i, row in enumerate(walk(root)):
            writer.writerow(row)
            fj.write(json.dumps(row, ensure_ascii=False) + "\n")
            depth = row["relative_path"].count("/")
            ft.write("  " * depth + ("[D] " if row["kind"] == "directory" else "[L] " if row["kind"] == "symlink" else "[F] ") + row["relative_path"].rsplit("/", 1)[-1] + "\n")
            if row["kind"] == "directory":
                dircount += 1
            elif row["kind"] == "file":
                filecount += 1
                total += row["size_bytes"]
                exts[row["extension"] or "(no extension)"] += 1
                topdirs[row["relative_path"].split("/")[0]] += row["size_bytes"]
                hashcount += bool(row["sha256"])
            elif row["kind"] == "symlink":
                symlinks += 1
            if len(samples) < 12:
                samples.append(row["relative_path"])
            if (i+1) % 20000 == 0:
                print(f"Scanned {i+1:,} paths...", flush=True)

    result = {
        "root": str(root), "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "path_index_format": "UTF-8 CSV + one JSON object per line JSONL",
        "files": filecount, "directories": dircount, "symlinks": symlinks,
        "file_bytes_total": total, "file_size_human": human(total),
        "hashed_files": hashcount, "extensions_file_counts": dict(exts.most_common()),
        "top_level_file_bytes": dict(topdirs.most_common()),
        "errors": errors, "sample_paths": samples,
        "outputs": {"paths_csv": str(csv_path), "paths_jsonl": str(jsonl_path), "tree": str(tree_path)}
    }
    (out / "INVENTORY_SUMMARY.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# Dataset / repository inventory", "",
             f"- Root: `{root}`", f"- Files: **{filecount:,}**; directories: **{dircount:,}**; symlinks: **{symlinks:,}**",
             f"- Total regular-file size: **{human(total)}**", f"- Hashed files: {hashcount:,}",
             "- Full paths: `ALL_PATHS.csv` or `ALL_PATHS.jsonl`", "- Complete hierarchy: `DIRECTORY_TREE.txt`", "",
             "## File counts by extension", "", "| Type | Files |", "|---|---:|"]
    lines += [f"| `{k}` | {v:,} |" for k, v in exts.most_common()]
    if errors:
        lines.extend(["", "## Errors", "", f"Encountered {len(errors)} inaccessible/unreadable items; see INVENTORY_SUMMARY.json."])
    (out / "INVENTORY_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"root": str(root), "files": filecount, "directories": dircount, "size_gib": round(total / 1024**3, 3), "errors": len(errors), "out": str(out)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
