"""Hydrate only approved missing media while preserving every report control file."""
import os
from pathlib import Path
import shutil
import tarfile
import tempfile

from dynamuhsim.analysis.evidence import relative_name, sha256_file
from .catalog import GitBlobHashes, revision, tree_files, tree_identity
from .history import history_groups, _sha, _GitPayloads
from .hydrate import _manifest, _target
from .report_plan import MEDIA, validate_report_plan


def create_media_plan(repo, index, report_plan, ref="HEAD"):
    groups = validate_report_plan(index, report_plan)
    commit = revision(repo, ref)
    files = tree_files(repo, commit)
    summaries = history_groups(index)
    result = {}
    with GitBlobHashes(repo) as hashes:
        for name, group in groups.items():
            if group["action"] != "retain": continue
            root = group["root"]
            if tree_identity(repo, commit, root) != summaries[name]["git_tree"]:
                raise ValueError("Retained report tree differs from archive: " + name)
            media = {path[len(root)+1:]: dict(**row, sha256=hashes.sha(row)) for path, row in files.items()
                     if path.startswith(root + "/") and Path(path).suffix.lower() in MEDIA}
            result[name] = dict(id=name, root=root, object_id=group["object_id"], media=media,
                                files=len(media), logical_bytes=sum(row["bytes"] for row in media.values()))
    return dict(schema="dynamuhsim_report_media_plan/1", revision=commit, history_index_sha256=_sha(index),
                report_dependency_plan_sha256=_sha(report_plan), groups=result,
                files=sum(g["files"] for g in result.values()), logical_bytes=sum(g["logical_bytes"] for g in result.values()),
                preserved="All non-media files, including gates, model arrays, frozen manifests, test_opened seals and source snapshots")


def validate_media_plan(index, plan):
    groups = history_groups(index)
    if plan.get("schema") != "dynamuhsim_report_media_plan/1" or plan["history_index_sha256"] != _sha(index):
        raise ValueError("Media plan/index identity mismatch")
    bundles = {g["id"]: g for g in index["bundles"]}
    for name, group in plan["groups"].items():
        if name not in groups or not name.startswith("reports:") or group["id"] != name or group["root"] != groups[name]["root"] or group["object_id"] != bundles[name]["object_id"]:
            raise ValueError("Media plan group identity mismatch")
        for path, row in group["media"].items():
            relative_name(path)
            if Path(path).suffix.lower() not in MEDIA or row["mode"] not in ["100644", "100755"]:
                raise ValueError("Media plan cannot select controls, model arrays or code")
        if group["files"] != len(group["media"]) or group["logical_bytes"] != sum(row["bytes"] for row in group["media"].values()):
            raise ValueError("Media group statistics mismatch")
    if plan["files"] != sum(g["files"] for g in plan["groups"].values()) or plan["logical_bytes"] != sum(g["logical_bytes"] for g in plan["groups"].values()):
        raise ValueError("Media plan totals mismatch")
    return plan["groups"]


def _inspect(checkout, target, inventory, allowed_missing):
    if not target.is_dir(): raise ValueError("Missing report controls; hydrate the complete report group first")
    expected = inventory["entries"]
    actual = {p.relative_to(target).as_posix() for p in target.rglob("*") if p.is_file() or p.is_symlink()}
    if actual - set(expected): raise ValueError("Extra report files; preserve them and use a separate checkout")
    missing = set(expected) - actual
    if missing - set(allowed_missing): raise ValueError("Missing report control files; refusing partial recovery")
    for name in actual:
        _, path = _target(checkout, inventory["root"] + "/" + name)
        row = expected[name]
        if (not path.is_file() or path.stat().st_size != row["bytes"] or sha256_file(path) != row["sha256"]
                or path.stat().st_mode & 0o777 != (0o755 if row["mode"] == "100755" else 0o644)):
            raise ValueError("Existing report file differs; never overwrite: " + name)
    return sorted(missing)


def hydrate_report_media(checkout, index, plan, selected, *, source="git", store=None, git_repo=None, check=False):
    groups = validate_media_plan(index, plan)
    if not set(selected).issubset(groups): raise ValueError("Unknown media group")
    receipts = []
    for name in sorted(set(selected)):
        group = groups[name]
        checkout, target = _target(checkout, group["root"])
        manifest = _manifest(git_repo or checkout, store, index, name, source)
        inventory = manifest["inventory"]
        if any(inventory["entries"].get(path) != row for path, row in group["media"].items()):
            raise ValueError("Media selection differs from archived inventory")
        missing = _inspect(checkout, target, inventory, group["media"])
        published = []
        if missing and not check:
            with tempfile.TemporaryDirectory(prefix=".hydrate-media-", dir=target.parent) as directory:
                staging = Path(directory)
                if source == "archive":
                    wanted = {"payload/" + group["root"] + "/" + path: path for path in missing}
                    with tarfile.open(Path(store) / "history/objects" / (group["object_id"] + ".tar.gz"), "r|gz") as archive:
                        for member in archive:
                            if member.name not in wanted: continue
                            path = staging / wanted[member.name]
                            path.parent.mkdir(parents=True, exist_ok=True)
                            with archive.extractfile(member) as src, path.open("xb") as dst: shutil.copyfileobj(src, dst)
                            path.chmod(0o755 if group["media"][wanted[member.name]]["mode"] == "100755" else 0o644)
                else:
                    with _GitPayloads(git_repo or checkout) as git:
                        for path in missing:
                            destination = staging / path;destination.parent.mkdir(parents=True, exist_ok=True)
                            row = group["media"][path]
                            with git.blob(row) as src, destination.open("xb") as dst:
                                while src.left: dst.write(src.read(min(src.left, 1024 * 1024)))
                            destination.chmod(0o755 if row["mode"] == "100755" else 0o644)
                for path in missing:
                    row = group["media"][path];staged = staging / path
                    if staged.stat().st_size != row["bytes"] or sha256_file(staged) != row["sha256"]:
                        raise ValueError("Staged media hash mismatch")
                # Verify controls again immediately before publishing media.
                _inspect(checkout, target, inventory, group["media"])
                for path in missing:
                    _, destination = _target(checkout, group["root"] + "/" + path)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    _target(checkout, group["root"] + "/" + path)
                    os.link(staging / path, destination)  # Atomic, fails if another writer created it.
                    published.append(path)
            if _inspect(checkout, target, inventory, []) != []: raise ValueError("Media recovery incomplete")
        receipts.append(dict(group=name, missing_files=missing if check else [], restored_files=published,
                             restored_bytes=sum(group["media"][p]["bytes"] for p in published),
                             controls_unchanged=True, status="MEDIA_MISSING" if check and missing else "PASS"))
    return dict(schema="dynamuhsim_report_media_hydration/1", status="MEDIA_MISSING" if any(r["status"] != "PASS" for r in receipts) else "PASS",
                source=source, groups=receipts, changed_groups=sum(bool(r["restored_files"]) for r in receipts),
                changed_files=sum(len(r["restored_files"]) for r in receipts), changed_bytes=sum(r["restored_bytes"] for r in receipts))
