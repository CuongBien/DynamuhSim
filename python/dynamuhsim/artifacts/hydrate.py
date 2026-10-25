"""Hydrate exact historical groups into absent checkout paths; never overwrite."""
import ctypes
import errno
import os
from pathlib import Path
import tempfile

from dynamuhsim.analysis.evidence import relative_name, sha256_file
from .catalog import GitBlobHashes, tree_files, tree_identity, revision
from .history import history_groups, verify_history, restore_history, _members, _GitPayloads


def _selection(index, group_id):
    groups = history_groups(index)
    if group_id not in groups: raise ValueError("Unknown historical group: " + group_id)
    return groups[group_id], next(row for row in index["bundles"] if row["id"] == group_id)


def _manifest(repo, store, index, group_id, source):
    group, bundle = _selection(index, group_id)
    if source == "archive":
        if store is None: raise ValueError("Archive hydration requires --store")
        manifest = verify_history(store, bundle["object_id"])
        if manifest["catalog_sha256"] != index["catalog_sha256"] or manifest["group"] != group:
            raise ValueError("Hydration archive/index identity mismatch")
    elif source == "git":
        commit = revision(repo, index["catalog"]["revision"])
        selected = tree_files(repo, commit, group["root"])
        entries = {}
        with GitBlobHashes(repo) as hashes:
            for name, row in sorted(selected.items()):
                if not name.startswith(group["root"] + "/"): raise ValueError("Git payload outside group")
                entries[name[len(group["root"])+1:]] = dict(**row, sha256=hashes.sha(row))
        inventory = dict(schema="dynamuhsim_asset_inventory/1", revision=commit, root=group["root"],
                         git_tree=tree_identity(repo, commit, group["root"]), entries=entries)
        manifest = dict(schema="dynamuhsim_history_bundle/1", catalog_sha256=index["catalog_sha256"], group=group, inventory=inventory)
    else: raise ValueError("Hydration source must be archive or git")
    if manifest["inventory"]["revision"] != index["catalog"]["revision"]:
        raise ValueError("Hydration revision mismatch")
    _members(manifest)
    return manifest


def _target(checkout, root):
    checkout = Path(checkout).resolve(strict=True)
    if not checkout.is_dir(): raise ValueError("Checkout must be an existing directory")
    relative = relative_name(root)
    target = checkout.joinpath(*relative.parts)
    for path in [target, *target.parents]:
        if path == checkout: break
        if path.is_symlink(): raise ValueError("Hydration path contains a symlink")
    target.resolve().relative_to(checkout)
    return checkout, target


def _compare(target, inventory):
    if not target.exists(): return "absent"
    if not target.is_dir() or target.is_symlink(): return "conflict"
    actual = {path.relative_to(target).as_posix() for path in target.rglob("*") if path.is_file() or path.is_symlink()}
    if actual != set(inventory["entries"]): return "conflict"
    for name, row in inventory["entries"].items():
        path = target / name
        if (path.is_symlink() or path.stat().st_size != row["bytes"] or sha256_file(path) != row["sha256"]
                or path.stat().st_mode & 0o777 != (0o755 if row["mode"] == "100755" else 0o644)):
            return "conflict"
    return "present_verified"


def _publish_directory(source, target):
    # Linux renameat2 provides an atomic no-replace operation even for empty dirs.
    if os.name == "nt":
        os.rename(source, target)  # Windows rename refuses existing destinations.
        return
    library = ctypes.CDLL(None, use_errno=True)
    rename = getattr(library, "renameat2", None)
    if rename is None: raise RuntimeError("Atomic hydrate publish requires renameat2 on this platform")
    if rename(-100, os.fsencode(source), -100, os.fsencode(target), 1):
        code = ctypes.get_errno()
        if code == errno.EEXIST: raise FileExistsError("Hydration destination appeared during publish")
        raise OSError(code, os.strerror(code), str(target))


def hydrate_history(checkout, index, group_id, *, source="archive", store=None, git_repo=None, check=False):
    group, bundle = _selection(index, group_id)
    checkout, target = _target(checkout, group["root"])
    manifest = _manifest(git_repo or checkout, store, index, group_id, source)
    state = _compare(target, manifest["inventory"])
    if state == "conflict": raise ValueError("Existing payload differs; preserve it and use a separate checkout: " + str(target))
    report = dict(schema="dynamuhsim_history_hydration/1", group=group_id, root=group["root"],
                  revision=manifest["inventory"]["revision"], inventory_sha256=group["inventory_sha256"],
                  object_id=bundle["object_id"], source=source, state=state, files=group["files"], logical_bytes=group["logical_bytes"], changed=False)
    if check or state == "present_verified": return report
    target.parent.mkdir(parents=True, exist_ok=True)
    # Recheck ancestors after mkdir; staging stays on the destination filesystem.
    _target(checkout, group["root"])
    with tempfile.TemporaryDirectory(prefix=".hydrate-", dir=target.parent) as directory:
        staging = Path(directory)
        if source == "archive":
            restore_history(store, bundle["object_id"], staging / "restore")
            payload = staging / "restore/payload" / group["root"]
        else:
            payload = staging / "payload"
            payload.mkdir()
            with _GitPayloads(git_repo or checkout) as git:
                for name, row in manifest["inventory"]["entries"].items():
                    destination = payload / name
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    with git.blob(row) as reader, destination.open("xb") as stream:
                        while reader.left: stream.write(reader.read(min(reader.left, 1024 * 1024)))
                    destination.chmod(0o755 if row["mode"] == "100755" else 0o644)
        if _compare(payload, manifest["inventory"]) != "present_verified":
            raise ValueError("Staged hydration failed byte/mode verification")
        _target(checkout, group["root"])
        _publish_directory(payload, target)
    if _compare(target, manifest["inventory"]) != "present_verified":
        raise ValueError("Hydrated payload changed after publication")
    report.update(state="present_verified", changed=True)
    return report
