"""Content-addressed capture bundles. Never evict before a verified round trip."""
import gzip
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import posixpath
import re
import subprocess
import tarfile
import tempfile
import uuid

from dynamuhsim.analysis.evidence import load_json, dump_json, relative_name, sha256_file, resolve_sources


def _paths(store, object_id=None, *, create=False):
    store = Path(store).resolve()
    for name in ["objects", "manifests", "restores"]:
        path = store / name
        if create:
            path.mkdir(parents=True, exist_ok=True)
        path.resolve().relative_to(store)
    if object_id is not None and not re.fullmatch(r"[0-9a-f]{64}", object_id):
        raise ValueError("Invalid artifact object ID")
    return store


def _link_target(name, target):
    if not target or target.startswith("/") or "\\" in target or ":" in target:
        raise ValueError("Unsafe archive symlink")
    combined = posixpath.normpath(posixpath.join(posixpath.dirname(name), target))
    relative_name(combined)
    if not combined.startswith("run/"):
        raise ValueError("Symlink leaves capture payload")
    return combined


def _inventory(run):
    entries, paths = {}, {}
    for base, directories, files in os.walk(run, followlinks=False):
        for item in sorted(directories + files):
            path = Path(base) / item
            name = "run/" + path.relative_to(run).as_posix()
            relative_name(name)
            if path.is_symlink():
                target = path.resolve(strict=True)
                target.relative_to(run)
                portable = os.path.relpath(target, path.parent).replace(os.sep, "/")
                _link_target(name, portable)
                entries[name] = dict(type="symlink", target=portable, original_target=os.readlink(path))
            elif path.is_file():
                entries[name] = dict(type="file", bytes=path.stat().st_size, sha256=sha256_file(path), mode=path.stat().st_mode & 0o777)
                paths[name] = path
            elif not path.is_dir():
                raise ValueError("Unsupported capture member: " + name)
    return entries, paths


def _publish(path, data):
    temporary = path.with_name(path.name + ".partial-" + uuid.uuid4().hex)
    try:
        with temporary.open("xb") as stream:
            stream.write(data)
        try:
            os.link(temporary, path)
        except FileExistsError:
            if path.read_bytes() != data:
                raise RuntimeError("Refusing to overwrite a different immutable manifest")
    finally:
        temporary.unlink(missing_ok=True)


def pack_capture(run, repo, store, *, source_roots=()):
    run, repo, store = Path(run).resolve(), Path(repo).resolve(), Path(store).resolve()
    original_relative = run.relative_to(repo).as_posix()
    if store == run or store.is_relative_to(run) or run.is_relative_to(store):
        raise ValueError("Artifact store and source run must be separate")
    store = _paths(store, create=True)
    if (run / "artifact_ref.json").exists():
        raise ValueError("Restore the complete artifact before repacking an evicted run")
    result = load_json(run / "episode/result.json")
    sources, missing = resolve_sources(result["source_sha256"], [repo, *source_roots], git_repo=repo, commit=result["head"])
    if missing:
        raise FileNotFoundError("Cannot archive incomplete provenance: " + ", ".join(missing))
    entries, paths = _inventory(run)
    blobs = {}
    for name, value in sources.items():
        member = "source/" + name
        blobs[member] = value["data"]
        entries[member] = dict(type="file", bytes=len(value["data"]), sha256=hashlib.sha256(value["data"]).hexdigest(), mode=value["mode"])
    capture = dict(schema="dynamuhsim_capture_bundle/1", original_run_relative=original_relative,
                   runtime_commit=result["head"], raw_result_sha256=sha256_file(run / "episode/result.json"),
                   source_sha256=result["source_sha256"],
                   closed=bool(result.get("cleanup", {}).get("status") == "PASS" and not result.get("cleanup", {}).get("remaining")
                               and result.get("recording", {}).get("returncode") == 0 and result.get("recording", {}).get("metadata_present")),
                   original_links={name: row["original_target"] for name, row in entries.items() if row["type"] == "symlink"},
                   symlink_policy="Internal links become relative; original link text retained in manifest")
    metadata = json.dumps(capture, sort_keys=True, indent=2, allow_nan=False).encode() + b"\n"
    blobs[".artifact/capture.json"] = metadata
    entries[".artifact/capture.json"] = dict(type="file", bytes=len(metadata), sha256=hashlib.sha256(metadata).hexdigest(), mode=0o644)
    handle, temporary = tempfile.mkstemp(prefix="packing-", suffix=".partial", dir=store / "objects")
    os.close(handle)
    temporary = Path(temporary)
    try:
        with temporary.open("wb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0, compresslevel=6) as zipped:
            with tarfile.open(mode="w|", fileobj=zipped, format=tarfile.PAX_FORMAT) as archive:
                for name, record in sorted(entries.items()):
                    member = tarfile.TarInfo(name)
                    member.mtime = member.uid = member.gid = 0
                    member.uname = member.gname = ""
                    if record["type"] == "symlink":
                        member.type, member.linkname, member.mode = tarfile.SYMTYPE, record["target"], 0o777
                        archive.addfile(member)
                    else:
                        member.size, member.mode = record["bytes"], record["mode"]
                        if name in blobs:
                            archive.addfile(member, io.BytesIO(blobs[name]))
                        else:
                            with paths[name].open("rb") as stream:
                                archive.addfile(member, stream)
        after, _ = _inventory(run)
        if after != {name: row for name, row in entries.items() if name.startswith("run/")}:
            raise RuntimeError("Capture changed during packing; original retained")
        object_id = sha256_file(temporary)
        target = store / "objects" / (object_id + ".tar.gz")
        manifest = dict(schema="dynamuhsim_artifact/1", object_id=object_id, archive_bytes=temporary.stat().st_size,
                        capture=capture, entries=entries)
        _verify_archive(temporary, manifest, object_id)
        try:
            os.link(temporary, target)
        except FileExistsError:
            if sha256_file(target) != object_id:
                raise RuntimeError("Existing content-addressed object is corrupt")
        _publish(store / "manifests" / (object_id + ".json"), json.dumps(manifest, sort_keys=True, indent=2).encode() + b"\n")
        verify(store, object_id)
        return manifest
    finally:
        temporary.unlink(missing_ok=True)


def verify(store, object_id):
    store = _paths(store, object_id)
    manifest = load_json(store / "manifests" / (object_id + ".json"))
    path = store / "objects" / (object_id + ".tar.gz")
    return _verify_archive(path, manifest, object_id)


def _verify_archive(path, manifest, object_id):
    if manifest.get("schema") != "dynamuhsim_artifact/1" or manifest.get("object_id") != object_id or path.stat().st_size != manifest.get("archive_bytes") or sha256_file(path) != object_id:
        raise ValueError("Artifact identity/hash failed")
    seen, metadata = set(), None
    entries = manifest["entries"]
    links = {name for name, row in entries.items() if row["type"] == "symlink"}
    for name in entries:
        relative_name(name)
        if not name.startswith(("run/", "source/", ".artifact/")) or any(str(parent) in links for parent in PurePosixPath(name).parents):
            raise ValueError("Unsafe artifact member ancestry")
    with tarfile.open(path, "r|gz") as archive:
        for member in archive:
            name = member.name
            relative_name(name)
            if name in seen or name not in entries:
                raise ValueError("Unexpected/duplicate archive member")
            seen.add(name)
            record = entries[name]
            if record["type"] == "symlink":
                _link_target(name, record["target"])
                if not member.issym() or member.linkname != record["target"]:
                    raise ValueError("Symlink manifest mismatch")
                continue
            if record["type"] != "file" or not member.isfile() or member.size != record["bytes"] or member.mode != record["mode"] or member.mode & ~0o777:
                raise ValueError("Artifact member type/size/mode failed")
            digest = hashlib.sha256()
            stream = archive.extractfile(member)
            content = [] if name == ".artifact/capture.json" else None
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
                if content is not None:
                    content.append(block)
            if digest.hexdigest() != record["sha256"]:
                raise ValueError("Artifact member hash failed: " + name)
            if content is not None:
                metadata = json.loads(b"".join(content))
    if seen != set(entries) or metadata != manifest["capture"]:
        raise ValueError("Incomplete archive or capture metadata mismatch")
    if metadata.get("original_links") != {name: row["original_target"] for name, row in entries.items() if row["type"] == "symlink"}:
        raise ValueError("Original symlink provenance mismatch")
    if entries.get("run/episode/result.json", {}).get("sha256") != metadata["raw_result_sha256"]:
        raise ValueError("Result identity mismatch")
    for name, digest in metadata["source_sha256"].items():
        relative_name(name)
        if entries.get("source/" + name, {}).get("sha256") != digest:
            raise ValueError("Incomplete source snapshot")
    return manifest


def restore(store, object_id, output):
    manifest = verify(store, object_id)
    store, output = Path(store).resolve(), Path(output).resolve()
    if output == store or output.is_relative_to(store) or store.is_relative_to(output):
        raise ValueError("Restore destination must be separate from store")
    output.mkdir(parents=True, exist_ok=False)
    links = []
    with tarfile.open(store / "objects" / (object_id + ".tar.gz"), "r|gz") as archive:
        for member in archive:
            target = output / member.name
            target.resolve().relative_to(output)
            target.parent.mkdir(parents=True, exist_ok=True)
            if member.issym():
                links.append((target, member.linkname))
                continue
            record = manifest["entries"][member.name]
            digest = hashlib.sha256()
            with archive.extractfile(member) as source, target.open("xb") as destination:
                for block in iter(lambda: source.read(1024 * 1024), b""):
                    destination.write(block)
                    digest.update(block)
            if digest.hexdigest() != record["sha256"]:
                raise ValueError("Restore member hash failed")
            target.chmod(record["mode"])
    for target, link in links:
        (target.parent / link).resolve().relative_to(output)
        target.symlink_to(link)
        if not target.exists():
            raise ValueError("Restored internal symlink target missing")
    receipt = dict(schema="dynamuhsim_artifact_restore/1", object_id=object_id, output=str(output),
                   verified_files=sum(row["type"] == "file" for row in manifest["entries"].values()),
                   verified_links=sum(row["type"] == "symlink" for row in manifest["entries"].values()))
    dump_json(output / "restore_receipt.json", receipt)
    # Eviction requires this independent verified extraction, not just archive CRC.
    _publish(store / "restores" / (object_id + ".json"), json.dumps(dict(schema=receipt["schema"], object_id=object_id, verified_files=receipt["verified_files"], verified_links=receipt["verified_links"]), sort_keys=True).encode() + b"\n")
    return receipt


def evict_raw(store, object_id, run, repo):
    """Remove only ignored .npy/.mcap payload after verified pack AND restore."""
    manifest = verify(store, object_id)
    store, run, repo = Path(store).resolve(), Path(run).resolve(), Path(repo).resolve()
    restored_path = store / "restores" / (object_id + ".json")
    if not restored_path.is_file():
        raise ValueError("A verified restore round trip is required before eviction")
    restored = load_json(restored_path)
    if (restored.get("schema") != "dynamuhsim_artifact_restore/1" or restored.get("object_id") != object_id
            or restored.get("verified_files") != sum(row["type"] == "file" for row in manifest["entries"].values())
            or not manifest["capture"]["closed"]):
        raise ValueError("Verified round trip and closed recording are required")
    if run.relative_to(repo).as_posix() != manifest["capture"]["original_run_relative"]:
        raise ValueError("Eviction target differs from archived run")
    selected = {}
    for name, record in manifest["entries"].items():
        if record["type"] != "file":
            continue
        if ((name.startswith("run/episode/paired_depth/on/") or name.startswith("run/episode/paired_depth/off/")) and name.endswith(".npy")) or (name.startswith("run/episode/raw_bag/") and name.endswith(".mcap")):
            selected[name[4:]] = record
    if not selected:
        raise ValueError("No raw capture payload selected")
    tracked = subprocess.check_output(["git", "-C", str(repo), "ls-files", "--", str(run)], text=True)
    if tracked:
        raise ValueError("Refusing to evict a run containing tracked files")
    ref = run / "artifact_ref.json"
    if ref.is_symlink():
        raise ValueError("Artifact reference must not be a symlink")
    prior = load_json(ref) if ref.exists() else None
    if prior is not None and prior.get("object_id") != object_id:
        raise ValueError("Different artifact reference already exists")
    if prior is not None and (prior.get("schema") != "dynamuhsim_artifact_ref/1" or prior.get("raw_files") != sorted(selected) or prior.get("state") not in ["evicting", "evicted"]):
        raise ValueError("Artifact reference does not describe this raw payload")
    actual_raw = {p.relative_to(run).as_posix() for pattern in ["episode/paired_depth/on/*.npy", "episode/paired_depth/off/*.npy", "episode/raw_bag/*.mcap"] for p in run.glob(pattern)}
    if actual_raw - set(selected):
        raise ValueError("New raw files appeared after packing; preserve and repack")
    for name, record in selected.items():
        path = run / name
        path.resolve().relative_to(run)
        if path.is_symlink() or (not path.exists() and prior is None):
            raise ValueError("Eviction source missing or symlinked")
        if path.exists() and (sha256_file(path) != record["sha256"] or subprocess.run(["git", "-C", str(repo), "check-ignore", "--quiet", str(path)]).returncode != 0):
            raise ValueError("Raw payload changed or is not ignored: " + name)
    current, _ = _inventory(run)
    current.pop("run/artifact_ref.json", None)
    expected_run = {name: row for name, row in manifest["entries"].items() if name.startswith("run/")}
    allowed_missing = {"run/" + name for name in selected} if prior is not None else set()
    if any(expected_run.get(name) != row for name, row in current.items()) or set(expected_run) - set(current) - allowed_missing:
        raise ValueError("Capture metadata or payload differs from the archived run")
    reference = dict(schema="dynamuhsim_artifact_ref/1", object_id=object_id, store=str(store), state="evicting", raw_files=sorted(selected), raw_bytes=sum(row["bytes"] for row in selected.values()),
                     restore="Restore into a new directory; capture is under run/ and source snapshots under source/.")
    dump_json(ref, reference)
    for name, record in selected.items():
        path = run / name
        if path.exists():
            # Recheck immediately before each deletion; no recursive filesystem removal.
            path.resolve().relative_to(run)
            if path.is_symlink() or sha256_file(path) != record["sha256"]:
                raise RuntimeError("Raw payload changed during eviction; reference allows recovery")
            path.unlink()
    reference["state"] = "evicted"
    dump_json(ref, reference)
    return reference
