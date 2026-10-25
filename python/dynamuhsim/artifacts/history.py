"""Git-pinned historical payload backups; never remove or overwrite source data.

Bundles restore without Git/ROS, preserving repository-relative payload paths.
They do not provide a complete executable environment for historical replay.
"""
import gzip
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import tarfile
import tempfile

from dynamuhsim.analysis.evidence import load_json, relative_name, sha256_file
from .catalog import GitBlobHashes, verify_catalog
from .store import _publish


def _json(value):
    return json.dumps(value, sort_keys=True, indent=2, allow_nan=False).encode() + b"\n"


def _sha(value):
    return hashlib.sha256(_json(value)).hexdigest()


def _digest(value, length=64):
    if not isinstance(value, str) or not re.fullmatch("[0-9a-f]{%d}" % length, value):
        raise ValueError("Invalid historical artifact digest")
    return value


def _store(store, create=False):
    store = Path(store).resolve()
    for name in ["history/objects", "history/manifests"]:
        path = store / name
        if create: path.mkdir(parents=True, exist_ok=True)
        path.resolve().relative_to(store)
    return store


def _inventory(store, group):
    relative_name(group["inventory_object"])
    path = (store / group["inventory_object"]).resolve()
    path.relative_to(store)
    compressed = path.read_bytes()
    if hashlib.sha256(compressed).hexdigest() != group["inventory_gzip_sha256"]:
        raise ValueError("Compressed inventory hash mismatch")
    data = gzip.decompress(compressed)
    if hashlib.sha256(data).hexdigest() != group["inventory_sha256"]:
        raise ValueError("Inventory hash mismatch")
    inventory = json.loads(data)
    if data != _json(inventory):
        raise ValueError("Expected canonical catalog inventory")
    return inventory


def _members(manifest):
    """Derive exact allowed tar members from the hash-pinned inventory."""
    if manifest.get("schema") != "dynamuhsim_history_bundle/1":
        raise ValueError("Unsupported history bundle")
    _digest(manifest["catalog_sha256"])
    group, inventory = manifest["group"], manifest["inventory"]
    root = group["root"]
    relative_name(root)
    _digest(inventory["revision"], 40)
    if (inventory.get("schema") != "dynamuhsim_asset_inventory/1" or inventory["root"] != root
            or inventory["git_tree"] != group["git_tree"] or _sha(inventory) != group["inventory_sha256"]):
        raise ValueError("History inventory identity/hash mismatch")
    if group["state"] != "inventory_only" or group["payload_externalized"] is not False or group["removal_allowed"] is not False:
        raise ValueError("Backup does not authorize removal")
    members = {}
    for name, row in inventory["entries"].items():
        relative_name(name)
        if row["mode"] not in ["100644", "100755"]:
            raise ValueError("Historical payload supports regular Git files only")
        _digest(row["git_blob"], 40)
        _digest(row["sha256"])
        if type(row["bytes"]) is not int or row["bytes"] < 0:
            raise ValueError("Invalid inventory size")
        members["payload/" + root + "/" + name] = dict(bytes=row["bytes"], sha256=row["sha256"],
                                                        mode=0o755 if row["mode"] == "100755" else 0o644,
                                                        git_blob=row["git_blob"])
    if group["files"] != len(members) or group["logical_bytes"] != sum(row["bytes"] for row in members.values()):
        raise ValueError("History group statistics mismatch")
    if any(str(parent) in members for name in members for parent in PurePosixPath(name).parents):
        raise ValueError("Conflicting payload file ancestry")
    provenance = dict(schema="dynamuhsim_history_provenance/1", catalog_sha256=manifest["catalog_sha256"],
                      revision=inventory["revision"], group=group)
    metadata = {".artifact/inventory.json": _json(inventory), ".artifact/provenance.json": _json(provenance)}
    for name, data in metadata.items():
        members[name] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest(), mode=0o644)
    return members, metadata


class _HashReader:
    def __init__(self, stream, size):
        self.stream, self.left = stream, size
        self.sha256 = hashlib.sha256()
        self.git = hashlib.sha1(("blob %d\0" % size).encode())

    def read(self, size=-1):
        size = self.left if size < 0 else min(size, self.left)
        data = self.stream.read(size)
        if len(data) != size:
            raise ValueError("Truncated historical payload")
        self.left -= len(data)
        self.sha256.update(data)
        self.git.update(data)
        return data


class _GitPayloads(GitBlobHashes):
    def add(self, archive, member, row):
        self.process.stdin.write((row["git_blob"] + "\n").encode())
        self.process.stdin.flush()
        header = self.process.stdout.readline().split()
        if header != [row["git_blob"].encode(), b"blob", str(row["bytes"]).encode()]:
            raise ValueError("Git payload identity mismatch")
        reader = _HashReader(self.process.stdout, row["bytes"])
        archive.addfile(member, reader)
        if (reader.left or reader.sha256.hexdigest() != row["sha256"] or reader.git.hexdigest() != row["git_blob"]
                or self.process.stdout.read(1) != b"\n"):
            raise ValueError("Git payload hash mismatch")


def _tar_info(name, row):
    member = tarfile.TarInfo(name)
    member.size, member.mode = row["bytes"], row["mode"]
    member.uid = member.gid = member.mtime = 0
    member.uname = member.gname = ""
    return member


def _check_archive(path, manifest, output=None):
    members, _ = _members(manifest)
    if (path.stat().st_size != manifest["archive_bytes"] or sha256_file(path) != _digest(manifest["object_id"])):
        raise ValueError("Historical archive identity/hash mismatch")
    seen = set()
    with tarfile.open(path, "r|gz") as archive:
        for member in archive:
            relative_name(member.name)
            if member.name not in members or member.name in seen:
                raise ValueError("Unexpected/duplicate historical archive member")
            seen.add(member.name)
            row = members[member.name]
            if not member.isfile() or member.size != row["bytes"] or member.mode != row["mode"]:
                raise ValueError("Historical archive type/size/mode mismatch")
            stream = _HashReader(archive.extractfile(member), member.size)
            destination = None
            try:
                if output is not None:
                    target = output / member.name
                    target.resolve().relative_to(output)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    destination = target.open("xb")
                while stream.left:
                    data = stream.read(min(stream.left, 1024 * 1024))
                    if destination is not None: destination.write(data)
            finally:
                if destination is not None: destination.close()
            if stream.sha256.hexdigest() != row["sha256"] or ("git_blob" in row and stream.git.hexdigest() != row["git_blob"]):
                raise ValueError("Historical payload SHA256/Git blob mismatch: " + member.name)
            if output is not None: target.chmod(row["mode"])
    if seen != set(members):
        raise ValueError("Incomplete historical archive")
    if output is not None:
        # Independently reread extracted bytes and modes, not just the tar stream.
        actual = {path.relative_to(output).as_posix() for path in output.rglob("*") if path.is_file() or path.is_symlink()}
        if actual != set(members): raise ValueError("Restored payload file coverage mismatch")
        for name, row in members.items():
            path = output / name
            if path.is_symlink() or path.stat().st_size != row["bytes"] or path.stat().st_mode & 0o777 != row["mode"] or sha256_file(path) != row["sha256"]:
                raise ValueError("Restored payload bytes/mode mismatch")
    return manifest


def pack_history(repo, store, catalog):
    """Archive every catalog group from committed blobs; source checkout is read-only."""
    repo, store = Path(repo).resolve(), _store(store, create=True)
    verify_catalog(repo, store, catalog)
    bundles = []
    catalog_sha = _sha(catalog)
    with _GitPayloads(repo) as git:
        for group in catalog["entries"]:
            inventory = _inventory(store, group)
            manifest = dict(schema="dynamuhsim_history_bundle/1", catalog_sha256=catalog_sha, group=group, inventory=inventory)
            members, metadata = _members(manifest)
            handle, temporary = tempfile.mkstemp(prefix="history-", suffix=".partial", dir=store / "history/objects")
            os.close(handle)
            temporary = Path(temporary)
            try:
                with temporary.open("wb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0, compresslevel=6) as zipped:
                    with tarfile.open(mode="w|", fileobj=zipped, format=tarfile.PAX_FORMAT) as archive:
                        for name, row in sorted(members.items()):
                            member = _tar_info(name, row)
                            if name in metadata: archive.addfile(member, io.BytesIO(metadata[name]))
                            else: git.add(archive, member, row)
                object_id = sha256_file(temporary)
                manifest.update(object_id=object_id, archive_bytes=temporary.stat().st_size)
                _check_archive(temporary, manifest)
                target = store / "history/objects" / (object_id + ".tar.gz")
                try: os.link(temporary, target)
                except FileExistsError:
                    if sha256_file(target) != object_id: raise ValueError("Existing history object is corrupt")
                _publish(store / "history/manifests" / (object_id + ".json"), _json(manifest))
                bundles.append(dict(id=group["id"], root=group["root"], object_id=object_id,
                                    archive_bytes=manifest["archive_bytes"], files=group["files"], logical_bytes=group["logical_bytes"]))
            finally:
                temporary.unlink(missing_ok=True)
    return dict(schema="dynamuhsim_history_index/1", catalog=catalog, catalog_sha256=catalog_sha, bundles=bundles,
                archive_bytes=sum(row["archive_bytes"] for row in bundles), removal_allowed=False,
                storage="Local content-addressed backups; no remote replication is claimed")


def verify_history(store, object_id):
    store = _store(store)
    _digest(object_id)
    manifest_path = store / "history/manifests" / (object_id + ".json")
    manifest_path.resolve().relative_to(store)
    manifest = load_json(manifest_path)
    if manifest["object_id"] != object_id: raise ValueError("History manifest object ID mismatch")
    archive = store / "history/objects" / (object_id + ".tar.gz")
    archive.resolve().relative_to(store)
    return _check_archive(archive, manifest)


def verify_history_index(store, index):
    if index.get("schema") != "dynamuhsim_history_index/1" or index["removal_allowed"] is not False:
        raise ValueError("Unsupported/removal-authorizing history index")
    catalog = index["catalog"]
    if catalog.get("schema") != "dynamuhsim_asset_catalog/1" or _sha(catalog) != index["catalog_sha256"]:
        raise ValueError("History index catalog identity mismatch")
    groups = {row["id"]: row for row in catalog["entries"]}
    roots = [row["root"] for row in catalog["entries"]]
    if (len(groups) != len(catalog["entries"]) or len(roots) != len(set(roots))
            or any(a.startswith(b + "/") for a in roots for b in roots if a != b)):
        raise ValueError("Duplicate/overlapping history groups")
    if len(index["bundles"]) != len(groups) or {row["id"] for row in index["bundles"]} != set(groups):
        raise ValueError("History index group coverage mismatch")
    total = count = packed = 0
    for bundle in index["bundles"]:
        manifest = verify_history(store, bundle["object_id"])
        group = groups[bundle["id"]]
        if (manifest["catalog_sha256"] != index["catalog_sha256"] or manifest["group"] != group
                or manifest["inventory"]["revision"] != catalog["revision"]
                or any(bundle[key] != group[key] for key in ["id", "root", "files", "logical_bytes"])
                or bundle["archive_bytes"] != manifest["archive_bytes"]):
            raise ValueError("History index bundle binding mismatch")
        total += group["logical_bytes"]
        count += group["files"]
        packed += bundle["archive_bytes"]
    if total != catalog["logical_bytes"] or count != catalog["files"] or packed != index["archive_bytes"]:
        raise ValueError("History index total mismatch")
    return dict(status="PASS", revision=catalog["revision"], catalog_sha256=index["catalog_sha256"],
                groups=len(groups), files=count, logical_bytes=total, archive_bytes=packed, removal_allowed=False)


def restore_history(store, object_id, output):
    """Fresh destination only. No Git, catalog cache, ROS, imports or replay execution."""
    manifest = verify_history(store, object_id)
    store, output_input = _store(store), Path(output)
    if output_input.exists() or output_input.is_symlink():
        raise FileExistsError("Restore destination already exists")
    output = output_input.resolve()
    if output == store or output.is_relative_to(store) or store.is_relative_to(output):
        raise ValueError("Restore destination must be separate from store")
    output.mkdir(parents=True, exist_ok=False)
    archive = store / "history/objects" / (object_id + ".tar.gz")
    _check_archive(archive, manifest, output=output)
    receipt = dict(schema="dynamuhsim_history_restore/1", status="PASS", object_id=object_id,
                   catalog_sha256=manifest["catalog_sha256"], revision=manifest["inventory"]["revision"],
                   root=manifest["group"]["root"], verified_payload_files=manifest["group"]["files"],
                   verified_payload_bytes=manifest["group"]["logical_bytes"], verified_metadata_files=2,
                   removal_allowed=False)
    with (output / "restore_receipt.json").open("xb") as stream: stream.write(_json(receipt))
    return receipt
