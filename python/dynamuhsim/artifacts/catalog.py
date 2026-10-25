"""Catalog committed historical payloads and verify lean/full package installs.

Inventories are not payload backups and never authorize removing tracked files.
"""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

from dynamuhsim.analysis.evidence import relative_name, sha256_file
from .store import _publish

PACKAGE = "src/custom_corridor"
WORLDS = PACKAGE + "/worlds"
POLICY = PACKAGE + "/config/world_payloads.txt"
BENCHMARK = "experiments/social_navigation/benchmark"
ASSET_DIRS = ["worlds", "launch", "models", "urdf", "rviz", "config", "maps"]
GATE_FILES = [
    "socnavgym_nav2_pilot/reports/gate01/validation.json",
    "socnavgym_nav2_pilot/reports/empty01/validation.json",
    "socnavgym_group_nav2/reports/reaction_gate.json",
    "socnavgym_group_nav2/reports/gate03/validation.json",
    "socnavgym_group_nav2/reports/offline05/validation.json",
    "shared_nav2_pilot/reports/gate01/validation.json",
    "common_response/reports/run01/protocol.json",
]


def revision(repo, ref):
    return subprocess.check_output(["git", "-C", str(repo), "rev-parse", "--verify", "--end-of-options", ref + "^{commit}"], text=True).strip()


def tree_files(repo, commit, root=None):
    command = ["git", "-C", str(repo), "ls-tree", "-rlz", commit]
    if root is not None:
        relative_name(root)
        command += ["--", root]
    records = {}
    for row in subprocess.check_output(command).split(b"\0"):
        if not row:
            continue
        header, name = row.split(b"\t", 1)
        mode, kind, oid, size = header.split()
        if kind != b"blob":
            continue
        name = name.decode("utf-8")
        relative_name(name)
        records[name] = dict(mode=mode.decode(), git_blob=oid.decode(), bytes=int(size))
    return records


def policy_at(repo, commit):
    data = subprocess.check_output(["git", "-C", str(repo), "show", commit + ":" + POLICY])
    paths = [line.split("#", 1)[0].strip() for line in data.decode().splitlines()]
    paths = [path for path in paths if path]
    for path in paths:
        relative_name(path)
    if not paths or len(set(paths)) != len(paths) or any(a.startswith(b + "/") for a in paths for b in paths if a != b):
        raise ValueError("Invalid/overlapping world payload policy")
    return paths, hashlib.sha256(data).hexdigest()


class GitBlobHashes:
    def __init__(self, repo):
        self.process = subprocess.Popen(["git", "-C", str(repo), "cat-file", "--batch"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.cache = {}

    def __enter__(self): return self

    def __exit__(self, exc_type, exc, tb):
        self.process.stdin.close()
        if exc_type is not None:
            self.process.terminate()
        try:
            code = self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
            raise RuntimeError("Git blob reader did not stop")
        if code and exc_type is None:
            raise RuntimeError("Git blob reader failed")
        self.process.stdout.close()
        self.process.stderr.close()

    def sha(self, row):
        oid = row["git_blob"]
        if oid in self.cache:
            return self.cache[oid]
        self.process.stdin.write((oid + "\n").encode())
        self.process.stdin.flush()
        header = self.process.stdout.readline().split()
        if len(header) != 3 or header[0].decode() != oid or header[1] != b"blob" or int(header[2]) != row["bytes"]:
            raise ValueError("Git blob identity/size mismatch")
        left, digest = row["bytes"], hashlib.sha256()
        while left:
            block = self.process.stdout.read(min(left, 1024 * 1024))
            if not block:
                raise ValueError("Truncated Git blob")
            digest.update(block)
            left -= len(block)
        if self.process.stdout.read(1) != b"\n":
            raise ValueError("Invalid Git batch delimiter")
        self.cache[oid] = digest.hexdigest()
        return self.cache[oid]


def groups_at(files, payloads):
    groups = {"world:" + path: WORLDS + "/" + path for path in payloads}
    for path in files:
        if not path.startswith(BENCHMARK + "/"):
            continue
        parts = path[len(BENCHMARK) + 1:].split("/")
        if "reports" in parts[:-1]:
            i = parts.index("reports")
            groups["reports:" + ("/".join(parts[:i]) or "root")] = BENCHMARK + "/" + "/".join(parts[:i+1])
    return groups


def frozen_references(repo, commit, files):
    references = {}
    for name in files:
        if name.endswith(".json") and "frozen" in Path(name).name:
            document = json.loads(subprocess.check_output(["git", "-C", str(repo), "show", commit + ":" + name]))
            references[name] = list(document.get("source_sha256", {}))
    return references


def tree_identity(repo, commit, root):
    result = subprocess.run(["git", "-C", str(repo), "rev-parse", "--verify", commit + ":" + root], capture_output=True, text=True)
    if result.returncode:
        return None
    oid = result.stdout.strip()
    kind = subprocess.check_output(["git", "-C", str(repo), "cat-file", "-t", oid], text=True).strip()
    if kind != "tree":
        raise ValueError("Payload root is not a Git directory: " + root)
    return oid


def consumer_metadata(identifier, root, selected, files, references):
    gates = [BENCHMARK + "/" + name for name in GATE_FILES if (BENCHMARK + "/" + name) in selected]
    source_refs = {name: sum(path in selected for path in paths) for name, paths in references.items() if any(path in selected for path in paths)}
    consumers = []
    if identifier.startswith("world:"):
        demo = WORLDS + "/school_arena/demo_1/"
        names = (["configs/dataset_v1.yaml", "produce_dataset_v1.py", "dataset_production/plan.py"] if root.endswith("/dataset_v1.0")
                 else ["scenario_generator/launch_episode.sh", "scenario_generator/install_episode.sh", "scenario_generator/scenario_coverage.py"])
        consumers = [demo + name for name in names if demo + name in files]
    return dict(current_gate_files=gates, known_source_consumers=consumers, historical_freeze_references=source_refs)


def create_catalog(repo, store, ref="HEAD"):
    repo, store = Path(repo).resolve(), Path(store).resolve()
    commit = revision(repo, ref)
    files = tree_files(repo, commit)
    payloads, policy_sha = policy_at(repo, commit)
    groups = groups_at(files, payloads)
    references = frozen_references(repo, commit, files)
    object_dir = store / "catalogs/objects"
    object_dir.mkdir(parents=True, exist_ok=True)
    object_dir.resolve().relative_to(store)
    summaries = []
    with GitBlobHashes(repo) as hashes:
        for identifier, root in sorted(groups.items()):
            selected = {name: row for name, row in files.items() if name.startswith(root + "/")}
            entries = {name[len(root)+1:]: dict(**row, sha256=hashes.sha(row)) for name, row in sorted(selected.items())}
            inventory = dict(schema="dynamuhsim_asset_inventory/1", revision=commit, root=root,
                             git_tree=tree_identity(repo, commit, root), entries=entries)
            encoded = json.dumps(inventory, sort_keys=True, indent=2).encode() + b"\n"
            digest = hashlib.sha256(encoded).hexdigest()
            compressed = gzip.compress(encoded, mtime=0)
            _publish(object_dir / (digest + ".json.gz"), compressed)
            consumers = consumer_metadata(identifier, root, selected, files, references)
            summaries.append(dict(id=identifier, root=root, git_tree=inventory["git_tree"], files=len(entries), logical_bytes=sum(row["bytes"] for row in entries.values()),
                                  inventory_sha256=digest, inventory_gzip_sha256=hashlib.sha256(compressed).hexdigest(),
                                  inventory_object="catalogs/objects/" + digest + ".json.gz", **consumers,
                                  state="inventory_only", payload_externalized=False, removal_allowed=False,
                                  blockers=["Payload archive and verified restore are not yet provided", "Retain runtime gates and exact historical source replay paths"]))
        unique_blobs = len(hashes.cache)
    return dict(schema="dynamuhsim_asset_catalog/1", revision=commit, world_policy_sha256=policy_sha, entries=summaries,
                files=sum(row["files"] for row in summaries), logical_bytes=sum(row["logical_bytes"] for row in summaries),
                unique_payload_blobs=unique_blobs, consumer_scope="Known current gates, explicit source-relative workflows and frozen JSON source maps; not a complete dynamic dependency graph")


def verify_catalog(repo, store, catalog):
    if catalog.get("schema") != "dynamuhsim_asset_catalog/1":
        raise ValueError("Unsupported catalog schema")
    commit = revision(repo, catalog["revision"])
    if commit != catalog["revision"]:
        raise ValueError("Catalog must pin a full commit")
    files = tree_files(repo, commit)
    payloads, policy_sha = policy_at(repo, commit)
    expected_groups = groups_at(files, payloads)
    if catalog["world_policy_sha256"] != policy_sha or {row["id"]: row["root"] for row in catalog["entries"]} != expected_groups or len(catalog["entries"]) != len(expected_groups):
        raise ValueError("Catalog policy/group coverage changed")
    references = frozen_references(repo, commit, files)
    seen, count, total = set(), 0, 0
    with GitBlobHashes(repo) as hashes:
        for summary in catalog["entries"]:
            if summary["state"] != "inventory_only" or summary["payload_externalized"] is not False or summary["removal_allowed"] is not False:
                raise ValueError("An inventory is not authorization to remove payloads")
            relative_name(summary["inventory_object"])
            if summary["inventory_object"] != "catalogs/objects/" + summary["inventory_sha256"] + ".json.gz":
                raise ValueError("Inventory object is not content-addressed")
            object_path = (Path(store).resolve() / summary["inventory_object"]).resolve()
            object_path.relative_to(Path(store).resolve())
            data = object_path.read_bytes()
            if hashlib.sha256(data).hexdigest() != summary["inventory_gzip_sha256"]:
                raise ValueError("Compressed inventory hash mismatch")
            raw = gzip.decompress(data)
            if hashlib.sha256(raw).hexdigest() != summary["inventory_sha256"]:
                raise ValueError("Inventory hash mismatch")
            inventory = json.loads(raw)
            if inventory["schema"] != "dynamuhsim_asset_inventory/1" or inventory["revision"] != commit or inventory["root"] != summary["root"] or inventory["git_tree"] != summary["git_tree"]:
                raise ValueError("Inventory identity mismatch")
            selected = {name: row for name, row in files.items() if name.startswith(summary["root"] + "/")}
            consumers = consumer_metadata(summary["id"], summary["root"], selected, files, references)
            if summary["git_tree"] != tree_identity(repo, commit, summary["root"]) or any(summary[key] != value for key, value in consumers.items()):
                raise ValueError("Catalog tree/consumer identity mismatch")
            expected = {name[len(summary["root"])+1:]: row for name, row in selected.items()}
            if set(inventory["entries"]) != set(expected):
                raise ValueError("Inventory file coverage mismatch")
            for name, row in inventory["entries"].items():
                relative_name(name)
                absolute_name = summary["root"] + "/" + name
                wanted = expected[name]
                if absolute_name in seen or row != dict(**wanted, sha256=hashes.sha(wanted)):
                    raise ValueError("Inventory Git blob/SHA256 mismatch")
                seen.add(absolute_name)
            amount = sum(row["bytes"] for row in expected.values())
            if summary["files"] != len(expected) or summary["logical_bytes"] != amount:
                raise ValueError("Catalog statistics mismatch")
            count += len(expected)
            total += amount
    if count != catalog["files"] or total != catalog["logical_bytes"]:
        raise ValueError("Catalog total mismatch")
    return dict(status="PASS", revision=commit, groups=len(expected_groups), files=count, logical_bytes=total, payloads_moved=False)


def verify_install(repo, share, *, legacy=False, ref="HEAD", history_index=None):
    """Verify committed package assets in a fresh copy or symlink install prefix."""
    repo, share = Path(repo).resolve(), Path(share).resolve()
    commit = revision(repo, ref)
    files = tree_files(repo, commit, PACKAGE)
    payloads, _ = policy_at(repo, commit)
    expected = {}
    for name, row in files.items():
        relative = name[len(PACKAGE)+1:]
        if relative.split("/", 1)[0] not in ASSET_DIRS:
            continue
        if not legacy and any(relative == "worlds/" + payload or relative.startswith("worlds/" + payload + "/") for payload in payloads):
            continue
        expected[relative] = row
    inventory_available = True
    if legacy:
        missing_roots = [payload for payload in payloads if not any(name.startswith(WORLDS + "/" + payload + "/") for name in files)]
        if missing_roots and history_index is None:
            inventory_available = False
        elif missing_roots:
            # Import lazily: the history module itself uses the Git catalog API.
            from .hydrate import _manifest
            for payload in missing_roots:
                manifest = _manifest(repo, None, history_index, "world:" + payload, "git")
                if manifest["group"]["root"] != WORLDS + "/" + payload:
                    raise ValueError("Historical install inventory root mismatch")
                expected.update({"worlds/" + payload + "/" + name: row for name, row in manifest["inventory"]["entries"].items()})
    missing, different = [], []
    with GitBlobHashes(repo) as hashes:
        for name, row in expected.items():
            path = share / name
            if not path.is_file(): missing.append(name)
            elif path.stat().st_size != row["bytes"] or sha256_file(path) != (row.get("sha256") or hashes.sha(row)): different.append(name)
    world_files = {name: row for name, row in expected.items() if name.startswith("worlds/")}
    present_payloads = [payload for payload in payloads if (share / "worlds" / payload).exists()]
    checks = dict(historical_payload_inventory_available=inventory_available, all_expected_assets_present=not missing, all_expected_assets_byte_equal=not different,
                  excluded_payloads_absent=legacy or not present_payloads)
    return dict(status="PASS" if all(checks.values()) else "FAIL", revision=commit, legacy=legacy, checks=checks,
                expected_assets=len(expected), expected_asset_bytes=sum(row["bytes"] for row in expected.values()), expected_world_files=len(world_files), expected_world_bytes=sum(row["bytes"] for row in world_files.values()),
                missing=missing, different=different, installed_payload_directories=present_payloads,
                note="Caches/untracked files outside excluded directories are not part of committed-asset comparison; old prefixes are not cleaned automatically.")
