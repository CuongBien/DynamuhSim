"""Hash-pinned controls, independent of reports and mutable evaluation folders."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
from dynamuhsim.workspace import source_path

from dynamuhsim.analysis.evidence import relative_name, sha256_file, load_json

BENCHMARK = "experiments/social_navigation/benchmark/"
CLASSIFIER = BENCHMARK + "depth_person_classifier/"
ORIGINS = {
    "classifier/model.npz": CLASSIFIER + "reports/selected_model.npz",
    "classifier/metadata.json": CLASSIFIER + "reports/selected_frozen.json",
    "classifier/source_freeze.json": CLASSIFIER + "frozen.json",
    "classifier/evaluation_receipt.json": CLASSIFIER + "reports/test_opened.json",
    "engines/identity.json": BENCHMARK + "depth_identity_guard/reports/final_engine/frozen.json",
    "engines/refinement.json": BENCHMARK + "depth_track_refinement/reports/frozen02.json",
}


def _hash(data): return hashlib.sha256(data).hexdigest()


def _digest(value):
    if not isinstance(value, str) or not re.fullmatch("[0-9a-f]{64}", value): raise ValueError("Expected SHA256")
    return value


def export_bundle(repo, ref, output):
    repo, output = Path(repo).resolve(), Path(output)
    if output.exists() or output.is_symlink(): raise FileExistsError("Preserve existing control bundle")
    commit = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "--verify", "--end-of-options", ref + "^{commit}"], text=True).strip()
    def blob(name):
        relative_name(name)
        return subprocess.check_output(["git", "-C", str(repo), "show", commit + ":" + name])
    contents = {name: blob(origin) for name, origin in ORIGINS.items()}
    selected = json.loads(contents["classifier/metadata.json"])
    if (selected["schema"] != "depth_person_selected_model/1" or selected["candidate"] != "depth_linear"
            or selected["hidden"] != 0 or selected["modality"] != "depth"):
        raise ValueError("Only the frozen depth-linear pilot is supported")
    if _hash(contents["classifier/model.npz"]) != selected["model_sha256"] or _hash(contents["classifier/source_freeze.json"]) != selected["source_freeze_sha256"]:
        raise ValueError("Historical classifier binding mismatch")
    receipt = json.loads(contents["classifier/evaluation_receipt.json"])
    if receipt["selected_frozen_sha256"] != _hash(contents["classifier/metadata.json"]): raise ValueError("Historical test receipt mismatch")
    provenance = {}
    for name, key in [("training.json", "training_sha256"), ("selected_model.pt", "torch_weights_sha256")]:
        origin = CLASSIFIER + "reports/" + name
        data = blob(origin)
        if _hash(data) != selected[key]: raise ValueError("Historical training/weights binding mismatch")
        provenance[origin] = dict(sha256=_hash(data), bytes=len(data))
    engines = {}
    for name, stage, schema in [("identity", "depth_identity_guard", "identity_guard_frozen/1"), ("refinement", "depth_track_refinement", "frozen_refinement/1")]:
        frozen = json.loads(contents["engines/" + name + ".json"])
        if frozen["schema"] != schema: raise ValueError("Unknown frozen engine")
        root = BENCHMARK + stage
        for path, digest in frozen["engine_sha256"].items():
            if _hash(blob(root + "/" + str(relative_name(path)))) != digest: raise ValueError("Historical engine source mismatch")
        engines[name] = dict(metadata="engines/" + name + ".json", source_root=root, schema=schema)
    manifest = dict(schema="dynamuhsim_sensor_control_bundle/1", source_revision=commit,
                    files={name: dict(bytes=len(data), sha256=_hash(data), original_path=ORIGINS[name]) for name, data in contents.items()},
                    classifier=dict(model="classifier/model.npz", metadata="classifier/metadata.json", source_freeze="classifier/source_freeze.json", evaluation_receipt="classifier/evaluation_receipt.json"),
                    engines=engines, provenance_only=provenance,
                    evaluation_state="Historical receipt only; no train/test operations. Legacy live test seal stays at its original path.")
    output.mkdir(parents=True, exist_ok=False)
    for name, data in contents.items():
        path = output / name;path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream: stream.write(data)
    raw = json.dumps(manifest, sort_keys=True, indent=2).encode() + b"\n"
    with (output / "manifest.json").open("xb") as stream: stream.write(raw)
    ControlBundle(output, _hash(raw))
    return dict(manifest_sha256=_hash(raw), source_revision=commit, files=len(contents), payload_bytes=sum(map(len, contents.values())), manifest_bytes=len(raw))


class ControlBundle:
    def __init__(self, root, expected_manifest_sha256):
        self.root = Path(root).resolve(strict=True)
        manifest_path = self._path("manifest.json")
        raw = manifest_path.read_bytes()
        if _hash(raw) != _digest(expected_manifest_sha256): raise ValueError("Control manifest hash mismatch")
        self.manifest = json.loads(raw)
        if self.manifest.get("schema") != "dynamuhsim_sensor_control_bundle/1": raise ValueError("Unsupported control bundle")
        for name, row in self.manifest["files"].items():
            path = self._path(name)
            if type(row["bytes"]) is not int or row["bytes"] < 0 or path.stat().st_size != row["bytes"] or sha256_file(path) != _digest(row["sha256"]):
                raise ValueError("Control file hash/size mismatch: " + name)
        self.manifest_sha256 = expected_manifest_sha256

    def _path(self, name):
        path = self.root / str(relative_name(name))
        if path.is_symlink(): raise ValueError("Control file cannot be a symlink")
        path.resolve(strict=True).relative_to(self.root)
        return path

    def read(self, name):
        if name not in self.manifest["files"]: raise ValueError("Unlisted control file")
        data = self._path(name).read_bytes()
        row = self.manifest["files"][name]
        if len(data) != row["bytes"] or _hash(data) != row["sha256"]: raise ValueError("Control changed after verification")
        return data

    def document(self, name): return json.loads(self.read(name))

    def verify_engine(self, name, repo):
        engine = self.manifest["engines"][name]
        frozen = self.document(engine["metadata"])
        if frozen["schema"] != engine["schema"]: raise ValueError("Engine schema mismatch")
        repo = Path(repo).resolve(strict=True)
        root = source_path(repo, str(relative_name(engine["source_root"])))
        for path, digest in frozen["engine_sha256"].items():
            source = root / str(relative_name(path))
            if source.is_symlink(): raise ValueError("Engine source cannot be a symlink")
            source.resolve(strict=True).relative_to(repo)
            if sha256_file(source) != _digest(digest): raise ValueError("Frozen engine changed: " + path)
        return frozen


def registered_bundle(repo, registry=None, name="sensor_pilot_v1"):
    repo = Path(repo).resolve(strict=True)
    registry = load_json(registry or repo / "configs/control_registry.json")
    if registry.get("schema") != "dynamuhsim_control_registry/1": raise ValueError("Unknown control registry")
    record = registry["bundles"][name]
    root = repo / str(relative_name(record["root"]))
    root.resolve(strict=True).relative_to(repo)
    return ControlBundle(root, record["manifest_sha256"])
