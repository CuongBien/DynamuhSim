#!/usr/bin/env python3
"""Additive handoff checks; reuses approved tools and never starts a simulator."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/handoff"


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def inside(root, name):
    relative = PurePosixPath(name)
    if relative.is_absolute() or ".." in relative.parts or "\\" in name or not relative.parts:
        raise ValueError("Unsafe relative artifact path: " + name)
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError("Artifact path escapes root: " + name)
    return path


def verify_baseline(root=ROOT):
    manifest = read(root / "configs/handoff/baseline.json")
    failures = []
    for name, expected in manifest["protected_sha256"].items():
        path = inside(root, name)
        if not path.is_file():
            failures.append(dict(path=name, reason="MISSING"))
        elif sha(path) != expected:
            failures.append(dict(path=name, reason="HASH_MISMATCH"))
    return dict(status="FAIL" if failures else "PASS", reference_commit=manifest["reference_commit"],
                protected_files=len(manifest["protected_sha256"]), failures=failures,
                scope="Protected accepted source/evidence bytes only; not a new simulator run")


def verify_dataset(dataset, pilot):
    dataset = Path(dataset).resolve()
    inventory_path = dataset / "bundle_inventory.json"
    if sha(inventory_path) != pilot["bundle_inventory_sha256"]:
        raise ValueError("Inventory differs from accepted pilot")
    inventory = read(inventory_path)
    for name, expected in inventory.items():
        path = inside(dataset, name)
        if (dataset / name).is_symlink() or not path.is_file() or path.stat().st_size != expected["bytes"] or sha(path) != expected["sha256"]:
            raise ValueError("Pilot member mismatch: " + name)
    allowed = set(inventory) | {"bundle_inventory.json"}
    for path in dataset.rglob("*"):
        if path.is_symlink():
            raise ValueError("Portable pilot cannot contain symlinks")
        name = path.relative_to(dataset).as_posix()
        if path.is_file() and "__pycache__" not in path.relative_to(dataset).parts and name not in allowed:
            raise ValueError("Unrecorded pilot member: " + name)
    return dict(status="PASS", files=len(inventory) + 1, inventory_sha256=sha(inventory_path))


def restore_archive(archive, work_dir, pilot):
    archive, work_dir = Path(archive).resolve(), Path(work_dir).resolve()
    if work_dir.exists():
        raise FileExistsError("Restore requires a NEW work directory: " + str(work_dir))
    if archive.stat().st_size != pilot["archive_bytes"] or sha(archive) != pilot["archive_sha256"]:
        raise ValueError("Archive differs from approved pilot; no extraction performed")
    with zipfile.ZipFile(archive) as source:
        for entry in source.infolist():
            name = PurePosixPath(entry.filename)
            inside(work_dir, entry.filename)
            if name.parts[0] != pilot["top_directory"] or stat.S_ISLNK(entry.external_attr >> 16):
                raise ValueError("Unexpected archive root or symlink")
        if source.testzip() is not None:
            raise ValueError("ZIP CRC failed")
        work_dir.mkdir(parents=True, exist_ok=False)
        source.extractall(work_dir)
    return work_dir / pilot["top_directory"]


def smoke_data(dataset, root=ROOT):
    pilot = read(root / "configs/handoff/baseline.json")["portable_pilot"]
    integrity = verify_dataset(dataset, pilot)
    # A fresh cache prefix prevents stale bundle bytecode from replacing verified source.
    # The existing approved inspector/loader runs unchanged in isolated Python.
    with tempfile.TemporaryDirectory(prefix="dynamuhsim_handoff_") as temporary:
        command = [sys.executable, "-I", "-B", "-X", "pycache_prefix=" + str(Path(temporary)/"cache"),
                   str(Path(dataset).resolve()/"tools/inspect_pilot.py"), "--dataset", str(Path(dataset).resolve())]
        result = subprocess.run(command, cwd=temporary, capture_output=True, text=True, timeout=60)
    if result.returncode:
        raise RuntimeError("Approved inspector failed: " + result.stderr[-2000:])
    inspected = json.loads(result.stdout)
    expected = inspected.get("status") == "PASS" and inspected.get("ROS_imported") is False and len(inspected.get("episodes", [])) == pilot["episodes"] and sum(e["windows"] for e in inspected["episodes"]) == pilot["windows"]
    return dict(status="PASS" if expected else "FAIL", dataset=str(Path(dataset).resolve()),
                integrity=integrity, inspector=inspected, new_simulator_runs=0, training_ready=False)


def probe_modules(python, modules):
    program = """import importlib,importlib.metadata,json,sys
out=[]
for name in json.loads(sys.argv[1]):
 try:
  m=importlib.import_module(name);out.append(dict(module=name,status='PASS',version=getattr(m,'__version__',None)))
 except Exception as e:out.append(dict(module=name,status='FAIL',error=type(e).__name__+': '+str(e)))
print('HANDOFF_PROBE_JSON:'+json.dumps(out))
"""
    environment = os.environ.copy()
    environment.update(OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
    result = subprocess.run([str(python), "-c", program, json.dumps(modules)], env=environment,
                            capture_output=True, text=True, timeout=45)
    marker = "HANDOFF_PROBE_JSON:"
    payload = next((line[len(marker):] for line in reversed(result.stdout.splitlines()) if line.startswith(marker)), None)
    if result.returncode or payload is None:
        return [dict(module="probe_process", status="FAIL", error=result.stderr[-2000:] or "Probe produced no JSON")]
    return json.loads(payload)


def doctor(scope, dataset=None, root=ROOT):
    config = read(root / "configs/handoff/dependencies.json")
    profile = config["profiles"][scope]
    checks = [dict(name="python>=3.10", status="PASS" if sys.version_info >= (3,10) else "FAIL", value=sys.version)]
    checks += [dict(name=v.pop("module"), **v) for v in probe_modules(sys.executable, profile["modules"])]
    if scope != "data":
        checks.append(dict(name="ROS_DISTRO=jazzy", status="PASS" if os.environ.get("ROS_DISTRO") == "jazzy" else "FAIL", value=os.environ.get("ROS_DISTRO"), remedy="source /opt/ros/jazzy/setup.bash"))
    for command in profile["tools"]:
        found = shutil.which(command)
        checks.append(dict(name=command, status="PASS" if found else "FAIL", path=found))
    for name in profile["paths"]:
        path = Path(name) if Path(name).is_absolute() else root/name
        checks.append(dict(name=name, status="PASS" if path.is_file() else "FAIL"))
    gpu = None
    if scope == "gazebo":
        python = root/config["socnav_venv"]
        if python.is_file():
            checks += [dict(name="socnav_venv:"+v.pop("module"), **v) for v in probe_modules(python, config["socnav_modules"])]
        else:
            checks.append(dict(name="socnav_venv", status="FAIL", remedy="See HANDOFF_SETUP.md; setup_socnavgym.sh manages its own venv"))
        executable = shutil.which("nvidia-smi") or config["reference_driver_query"]
        if Path(executable).is_file():
            value = subprocess.run([executable, "--query-gpu=name,driver_version", "--format=csv,noheader"], capture_output=True, text=True, timeout=10)
            gpu = dict(status="PASS" if value.returncode == 0 else "FAIL", device=value.stdout.strip(), error=value.stderr.strip())
        else:
            gpu = dict(status="FAIL", error="NVIDIA device query unavailable on current reference profile")
        checks.append(dict(name="NVIDIA_device_query", **gpu))
    if dataset is not None:
        try:
            check = verify_dataset(dataset, read(root/"configs/handoff/baseline.json")["portable_pilot"])
        except (OSError, ValueError) as error:
            check = dict(status="FAIL", error=str(error))
        checks.append(dict(name="portable_dataset", **check))
    return dict(status="PASS" if all(c["status"] == "PASS" for c in checks) else "FAIL", scope=scope,
                checks=checks, validated_reference_python=config["validated_python"],
                actual_rendering_test="NOT_RUN", new_simulator_runs=0,
                meaning=profile["scope"], auto_install=False)


def reference_commands(mode, run_dir, seed=46, root=ROOT):
    run_dir = Path(run_dir).resolve()
    allowed = (root/"experiments/social_navigation/benchmark/generated/handoff_reference").resolve()
    if not run_dir.is_relative_to(allowed) or run_dir == allowed or run_dir.exists():
        raise ValueError("Use a NEW run under " + str(allowed))
    recipe = read(root/"docs/demos/2026-10-29/transport_order/collection_profile.json")
    selected = recipe["modes"][mode]
    config = root/recipe["capture_config"]
    if sha(config) != recipe["capture_config_sha256"]:
        raise ValueError("Accepted capture config changed")
    if seed not in read(config)["allowed_seeds"]:
        raise ValueError("Seed not allowed by frozen capture config")
    prepare = [sys.executable, str(root/"tools/capture_episode.py"), "prepare", "--run-dir", str(run_dir),
               "--config", str(config), "--scenario", selected["scenario"], "--seed", str(seed), "--planner-diagnostics",
               "--human-motion-profile", selected["human_motion_profile"], "--human-goal-clock", selected["human_goal_clock"],
               "--human-lifecycle-profile", selected["human_lifecycle_profile"]]
    if "encounter_profile" in selected:
        prepare += ["--encounter-profile", selected["encounter_profile"]]
    commands = [prepare,
                [sys.executable, str(root/"tools/capture_episode.py"), "verify", "--run-dir", str(run_dir)],
                [sys.executable, str(root/"tools/capture_episode.py"), "run", "--run-dir", str(run_dir)],
                [sys.executable, str(root/"tools/audit_episode.py"), "--run-dir", str(run_dir), "--source-root", str(root),
                 "--output", str(run_dir.with_name(run_dir.name+"_audit.json"))]]
    return dict(status="COMMANDS_ONLY", mode=mode, commands=commands, bash=[shlex.join(c) for c in commands],
                simulator_started=False, note="Commands are printed, never executed; source ROS/workspace explicitly before use")


def smoke_contract(root=ROOT):
    names = ["test_dataset_normalization", "test_goal_clock", "test_persistent_conversation", "test_worker_readiness",
             "test_transport_shutdown", "test_handoff_pilot", "test_portable_reader"]
    environment = os.environ.copy()
    environment.update(PYTHONPATH=os.pathsep.join([str(root/"python"), str(root/"tests")]),
                       OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
    result = subprocess.run([sys.executable, "-m", "unittest", *names], cwd=root, env=environment,
                            capture_output=True, text=True, timeout=90)
    return dict(status="PASS" if result.returncode == 0 else "FAIL", modules=names,
                output=(result.stdout+result.stderr)[-12000:], new_simulator_runs=0)


def save_report(report, path=None):
    if path is not None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x") as stream:
            json.dump(report, stream, indent=2)
            stream.write("\n")
    print(json.dumps(report, indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    baseline = sub.add_parser("baseline", help="Check accepted protected bytes")
    doc = sub.add_parser("doctor", help="Probe dependencies only; no installs/simulation")
    doc.add_argument("--scope", choices=["data", "export", "gazebo"], default="data")
    doc.add_argument("--dataset", type=Path)
    data = sub.add_parser("smoke-data", help="Verify known portable pilot and call unchanged inspector")
    source = data.add_mutually_exclusive_group(required=True)
    source.add_argument("--dataset", type=Path)
    source.add_argument("--archive", type=Path)
    data.add_argument("--work-dir", type=Path)
    contract = sub.add_parser("smoke-contract", help="Run existing small unit contract suite; no simulation")
    reference = sub.add_parser("reference-command", help="Print explicit accepted Gazebo commands only")
    reference.add_argument("--mode", choices=["conversation", "natural_group_walking"], required=True)
    reference.add_argument("--run-dir", type=Path, required=True)
    reference.add_argument("--seed", type=int, default=46)
    for p in [baseline, doc, data, contract, reference]:
        p.add_argument("--output", type=Path, help="Optional NEW JSON receipt, never overwritten")
    args = parser.parse_args(argv)
    try:
        if args.output and args.output.exists():
            raise FileExistsError("Receipt already exists: " + str(args.output))
        if args.command == "baseline":report = verify_baseline()
        elif args.command == "doctor":report = doctor(args.scope, args.dataset)
        elif args.command == "smoke-contract":report = smoke_contract()
        elif args.command == "reference-command":report = reference_commands(args.mode, args.run_dir, args.seed)
        else:
            if args.archive:
                if args.work_dir is None:raise ValueError("--archive requires a NEW --work-dir")
                dataset = restore_archive(args.archive, args.work_dir, read(CONFIG/"baseline.json")["portable_pilot"])
            else:
                if args.work_dir is not None:raise ValueError("--work-dir is only used with --archive")
                dataset = args.dataset
            report = smoke_data(dataset)
        save_report(report, args.output)
        return 0 if report["status"] in ["PASS", "COMMANDS_ONLY"] else 1
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, zipfile.BadZipFile) as error:
        report = dict(status="ERROR", error=type(error).__name__+": "+str(error), command=args.command)
        # Do not overwrite an existing requested receipt, even on error.
        if args.output is None or not args.output.exists():save_report(report, args.output)
        else:print(json.dumps(report, indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
