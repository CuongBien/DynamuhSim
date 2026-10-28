"""Strict evidence I/O and hash-checked source resolution; snapshots are never executed."""
import gzip
import builtins
import importlib.util
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
from dynamuhsim.workspace import source_path


def _reject_constant(value):
    raise ValueError("Non-finite JSON: " + value)


def relative_name(name):
    path = PurePosixPath(name)
    if not name or not path.parts or "\x00" in name or path.is_absolute() or ".." in path.parts or "\\" in name or ":" in name or str(path) != name:
        raise ValueError("Expected a normalized relative evidence path: " + name)
    return path


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path):
    return json.loads(Path(path).read_text(), parse_constant=_reject_constant)


def dump_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def jsonl_rows(path):
    path = Path(path)
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                yield json.loads(line, parse_constant=_reject_constant)
            except (ValueError, json.JSONDecodeError) as error:
                raise ValueError(f"{path}:{number}: {error}") from error


def resolve_sources(expected, roots=(), *, git_repo=None, commit=None):
    """Return exact bytes from explicit roots or a recorded Git commit, never HEAD fallback."""
    resolved, missing = {}, []
    if commit is not None and not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError("Expected recorded full Git commit SHA")
    for name, digest in expected.items():
        relative_name(name)
        if not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError("Invalid source SHA256: " + name)
        found = None
        for root in roots:
            path = source_path(root, name)
            if path.is_file() and sha256_file(path) == digest:
                found = dict(data=path.read_bytes(), mode=path.stat().st_mode & 0o777, origin="file")
                break
        if found is None and git_repo is not None and commit is not None:
            result = subprocess.run(["git", "-C", str(git_repo), "show", commit + ":" + name], capture_output=True)
            if result.returncode == 0 and hashlib.sha256(result.stdout).hexdigest() == digest:
                mode = subprocess.check_output(["git", "-C", str(git_repo), "ls-tree", commit, "--", name], text=True).split()[0]
                found = dict(data=result.stdout, mode=0o755 if mode == "100755" else 0o644, origin="recorded_git")
        if found is None:
            missing.append(name)
        else:
            # The file may have changed between hashing and reading.
            if hashlib.sha256(found["data"]).hexdigest() != digest:
                raise RuntimeError("Source changed while resolving: " + name)
            resolved[name] = found
    return resolved, missing


def module_from_path(path, name, bindings=None):
    """Load current code by explicit path, with module-local legacy import bindings."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    if bindings:
        def imported(name, *args, **kwargs):
            return bindings[name] if name in bindings else builtins.__import__(name, *args, **kwargs)
        module.__dict__['__builtins__'] = dict(vars(builtins), __import__=imported)
    spec.loader.exec_module(module)
    return module


def reserve_audit_directory(output, dataset, raw_root, repository, stage, source_roots=()):
    """Reserve a fresh output outside datasets, raw captures and archived history."""
    output, repository = Path(output).resolve(), Path(repository).resolve()
    forbidden = [Path(dataset).resolve(), Path(raw_root).resolve()]
    for root in {repository, *(Path(root).resolve() for root in source_roots)}:
        forbidden.extend([(root/'experiments/archive').resolve(), (root/stage/'reports').resolve(), source_path(root, str(Path(stage)/'reports')).resolve()])
    if any(output == p or output.is_relative_to(p) or p.is_relative_to(output) for p in forbidden):
        raise ValueError('Audit output must be outside datasets, raw captures and archived evidence')
    output.mkdir(parents=True, exist_ok=False)
    return output


def verify_sources(expected, *, repository, roots=(), revision):
    """Require exact recorded bytes; this proves provenance, not executable code parity."""
    found, missing = resolve_sources(expected, roots=[repository,*roots], git_repo=repository, commit=revision)
    if missing: raise ValueError('Historical source unavailable: '+', '.join(missing))
    return {name:value['origin'] for name,value in found.items()}
