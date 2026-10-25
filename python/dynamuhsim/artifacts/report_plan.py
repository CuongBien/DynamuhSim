"""Conservative report inventory and static path dependencies; never execute sources."""
import ast
from collections import defaultdict
import json
from pathlib import Path, PurePosixPath
import posixpath
import subprocess

from .catalog import BENCHMARK, revision, tree_files, tree_identity
from .history import history_groups, _sha

MEDIA = {".gif", ".png", ".jpg", ".jpeg", ".mp4", ".webm"}
SNAPSHOT = {".py", ".cc", ".sh", ".diff", ".patch"}
# These pipelines retain report files until their dynamic engine/capture imports
# and ignored raw inputs can be verified separately. This is a retention policy,
# not an inference that every report in these families is a live runtime input.
PRESERVE_STAGES = {"camera_height_ab", "dataset_pilot", "early_avoidance_pilot",
                   "lidar_person_pilot", "perception_target_pilot", "portable_episode"}
PIPELINES = {"dataset_pilot", "depth_proxy_lifecycle", "early_avoidance_pilot", "wm_episode_batch"}


def dependency_closure(groups, selected):
    if not set(selected).issubset(groups): raise ValueError("Unknown report group")
    found = set(selected)
    while True:
        expanded = found | {target for name in found for target in groups[name]["dependencies"]}
        if not expanded.issubset(groups): raise ValueError("Unknown report dependency")
        if expanded == found: return sorted(found)
        found = expanded


def _value(node, environment):
    """Evaluate only constants and lexical Path operations, never Python calls."""
    try:
        if isinstance(node, ast.Constant): return node.value
        if isinstance(node, ast.Name): return environment.get(node.id)
        if isinstance(node, ast.Attribute):
            value = _value(node.value, environment)
            if isinstance(value, PurePosixPath):
                if node.attr == "parent": return value.parent
                if node.attr == "parents": return list(value.parents)
        if isinstance(node, ast.Subscript): return _value(node.value, environment)[_value(node.slice, environment)]
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
            left, right = _value(node.left, environment), _value(node.right, environment)
            if isinstance(left, PurePosixPath) and isinstance(right, str):
                return PurePosixPath(posixpath.normpath(str(left / right)))
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name) and node.func.id in ["Path", "str"] and len(node.args) == 1:
                value = _value(node.args[0], environment)
                if node.func.id == "Path" and isinstance(value, (str, PurePosixPath)): return PurePosixPath(value)
                if node.func.id == "str" and value is not None: return str(value)
            if isinstance(node.func, ast.Attribute) and node.func.attr in ["resolve", "absolute"] and not node.args:
                return _value(node.func.value, environment)
    except (ValueError, TypeError, IndexError, KeyError):
        pass
    return None


def python_references(text, name, roots):
    tree = ast.parse(text)
    environment = {"__file__": "/CHECKOUT/" + name}
    references = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            value = _value(node.value, environment)
            for target in node.targets:
                if isinstance(target, ast.Name) and value is not None: environment[target.id] = value
        value = _value(node, environment)
        if not isinstance(value, PurePosixPath): continue
        value = str(value)
        # Also recognize archived absolute paths from the producer's checkout.
        for identifier, root in roots.items():
            if value.endswith("/" + root) or "/" + root + "/" in value:
                references.add((identifier, getattr(node, "lineno", 0)))
    return sorted(references)


def source_pins(value):
    if isinstance(value, dict):
        for key, nested in value.items():
            if key == "source_sha256" and isinstance(nested, dict):
                for name, digest in nested.items():
                    if isinstance(name, str) and isinstance(digest, str): yield name, digest
            yield from source_pins(nested)
    elif isinstance(value, list):
        for nested in value: yield from source_pins(nested)


def create_report_plan(repo, index, ref="HEAD"):
    repo = Path(repo).resolve()
    history_groups(index)
    commit = revision(repo, ref)
    files = tree_files(repo, commit)
    summaries = {row["id"]: row for row in index["catalog"]["entries"] if row["id"].startswith("reports:")}
    roots = {key: row["root"] for key, row in summaries.items()}
    bundles = {row["id"]: row for row in index["bundles"]}
    groups = {}
    for identifier, summary in summaries.items():
        selected = {name: row for name, row in files.items() if name.startswith(summary["root"] + "/")}
        if tree_identity(repo, commit, summary["root"]) != summary["git_tree"] or len(selected) != summary["files"]:
            raise ValueError("Report tree differs from archived baseline: " + identifier)
        categories = defaultdict(lambda: dict(files=0, logical_bytes=0))
        for name, row in selected.items():
            suffix = PurePosixPath(name).suffix.lower()
            category = "media" if suffix in MEDIA else "snapshot" if suffix in SNAPSHOT else "data_and_metadata"
            categories[category]["files"] += 1
            categories[category]["logical_bytes"] += row["bytes"]
        groups[identifier] = dict(id=identifier, root=summary["root"], files=summary["files"], logical_bytes=summary["logical_bytes"],
                                  object_id=bundles[identifier]["object_id"], categories=dict(categories), dependencies=[],
                                  retention_reasons=[], source_hash_references=[], static_references=[])
    edges = defaultdict(set)
    seed_files, parse_errors = [], []
    def owner(name):
        for identifier, root in roots.items():
            if name.startswith(root + "/"): return identifier
        if name.startswith(BENCHMARK + "/"):
            relative = name[len(BENCHMARK)+1:]
            return "reports:" + (relative.split("/", 1)[0] if "/" in relative else "root")
        return None
    scan = [name for name in files if
            (name.endswith(".py") and name.startswith((BENCHMARK + "/", "python/", "tools/", "configs/")))
            or (name.endswith(".json") and not name.startswith(("docs/", "tests/")) and "/vendor/" not in name)]
    # Git show reads committed blobs only. No source is imported or evaluated.
    for name in sorted(scan):
        data = subprocess.check_output(["git", "-C", str(repo), "show", commit + ":" + name])
        try: text = data.decode("utf-8")
        except UnicodeError:
            parse_errors.append(dict(file=name, error="not UTF-8")); continue
        if name.endswith(".py"):
            try: references = python_references(text, name, roots)
            except SyntaxError:
                parse_errors.append(dict(file=name, error="Python syntax")); continue
            for identifier, line in references:
                groups[identifier]["static_references"].append(dict(file=name, line=line))
                origin = owner(name)
                if origin in groups and origin != identifier: edges[origin].add(identifier)
            stage = name[len(BENCHMARK)+1:].split("/", 1)[0] if name.startswith(BENCHMARK + "/") else None
            if "/reports/" not in name and (name.startswith(("python/", "tools/")) or stage in PIPELINES):
                seed_files.extend(identifier for identifier, _ in references)
        else:
            try: document = json.loads(text)
            except ValueError:
                parse_errors.append(dict(file=name, error="JSON syntax")); continue
            for path, digest in source_pins(document):
                for identifier, root in roots.items():
                    if path.startswith(root + "/"):
                        groups[identifier]["source_hash_references"].append(dict(manifest=name, path=path, sha256=digest))
    if parse_errors:
        raise ValueError("Resolve source parsing errors before planning: " + json.dumps(parse_errors))
    for identifier, row in groups.items():
        row["dependencies"] = sorted(edges[identifier])
        stage = identifier.removeprefix("reports:")
        if summaries[identifier]["current_gate_files"]: row["retention_reasons"].append("current runtime gate/protocol")
        if row["source_hash_references"]: row["retention_reasons"].append("source_sha256 references into this report tree")
        if stage.startswith(("depth_", "wm_")) or stage in PRESERVE_STAGES:
            row["retention_reasons"].append("capture/perception/WM replay retained pending dynamic dependency verification")
        if identifier in seed_files: row["retention_reasons"].append("static reference from shared tooling or preserved capture pipeline")
    retained = dependency_closure(groups, [name for name, row in groups.items() if row["retention_reasons"]])
    for identifier, row in groups.items():
        if identifier in retained and not row["retention_reasons"]: row["retention_reasons"].append("dependency of a retained report group")
        row["action"] = "retain" if identifier in retained else "externalize_whole_group"
    return dict(schema="dynamuhsim_report_dependency_plan/1", revision=commit, history_index_sha256=_sha(index),
                groups=groups, source_files_scanned=len(scan),
                limits="Static lexical Python Path references and JSON source_sha256 only; no claim of a complete dynamic/import/shell dependency graph. Capture/perception/WM trees retained conservatively.")


def validate_report_plan(index, plan):
    history_groups(index)
    if plan.get("schema") != "dynamuhsim_report_dependency_plan/1" or plan["history_index_sha256"] != _sha(index):
        raise ValueError("Report plan/index identity mismatch")
    expected = {row["id"]: row for row in index["catalog"]["entries"] if row["id"].startswith("reports:")}
    bundles = {row["id"]: row for row in index["bundles"]}
    if set(plan["groups"]) != set(expected): raise ValueError("Report plan coverage mismatch")
    for identifier, group in plan["groups"].items():
        if (any(group[key] != expected[identifier][key] for key in ["id", "root", "files", "logical_bytes"])
                or group["object_id"] != bundles[identifier]["object_id"]
                or group["action"] not in ["retain", "externalize_whole_group"]):
            raise ValueError("Report plan group identity mismatch")
        if expected[identifier]["current_gate_files"] and group["action"] != "retain":
            raise ValueError("Current gates cannot be externalized")
        if not set(group["dependencies"]).issubset(expected): raise ValueError("Unknown report dependency")
    return plan["groups"]


def hydrate_reports(checkout, index, plan, selected, *, source="git", store=None):
    from .hydrate import hydrate_history
    groups = validate_report_plan(index, plan)
    closure = dependency_closure(groups, selected)
    receipts = [hydrate_history(checkout, index, name, source=source, store=store) for name in closure]
    return dict(schema="dynamuhsim_report_hydration/1", status="PASS", requested=sorted(set(selected)),
                dependency_closure=closure, source=source, groups=receipts,
                changed_groups=sum(row["changed"] for row in receipts),
                changed_files=sum(row["files"] for row in receipts if row["changed"]),
                changed_bytes=sum(row["logical_bytes"] for row in receipts if row["changed"]))
