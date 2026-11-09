"""Plan, validate, execute and report NarrowNav human navigation benchmarks."""

import argparse
import csv
import fcntl
import hashlib
import json
import platform
import subprocess
import sys
from dataclasses import asdict
from importlib.metadata import PackageNotFoundError, version
from importlib.util import find_spec
from math import isclose, isfinite
from pathlib import Path

from narrow_nav.core.types import Point2D
from narrow_nav.evaluation.qa import schedule
from narrow_nav.evaluation.runner import execute, write_json
from narrow_nav.evaluation.scenario_cards import _yaml, compile_plan, fingerprint

ROOT = Path(__file__).resolve().parents[2]
MODELS = {
    "sfm_jupedsim": "SocialForceModel",
    "avm_jupedsim": "AnticipationVelocityModel",
    "csm_jupedsim": "CollisionFreeSpeedModelV2",
}
TIMING = {
    "physics_dt_ns": 2_000_000,
    "robot_control_dt_ns": 50_000_000,
    "human_control_dt_ns": 100_000_000,
}


def _git(*args):
    return subprocess.check_output(
        ["git", "--no-optional-locks", *args], cwd=ROOT, text=True
    ).strip()


def _runtime():
    paths = [
        path
        for folder in ("configs", "src/narrow_nav", "scripts/benchmarks", "sim/mujoco")
        for path in (ROOT / folder).rglob("*")
        if path.suffix in (".py", ".yaml", ".xml") and path.is_file()
    ] + [ROOT / name for name in ("uv.lock", "pyproject.toml")]
    code = fingerprint(
        {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(paths)
        }
    )
    packages = {}
    for name in (
        "mujoco",
        "narrow-nav",
        "jupedsim",
        "pyrvo2",
        "numpy",
        "shapely",
        "hydra-core",
        "omegaconf",
    ):
        try:
            packages[name] = version(name)
        except PackageNotFoundError:
            packages[name] = None
    return {
        "git_commit": _git("rev-parse", "HEAD"),
        "git_dirty": bool(_git("status", "--porcelain")),
        "git_branch": _git("branch", "--show-current"),
        "code_sha256": code,
        "python": sys.executable,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "packages": packages,
        "rvo2_binary_sha256": (
            hashlib.sha256(Path(spec.origin).read_bytes()).hexdigest()
            if (spec := find_spec("rvo2")) and spec.origin
            else None
        ),
    }


def _backend(name, catalog):
    from narrow_nav.humans.registry import create_human_backend

    gate = catalog[name]
    if (
        gate["integration"] not in ("PASS", "MODEL_INTEGRATION_PASS")
        or not (ROOT / gate["evidence"]).is_file()
    ):
        raise ValueError(f"BLOCKED_CAPABILITY: no real integration evidence for {name}")
    backend = create_human_backend(name)
    if name in MODELS and (
        getattr(backend, "model_class", None) != MODELS[name]
        or backend.build.distribution != "1.4.2+task04u"
    ):
        raise ValueError(f"BLOCKED_CAPABILITY: {name} requires audited 04U model build")
    return backend


def _prepare(slot, card):
    from omegaconf import OmegaConf

    from narrow_nav.simulation.configuration import compose_config
    from narrow_nav.simulation.geometry_inspector import inspect_world
    from narrow_nav.simulation.world_builder import build_world

    bind = slot["binding"]
    overrides = [
        f"humans={slot['backend']}",
        "robots=mecanum_robot",
        f"seed={slot['seed']}",
    ]
    if card is not None:
        overrides += [
            "simulation.observation_sensors=[]",
            "experiments.duration_ns=30000000000",
        ]
        if card["id"].startswith("OT"):
            speed = card["subtypes"][bind["subtype"]]["human_speed_mps"]
            field = "speed_mps" if slot["backend"] == "orca" else "desired_speed"
            overrides.append(f"humans.{field}={speed}")
    config = compose_config(ROOT / "configs", (*overrides, *bind["overrides"]))
    world = build_world(config)
    scene = world.scene
    if config.humans.backend != slot["backend"] or (
        config.simulation.physics_dt_ns != TIMING["physics_dt_ns"]
        or config.simulation.control_dt_ns != TIMING["robot_control_dt_ns"]
        or config.simulation.human_control_dt_ns != TIMING["human_control_dt_ns"]
    ):
        raise ValueError("runtime does not match the frozen backend or clock contract")
    labels = bind.get("label_context", {})
    visible = None
    geometry_check = None
    if card is not None:
        if (
            world.episode.scenario_id != card["runtime_id"]
            or len(scene.humans) != bind["density"]
        ):
            raise ValueError(
                "runtime scene ID or density disagrees with approved binding"
            )
        if card["geometry"].get("min_length_m", 0) > OmegaConf.select(
            config, "worlds.length_m", default=0
        ):
            raise ValueError("corridor length is below the Card minimum")
        collision = inspect_world(world, allow_rotated_walls=True)
        from narrow_nav.humans.jupedsim_bridge.geometry import build_walkable

        walkable = build_walkable(collision.planar_world())
        for actor in (scene.robot, *scene.humans):
            walkable.validate_route(
                Point2D(actor.pose.x, actor.pose.y), actor.goal, actor.radius_m
            )
        width = collision.clear_width(*bind["width_probe_xy_m"])
        if not isclose(width, bind["width_m"], abs_tol=1e-9, rel_tol=0):
            raise ValueError(
                f"measured aperture {width} differs from binding {bind['width_m']}"
            )
        human_radius = max(actor.radius_m for actor in scene.humans)
        single_clearance = width - 2 * max(scene.robot.radius_m, human_radius)
        if single_clearance <= 0:
            raise ValueError("aperture cannot admit at least one actor footprint")
        geometry_check = {
            "aperture_m": width,
            "largest_actor_clearance_m": single_clearance,
            "robot_human_side_by_side_clearance_m": (
                width - 2 * (scene.robot.radius_m + human_radius)
            ),
        }
        code = card["id"][:2]
        if code in ("BC", "IN", "ND"):
            center = labels.get("center_xy_m", [])
            if len(center) != 2 or not all(isfinite(value) for value in center):
                raise ValueError("this Card requires a surveyed XY center for labels")
        if code == "ND" and labels.get("crossing_axis") not in (0, 1):
            raise ValueError("doorway label needs an axis")
        if code == "IN":
            speeds = labels.get("approach_speed_mps", {})
            if set(speeds) != {
                "robot",
                *(actor.entity_id for actor in scene.humans),
            } or any(not isfinite(speed) or speed <= 0 for speed in speeds.values()):
                raise ValueError(
                    "intersection label needs approved positive approach speeds"
                )
        if code == "BC":
            from shapely.geometry import LineString

            polygon = walkable.native_polygon()

            def visible(a, b):
                return polygon.covers(LineString((a, b)))

    shortest = bind.get("shortest_path_m")
    if shortest is not None and (
        not isfinite(shortest) or shortest <= 0 or not bind.get("shortest_path_source")
    ):
        raise ValueError("SPL reference path needs a positive length and source")
    commands = schedule(
        bind["commands"], world.episode.control_dt_ns, world.episode.duration_ns
    )
    normalized = OmegaConf.to_container(config, resolve=True)
    del normalized["humans"]
    model_sha = hashlib.sha256(world.mjcf.encode()).hexdigest()
    yaml = OmegaConf.to_yaml(config, resolve=True, sort_keys=True)
    return config, {
        "yaml": yaml,
        "card": card,
        "visible": visible,
        "geometry_check": geometry_check,
        "timing": TIMING,
        "schedule": commands,
        "goal_tolerance": config.humans.goal_tolerance_m,
        "goals": {
            "robot": (scene.robot.goal.x, scene.robot.goal.y),
            **{actor.entity_id: (actor.goal.x, actor.goal.y) for actor in scene.humans},
        },
        "radii": {
            "robot": scene.robot.radius_m,
            **{actor.entity_id: actor.radius_m for actor in scene.humans},
        },
        "config_sha256": hashlib.sha256(yaml.encode()).hexdigest(),
        "model_sha256": model_sha,
        "asset_hashes": dict(world.source_sha256),
        "paired_key": fingerprint(
            (
                slot["card_id"],
                slot["level"],
                slot["action"],
                slot["seed"],
                bind.get("subtype"),
                bind.get("human_policy"),
                bind["commands"],
                labels,
                normalized,
                model_sha,
                world.source_sha256,
            )
        ),
    }


def _manifest(path):
    data = json.loads(path.read_text())
    if data != compile_plan(ROOT, data["phase"], data["bindings"]):
        raise ValueError(
            "manifest differs from deterministic compiler or source fingerprint"
        )
    return data


def _select(data, args):
    selected = [
        slot
        for slot in data["slots"]
        if (args.episode is None or slot["slot_id"] == args.episode)
        and (args.backend is None or slot["backend"] == args.backend)
        and (args.scenario is None or slot["card_id"] == args.scenario)
        and (args.seed is None or slot["seed"] == args.seed)
    ]
    if not selected:
        raise ValueError("selection matches no planned slots")
    return selected


def _validate(data, selected, runtime=False, prepare=False):
    blocked = [slot for slot in selected if slot["blocked"]]
    availability = {}
    preparation_errors = []
    if runtime:
        for name in sorted({slot["backend"] for slot in selected}):
            try:
                backend = _backend(name, data["matrix"]["backends"])
                availability[name] = {"version": backend.backend_version}
            except (ImportError, RuntimeError, ValueError) as exc:
                availability[name] = {"blocked": str(exc)}
    if prepare:
        for slot in selected:
            if slot["blocked"] or "blocked" in availability.get(slot["backend"], {}):
                continue
            try:
                _prepare(slot, data["cards"].get(slot["card_id"]))
            except (ImportError, RuntimeError, ValueError, KeyError, TypeError) as exc:
                preparation_errors.append(
                    {"slot_id": slot["slot_id"], "error": str(exc)}
                )
    summary = {
        "phase": data["phase"],
        "selected": len(selected),
        "nominal_cumulative": len(data["slots"]),
        "nominal_new": sum(slot.get("is_new", True) for slot in selected),
        "bound_slots": len(selected) - len(blocked),
        "blocked_slots": len(blocked),
        "block_reasons": sorted(
            {reason for slot in blocked for reason in slot["blocked"]}
        ),
        "runtime": availability,
        "preparation_errors": preparation_errors,
    }
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return (
        not blocked
        and not preparation_errors
        and not any("blocked" in status for status in availability.values())
    )


def _previous(root, slot):
    return sorted((root / "attempts" / slot["slot_id"]).glob("attempt-*"))


def _reuse(history, execution_key):
    reusable = False
    for path in history:
        intent = (
            json.loads((path / "intent.json").read_text())
            if (path / "intent.json").exists()
            else {}
        )
        if intent.get("execution_key") not in (None, execution_key):
            raise ValueError(f"attempt fingerprint differs at {path}")
        validity = path / "validity.json"
        if (
            validity.exists()
            and (path / "manifest.json").exists()
            and intent.get("execution_key") == execution_key
            and json.loads(validity.read_text())["infrastructure_valid"]
        ):
            reusable = True
    return reusable


def _run(data, selected, args):
    from narrow_nav.simulation.mujoco_engine import MuJoCoEngine

    if not args.execute:
        raise ValueError("run requires explicit --execute")
    if not _validate(data, selected, runtime=True):
        raise ValueError(
            "selection contains blocked slots or unavailable required backends"
        )
    runtime = _runtime()
    if data["phase"] != "smoke" and runtime["git_dirty"]:
        raise ValueError("research execution requires a clean checkout")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    lock = args.output_dir / ".lock"
    with lock.open("a") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return _run_locked(data, selected, args, runtime, MuJoCoEngine)


def _run_locked(data, selected, args, runtime, engine_factory):
    root = args.output_dir
    marker = root / "run.json"
    lock = {"schema_version": 1, "plan_id": data["plan_id"], "runtime": runtime}
    if marker.exists():
        if not args.resume or json.loads(marker.read_text()) != lock:
            raise ValueError(
                "existing run requires --resume and identical plan/runtime provenance"
            )
    else:
        if args.resume or any(path.name != ".lock" for path in root.iterdir()):
            raise ValueError("new run needs an empty directory")
        write_json(marker, lock)
    plan_path = root / f"phase_{data['phase']}.json"
    if not plan_path.exists():
        write_json(plan_path, data)
    elif json.loads(plan_path.read_text()) != data:
        raise ValueError("saved phase manifest changed")
    candidates = []
    for slot in selected:
        backend = _backend(slot["backend"], data["matrix"]["backends"])
        config, case = _prepare(slot, data["cards"].get(slot["card_id"]))
        key = fingerprint(
            (
                lock,
                slot["slot_id"],
                backend.backend_version,
                case["config_sha256"],
                case["model_sha256"],
                case["asset_hashes"],
                slot["binding"],
            )
        )
        history = _previous(root, slot)
        reusable = _reuse(history, key)
        if args.only_new and not slot.get("is_new", True):
            if not reusable:
                raise ValueError(
                    f"cannot skip prior slot without reusable valid evidence: {slot['slot_id']}"
                )
            continue
        if reusable or (history and not args.retry_invalid):
            continue
        candidates.append((slot, backend, config, case, key, len(history)))
    invalid = False
    for slot, backend, config, case, key, prior in candidates[: args.max_episodes]:
        attempt = root / "attempts" / slot["slot_id"] / f"attempt-{prior + 1:04}"
        attempt.mkdir(parents=True)
        context = {
            "slot": slot,
            "phase": data["phase"],
            "attempt": prior + 1,
            "execution_key": key,
            "paired_key": case["paired_key"],
            "run_provenance": runtime,
            "backend_version": backend.backend_version,
            "backend_build": asdict(backend.build)
            if slot["backend"] in MODELS
            else None,
            "geometry_check": case["geometry_check"],
            "config_sha256": case["config_sha256"],
            "model_sha256": case["model_sha256"],
            "asset_hashes": case["asset_hashes"],
        }
        metrics = execute(
            engine_factory(human_backend=backend), config, slot, case, attempt, context
        )
        print(slot["slot_id"], metrics["outcome"], flush=True)
        invalid |= not metrics["infrastructure_valid"]
    return 2 if invalid else 0


def _filters(parser):
    parser.add_argument("--episode")
    parser.add_argument("--backend")
    parser.add_argument("--scenario")
    parser.add_argument("--seed", type=int)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    plan = commands.add_parser("plan", help="enumerate without physics")
    plan.add_argument("--phase", required=True, choices=("0", "1", "2", "smoke"))
    plan.add_argument("--output", required=True, type=Path)
    plan.add_argument("--bindings", type=Path)
    for name in ("validate", "dry-run", "run"):
        sub = commands.add_parser(name)
        sub.add_argument("--manifest", required=True, type=Path)
        _filters(sub)
        if name == "run":
            sub.add_argument("--output-dir", required=True, type=Path)
            for option in ("execute", "resume", "retry-invalid", "only-new"):
                sub.add_argument(f"--{option}", action="store_true")
            sub.add_argument("--max-episodes", type=int)
        else:
            sub.add_argument("--runtime", action="store_true")
    report = commands.add_parser("report")
    report.add_argument("--run-dir", required=True, type=Path)
    report.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "plan":
            phase = args.phase if args.phase == "smoke" else int(args.phase)
            result = compile_plan(
                ROOT, phase, _yaml(args.bindings) if args.bindings else None
            )
            args.output.parent.mkdir(parents=True, exist_ok=True)
            write_json(args.output, result)
            fields = (
                "slot_id",
                "backend",
                "card_id",
                "branch",
                "level",
                "action",
                "seed_id",
                "seed",
                "introduced",
                "is_new",
                "reused_from_phase",
                "blocked",
            )
            with args.output.with_suffix(".csv").open("w", newline="") as file:
                writer = csv.DictWriter(file, fields, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(result["slots"])
            _validate(result, result["slots"])
        elif args.command == "report":
            from narrow_nav.evaluation.reporting import report_run

            report_run(args.run_dir, args.output_dir)
        else:
            result = _manifest(args.manifest)
            selected = _select(result, args)
            if args.command != "run":
                return (
                    0
                    if _validate(
                        result,
                        selected,
                        args.runtime or args.command == "dry-run",
                        prepare=args.command == "dry-run",
                    )
                    else 2
                )
            if args.max_episodes is not None and args.max_episodes < 1:
                raise ValueError("--max-episodes must be positive")
            if (args.retry_invalid or args.only_new) and not args.resume:
                raise ValueError("--retry-invalid and --only-new require --resume")
            return _run(result, selected, args)
    except (ImportError, OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        print(f"benchmark: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
