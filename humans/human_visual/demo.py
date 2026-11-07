"""Run the isolated 150-second Quaternius animation test fixture under MuJoCo."""

import json
import resource
from argparse import ArgumentParser
from contextlib import ExitStack
from hashlib import sha256
from math import ceil, isfinite
from pathlib import Path
from time import perf_counter, sleep

import mujoco
import numpy as np
from PIL import Image

from .bundle import AssetError, load_character
from .diagnostics import FootDiagnostics
from .mujoco_skin import build_preview, sole_probes
from .replay import CorridorReplay, Replay

ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 800, 600


def load_actors(asset_directory, ids):
    male, female = (
        load_character(asset_directory / name)
        for name in ("casual_male", "casual_female")
    )
    if (male.metadata["gender"], female.metadata["gender"]) != ("male", "female"):
        raise AssetError("expected one male and one female character")
    return {actor_id: (male, female)[index % 2] for index, actor_id in enumerate(ids)}


def frame_camera(camera, commands):
    xs = [command.x for command in commands]
    ys = [command.y for command in commands]
    camera.lookat[:] = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, 0.85)
    camera.azimuth = 137
    camera.elevation = -12
    camera.distance = max(4.2, 1.5 * max(max(xs) - min(xs), max(ys) - min(ys)) + 2.1)


def run_demo(
    spec,
    assets_dir,
    duration,
    agents,
    seed,
    fps,
    *,
    scene_name="studio",
    check_only=False,
    record_dir=None,
    record_every=30,
):
    corridor = scene_name == "corridor"
    replay = (
        CorridorReplay(agents, seed)
        if corridor
        else Replay(spec, duration, agents, seed)
    )
    scene_path = ROOT / "scenes/corridor.xml" if corridor else None
    stage_names = ("corridor",) if corridor else spec["stage_names"]
    actors = load_actors(assets_dir, replay.ids)
    scene = build_preview(actors, WIDTH, HEIGHT, scene_path=scene_path)
    diagnostics = FootDiagnostics()
    probes_by_character = {}
    for character in actors.values():
        if character.identifier not in probes_by_character:
            probes_by_character[character.identifier] = sole_probes(character)
    if not corridor:
        for actor_id in replay.ids[2:]:
            scene.update(actor_id, 1000.0, 0.0, 0.0, 0.0, "idle", 0.0)
    digest = sha256()
    stage_frames = dict.fromkeys(stage_names, 0)
    camera = mujoco.MjvCamera()
    if corridor:
        camera.lookat[:] = (0, 0, 0.65)
        camera.azimuth = 114
        camera.elevation = -31
        camera.distance = 13.8
    steps = ceil(duration * fps)
    update_ms = []
    previous = simulated = 0.0
    rendered = active_agents_at_start = 0
    started = perf_counter()
    with ExitStack() as stack:
        window = renderer = None
        if record_dir is not None:
            record_dir.mkdir(parents=True, exist_ok=True)
            renderer = stack.enter_context(
                mujoco.Renderer(scene.model, width=WIDTH, height=HEIGHT)
            )
        elif not check_only:
            from .window_view import open_window

            window = stack.enter_context(
                open_window(
                    scene.model,
                    WIDTH,
                    HEIGHT,
                    orbit_camera=camera if corridor else None,
                )
            )
        wall_start = perf_counter()
        for frame in range(steps + 1):
            if window is not None and not window.is_running():
                break
            simulated = min(frame / fps, duration)
            dt = simulated - previous
            previous = simulated
            commands = replay.update(simulated, dt)
            if frame == 0:
                active_agents_at_start = len(commands)
            stage = stage_names[replay.stage]
            stage_frames[stage] += 1
            before = perf_counter()
            poses = []
            for cmd in commands:
                pose = scene.update(
                    cmd.human_id,
                    cmd.x,
                    cmd.y,
                    cmd.heading,
                    cmd.speed,
                    cmd.state,
                    dt,
                )
                poses.append(pose)
            scene.forward()
            update_ms.append((perf_counter() - before) * 1e3)
            for cmd, pose in zip(commands, poses, strict=True):
                digest.update(pose.bone_positions.tobytes())
                digest.update(pose.bone_quaternions.tobytes())
                character = actors[cmd.human_id]
                for side, probe in enumerate(probes_by_character[character.identifier]):
                    diagnostics.update(
                        cmd.human_id,
                        side,
                        probe.position(pose),
                        cmd.speed,
                        dt,
                        (cmd.x, cmd.y),
                        cmd.heading,
                    )
            if renderer is not None and (frame % record_every == 0 or frame == steps):
                if not corridor:
                    frame_camera(camera, commands)
                renderer.update_scene(scene.data, camera=camera)
                Image.fromarray(renderer.render()).save(
                    record_dir / f"{frame:06d}-{stage}.png"
                )
                rendered += 1
            if window is not None:
                if not corridor:
                    frame_camera(camera, commands)
                window.draw(scene.model, scene.data, camera)
                rendered += 1
                if window.paused:
                    pause_started = perf_counter()
                    while window.is_running() and window.paused:
                        window.draw(scene.model, scene.data, camera)
                        sleep(1 / fps)
                    wall_start += perf_counter() - pause_started
                sleep(max(0, wall_start + (frame + 1) / fps - perf_counter()))
    return {
        "mujoco": mujoco.__version__,
        "mode": "record"
        if record_dir is not None
        else "check_only"
        if check_only
        else "gui",
        "duration_s": duration,
        "simulated_s": simulated,
        "completed": simulated == duration,
        "scene": scene_name,
        "agents": agents,
        "active_agents_at_start": active_agents_at_start,
        "seed": seed,
        "fps_config": fps,
        "frames": len(update_ms),
        "stage_frames": stage_frames,
        "skins": scene.model.nskin,
        "mocap_bodies": scene.model.nmocap,
        "visible_geom_count_excluding_skin": scene.model.ngeom,
        "rendered_frames": rendered,
        "pose_digest_sha256": digest.hexdigest(),
        "wall_s": perf_counter() - started,
        "update_p50_ms": float(np.median(update_ms)) if update_ms else None,
        "update_p95_ms": float(np.percentile(update_ms, 95)) if update_ms else None,
        "peak_rss_kib_linux": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        **(diagnostics.summary() if update_ms else {}),
    }


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--assets-dir", type=Path, default=ROOT / "assets/generated")
    parser.add_argument("--spec", type=Path, default=ROOT / "replay.json")
    parser.add_argument(
        "--scene",
        choices=("studio", "corridor"),
        default="studio",
        help="studio stages or an interactive corridor with all actors visible from the start",
    )
    parser.add_argument("--duration", type=float)
    parser.add_argument("--agents", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--fps", type=int)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument(
        "--check-only",
        action="store_true",
        help="validate and advance all frames without a GL context",
    )
    modes.add_argument(
        "--record-dir",
        type=Path,
        help="write real MuJoCo PNG frames inside humans/ using offscreen GL",
    )
    parser.add_argument(
        "--record-every", type=int, default=30, help="record every N simulation frames"
    )
    args = parser.parse_args()
    spec = json.loads(args.spec.read_text(encoding="utf-8"))
    duration = spec["duration_s"] if args.duration is None else args.duration
    agents = spec["agents"] if args.agents is None else args.agents
    seed = spec["seed"] if args.seed is None else args.seed
    fps = spec["fps"] if args.fps is None else args.fps
    if (
        not isfinite(duration)
        or duration <= 0
        or not 1 <= agents <= 10
        or min(fps, args.record_every) < 1
        or seed < 0
    ):
        parser.error(
            "positive duration/fps/record interval, nonnegative seed and 1–10 agents required"
        )
    record_dir = args.record_dir.resolve() if args.record_dir else None
    if record_dir is not None and not record_dir.is_relative_to(ROOT):
        parser.error("record directory must stay inside humans/")
    try:
        result = run_demo(
            spec,
            args.assets_dir,
            duration,
            agents,
            seed,
            fps,
            scene_name=args.scene,
            check_only=args.check_only,
            record_dir=record_dir,
            record_every=args.record_every,
        )
    except (AssetError, FileNotFoundError) as error:
        parser.exit(2, f"BLOCKED: {error}\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
