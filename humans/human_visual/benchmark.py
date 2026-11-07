"""Measure actual offscreen MuJoCo rendering and visual update costs."""

import json
import resource
from argparse import ArgumentParser
from time import perf_counter

import mujoco
import numpy as np
from OpenGL import GL

from .demo import ROOT, load_actors
from .mujoco_skin import build_preview


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--agents", type=int, choices=(1, 2, 5, 10), required=True)
    parser.add_argument("--frames", type=int, default=90)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    width, height = 640, 480
    ids = tuple(f"human_{i}" for i in range(args.agents))
    actors = load_actors(ROOT / "assets/generated", ids)
    scene = build_preview(actors, width, height)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (0, 0, 0.85)
    camera.azimuth = 137
    camera.elevation = -14
    camera.distance = max(4.4, args.agents * 1.18)
    elapsed, updates = [], []
    with mujoco.Renderer(scene.model, width=width, height=height) as renderer:
        gl_renderer = GL.glGetString(GL.GL_RENDERER).decode()
        for frame in range(12 + args.frames):
            start = perf_counter()
            for i, actor_id in enumerate(ids):
                scene.update(
                    actor_id,
                    0,
                    (i - (args.agents - 1) / 2) * 0.85,
                    0,
                    1.15,
                    "walk",
                    1 / 30,
                )
            scene.forward()
            updated = perf_counter()
            renderer.update_scene(scene.data, camera=camera)
            pixels = renderer.render()
            end = perf_counter()
            if frame >= 12:
                updates.append((updated - start) * 1000)
                elapsed.append(end - start)
    if pixels.shape != (height, width, 3) or not pixels.any():
        raise RuntimeError("empty MuJoCo framebuffer")
    print(
        json.dumps(
            {
                "mujoco": mujoco.__version__,
                "gl_renderer": gl_renderer,
                "agents": args.agents,
                "width": width,
                "height": height,
                "frames": args.frames,
                "rendered_fps": args.frames / sum(elapsed),
                "frame_p95_ms": float(np.percentile(elapsed, 95) * 1000),
                "update_p50_ms": float(np.median(updates)),
                "update_p95_ms": float(np.percentile(updates, 95)),
                "peak_rss_kib_linux": resource.getrusage(
                    resource.RUSAGE_SELF
                ).ru_maxrss,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
