"""Open a configured world in the interactive MuJoCo viewer."""

from argparse import ArgumentParser
from math import atan, degrees
from pathlib import Path
from time import perf_counter, sleep

import mujoco
from mujoco import viewer

from ..configuration import HumanProp, compose_config
from ..world_builder import MJCF_ROOT, build_world
from ..world_geometry import Circle
from .human_visuals import HumanVisuals


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config-dir", type=Path, default=MJCF_ROOT.parents[1] / "configs"
    )
    parser.add_argument(
        "overrides", nargs="*", help="Hydra overrides, e.g. robots=mecanum_robot"
    )
    focus_options = parser.add_mutually_exclusive_group()
    focus_options.add_argument(
        "--focus-human", metavar="ID", help="start the camera near a human"
    )
    focus_options.add_argument(
        "--focus-robot", action="store_true", help="start with a close-up of the robot"
    )
    args = parser.parse_args()

    world = build_world(compose_config(args.config_dir, args.overrides))
    focus = next(
        (
            spawn
            for spawn in (*world.scene.humans, *world.scene.human_props)
            if spawn.entity_id == args.focus_human
        ),
        None,
    )
    if args.focus_human is not None and focus is None:
        parser.error(f"human {args.focus_human!r} is not in the selected scenario")
    model = mujoco.MjSpec.from_string(
        world.mjcf, assets=dict(world.visual_assets)
    ).compile()
    walls = tuple(
        wall.bounds if isinstance(wall, Circle) else wall
        for wall in world.scene.collision.walls
    )
    x_min, x_max = min(wall.x_min for wall in walls), max(wall.x_max for wall in walls)
    y_min, y_max = min(wall.y_min for wall in walls), max(wall.y_max for wall in walls)
    model.stat.center[:2] = ((x_min + x_max) / 2, (y_min + y_max) / 2)
    view = model.vis.global_
    view.elevation = -89
    view.orthographic = 0
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    frame_height = 1.15 * max(y_max - y_min, (x_max - x_min) / 1.25)
    view.fovy = (
        42
        if focus is not None or args.focus_robot
        else degrees(2 * atan(frame_height / (2 * camera.distance)))
    )
    model.vis.headlight.active = 1
    model.vis.headlight.ambient[:] = (0.65, 0.65, 0.65)
    model.vis.headlight.diffuse[:] = (0.55, 0.55, 0.55)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    visuals = HumanVisuals(model, world.skin_actors)
    with viewer.launch_passive(
        model, data, show_left_ui=False, show_right_ui=False
    ) as window:
        with window.lock():
            mujoco.mjv_defaultFreeCamera(model, window.cam)
            window.opt.geomgroup[3] = 0  # Human collision footprints.
            window.opt.geomgroup[5] = 1  # Actor visual meshes and bodies.
            if args.focus_robot:
                pose = world.scene.robot.pose
                window.cam.lookat[:] = (pose.x, pose.y, 0.25)
                window.cam.azimuth = degrees(pose.yaw) + 145
                window.cam.elevation = -18
                window.cam.distance = 1.05
            elif focus is not None:
                seated = (
                    isinstance(focus, HumanProp) and focus.seat_height_m is not None
                )
                window.cam.lookat[:] = (focus.pose.x, focus.pose.y, 0.85)
                window.cam.azimuth = degrees(focus.pose.yaw) + (145 if seated else 180)
                window.cam.elevation = -24 if seated else -16
                window.cam.distance = 2.8
        start = perf_counter()
        while window.is_running():
            with window.lock():
                visuals.update(model, data, perf_counter() - start)
            window.sync()
            sleep(1 / 60)


if __name__ == "__main__":
    main()
