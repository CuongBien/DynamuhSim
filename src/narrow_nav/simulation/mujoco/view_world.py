"""Open a configured world in the interactive MuJoCo viewer."""

from argparse import ArgumentParser
from math import atan, degrees
from pathlib import Path
from time import sleep

import mujoco
from mujoco import viewer

from ..configuration import compose_config
from ..world_builder import MJCF_ROOT, build_world


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config-dir", type=Path, default=MJCF_ROOT.parents[1] / "configs"
    )
    parser.add_argument(
        "overrides", nargs="*", help="Hydra overrides, e.g. robots=mecanum_robot"
    )
    args = parser.parse_args()

    world = build_world(compose_config(args.config_dir, args.overrides))
    model = mujoco.MjSpec.from_string(world.mjcf).compile()
    walls = world.scene.collision.walls
    x_min, x_max = min(wall.x_min for wall in walls), max(wall.x_max for wall in walls)
    y_min, y_max = min(wall.y_min for wall in walls), max(wall.y_max for wall in walls)
    model.stat.center[:2] = ((x_min + x_max) / 2, (y_min + y_max) / 2)
    view = model.vis.global_
    view.elevation = -89
    view.orthographic = 0
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    frame_height = 1.15 * max(y_max - y_min, (x_max - x_min) / 1.25)
    view.fovy = degrees(2 * atan(frame_height / (2 * camera.distance)))
    model.vis.headlight.active = 1
    model.vis.headlight.ambient[:] = (0.55, 0.55, 0.55)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with viewer.launch_passive(
        model, data, show_left_ui=False, show_right_ui=False
    ) as window:
        with window.lock():
            mujoco.mjv_defaultFreeCamera(model, window.cam)
            window.opt.geomgroup[5] = 1  # Actor visual meshes and bodies.
        while window.is_running():
            window.sync()
            sleep(1 / 60)


if __name__ == "__main__":
    main()
