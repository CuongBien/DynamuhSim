"""Compile MJCF and copy static collision geometry out of MuJoCo."""

from pathlib import Path

import mujoco

from narrow_nav.core.errors import ConfigurationError

from ..world_geometry import Circle, Rectangle, WorldCollision


def inspect_mjcf(
    source: str | Path, assets: tuple[tuple[str, bytes], ...] = ()
) -> WorldCollision:
    if isinstance(source, Path):
        model = mujoco.MjModel.from_xml_path(str(source.resolve()))
    else:
        model = mujoco.MjModel.from_xml_string(source, dict(assets))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return collision_geometry(model, data)


def collision_geometry(model: mujoco.MjModel, data: mujoco.MjData) -> WorldCollision:
    floors, walls, wall_ids = [], [], []
    for geom_id in range(model.ngeom):
        group = int(model.geom_group[geom_id])
        if group not in (0, 1, 3) or model.geom_bodyid[geom_id] != 0:
            continue
        if not (model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]):
            continue
        x, y, _ = map(float, data.geom_xpos[geom_id])
        half_x, half_y, _ = map(float, model.geom_size[geom_id])
        orientation = tuple(data.geom_xmat[geom_id])
        shape = model.geom_type[geom_id]
        if group != 0:
            wall_ids.append(model.geom(geom_id).name)
        if (
            group != 0
            and shape == mujoco.mjtGeom.mjGEOM_CYLINDER
            and orientation[8] == 1
        ):
            walls.append(Circle(x, y, half_x))
            continue
        if shape != mujoco.mjtGeom.mjGEOM_BOX or orientation != (
            1,
            0,
            0,
            0,
            1,
            0,
            0,
            0,
            1,
        ):
            raise ConfigurationError(
                "expected axis-aligned collision boxes or upright obstacle cylinders"
            )
        boxes = floors if group == 0 else walls
        boxes.append(Rectangle(x - half_x, x + half_x, y - half_y, y + half_y))
    if not floors or not walls:
        raise ConfigurationError(
            "compiled world must contain collision floors and walls"
        )
    return WorldCollision(tuple(floors), tuple(walls), tuple(wall_ids))
