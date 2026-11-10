"""Compile MJCF and copy static collision geometry out of MuJoCo."""

from math import atan2, isclose
from pathlib import Path

import mujoco

from narrow_nav.core.errors import ConfigurationError

from ..world_geometry import (
    GEOMETRY_TOLERANCE_M,
    Circle,
    Rectangle,
    RotatedRectangle,
    WorldCollision,
    is_yield_bay_cap,
)


def inspect_mjcf(
    source: str | Path,
    assets: tuple[tuple[str, bytes], ...] = (),
    *,
    allow_rotated_walls: bool = False,
) -> WorldCollision:
    if isinstance(source, Path):
        model = mujoco.MjModel.from_xml_path(str(source.resolve()))
    else:
        model = mujoco.MjModel.from_xml_string(source, dict(assets))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return collision_geometry(model, data, allow_rotated_walls=allow_rotated_walls)


def collision_geometry(
    model: mujoco.MjModel, data: mujoco.MjData, *, allow_rotated_walls: bool = False
) -> WorldCollision:
    floors, walls, wall_ids = [], [], []
    for geom_id in range(model.ngeom):
        group = int(model.geom_group[geom_id])
        if group not in (0, 1, 3) or model.geom_bodyid[geom_id] != 0:
            continue
        if not (model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]):
            continue
        if is_yield_bay_cap(model.geom(geom_id).name):
            continue
        x, y, z = map(float, data.geom_xpos[geom_id])
        half_x, half_y, half_z = map(float, model.geom_size[geom_id])
        orientation = tuple(data.geom_xmat[geom_id])
        shape = model.geom_type[geom_id]
        upright = all(
            isclose(orientation[index], value, abs_tol=GEOMETRY_TOLERANCE_M, rel_tol=0)
            for index, value in ((2, 0), (5, 0), (6, 0), (7, 0), (8, 1))
        )
        if not upright:
            raise ConfigurationError("collision geometry must be upright in world XY")
        if group != 0 and shape == mujoco.mjtGeom.mjGEOM_CYLINDER:
            # MuJoCo cylinders store radius and half-height in the first two slots.
            if z + half_y <= GEOMETRY_TOLERANCE_M:
                raise ConfigurationError("collision cylinder must extend above Z=0")
            walls.append(Circle(x, y, half_x))
        elif shape == mujoco.mjtGeom.mjGEOM_BOX:
            yaw = atan2(orientation[3], orientation[0])
            if group == 0:
                if (
                    abs(yaw) > GEOMETRY_TOLERANCE_M
                    or abs(z + half_z) > GEOMETRY_TOLERANCE_M
                ):
                    raise ConfigurationError(
                        "collision floor must be axis-aligned with top at Z=0"
                    )
                floors.append(Rectangle(x - half_x, x + half_x, y - half_y, y + half_y))
                continue
            if z + half_z <= GEOMETRY_TOLERANCE_M:
                raise ConfigurationError("collision wall must extend above Z=0")
            if abs(yaw) <= GEOMETRY_TOLERANCE_M:
                walls.append(Rectangle(x - half_x, x + half_x, y - half_y, y + half_y))
            elif allow_rotated_walls:
                walls.append(
                    RotatedRectangle(
                        Rectangle(-half_x, half_x, -half_y, half_y), x, y, yaw
                    )
                )
            else:
                raise ConfigurationError("runtime collision walls must be axis-aligned")
        else:
            raise ConfigurationError(
                "expected collision boxes or upright obstacle cylinders"
            )
        wall_ids.append(model.geom(geom_id).name)
    if not floors or not walls:
        raise ConfigurationError(
            "compiled world must contain collision floors and walls"
        )
    return WorldCollision(tuple(floors), tuple(walls), tuple(wall_ids))
