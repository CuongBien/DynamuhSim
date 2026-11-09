"""Inspect compiled collision geometry without exposing native simulator objects."""

from pathlib import Path

from .mujoco.geometry import inspect_mjcf
from .world_builder import BuiltWorld
from .world_geometry import WorldCollision


def inspect_world(
    world: BuiltWorld | Path, *, allow_rotated_walls: bool = False
) -> WorldCollision:
    """Return free-space and clear-width queries for a built or on-disk scene."""
    if isinstance(world, BuiltWorld):
        return inspect_mjcf(
            world.mjcf, world.visual_assets, allow_rotated_walls=allow_rotated_walls
        )
    return inspect_mjcf(world, allow_rotated_walls=allow_rotated_walls)
