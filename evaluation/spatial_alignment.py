"""Spatial alignment and coordinate transformation utilities between Gazebo, Odom, and Map frames."""
from __future__ import annotations

from typing import List, Optional, Tuple

# Default corridor parameters for corridor_090
DEFAULT_SPAWN_X = -13.0
DEFAULT_SPAWN_Y = 0.0

# Alcove bounding box in Gazebo World coordinates:
# Model recess_side_left at x=1.35, recess_side_right at x=2.65, recess_back at y=1.10
ALCOVE_WORLD_X_MIN = 1.35
ALCOVE_WORLD_X_MAX = 2.65
ALCOVE_WORLD_Y_MIN = 0.45   # Corridor wall line is at y = 0.45m
ALCOVE_WORLD_Y_MAX = 1.15   # Back wall is at y = 1.10m (+ wall thickness)


def align_obstacle_to_robot_frame(
    gt_samples: List[Tuple[float, float, float]],
    spawn_x: float = DEFAULT_SPAWN_X,
    spawn_y: float = DEFAULT_SPAWN_Y,
) -> List[Tuple[float, float, float]]:
    """Transform ground-truth obstacle positions from Gazebo world coordinates into robot frame.

    Robot odometry starts at (0, 0) corresponding to (spawn_x, spawn_y) in world coordinates.
    x_robot_frame = x_world - spawn_x
    y_robot_frame = y_world - spawn_y
    """
    return [
        (t, x - spawn_x, y - spawn_y)
        for t, x, y in gt_samples
    ]


def align_robot_to_world_frame(
    robot_samples: List[Tuple[float, float, float]],
    spawn_x: float = DEFAULT_SPAWN_X,
    spawn_y: float = DEFAULT_SPAWN_Y,
) -> List[Tuple[float, float, float]]:
    """Transform robot odometry samples into Gazebo world frame."""
    return [
        (t, x + spawn_x, y + spawn_y)
        for t, x, y in robot_samples
    ]


def get_alcove_bounds(frame: str = "odom", spawn_x: float = DEFAULT_SPAWN_X) -> Tuple[float, float, float, float]:
    """Return (x_min, x_max, y_min, y_max) for the alcove recess."""
    if frame.lower() in ("world", "gazebo", "gz"):
        return ALCOVE_WORLD_X_MIN, ALCOVE_WORLD_X_MAX, ALCOVE_WORLD_Y_MIN, ALCOVE_WORLD_Y_MAX
    # Odom / Map frame
    return (
        ALCOVE_WORLD_X_MIN - spawn_x,
        ALCOVE_WORLD_X_MAX - spawn_x,
        ALCOVE_WORLD_Y_MIN,
        ALCOVE_WORLD_Y_MAX,
    )


def is_in_alcove(
    x: float,
    y: float,
    frame: str = "odom",
    spawn_x: float = DEFAULT_SPAWN_X,
    margin_x: float = 0.10,
    margin_y: float = 0.05,
) -> bool:
    """Check if a coordinate (x, y) is inside the alcove recess."""
    x_min, x_max, y_min, y_max = get_alcove_bounds(frame=frame, spawn_x=spawn_x)
    return (x_min - margin_x <= x <= x_max + margin_x) and (y >= y_min - margin_y)
