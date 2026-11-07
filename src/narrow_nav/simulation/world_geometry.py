"""Axis-aligned indoor layouts and planar clearance, in the world frame."""

from dataclasses import dataclass
from math import hypot

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.validation import finite, positive

XY = tuple[float, float]
GEOMETRY_TOLERANCE_M = 1e-9


@dataclass(frozen=True, slots=True)
class Rectangle:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self) -> None:
        positive(self.x_max - self.x_min, "rectangle length")
        positive(self.y_max - self.y_min, "rectangle width")

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def distance(self, x: float, y: float) -> float:
        return hypot(
            max(self.x_min - x, x - self.x_max, 0),
            max(self.y_min - y, y - self.y_max, 0),
        )


@dataclass(frozen=True, slots=True)
class WorldCollision:
    floors: tuple[Rectangle, ...]
    walls: tuple[Rectangle, ...]

    def contains(self, x: float, y: float, radius_m: float = 0.0) -> bool:
        """Test that a circular footprint lies in free space with positive clearance."""
        finite(x, "x")
        finite(y, "y")
        if finite(radius_m, "radius_m") < 0:
            raise ConfigurationError("radius_m must be nonnegative")
        return any(floor.contains(x, y) for floor in self.floors) and all(
            wall.distance(x, y) > radius_m + GEOMETRY_TOLERANCE_M for wall in self.walls
        )

    def clear_width(self, x: float, y: float = 0.0) -> float:
        """Measure the unobstructed span along Y through a free-space point."""
        if not self.contains(x, y):
            raise ConfigurationError("width measurement requires a free-space point")
        section = [
            wall
            for wall in self.walls
            if wall.x_min - GEOMETRY_TOLERANCE_M
            <= x
            <= wall.x_max + GEOMETRY_TOLERANCE_M
        ]
        south = max((wall.y_max for wall in section if wall.y_max < y), default=None)
        north = min((wall.y_min for wall in section if wall.y_min > y), default=None)
        if south is None or north is None:
            raise ConfigurationError("width measurement requires walls on both sides")
        return north - south


@dataclass(frozen=True, slots=True)
class WorldSpec:
    layout: str
    corridor_length_m: float
    corridor_width_m: float
    wall_height_m: float
    wall_thickness_m: float
    turn_length_m: float | None = None
    passing_bay_width_m: float | None = None
    passing_bay_length_m: float | None = None
    passing_bay_center_x_m: float | None = None

    def collision(self) -> WorldCollision:
        """Extrude the exterior of a counterclockwise floor boundary into solid walls."""
        half_length = positive(self.corridor_length_m, "corridor_length_m") / 2
        half_width = positive(self.corridor_width_m, "corridor_width_m") / 2
        positive(self.wall_height_m, "wall_height_m")
        thickness = positive(self.wall_thickness_m, "wall_thickness_m")
        left, right, south, north = -half_length, half_length, -half_width, half_width
        floor = Rectangle(left, right, south, north)
        floors = (floor,)
        boundary = ((left, south), (right, south), (right, north), (left, north))

        if self.layout == "blind_corner":
            turn = positive(self.turn_length_m, "turn_length_m")
            inner = right - 2 * half_width
            if inner - left <= thickness:
                raise ConfigurationError(
                    "blind_corner requires a main leg longer than its width and wall thickness"
                )
            top = north + turn
            floors += (Rectangle(inner, right, north, top),)
            boundary = (
                (left, south),
                (right, south),
                (right, top),
                (inner, top),
                (inner, north),
                (left, north),
            )
        elif self.layout == "passing_bay":
            bay_width = positive(self.passing_bay_width_m, "passing_bay_width_m")
            bay_length = positive(self.passing_bay_length_m, "passing_bay_length_m")
            center = finite(self.passing_bay_center_x_m, "passing_bay_center_x_m")
            entry, exit_ = center - bay_length / 2, center + bay_length / 2
            if bay_width <= 2 * half_width:
                raise ConfigurationError(
                    "passing_bay_width_m must exceed corridor_width_m"
                )
            if entry - left <= thickness or right - exit_ <= thickness:
                raise ConfigurationError(
                    "passing bay must fit inside the corridor with room for both wall returns"
                )
            top = bay_width - half_width
            floors += (Rectangle(entry, exit_, north, top),)
            boundary = (
                (left, south),
                (right, south),
                (right, north),
                (exit_, north),
                (exit_, top),
                (entry, top),
                (entry, north),
                (left, north),
            )
        elif self.layout != "corridor":
            raise ConfigurationError(f"unknown world layout {self.layout!r}")

        return WorldCollision(floors, _wall_boxes(boundary, thickness))


def _convex(previous: XY, corner: XY, following: XY) -> bool:
    ax, ay = corner[0] - previous[0], corner[1] - previous[1]
    bx, by = following[0] - corner[0], following[1] - corner[1]
    return ax * by - ay * bx > 0


def _wall_boxes(boundary: tuple[XY, ...], thickness: float) -> tuple[Rectangle, ...]:
    walls = []
    for index, (x0, y0) in enumerate(boundary):
        following = (index + 1) % len(boundary)
        x1, y1 = boundary[following]
        if y0 == y1:
            direction = 1 if x1 > x0 else -1
            # Horizontal walls fill the outside of convex corners and butt against vertical walls.
            start = (
                thickness
                if _convex(boundary[index - 1], (x0, y0), (x1, y1))
                else -thickness
            )
            end = (
                thickness
                if _convex(
                    (x0, y0), (x1, y1), boundary[(following + 1) % len(boundary)]
                )
                else -thickness
            )
            x0 -= direction * start
            x1 += direction * end
            y1 -= direction * thickness
        else:
            x1 += thickness if y1 > y0 else -thickness
        walls.append(Rectangle(min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)))
    return tuple(walls)
