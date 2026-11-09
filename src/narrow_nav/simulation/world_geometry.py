"""Axis-aligned indoor layouts and planar clearance, in the world frame."""

from dataclasses import dataclass, field
from itertools import combinations
from math import cos, hypot, pi, sin, sqrt

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.planar_world import PlanarWorld
from narrow_nav.core.states import Obstacle
from narrow_nav.core.types import Point2D
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

    def signed_distance(self, x: float, y: float) -> float:
        if self.contains(x, y):
            return -min(x - self.x_min, self.x_max - x, y - self.y_min, self.y_max - y)
        return self.distance(x, y)

    def y_span(self, x: float) -> tuple[float, float] | None:
        if self.x_min - GEOMETRY_TOLERANCE_M <= x <= self.x_max + GEOMETRY_TOLERANCE_M:
            return self.y_min, self.y_max
        return None


@dataclass(frozen=True, slots=True)
class RotatedRectangle:
    local: Rectangle
    x: float
    y: float
    yaw: float

    def _local_xy(self, x: float, y: float) -> XY:
        dx, dy = x - self.x, y - self.y
        c, s = cos(self.yaw), sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def distance(self, x: float, y: float) -> float:
        return self.local.distance(*self._local_xy(x, y))

    def signed_distance(self, x: float, y: float) -> float:
        return self.local.signed_distance(*self._local_xy(x, y))

    def y_span(self, x: float) -> tuple[float, float] | None:
        ys = []
        ring = _box_vertices(self)
        for (ax, ay), (bx, by) in zip(ring, (*ring[1:], ring[0]), strict=True):
            if min(ax, bx) <= x <= max(ax, bx):
                if ax == bx:
                    ys.extend((ay, by))
                else:
                    ys.append(ay + (by - ay) * (x - ax) / (bx - ax))
        return (min(ys), max(ys)) if ys else None


def _box_vertices(box: Rectangle | RotatedRectangle) -> tuple[XY, ...]:
    rect = box.local if isinstance(box, RotatedRectangle) else box
    ring = (
        (rect.x_min, rect.y_min),
        (rect.x_max, rect.y_min),
        (rect.x_max, rect.y_max),
        (rect.x_min, rect.y_max),
    )
    if isinstance(box, RotatedRectangle):
        c, s = cos(box.yaw), sin(box.yaw)
        return tuple((box.x + c * x - s * y, box.y + s * x + c * y) for x, y in ring)
    return ring


@dataclass(frozen=True, slots=True)
class Circle:
    x: float
    y: float
    radius_m: float

    def __post_init__(self) -> None:
        finite(self.x, "circle x")
        finite(self.y, "circle y")
        positive(self.radius_m, "circle radius_m")

    @property
    def bounds(self) -> Rectangle:
        r = self.radius_m
        return Rectangle(self.x - r, self.x + r, self.y - r, self.y + r)

    def distance(self, x: float, y: float) -> float:
        return max(0.0, self.signed_distance(x, y))

    def signed_distance(self, x: float, y: float) -> float:
        return hypot(self.x - x, self.y - y) - self.radius_m

    def y_span(self, x: float) -> tuple[float, float] | None:
        dx = abs(x - self.x)
        if dx > self.radius_m + GEOMETRY_TOLERANCE_M:
            return None
        dy = sqrt(max(0.0, self.radius_m**2 - dx**2))
        return self.y - dy, self.y + dy


@dataclass(frozen=True, slots=True)
class WorldCollision:
    floors: tuple[Rectangle, ...]
    walls: tuple[Rectangle | RotatedRectangle | Circle, ...]
    wall_ids: tuple[str, ...] = field(default=(), compare=False)

    def obstacles(self, *, circle_sides: int = 16) -> tuple[Obstacle, ...]:
        obstacles = []
        for entity_id, wall in zip(self.wall_ids, self.walls, strict=True):
            if isinstance(wall, Circle):
                radius = wall.radius_m / cos(pi / circle_sides)
                ring = tuple(
                    (
                        wall.x + radius * cos(2 * k * pi / circle_sides),
                        wall.y + radius * sin(2 * k * pi / circle_sides),
                    )
                    for k in range(circle_sides)
                )
            else:
                ring = _box_vertices(wall)
            obstacles.append(
                Obstacle(entity_id, (tuple(Point2D(x, y) for x, y in ring),))
            )
        return tuple(obstacles)

    def planar_world(self) -> PlanarWorld:
        """Export the compiled collision footprint for a 2D consumer."""
        sides = 64
        return PlanarWorld(
            tuple(
                tuple(Point2D(x, y) for x, y in _box_vertices(floor))
                for floor in self.floors
            ),
            self.obstacles(circle_sides=sides),
            max(
                (
                    wall.radius_m * (1 / cos(pi / sides) - 1)
                    for wall in self.walls
                    if isinstance(wall, Circle)
                ),
                default=0.0,
            ),
        )

    def human_paths_clear(
        self, starts: tuple[XY, ...], ends: tuple[XY, ...], radii: tuple[float, ...]
    ) -> bool:
        paths = tuple(zip(starts, ends, radii, strict=True))
        for (x, y), (ex, ey), radius in paths:
            if (x, y) == (ex, ey):
                continue
            mx, my = (x + ex) / 2, (y + ey) / 2
            # Distance to a solid is 1-Lipschitz along the entire segment.
            margin = radius + hypot(ex - x, ey - y) / 2 + GEOMETRY_TOLERANCE_M
            if (
                not any(floor.contains(mx, my) for floor in self.floors)
                or not self.contains(ex, ey, radius)
                or any(wall.distance(mx, my) <= margin for wall in self.walls)
            ):
                return False
        for ((ax, ay), (aex, aey), ar), ((bx, by), (bex, bey), br) in combinations(
            paths, 2
        ):
            dx, dy = ax - bx, ay - by
            vx, vy = aex - bex - dx, aey - bey - dy
            length_squared = vx * vx + vy * vy
            fraction = (
                min(1.0, max(0.0, -(dx * vx + dy * vy) / length_squared))
                if length_squared
                else 0.0
            )
            if hypot(dx + fraction * vx, dy + fraction * vy) < ar + br:
                return False
        return True

    def contains(self, x: float, y: float, radius_m: float = 0.0) -> bool:
        """Test that a circular footprint lies in free space with positive clearance."""
        finite(x, "x")
        finite(y, "y")
        if finite(radius_m, "radius_m") < 0:
            raise ConfigurationError("radius_m must be nonnegative")
        if len(self.floors) == 1:
            (floor,) = self.floors
            if not (
                floor.x_min + radius_m <= x <= floor.x_max - radius_m
                and floor.y_min + radius_m <= y <= floor.y_max - radius_m
            ):
                return False
        return any(floor.contains(x, y) for floor in self.floors) and all(
            wall.distance(x, y) > radius_m + GEOMETRY_TOLERANCE_M for wall in self.walls
        )

    def clear_width(self, x: float, y: float = 0.0) -> float:
        """Measure the unobstructed span along Y through a free-space point."""
        if not self.contains(x, y):
            raise ConfigurationError("width measurement requires a free-space point")
        section = [span for wall in self.walls if (span := wall.y_span(x)) is not None]
        south = max((high for _, high in section if high < y), default=None)
        north = min((low for low, _ in section if low > y), default=None)
        if south is None or north is None:
            raise ConfigurationError("width measurement requires walls on both sides")
        return north - south


def opening_clearance(width_m: float, radius_m: float, *other_radii_m: float) -> float:
    """Necessary transverse slack for discs at a straight opening, in metres."""
    width = positive(width_m, "opening width_m")
    return width - 2 * sum(
        positive(radius, "footprint radius_m") for radius in (radius_m, *other_radii_m)
    )


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
        end_of_straight_wall = right

        if self.layout == "blind_corner":
            turn = positive(self.turn_length_m, "turn_length_m")
            inner = right - 2 * half_width
            end_of_straight_wall = inner
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
        elif self.layout not in ("corridor", "passing_bay"):
            raise ConfigurationError(f"unknown world layout {self.layout!r}")

        if self.layout in ("blind_corner", "passing_bay"):
            bay_width = positive(self.passing_bay_width_m, "passing_bay_width_m")
            bay_length = positive(self.passing_bay_length_m, "passing_bay_length_m")
            center = finite(self.passing_bay_center_x_m, "passing_bay_center_x_m")
            entry, exit_ = center - bay_length / 2, center + bay_length / 2
            if bay_width <= 2 * half_width:
                raise ConfigurationError(
                    "passing_bay_width_m must exceed corridor_width_m"
                )
            if entry - left <= thickness or end_of_straight_wall - exit_ <= thickness:
                raise ConfigurationError(
                    "passing bay must fit inside the corridor with room for both wall returns"
                )
            top = bay_width - half_width
            floors += (Rectangle(entry, exit_, north, top),)
            # The last edge of the boundary is the north wall of the main leg.
            boundary = (
                *boundary[:-1],
                (exit_, north),
                (exit_, top),
                (entry, top),
                (entry, north),
                boundary[-1],
            )

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
