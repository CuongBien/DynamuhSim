"""Convert the world XY projection to JuPedSim's uninflated walkable polygon."""

from dataclasses import dataclass
from hashlib import sha256
from math import cos, hypot, pi

from shapely import normalize, to_wkb
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union
from shapely.validation import explain_validity

from narrow_nav.core.errors import ContractError
from narrow_nav.core.planar_world import PlanarWorld
from narrow_nav.core.types import WORLD_FRAME, Point2D
from narrow_nav.core.validation import frame_is, positive

_ARC_SEGMENTS = 32
_GEOMETRY_TOLERANCE_M = 1e-9


class BridgeCapabilityError(ContractError):
    """An operation outside the verified native API envelope."""

    status = "BLOCKED_CAPABILITY"


def _polygon(ring: tuple[Point2D, ...]) -> Polygon:
    polygon = Polygon([(point.x, point.y) for point in ring])
    if not polygon.is_valid or polygon.area <= 0:
        raise ContractError(f"invalid XY ring: {explain_validity(polygon)}")
    return polygon


@dataclass(frozen=True, slots=True)
class WalkableGeometry:
    polygon: Polygon | MultiPolygon
    geometry_hash: str
    projection_error_m: float

    @property
    def components(self) -> tuple[Polygon, ...]:
        if isinstance(self.polygon, Polygon):
            return (self.polygon,)
        return tuple(self.polygon.geoms)

    def native_polygon(self, endpoints: tuple[Point2D, ...] = ()) -> Polygon:
        """Select the one component shared by all specified actor endpoints."""
        for endpoint in endpoints:
            frame_is(endpoint.frame_id, WORLD_FRAME, "native endpoint")
        points = tuple(Point(point.x, point.y) for point in endpoints)
        candidates = tuple(
            part
            for part in self.components
            if all(part.contains(point) for point in points)
        )
        if len(candidates) != 1:
            raise BridgeCapabilityError(
                "native geometry requires exactly one connected Polygon containing all endpoints"
            )
        return candidates[0]

    def validate_route(self, spawn: Point2D, goal: Point2D, radius_m: float) -> None:
        """Check connectivity for one circular actor without inflating native walls."""
        radius = positive(radius_m, "radius_m")
        # Conservative chord correction for the arcs of concave corners.
        inset = (radius + _GEOMETRY_TOLERANCE_M) / cos(pi / (4 * _ARC_SEGMENTS))
        centre_space = self.polygon.buffer(-inset, quad_segs=_ARC_SEGMENTS)
        for endpoint in (spawn, goal):
            frame_is(endpoint.frame_id, WORLD_FRAME, "route endpoint")
        start, finish = Point(spawn.x, spawn.y), Point(goal.x, goal.y)
        components = (
            centre_space.geoms
            if centre_space.geom_type == "MultiPolygon"
            else (centre_space,)
        )
        if not any(part.contains(start) for part in components):
            raise ContractError("spawn footprint outside walkable clearance")
        if not any(part.contains(finish) for part in components):
            raise ContractError("goal footprint outside walkable clearance")
        if not any(
            part.contains(start) and part.contains(finish) for part in components
        ):
            raise ContractError("spawn and goal are disconnected for the actor radius")

    def opening_width(
        self, centre: Point2D, *, axis: tuple[float, float], half_span_m: float
    ) -> float:
        """Measure the connected uninflated cross-section at a supplied threshold."""
        frame_is(centre.frame_id, WORLD_FRAME, "threshold")
        half_span = positive(half_span_m, "half_span_m")
        norm = positive(hypot(*axis), "cross-section axis norm")
        dx, dy = (value * half_span / norm for value in axis)
        at = Point(centre.x, centre.y)
        if not self.polygon.contains(at):
            raise ContractError("threshold centre must lie inside walkable space")
        line = LineString(
            [(centre.x - dx, centre.y - dy), (centre.x + dx, centre.y + dy)]
        )
        section = self.polygon.intersection(line)
        parts = getattr(section, "geoms", (section,))
        for part in parts:
            if (
                part.geom_type == "LineString"
                and part.distance(at) <= _GEOMETRY_TOLERANCE_M
            ):
                if any(
                    part.distance(Point(endpoint)) <= _GEOMETRY_TOLERANCE_M
                    for endpoint in line.coords
                ):
                    raise ContractError(
                        "cross-section must reach both sides of the opening"
                    )
                return part.length
        raise ContractError("no walkable cross-section at threshold centre")


def build_walkable(world: PlanarWorld) -> WalkableGeometry:
    floors = unary_union([_polygon(ring) for ring in world.floors])
    solids = unary_union(
        [_polygon(ring) for solid in world.solids for ring in solid.polygons]
    )
    polygon = floors.difference(solids)
    if (
        not isinstance(polygon, (Polygon, MultiPolygon))
        or polygon.is_empty
        or not polygon.is_valid
    ):
        raise ContractError(f"invalid walkable area: {explain_validity(polygon)}")
    digest = sha256(to_wkb(normalize(polygon), byte_order=1)).hexdigest()
    return WalkableGeometry(polygon, digest, world.projection_error_m)


def require_connected_polygon(geometry) -> Polygon:
    if geometry.is_empty or not geometry.is_valid:
        raise ContractError(f"invalid walkable area: {explain_validity(geometry)}")
    if geometry.geom_type != "Polygon":
        raise BridgeCapabilityError(
            f"one JuPedSim simulation requires a connected Polygon, got {geometry.geom_type}"
        )
    return geometry
