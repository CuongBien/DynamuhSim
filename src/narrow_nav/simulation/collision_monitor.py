"""Domain contact classification and separate footprint clearance metrics."""

from itertools import combinations
from math import hypot

from narrow_nav.core.protocols import Clearance, ContactEvent, ContactKind
from narrow_nav.core.states import GroundTruthState

from .configuration import SceneGeometry
from .entity_registry import EntityRegistry
from .world_geometry import WorldCollision

_KINDS: dict[tuple[str, str], ContactKind] = {
    ("robot", "wall"): "robot_wall",
    ("human", "robot"): "robot_human",
    ("human", "wall"): "human_wall",
    ("human", "human"): "human_human",
}


def contact_events(
    pairs: set[tuple[str, str]], registry: EntityRegistry, time_ns: int
) -> tuple[ContactEvent, ...]:
    events = []
    for first, second in sorted(pairs):
        first_kind, second_kind = sorted((registry.kind(first), registry.kind(second)))
        kind = _KINDS.get((first_kind, second_kind))
        if kind is not None:
            events.append(ContactEvent(time_ns, first, second, kind))
    return tuple(events)


def measure_clearance(
    collision: WorldCollision, scene: SceneGeometry, state: GroundTruthState
) -> Clearance:
    x, y = state.robot.pose.x, state.robot.pose.y
    radius = scene.robot.radius_m
    humans = tuple(
        (human.pose.x, human.pose.y, spawn.radius_m)
        for human, spawn in zip(state.humans, scene.humans, strict=True)
    )
    return Clearance(
        robot_wall_m=min(wall.signed_distance(x, y) for wall in collision.walls)
        - radius,
        robot_human_m=min(
            (hypot(x - hx, y - hy) - radius - hr for hx, hy, hr in humans),
            default=None,
        ),
        human_wall_m=min(
            (
                wall.signed_distance(hx, hy) - hr
                for hx, hy, hr in humans
                for wall in collision.walls
            ),
            default=None,
        ),
        human_human_m=min(
            (
                hypot(ax - bx, ay - by) - ar - br
                for (ax, ay, ar), (bx, by, br) in combinations(humans, 2)
            ),
            default=None,
        ),
    )
