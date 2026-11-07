"""Domain contact classification and separate footprint clearance metrics."""

from dataclasses import dataclass
from math import hypot

from narrow_nav.core.protocols import ContactEvent, ContactKind
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


@dataclass(frozen=True, slots=True)
class Clearance:
    robot_wall_m: float
    robot_human_m: float | None


def measure_clearance(
    collision: WorldCollision, scene: SceneGeometry, state: GroundTruthState
) -> Clearance:
    x, y = state.robot.pose.x, state.robot.pose.y
    radius = scene.robot.radius_m
    return Clearance(
        robot_wall_m=min(wall.distance(x, y) for wall in collision.walls) - radius,
        robot_human_m=min(
            (
                hypot(x - human.pose.x, y - human.pose.y) - radius - spawn.radius_m
                for human, spawn in zip(state.humans, scene.humans, strict=True)
            ),
            default=None,
        ),
    )
