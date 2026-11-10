"""Deterministic straight-to-goal proposals for the technical baseline."""

from math import hypot

from narrow_nav.core.actions import HumanMotionRequest
from narrow_nav.core.errors import ContractError
from narrow_nav.core.protocols import HumanBackendConfig
from narrow_nav.core.states import HumanContext
from narrow_nav.core.types import NS_PER_SECOND, Velocity2D
from narrow_nav.core.validation import (
    finite,
    identifier,
    integer,
    ordered_ids,
    positive,
    require_type,
    uint64,
)

from .goal import goal_speed_limit


class ScriptedHumanBackend:
    backend_id = "scripted"
    backend_version = "1"

    def __init__(self) -> None:
        self._ids: tuple[str, ...] | None = None
        self._arrived: set[str] = set()

    def reset(
        self, config: HumanBackendConfig, seed: int, entity_ids: tuple[str, ...]
    ) -> None:
        uint64(seed, "human backend seed")
        require_type(entity_ids, tuple, "human IDs")
        ordered_ids(
            tuple(identifier(key, "human ID") for key in entity_ids), "human IDs"
        )
        speed = finite(config.parameters.get("speed_mps"), "humans.speed_mps")
        if speed < 0:
            raise ContractError("humans.speed_mps must be nonnegative")
        self._dt_s = (
            integer(config.control_dt_ns, "human control_dt_ns", minimum=1)
            / NS_PER_SECOND
        )
        self._tolerance = positive(config.goal_tolerance_m, "humans.goal_tolerance_m")
        self._acceleration = positive(
            config.max_acceleration_mps2, "human acceleration"
        )
        self._speed = speed
        self._ids = entity_ids
        self._arrived.clear()

    def propose(self, context: HumanContext) -> tuple[HumanMotionRequest, ...]:
        if self._ids != tuple(human.entity_id for human in context.humans):
            raise ContractError(
                "scripted backend requires reset with the current roster"
            )
        requests = []
        for human in context.humans:
            dx = human.goal.x - human.pose.x
            dy = human.goal.y - human.pose.y
            distance = hypot(dx, dy)
            if distance <= self._tolerance:
                self._arrived.add(human.entity_id)
            velocity = Velocity2D(0, 0)
            if human.entity_id not in self._arrived:
                braking_speed = goal_speed_limit(
                    distance, self._dt_s, self._acceleration
                )
                speed = min(self._speed, braking_speed)
                velocity = Velocity2D(dx / distance * speed, dy / distance * speed)
            requests.append(
                HumanMotionRequest(human.entity_id, velocity, context.stamp.timestamp)
            )
        return tuple(requests)
