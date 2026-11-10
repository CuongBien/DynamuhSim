"""Planar command limits and heading convention for human control."""

from dataclasses import dataclass
from math import atan2, hypot

from narrow_nav.core.errors import ContractError
from narrow_nav.core.geometry import wrap_yaw
from narrow_nav.core.types import NS_PER_SECOND, WORLD_FRAME, Velocity2D
from narrow_nav.core.validation import frame_is, integer, positive


@dataclass(frozen=True, slots=True)
class HumanMotionLimits:
    max_speed_mps: float
    max_acceleration_mps2: float

    def __post_init__(self) -> None:
        positive(self.max_speed_mps, "scenarios.human_profile.max_speed_mps")
        positive(
            self.max_acceleration_mps2, "scenarios.human_profile.max_acceleration_mps2"
        )

    def limit(
        self, requested: Velocity2D, previous: Velocity2D, human_dt_ns: int
    ) -> Velocity2D:
        frame_is(requested.frame_id, WORLD_FRAME, "requested")
        frame_is(previous.frame_id, WORLD_FRAME, "previous")
        dt_s = integer(human_dt_ns, "human_dt_ns", minimum=1) / NS_PER_SECOND
        speed = self.max_speed_mps
        if hypot(previous.linear_x, previous.linear_y) > speed * (1 + 1e-12):
            raise ContractError("previous limited velocity exceeds the human speed cap")

        x, y = requested.linear_x, requested.linear_y
        scale = max(abs(x), abs(y), speed)
        x, y = x / scale, y / scale
        bound = min(scale, speed / (hypot(x, y) or 1.0))
        dx, dy = x * bound - previous.linear_x, y * bound - previous.linear_y
        change = hypot(dx, dy)
        budget = self.max_acceleration_mps2 * dt_s
        fraction = budget / change if change > budget else 1.0
        return Velocity2D(
            previous.linear_x + fraction * dx, previous.linear_y + fraction * dy
        )


def heading_from_velocity(velocity: Velocity2D, previous_yaw: float) -> float:
    frame_is(velocity.frame_id, WORLD_FRAME, "velocity")
    if velocity.linear_x == velocity.linear_y == 0:
        return wrap_yaw(previous_yaw)
    return wrap_yaw(atan2(velocity.linear_y, velocity.linear_x))
