"""Integer clocks on the physics grid."""

from __future__ import annotations

from dataclasses import dataclass

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.validation import integer


@dataclass(frozen=True, slots=True)
class Timing:
    physics_dt_ns: int
    robot_control_dt_ns: int
    human_control_dt_ns: int
    observation_dt_ns: int

    def __post_init__(self) -> None:
        integer(self.physics_dt_ns, "physics_dt_ns", minimum=1)
        for name in ("robot_control_dt_ns", "human_control_dt_ns", "observation_dt_ns"):
            period = integer(getattr(self, name), name, minimum=1)
            if period % self.physics_dt_ns:
                raise ConfigurationError(f"{name} must be divisible by physics_dt_ns")

    @property
    def substeps_per_tick(self) -> int:
        return self.robot_control_dt_ns // self.physics_dt_ns

    def clocks(self, robot_tick: int) -> ClockState:
        time_ns = robot_tick * self.robot_control_dt_ns
        return ClockState(
            physics_steps=robot_tick * self.substeps_per_tick,
            last_human_control_time_ns=time_ns
            // self.human_control_dt_ns
            * self.human_control_dt_ns,
            last_observation_time_ns=time_ns
            // self.observation_dt_ns
            * self.observation_dt_ns,
        )


@dataclass(frozen=True, slots=True)
class ClockState:
    physics_steps: int
    last_human_control_time_ns: int
    last_observation_time_ns: int
