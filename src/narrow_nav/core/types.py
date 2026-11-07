"""Frame-aware planar quantities in metres, seconds and radians."""

from dataclasses import dataclass
from math import pi

from .errors import ContractError
from .validation import finite, identifier, integer

WORLD_FRAME = "world"
ROBOT_BASE_FRAME = "robot_base"
SCHEMA_VERSION = 1
NS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True, slots=True)
class Timestamp:
    tick: int
    time_ns: int

    def __post_init__(self) -> None:
        integer(self.tick, "tick")
        integer(self.time_ns, "time_ns")
        if self.tick == 0:
            if self.time_ns != 0:
                raise ContractError("tick zero must have time_ns zero")
        elif self.time_ns == 0 or self.time_ns % self.tick:
            raise ContractError(
                "time_ns must be tick times a positive integer control step"
            )

    @property
    def time_s(self) -> float:
        return self.time_ns / NS_PER_SECOND


@dataclass(frozen=True, slots=True)
class Point2D:
    """Position in metres in the declared frame."""

    x: float
    y: float
    frame_id: str = WORLD_FRAME

    def __post_init__(self) -> None:
        finite(self.x, "x")
        finite(self.y, "y")
        identifier(self.frame_id, "frame_id")


@dataclass(frozen=True, slots=True)
class Pose2D:
    """Position in metres and counter-clockwise yaw in radians."""

    x: float
    y: float
    yaw: float
    frame_id: str = WORLD_FRAME

    def __post_init__(self) -> None:
        finite(self.x, "x")
        finite(self.y, "y")
        if not -pi <= finite(self.yaw, "yaw") < pi:
            raise ContractError("yaw must be in [-pi, pi)")
        identifier(self.frame_id, "frame_id")


@dataclass(frozen=True, slots=True)
class Velocity2D:
    """Planar linear velocity in m/s."""

    linear_x: float
    linear_y: float
    frame_id: str = WORLD_FRAME

    def __post_init__(self) -> None:
        finite(self.linear_x, "linear_x")
        finite(self.linear_y, "linear_y")
        identifier(self.frame_id, "frame_id")


@dataclass(frozen=True, slots=True)
class Twist2D:
    """Planar linear velocity in m/s and angular velocity in rad/s."""

    linear_x: float
    linear_y: float
    angular_z: float
    frame_id: str = ROBOT_BASE_FRAME

    def __post_init__(self) -> None:
        finite(self.linear_x, "linear_x")
        finite(self.linear_y, "linear_y")
        finite(self.angular_z, "angular_z")
        identifier(self.frame_id, "frame_id")
