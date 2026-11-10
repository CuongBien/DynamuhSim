"""Sample a measured robot footprint for geometry probes."""

from dataclasses import dataclass
from math import cos, hypot, pi, sin

from shapely.geometry import Polygon

from narrow_nav.core.states import HumanContext
from narrow_nav.core.types import WORLD_FRAME, Point2D, Velocity2D
from narrow_nav.core.validation import frame_is, integer, positive, require_type

from .geometry import BridgeCapabilityError, require_connected_polygon

UNSUPPORTED_CAPABILITIES = (
    "external_agent_pose_write",
    "external_agent_velocity_coupling",
    "native_human_pose_resync",
)

_SIDES = 64


def require_dynamic_robot_support() -> None:
    raise BridgeCapabilityError(
        "JuPedSim 1.4.2 cannot accept the measured pose and velocity of an external robot"
    )


@dataclass(frozen=True, slots=True)
class RobotFootprintSample:
    centre: Point2D
    velocity: Velocity2D
    radius_m: float
    time_ns: int

    def __post_init__(self) -> None:
        frame_is(
            require_type(self.centre, Point2D, "robot centre").frame_id,
            WORLD_FRAME,
            "robot centre",
        )
        frame_is(
            require_type(self.velocity, Velocity2D, "robot velocity").frame_id,
            WORLD_FRAME,
            "robot velocity",
        )
        positive(self.radius_m, "robot radius_m")
        integer(self.time_ns, "robot time_ns")

    @classmethod
    def from_context(cls, context: HumanContext) -> "RobotFootprintSample":
        return cls(
            Point2D(context.robot_pose.x, context.robot_pose.y),
            context.robot_world_velocity,
            context.robot_radius_m,
            context.stamp.timestamp.time_ns,
        )

    def carve(self, geometry: Polygon) -> Polygon:
        outer = self.radius_m / cos(pi / _SIDES)
        footprint = Polygon(
            [
                (
                    self.centre.x + outer * cos(2 * k * pi / _SIDES),
                    self.centre.y + outer * sin(2 * k * pi / _SIDES),
                )
                for k in range(_SIDES)
            ]
        )
        return require_connected_polygon(geometry.difference(footprint))

    def error_bound_m(self, observed: "RobotFootprintSample") -> float:
        """Spatial error from holding the hole plus circumscribed polygon excess."""
        return (
            hypot(self.centre.x - observed.centre.x, self.centre.y - observed.centre.y)
            + abs(self.radius_m - observed.radius_m)
            + self.radius_m * (1 / cos(pi / _SIDES) - 1)
        )
