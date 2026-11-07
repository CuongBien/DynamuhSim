"""Drive conversion and motion limits with no simulator dependency."""

from dataclasses import dataclass
from math import hypot

from narrow_nav.core.errors import ContractError
from narrow_nav.core.types import Twist2D
from narrow_nav.core.validation import positive

DIFFERENTIAL_WHEELS = ("left", "right")
MECANUM_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


@dataclass(frozen=True, slots=True)
class DriveProfile:
    drive: str
    wheel_radius_m: float
    wheel_separation_m: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    max_linear_acceleration_mps2: float
    max_angular_acceleration_radps2: float
    wheel_speed_limit_radps: float
    wheelbase_m: float = 0.0

    def __post_init__(self) -> None:
        for name in (
            "wheel_radius_m",
            "wheel_separation_m",
            "max_linear_speed_mps",
            "max_angular_speed_radps",
            "max_linear_acceleration_mps2",
            "max_angular_acceleration_radps2",
            "wheel_speed_limit_radps",
        ):
            positive(getattr(self, name), name)
        if self.drive == "mecanum":
            positive(self.wheelbase_m, "wheelbase_m")
        elif self.drive != "differential":
            raise ContractError("robots.drive must be differential or mecanum")

    @property
    def wheels(self) -> tuple[str, ...]:
        return DIFFERENTIAL_WHEELS if self.drive == "differential" else MECANUM_WHEELS

    def wheel_speeds(self, velocity: Twist2D) -> tuple[float, ...]:
        x, y, z = velocity.linear_x, velocity.linear_y, velocity.angular_z
        turn = z * (self.wheelbase_m + self.wheel_separation_m) / 2
        if self.drive == "differential":
            if y != 0:
                raise ContractError("differential drive requires linear_y = 0")
            speeds = (x - turn, x + turn)
        else:
            speeds = (x - y - turn, x + y + turn, x + y - turn, x - y + turn)
        return tuple(speed / self.wheel_radius_m for speed in speeds)

    def limit(self, requested: Twist2D, previous: Twist2D, dt_s: float) -> Twist2D:
        x, y, z = requested.linear_x, requested.linear_y, requested.angular_z
        if self.drive == "differential" and y != 0:
            raise ContractError("differential drive requires linear_y = 0")
        # Normalize first so even large finite requests can be bounded safely.
        linear_scale = max(abs(x), abs(y), self.max_linear_speed_mps)
        x, y = x / linear_scale, y / linear_scale
        linear_bound = min(
            linear_scale, self.max_linear_speed_mps / (hypot(x, y) or 1.0)
        )
        x, y = x * linear_bound, y * linear_bound
        z = max(-self.max_angular_speed_radps, min(self.max_angular_speed_radps, z))
        wheel_peak = max(map(abs, self.wheel_speeds(Twist2D(x, y, z))))
        wheel_scale = self.wheel_speed_limit_radps / max(
            wheel_peak, self.wheel_speed_limit_radps
        )
        dx = x * wheel_scale - previous.linear_x
        dy = y * wheel_scale - previous.linear_y
        dz = z * wheel_scale - previous.angular_z
        linear_budget = self.max_linear_acceleration_mps2 * dt_s
        angular_budget = self.max_angular_acceleration_radps2 * dt_s
        fraction = min(
            linear_budget / max(linear_budget, hypot(dx, dy)),
            angular_budget / max(angular_budget, abs(dz)),
        )
        return Twist2D(
            previous.linear_x + fraction * dx,
            previous.linear_y + fraction * dy,
            previous.angular_z + fraction * dz,
        )
