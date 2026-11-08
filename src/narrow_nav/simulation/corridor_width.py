"""Resize corridor cross-sections while preserving lengths outside those bands."""

from dataclasses import dataclass

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.validation import finite, positive


@dataclass(frozen=True, slots=True)
class AxisResize:
    bands: tuple[tuple[float, float], ...] = ()
    width_m: float = 0.0
    anchor_m: float = 0.0

    def __post_init__(self) -> None:
        if self.bands:
            positive(self.width_m, "corridor width")
        previous = None
        for low, high in self.bands:
            finite(low, "corridor band lower bound")
            positive(high - low, "corridor band width")
            if previous is not None and low < previous:
                raise ConfigurationError("corridor bands must be sorted and disjoint")
            previous = high

    def _shift(self, value: float) -> float:
        return sum(
            (self.width_m - (high - low))
            * min(1.0, max(0.0, (value - low) / (high - low)))
            for low, high in self.bands
        )

    def map(self, value: float) -> float:
        return value + self._shift(value) - self._shift(self.anchor_m)

    def actor_coordinate(self, value: float, radius_m: float) -> float:
        mapped = self.map(value)
        margin = radius_m + 0.01
        for low, high in self.bands:
            if low < value < high and 2 * margin < self.width_m:
                return min(max(mapped, self.map(low) + margin), self.map(high) - margin)
        return mapped


@dataclass(frozen=True, slots=True)
class CorridorResize:
    x: AxisResize = AxisResize()
    y: AxisResize = AxisResize()
    x_applies_above_y_m: float | None = None

    def actor_point(self, x: float, y: float, radius_m: float) -> tuple[float, float]:
        # In an analytic L layout the X band belongs only to the north leg.
        if self.x_applies_above_y_m is None or (
            y >= self.x_applies_above_y_m and x >= self.x.bands[0][0]
        ):
            x = self.x.actor_coordinate(x, radius_m)
        return (
            x,
            self.y.actor_coordinate(y, radius_m),
        )
