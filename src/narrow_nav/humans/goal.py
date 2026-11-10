"""Shared stopping envelope for preferred goal velocity."""

from math import sqrt


def goal_speed_limit(distance_m: float, dt_s: float, acceleration_mps2: float) -> float:
    """Leave room for one held command and then braking to rest."""
    return 2 * distance_m / (dt_s + sqrt(dt_s**2 + 2 * distance_m / acceleration_mps2))
