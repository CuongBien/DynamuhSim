"""Shared simulation construction. Model and agent setup belong to the caller."""

import jupedsim
from shapely.geometry import Polygon

from .clock import BridgeClock


def create_simulation(
    model, geometry: Polygon, clock: BridgeClock
) -> jupedsim.Simulation:
    return jupedsim.Simulation(model=model, geometry=geometry, dt=clock.internal_dt_s)
