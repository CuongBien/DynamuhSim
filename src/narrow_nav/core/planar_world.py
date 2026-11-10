"""Engine-independent collision projection in world XY, metres, floor at Z=0."""

from dataclasses import dataclass

from .errors import ContractError
from .states import Obstacle
from .types import WORLD_FRAME, Point2D
from .validation import finite, require_type, unique


@dataclass(frozen=True, slots=True)
class PlanarWorld:
    floors: tuple[tuple[Point2D, ...], ...]
    solids: tuple[Obstacle, ...]
    projection_error_m: float

    def __post_init__(self) -> None:
        require_type(self.floors, tuple, "floors")
        if not self.floors:
            raise ContractError("planar world requires a floor")
        # Reuse the polygon ring checks shared with the context obstacle contract.
        Obstacle("floor_projection", self.floors)
        require_type(self.solids, tuple, "solids")
        for solid in self.solids:
            require_type(solid, Obstacle, "solid")
        unique((solid.entity_id for solid in self.solids), "solid IDs")
        if finite(self.projection_error_m, "projection_error_m") < 0:
            raise ContractError("projection_error_m must be nonnegative")

    @property
    def frame_id(self) -> str:
        return WORLD_FRAME

    @property
    def schema_version(self) -> int:
        return 1
