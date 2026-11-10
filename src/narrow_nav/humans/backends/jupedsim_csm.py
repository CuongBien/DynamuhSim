"""Native JuPedSim Collision Free Speed Model V2."""

from typing import ClassVar

from ..jupedsim_bridge.coupled_backend import CoupledHumanBackend


class CsmHumanBackend(CoupledHumanBackend):
    backend_id = "csm_jupedsim"
    model_class = "CollisionFreeSpeedModelV2"
    native_velocity = False
    agent_defaults: ClassVar[dict[str, float]] = {
        "desired_speed": 1.2,
        "time_gap": 1.0,
        "strength_neighbor_repulsion": 8.0,
        "range_neighbor_repulsion": 0.1,
        "strength_geometry_repulsion": 5.0,
        "range_geometry_repulsion": 0.02,
    }
