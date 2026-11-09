"""Native JuPedSim Anticipation Velocity Model."""

from typing import ClassVar

from ..jupedsim_bridge.coupled_backend import CoupledHumanBackend


class AvmHumanBackend(CoupledHumanBackend):
    backend_id = "avm_jupedsim"
    model_class = "AnticipationVelocityModel"
    seeded_model = True
    model_defaults: ClassVar[dict[str, float]] = {"pushout_strength": 0.3}
    agent_defaults: ClassVar[dict[str, float]] = {
        "desired_speed": 1.2,
        "time_gap": 1.06,
        "strength_neighbor_repulsion": 8.0,
        "range_neighbor_repulsion": 0.1,
        "wall_buffer_distance": 0.1,
        "anticipation_time": 1.0,
        "reaction_time": 0.3,
    }
