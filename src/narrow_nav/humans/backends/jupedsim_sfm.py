"""Native JuPedSim Social Force Model."""

from typing import ClassVar

from ..jupedsim_bridge.coupled_backend import CoupledHumanBackend


class SfmHumanBackend(CoupledHumanBackend):
    backend_id = "sfm_jupedsim"
    model_class = "SocialForceModel"
    allow_legacy = True
    model_defaults: ClassVar[dict[str, float]] = {
        "body_force": 120_000,
        "friction": 240_000,
    }
    agent_defaults: ClassVar[dict[str, float]] = {
        "mass": 80.0,
        "desired_speed": 0.8,
        "reaction_time": 0.5,
        "agent_scale": 2000,
        "obstacle_scale": 2000,
        "force_distance": 0.08,
    }
