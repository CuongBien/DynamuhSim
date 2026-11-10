"""Native JuPedSim Anticipation Velocity Model with goal stopping."""

from dataclasses import replace
from math import hypot
from typing import ClassVar

from narrow_nav.core.actions import HumanMotionRequest
from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.protocols import HumanBackendConfig
from narrow_nav.core.states import HumanContext
from narrow_nav.core.types import NS_PER_SECOND, Velocity2D
from narrow_nav.core.validation import positive

from ..goal import goal_speed_limit
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

    def reset(
        self, config: HumanBackendConfig, seed: int, entity_ids: tuple[str, ...]
    ) -> None:
        approach = config.parameters.get("goal_approach", "native")
        if approach not in ("braking_envelope", "native"):
            raise ConfigurationError("unsupported AVM goal approach")
        acceleration = positive(
            config.max_acceleration_mps2, "AVM goal braking acceleration"
        )
        super().reset(config, seed, entity_ids)
        self._goal_approach = approach
        self._goal_acceleration = acceleration

    def propose(self, context: HumanContext) -> tuple[HumanMotionRequest, ...]:
        motions = super().propose(context)
        if self._goal_approach == "native":
            return motions

        dt_s = self._clock.human_dt_ns / NS_PER_SECOND
        requests = []
        for human, motion in zip(context.humans, motions, strict=True):
            distance = hypot(human.goal.x - human.pose.x, human.goal.y - human.pose.y)
            vx, vy = motion.velocity.linear_x, motion.velocity.linear_y
            speed = hypot(vx, vy)
            limit = (
                0.0
                if distance <= self._goal_tolerance
                else goal_speed_limit(distance, dt_s, self._goal_acceleration)
            )
            if speed > limit:
                scale = limit / speed
                motion = replace(motion, velocity=Velocity2D(vx * scale, vy * scale))
            requests.append(motion)
        return tuple(requests)
