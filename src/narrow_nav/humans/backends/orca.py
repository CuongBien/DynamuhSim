"""RVO2-only human proposals, resynchronized to measured world state."""

from importlib import import_module
from importlib.metadata import version
from math import fsum, hypot

from narrow_nav.core.actions import HumanMotionRequest
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.protocols import HumanBackendConfig
from narrow_nav.core.states import HumanContext
from narrow_nav.core.types import NS_PER_SECOND, Velocity2D
from narrow_nav.core.validation import (
    finite,
    identifier,
    integer,
    ordered_ids,
    positive,
    require_type,
    uint64,
)

from ..goal import goal_speed_limit

_ZERO = (0.0, 0.0)
_ROBOT_PROFILE = "reciprocal_external_observed"


class OrcaHumanBackend:
    backend_id = "orca"

    def __init__(self) -> None:
        try:
            self._rvo2 = import_module("rvo2")
            self.backend_version = f"1+pyrvo2.{version('pyrvo2')}"
        except ImportError as exc:
            raise ConfigurationError(
                "ORCA requires the pinned pyrvo2 binding (human-orca extra)"
            ) from exc
        self._sim = None

    def reset(
        self, config: HumanBackendConfig, seed: int, entity_ids: tuple[str, ...]
    ) -> None:
        self._sim = None
        uint64(seed, "human backend seed")
        require_type(entity_ids, tuple, "human IDs")
        ordered_ids(
            tuple(identifier(key, "human ID") for key in entity_ids), "human IDs"
        )
        params = config.parameters
        if params.get("robot_profile") != _ROBOT_PROFILE:
            raise ConfigurationError(f"ORCA robot_profile must be {_ROBOT_PROFILE!r}")
        self._dt_s = (
            integer(config.control_dt_ns, "human control_dt_ns", minimum=1)
            / NS_PER_SECOND
        )
        self._speed = finite(params.get("speed_mps"), "humans.speed_mps")
        if self._speed < 0:
            raise ConfigurationError("humans.speed_mps must be nonnegative")
        self._goal_tolerance = positive(
            config.goal_tolerance_m, "humans.goal_tolerance_m"
        )
        self._max_speed = positive(config.max_speed_mps, "human max_speed_mps")
        self._acceleration = positive(
            config.max_acceleration_mps2, "human max_acceleration_mps2"
        )
        self._ids = entity_ids
        self._slots: dict[str, int] = {}
        self._robot_slot = None
        self._arrived: set[str] = set()
        self._sim = self._rvo2.PyRVOSimulator(
            self._dt_s,
            positive(params.get("neighbor_dist_m"), "humans.neighbor_dist_m"),
            integer(params.get("max_neighbors"), "humans.max_neighbors"),
            positive(params.get("time_horizon_s"), "humans.time_horizon_s"),
            positive(params.get("obstacle_horizon_s"), "humans.obstacle_horizon_s"),
            0.0,  # Per-agent radii arrive in the first HumanContext.
            self._max_speed,
        )

    def _initialize_scene(self, context: HumanContext) -> None:
        sim = self._sim
        for obstacle in sorted(context.obstacles, key=lambda item: item.entity_id):
            for polygon in obstacle.polygons:
                vertices = [(point.x, point.y) for point in polygon]
                twice_area = fsum(
                    x0 * y1 - x1 * y0
                    for (x0, y0), (x1, y1) in zip(
                        vertices, vertices[1:] + vertices[:1], strict=True
                    )
                )
                if twice_area == 0:
                    raise ContractError(
                        f"degenerate ORCA obstacle {obstacle.entity_id!r}"
                    )
                sim.addObstacle(vertices if twice_area > 0 else vertices[::-1])
        sim.processObstacles()
        for human in context.humans:
            self._slots[human.entity_id] = sim.addAgent((human.pose.x, human.pose.y))
        self._robot_slot = sim.addAgent((context.robot_pose.x, context.robot_pose.y))

    def propose(self, context: HumanContext) -> tuple[HumanMotionRequest, ...]:
        sim = self._sim
        if sim is None or self._ids != tuple(
            human.entity_id for human in context.humans
        ):
            raise ContractError("ORCA backend requires reset with the current roster")
        if not self._ids:
            return ()
        if self._robot_slot is None:
            self._initialize_scene(context)
        robot = self._robot_slot
        observed_robot_velocity = (
            context.robot_world_velocity.linear_x,
            context.robot_world_velocity.linear_y,
        )
        sim.setAgentPosition(robot, (context.robot_pose.x, context.robot_pose.y))
        sim.setAgentVelocity(robot, observed_robot_velocity)
        sim.setAgentPrefVelocity(robot, observed_robot_velocity)
        sim.setAgentRadius(robot, context.robot_radius_m)
        sim.setAgentMaxSpeed(robot, hypot(*observed_robot_velocity))

        for human in context.humans:
            slot = self._slots[human.entity_id]
            sim.setAgentPosition(slot, (human.pose.x, human.pose.y))
            sim.setAgentVelocity(
                slot, (human.velocity.linear_x, human.velocity.linear_y)
            )
            sim.setAgentRadius(slot, context.human_radius_m)
            dx, dy = human.goal.x - human.pose.x, human.goal.y - human.pose.y
            distance = hypot(dx, dy)
            if distance <= self._goal_tolerance:
                self._arrived.add(human.entity_id)
            if human.entity_id in self._arrived:
                sim.setAgentMaxSpeed(slot, 0.0)
                sim.setAgentPrefVelocity(slot, _ZERO)
            else:
                sim.setAgentMaxSpeed(slot, self._max_speed)
                speed = min(
                    self._speed,
                    goal_speed_limit(distance, self._dt_s, self._acceleration),
                )
                sim.setAgentPrefVelocity(
                    slot, (dx / distance * speed, dy / distance * speed)
                )
        sim.doStep()
        # Only velocities survive the solve. All internal positions/velocities
        # are overwritten from measured state on the next callback.
        return tuple(
            HumanMotionRequest(
                key,
                Velocity2D(
                    *(
                        _ZERO
                        if key in self._arrived
                        else sim.getAgentVelocity(self._slots[key])
                    )
                ),
                context.stamp.timestamp,
            )
            for key in self._ids
        )
