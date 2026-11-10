"""Shared measured-state coupling for JuPedSim native models."""

import json
import sys
from dataclasses import dataclass
from hashlib import sha256
from importlib.metadata import PackageNotFoundError, version
from math import cos, dist, isclose, sin
from pathlib import Path
from typing import ClassVar

from narrow_nav.core.actions import HumanMotionRequest
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.geometry import wrap_yaw
from narrow_nav.core.protocols import HumanBackendConfig
from narrow_nav.core.states import HumanContext, HumanState
from narrow_nav.core.types import NS_PER_SECOND, Point2D, Velocity2D
from narrow_nav.core.validation import finite, identifier, ordered_ids, positive, uint64

from .clock import BridgeClock

XY = tuple[float, float]
SFM_DISTRIBUTION = "1.4.2+task04s"
SFM_SOURCE_COMMIT = "80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe"
SFM_PATCH_SHA256 = "0d79faa084acf5b863a3944258499837b46f7f9ccae56d8119b3a555bab710e4"


@dataclass(frozen=True, slots=True)
class JupedsimBuild:
    distribution: str
    source_commit: str
    patch_sha256: str
    binary: str
    binary_sha256: str


@dataclass(frozen=True, slots=True)
class PedestrianSync:
    entity_id: str
    native_identity: tuple[int, int, int]
    position: XY
    velocity: XY
    prediction: XY
    pre_rebase_error_m: float


@dataclass(frozen=True, slots=True)
class CoupledSync:
    time_ns: int
    iteration_before: int
    iteration_after: int
    robot_id: str
    robot_native_id: int
    robot_position: XY
    robot_velocity: XY
    pedestrians: tuple[PedestrianSync, ...]


def _native_runtime(backend_id, allow_legacy):
    minimum = "1.4.2+task04s external/rebase APIs or " if allow_legacy else ""
    message = (
        f"{backend_id} requires isolated JuPedSim {minimum}1.4.2+task04u coupling APIs"
    )
    try:
        import jupedsim
        from jupedsim.native import py_jps

        distribution = version("jupedsim")
    except (ImportError, PackageNotFoundError) as error:
        raise ConfigurationError(message) from error
    binary = Path(py_jps.__file__).resolve()
    package = Path(jupedsim.__file__).resolve().parent
    legacy = allow_legacy and distribution == SFM_DISTRIBUTION
    methods = (
        ("register_external_sfm_actor", "apply_sfm_coupled_state", "external_sfm_actor")
        if legacy
        else (
            "register_external_actor",
            "apply_coupled_state",
            "external_actor",
            "coupled_human_velocity",
        )
    )
    if (
        (not legacy and distribution != "1.4.2+task04u")
        or jupedsim.__version__ != "1.4.2"
        or not package.is_relative_to(Path(sys.prefix).resolve())
        or binary.parent != package
        or not all(
            hasattr(simulation, method)
            for simulation in (jupedsim.Simulation, py_jps.Simulation)
            for method in methods
        )
    ):
        raise ConfigurationError(message)
    binary_hash = sha256(binary.read_bytes()).hexdigest()
    patch_hash = SFM_PATCH_SHA256
    if not legacy:
        try:
            provenance = json.loads((package / "_task04u.json").read_text())
            if (
                provenance["source_commit"] != SFM_SOURCE_COMMIT
                or provenance["binary_sha256"] != binary_hash
            ):
                raise ValueError("native provenance mismatch")
            patch_hash = provenance["patch_sha256"]
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise ConfigurationError("invalid JuPedSim 04U build provenance") from error
    return (
        jupedsim,
        JupedsimBuild(
            distribution, SFM_SOURCE_COMMIT, patch_hash, str(binary), binary_hash
        ),
        legacy,
    )


def _identity(agent) -> tuple[int, int, int]:
    return agent.id, agent.journey_id, agent.stage_id


def _parameters(values, defaults):
    resolved = {}
    for name, default in defaults.items():
        value = finite(values.get(name, default), f"humans.{name}")
        if name in ("mass", "reaction_time", "force_distance"):
            positive(value, f"humans.{name}")
        elif value < 0:
            raise ContractError(f"humans.{name} must be nonnegative")
        resolved[name] = value
    return resolved


class CoupledHumanBackend:
    backend_id: str
    model_class: str
    model_defaults: ClassVar[dict[str, float]] = {}
    agent_defaults: ClassVar[dict[str, float]]
    allow_legacy = False
    seeded_model = False
    native_velocity = True
    coupling_profile = "external_actor_hold"

    def __init__(self) -> None:
        self._jps, self.build, self._legacy = _native_runtime(
            self.backend_id, self.allow_legacy
        )
        self.backend_version = (
            f"{self.build.distribution}:sha256:{self.build.binary_sha256}"
        )
        self._simulation = None
        self._episode_id = None
        self.last_sync: CoupledSync | None = None

    def reset(
        self, config: HumanBackendConfig, seed: int, entity_ids: tuple[str, ...]
    ) -> None:
        from .geometry import build_walkable

        uint64(seed, "human backend seed")
        ordered_ids(
            tuple(identifier(key, "human ID") for key in entity_ids), "human IDs"
        )
        if config.planar_world is None:
            raise ConfigurationError(
                f"{self.backend_id} requires the compiled planar world"
            )
        if config.control_dt_ns not in (50_000_000, 100_000_000):
            raise ConfigurationError(
                f"{self.backend_id} supports human periods of 50 or 100 ms"
            )
        if (
            config.parameters.get("coupling_profile", self.coupling_profile)
            != self.coupling_profile
        ):
            raise ConfigurationError("unsupported JuPedSim coupling profile")
        self._clock = BridgeClock(config.control_dt_ns)
        self._geometry = build_walkable(config.planar_world)
        self._model = _parameters(config.parameters, self.model_defaults)
        if self.seeded_model:
            self._model["rng_seed"] = seed
        self._parameters = _parameters(config.parameters, self.agent_defaults)
        self._robot_anticipation = finite(
            config.parameters.get("robot_anticipation_time", 1.0),
            "humans.robot_anticipation_time",
        )
        self._goal_tolerance = positive(
            config.goal_tolerance_m, "JuPedSim waypoint tolerance"
        )
        self._entity_ids = entity_ids
        self._simulation = self._episode_id = self.last_sync = None

    @property
    def geometry_hash(self) -> str:
        return self._geometry.geometry_hash

    def _add_pedestrian(self, sim, human: HumanState, radius: float) -> int:
        goal = human.goal.x, human.goal.y
        stage = sim.add_waypoint_stage(goal, self._goal_tolerance)
        route = sim.add_journey(self._jps.JourneyDescription([stage]))
        parameters = getattr(self._jps, f"{self.model_class}AgentParameters")
        kinematics = {}
        if self.model_class == "SocialForceModel":
            kinematics = {
                "orientation": (cos(human.pose.yaw), sin(human.pose.yaw)),
                "velocity": (human.velocity.linear_x, human.velocity.linear_y),
            }
        return sim.add_agent(
            parameters(
                position=(human.pose.x, human.pose.y),
                radius=radius,
                journey_id=route,
                stage_id=stage,
                **self._parameters,
                **kinematics,
            )
        )

    def _initialize(self, context: HumanContext) -> None:
        from .model_factory import create_simulation

        endpoints = [Point2D(context.robot_pose.x, context.robot_pose.y)]
        for human in context.humans:
            spawn = Point2D(human.pose.x, human.pose.y)
            self._geometry.validate_route(spawn, human.goal, context.human_radius_m)
            endpoints.extend((spawn, human.goal))
        polygon = self._geometry.native_polygon(tuple(endpoints))
        model = getattr(self._jps, self.model_class)(**self._model)
        sim = create_simulation(model, polygon, self._clock)
        self._human_ids = tuple(
            self._add_pedestrian(sim, human, context.human_radius_m)
            for human in context.humans
        )
        self._human_radius = context.human_radius_m
        self._robot = context.robot_entity_id, context.robot_radius_m
        self._robot_native_id = (
            sim.register_external_sfm_actor(context.robot_radius_m)
            if self._legacy
            else sim.register_external_actor(
                context.robot_radius_m, self._robot_anticipation
            )
        )
        self._episode_id = context.stamp.episode_id
        self._simulation = sim

    def propose(self, context: HumanContext) -> tuple[HumanMotionRequest, ...]:
        stamp = context.stamp.timestamp
        if getattr(self, "_entity_ids", None) != tuple(
            h.entity_id for h in context.humans
        ):
            raise ContractError(
                "JuPedSim requires reset with the current pedestrian roster"
            )
        if self._simulation is None:
            if stamp.time_ns != 0:
                raise ContractError("JuPedSim first callback must start at time zero")
            self._initialize(context)
        sim = self._simulation
        before = self._clock.iterations_at(stamp.time_ns)
        if (
            context.stamp.episode_id != self._episode_id
            or context.human_radius_m != self._human_radius
            or (context.robot_entity_id, context.robot_radius_m) != self._robot
            or sim.iteration_count() != before
        ):
            raise ContractError("JuPedSim episode, external actor or clock changed")
        robot_xy = context.robot_pose.x, context.robot_pose.y
        robot_v = (
            context.robot_world_velocity.linear_x,
            context.robot_world_velocity.linear_y,
        )
        states = [
            (
                native_id,
                (h.pose.x, h.pose.y),
                (h.velocity.linear_x, h.velocity.linear_y),
                h.pose.yaw,
            )
            for native_id, h in zip(self._human_ids, context.humans, strict=True)
        ]
        old_agents = [sim.agent(native_id) for native_id in self._human_ids]
        identities = [_identity(agent) for agent in old_agents]
        errors = [
            dist(agent.position, state[1])
            for agent, state in zip(old_agents, states, strict=True)
        ]
        steps = self._clock.iterations_per_callback
        robot_yaw = context.robot_pose.yaw
        if self._legacy:
            sim.apply_sfm_coupled_state(
                stamp.time_ns,
                steps,
                external=[(self._robot_native_id, robot_xy, robot_v)],
                pedestrians=[state[:3] for state in states],
            )
            read_actor = sim.external_sfm_actor
            expected_actor = (
                self._robot_native_id,
                robot_xy,
                robot_v,
                self._robot[1],
                stamp.time_ns,
                True,
            )
        else:
            sim.apply_coupled_state(
                stamp.time_ns,
                steps,
                external=[(self._robot_native_id, robot_xy, robot_v, robot_yaw)],
                pedestrians=states,
            )
            read_actor = sim.external_actor
            expected_actor = (
                self._robot_native_id,
                robot_xy,
                robot_v,
                self._robot[1],
                stamp.time_ns,
            )
        for agent, (_, xy, v, yaw), identity in zip(
            old_agents, states, identities, strict=True
        ):
            observed_v = (
                agent.model.velocity
                if self._legacy
                else sim.coupled_human_velocity(agent.id)
            )
            if (agent.position, observed_v, _identity(agent)) != (
                xy,
                v if self.native_velocity else None,
                identity,
            ):
                raise ContractError("JuPedSim native rebase readback mismatch")
            if (
                not self.allow_legacy
                and dist(agent.orientation, (cos(yaw), sin(yaw))) > 1e-12
            ):
                raise ContractError("JuPedSim human heading readback mismatch")
        robot_sample = read_actor(self._robot_native_id)
        if self._legacy:
            matches_robot = robot_sample == expected_actor
        else:
            matches_robot = (
                *robot_sample[:3],
                *robot_sample[4:],
            ) == expected_actor and isclose(
                wrap_yaw(robot_sample[3] - robot_yaw), 0, abs_tol=1e-12
            )
        if not matches_robot:
            raise ContractError("JuPedSim external actor boundary readback mismatch")
        sim.iterate(steps)
        if (
            sim.iteration_count() != before + steps
            or read_actor(self._robot_native_id) != robot_sample
        ):
            raise ContractError(
                "JuPedSim native iteration count or held robot state changed"
            )
        dt = self._clock.human_dt_ns / NS_PER_SECOND
        samples, motions = [], []
        for entity_id, agent, (_, xy, v, _), error in zip(
            self._entity_ids, old_agents, states, errors, strict=True
        ):
            px, py = agent.position
            samples.append(
                PedestrianSync(entity_id, _identity(agent), xy, v, (px, py), error)
            )
            motions.append(
                HumanMotionRequest(
                    entity_id, Velocity2D((px - xy[0]) / dt, (py - xy[1]) / dt), stamp
                )
            )
        self.last_sync = CoupledSync(
            stamp.time_ns,
            before,
            before + steps,
            self._robot[0],
            self._robot_native_id,
            robot_sample[1],
            robot_sample[2],
            tuple(samples),
        )
        return tuple(motions)
