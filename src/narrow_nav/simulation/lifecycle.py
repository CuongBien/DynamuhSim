"""Engine-independent orchestration of one loaded world and its episodes."""

from dataclasses import dataclass, replace
from hashlib import sha256
from importlib.metadata import version
from math import hypot
from pathlib import Path
from platform import python_version
from subprocess import CalledProcessError, check_output
from traceback import format_exception
from typing import Protocol
from uuid import uuid4

from omegaconf import DictConfig, ListConfig, OmegaConf

from narrow_nav.core.actions import (
    HumanMotion,
    MotionBatch,
    RobotAction,
    validate_human_requests,
)
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.metadata import EpisodeConfig, EpisodeMetadata
from narrow_nav.core.observations import Observation, SensorSlot
from narrow_nav.core.protocols import (
    AdapterSnapshot,
    AdapterStepResult,
    HumanBackendConfig,
    RegisteredHumanBackend,
    SimulationStepResult,
    SimulatorAdapter,
)
from narrow_nav.core.seed import derive_seed
from narrow_nav.core.states import GroundTruthState, HumanContext, Obstacle
from narrow_nav.core.types import NS_PER_SECOND, Twist2D, Velocity2D
from narrow_nav.core.validation import positive, require_type, uint64, unique

from .backend import SimulationError, SimulationMetadata
from .collision_monitor import Clearance, measure_clearance
from .configuration import drive_from_config, timing_from_config
from .human_motion import HumanMotionLimits
from .robot_controller import DriveProfile
from .state_extractor import executed_motion
from .timing import Timing
from .world_builder import MJCF_ROOT, BuiltWorld, build_world
from .world_geometry import GEOMETRY_TOLERANCE_M, WorldCollision

_UNAVAILABLE_SENSORS = ("depth", "imu", "lidar", "odometry")
_STOP = Twist2D(0, 0, 0)
_HUMAN_STOP = Velocity2D(0, 0)


class RuntimeAdapter(SimulatorAdapter[BuiltWorld], Protocol):
    @property
    def version(self) -> str: ...

    def load(self, world: BuiltWorld, drive: DriveProfile) -> WorldCollision: ...

    def close(self) -> None: ...


@dataclass(frozen=True, slots=True)
class _Loaded:
    config: DictConfig
    episode: EpisodeConfig
    world: BuiltWorld
    timing: Timing
    drive: DriveProfile
    collision: WorldCollision
    sensor_slots: tuple[SensorSlot, ...]
    goal_tolerance_m: float
    human_limits: HumanMotionLimits
    human_obstacles: tuple[Obstacle, ...]


def _git_provenance(repo: Path) -> tuple[str, bool]:
    try:
        commit = check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
        dirty = bool(
            check_output(
                ["git", "--no-optional-locks", "status", "--porcelain"],
                cwd=repo,
                text=True,
            )
        )
    except (OSError, CalledProcessError) as exc:
        raise SimulationError(
            "episode provenance requires a Git checkout with a commit"
        ) from exc
    return commit, dirty


class SimulationLifecycle:
    """Typed episode facade, with native operations delegated to the adapter."""

    def __init__(
        self,
        adapter: RuntimeAdapter,
        *,
        mjcf_root: Path = MJCF_ROOT,
        human_backend: RegisteredHumanBackend | None = None,
    ) -> None:
        self._adapter = adapter
        self._human_backend = human_backend
        self._mjcf_root = mjcf_root.resolve()
        self._run_id = uuid4().hex
        self._ordinal = 0
        self._closed = False
        self._loaded: _Loaded | None = None
        self._metadata: SimulationMetadata | None = None
        self._truth: GroundTruthState | None = None
        self._observation: Observation | None = None
        self._last_limited = _STOP
        self._human_commands: tuple[
            tuple[HumanMotion, ...], tuple[HumanMotion, ...]
        ] = ((), ())

    def _require_loaded(self) -> _Loaded:
        if self._closed:
            raise SimulationError("simulation is closed")
        if self._loaded is None:
            raise SimulationError("load a configuration first")
        return self._loaded

    def load(self, config: DictConfig) -> None:
        if self._closed:
            raise SimulationError("simulation is closed")
        resolved = OmegaConf.create(
            OmegaConf.to_container(config, resolve=True, throw_on_missing=True)
        )
        built = build_world(resolved, mjcf_root=self._mjcf_root)
        episode = built.episode
        timing = timing_from_config(resolved, episode)
        if self._human_backend is not None:
            if resolved.humans.backend != self._human_backend.backend_id:
                raise ConfigurationError(
                    "humans.backend does not match the injected backend"
                )
            timing.human_control_due(0)
        profile = resolved.scenarios.human_profile
        human_limits = HumanMotionLimits(
            profile.max_speed_mps, profile.max_acceleration_mps2
        )
        drive = drive_from_config(resolved)
        sensors = require_type(
            OmegaConf.select(
                resolved, "simulation.observation_sensors", default=OmegaConf.create([])
            ),
            ListConfig,
            "observation_sensors",
        )
        if any(sensor not in _UNAVAILABLE_SENSORS for sensor in sensors):
            raise ConfigurationError(
                "observation_sensors must be selected from depth, imu, lidar, odometry"
            )
        unique(sensors, "observation_sensors")
        slots = tuple(
            SensorSlot(sensor, "missing", reason="capability_unavailable")
            for sensor in sorted(sensors)
        )
        goal_tolerance = positive(
            OmegaConf.select(resolved, "experiments.goal_tolerance_m", default=0.1),
            "goal_tolerance_m",
        )
        collision = self._adapter.load(built, drive)
        obstacles = collision.obstacles() if self._human_backend is not None else ()
        OmegaConf.set_readonly(resolved, True)
        self._loaded = _Loaded(
            resolved,
            episode,
            built,
            timing,
            drive,
            collision,
            slots,
            goal_tolerance,
            human_limits,
            obstacles,
        )
        self._metadata = self._truth = self._observation = None
        self._last_limited = _STOP
        self._human_commands = ((), ())

    def reset(
        self, seed: int | None = None, scenario: str | None = None
    ) -> Observation:
        loaded = self._require_loaded()
        if scenario is not None and scenario != loaded.episode.scenario_id:
            raise ConfigurationError(
                "load a configuration for the requested scenario before reset"
            )
        seed = loaded.episode.episode_seed if seed is None else uint64(seed)
        episode = replace(loaded.episode, episode_seed=seed)
        config = OmegaConf.merge(loaded.config, {"seed": seed})
        yaml = OmegaConf.to_yaml(config, resolve=True, sort_keys=True)
        commit, dirty = _git_provenance(MJCF_ROOT.parents[1])
        header = EpisodeMetadata(
            self._run_id,
            self._ordinal,
            episode,
            sha256(yaml.encode()).hexdigest(),
            commit,
            dirty,
        )
        metadata = SimulationMetadata(
            episode=header,
            resolved_config=yaml,
            model_sha256=sha256(loaded.world.mjcf.encode()).hexdigest(),
            mjcf_source_sha256=loaded.world.source_sha256,
            engine_version=self._adapter.version,
            python_version=python_version(),
            project_version=version("narrow-nav"),
            engine_seed=derive_seed(seed, "engine"),
            timing=loaded.timing,
            clocks=loaded.timing.clocks(0),
            unavailable_sensors=_UNAVAILABLE_SENSORS,
            human_motion_mode="stationary"
            if self._human_backend is None
            else "kinematic",
            human_backend_id=None
            if self._human_backend is None
            else self._human_backend.backend_id,
            human_backend_version=None
            if self._human_backend is None
            else self._human_backend.backend_version,
            human_backend_seed=None
            if self._human_backend is None
            else derive_seed(seed, "human_backend"),
            not_observable_solver=getattr(self._adapter, "not_observable_solver", ()),
        )
        self._ordinal += 1
        self._metadata = metadata
        self._truth = self._observation = None
        self._last_limited = _STOP
        self._human_commands = ((), ())
        try:
            snapshot = require_type(
                self._adapter.reset(loaded.world, header, metadata.engine_seed, ()),
                AdapterSnapshot,
                "reset snapshot",
            )
            truth = snapshot.ground_truth
            self._validate_truth(loaded, header, 0, truth)
            if self._goal_reached(loaded, truth):
                raise SimulationError("initial state already satisfies the goal")
            observation = Observation(truth.stamp, loaded.sensor_slots)
        except Exception as exc:
            self._fail("reset_failed", 0, exc)
            raise
        zero = tuple(
            HumanMotion(human.entity_id, _HUMAN_STOP, truth.stamp.timestamp)
            for human in truth.humans
        )
        if self._human_backend is not None:
            try:
                self._human_backend.reset(
                    HumanBackendConfig(
                        OmegaConf.to_container(loaded.config.humans, resolve=True),
                        loaded.timing.human_control_dt_ns,
                        positive(
                            loaded.config.humans.get("goal_tolerance_m", 0.1),
                            "humans.goal_tolerance_m",
                        ),
                        loaded.human_limits.max_acceleration_mps2,
                        max_speed_mps=loaded.human_limits.max_speed_mps,
                    ),
                    metadata.human_backend_seed,
                    tuple(human.entity_id for human in truth.humans),
                )
            except Exception as exc:
                self._fail("human_backend_reset_failed", 0, exc)
                raise
        self._human_commands = (zero, zero)
        self._truth, self._observation = truth, observation
        self._metadata = replace(metadata, last_valid_tick=0)
        return observation

    @staticmethod
    def _validate_truth(
        loaded: _Loaded, header: EpisodeMetadata, tick: int, truth: GroundTruthState
    ) -> None:
        if truth.stamp != header.stamp(tick):
            raise ContractError("adapter snapshot must match the episode stamp")
        scene = loaded.world.scene
        if truth.entity_ids != (
            scene.robot.entity_id,
            *(human.entity_id for human in scene.humans),
        ):
            raise ContractError("adapter snapshot must match the configured actor IDs")

    @staticmethod
    def _goal_reached(loaded: _Loaded, truth: GroundTruthState) -> bool:
        pose, goal = truth.robot.pose, loaded.world.scene.robot.goal
        return hypot(pose.x - goal.x, pose.y - goal.y) <= loaded.goal_tolerance_m

    def _fail(self, reason: str, tick: int, error: Exception) -> None:
        self._metadata = replace(
            self.get_metadata(),
            status="failed",
            end_reason=reason,
            failure_tick=tick,
            failure=f"{type(error).__name__}: {error}",
            failure_traceback="".join(format_exception(error)),
        )

    def _human_motions(
        self, loaded: _Loaded, start: GroundTruthState
    ) -> tuple[tuple[HumanMotion, ...], tuple[HumanMotion, ...]]:
        stamp = start.stamp.timestamp
        requested, limited = self._human_commands
        if self._human_backend is None or not loaded.timing.human_control_due(
            stamp.tick
        ):
            return (
                tuple(replace(motion, timestamp=stamp) for motion in requested),
                tuple(replace(motion, timestamp=stamp) for motion in limited),
            )
        context = HumanContext.from_ground_truth(
            start,
            robot_radius_m=loaded.world.scene.robot.radius_m,
            human_radius_m=loaded.config.scenarios.human_profile.radius_m,
            obstacles=loaded.human_obstacles,
        )
        requested = self._human_backend.propose(context)
        validate_human_requests(
            requested,
            stamp,
            entity_ids=tuple(human.entity_id for human in start.humans),
        )
        return requested, tuple(
            replace(
                motion,
                velocity=loaded.human_limits.limit(
                    motion.velocity,
                    previous.velocity,
                    loaded.timing.human_control_dt_ns,
                ),
            )
            for motion, previous in zip(requested, limited, strict=True)
        )

    def step(self, robot_action: RobotAction) -> SimulationStepResult:
        loaded = self._require_loaded()
        metadata = self.get_metadata()
        if metadata.status != "running":
            raise SimulationError("episode is closed; reset before stepping again")
        start = self.get_ground_truth()
        require_type(robot_action, RobotAction, "robot_action")
        if (
            robot_action.entity_id != start.robot.entity_id
            or robot_action.timestamp != start.stamp.timestamp
        ):
            raise ContractError(
                "robot action must match the current robot and timestamp"
            )
        tick = start.stamp.timestamp.tick + 1
        velocity = loaded.drive.limit(
            robot_action.velocity,
            self._last_limited,
            loaded.timing.robot_control_dt_ns / NS_PER_SECOND,
        )
        try:
            human_requests, human_limits = self._human_motions(loaded, start)
        except Exception as exc:
            self._fail("human_backend_failed", tick, exc)
            raise
        requested = MotionBatch(start.stamp, robot_action, human_requests)
        limited = MotionBatch(
            start.stamp, replace(robot_action, velocity=velocity), human_limits
        )
        try:
            advance = require_type(
                self._adapter.advance(limited), AdapterStepResult, "adapter step"
            )
            truth = advance.snapshot.ground_truth
            self._validate_truth(loaded, metadata.episode, tick, truth)
            speed_bound = (
                loaded.human_limits.max_speed_mps
                * loaded.timing.robot_control_dt_ns
                / NS_PER_SECOND
            )
            if any(
                hypot(after.pose.x - before.pose.x, after.pose.y - before.pose.y)
                > speed_bound + GEOMETRY_TOLERANCE_M
                for before, after in zip(start.humans, truth.humans, strict=True)
            ):
                raise SimulationError(
                    "human displacement exceeds max speed for the control tick"
                )
            clearance = measure_clearance(loaded.collision, loaded.world.scene, truth)
            if self._human_backend is not None and any(
                distance is not None and distance < -GEOMETRY_TOLERANCE_M
                for distance in (clearance.human_wall_m, clearance.human_human_m)
            ):
                raise SimulationError(
                    "executed human footprints overlap solid geometry"
                )
            if advance.human_motion_blocked_substeps > loaded.timing.substeps_per_tick:
                raise ContractError(
                    "blocked human substeps exceed the control tick budget"
                )
            observation = Observation(truth.stamp, loaded.sensor_slots)
            reason = None
            if loaded.config.experiments.terminate_on_contact and any(
                event.kind in ("robot_wall", "robot_human")
                for event in advance.contacts
            ):
                reason = "robot_contact"
            elif self._goal_reached(loaded, truth):
                reason = "goal_reached"
            elif tick == metadata.episode.config.max_ticks:
                reason = "time_budget"
            result = SimulationStepResult(
                requested,
                limited,
                executed_motion(start, truth, advance.robot_delta_yaw),
                observation,
                truth,
                contacts=advance.contacts,
                terminated=reason in ("robot_contact", "goal_reached"),
                truncated=reason == "time_budget",
                end_reason=reason,
                clearance=clearance,
                human_motion_blocked_substeps=advance.human_motion_blocked_substeps,
            )
        except Exception as exc:
            self._fail("step_failed", tick, exc)
            raise
        self._last_limited = velocity
        self._human_commands = (human_requests, human_limits)
        self._truth, self._observation = truth, observation
        status = (
            "terminated"
            if result.terminated
            else "truncated"
            if result.truncated
            else "running"
        )
        self._metadata = replace(
            metadata,
            clocks=loaded.timing.clocks(tick),
            last_valid_tick=tick,
            status=status,
            end_reason=reason,
        )
        return result

    def stop(self) -> SimulationStepResult:
        truth = self.get_ground_truth()
        return self.step(
            RobotAction(truth.robot.entity_id, _STOP, truth.stamp.timestamp)
        )

    def get_observation(self) -> Observation:
        if self._observation is None:
            raise SimulationError("no valid observation; reset first")
        return self._observation

    def get_ground_truth(self) -> GroundTruthState:
        if self._truth is None:
            raise SimulationError("no valid ground truth; reset first")
        return self._truth

    def get_metadata(self) -> SimulationMetadata:
        if self._metadata is None:
            raise SimulationError("no episode metadata; reset first")
        return self._metadata

    def get_clearance(self) -> Clearance:
        loaded = self._require_loaded()
        return measure_clearance(
            loaded.collision, loaded.world.scene, self.get_ground_truth()
        )

    def close(self) -> None:
        if not self._closed:
            self._adapter.close()
            self._loaded = self._metadata = self._truth = self._observation = None
            self._closed = True
