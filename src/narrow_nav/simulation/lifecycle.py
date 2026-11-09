"""Engine-independent orchestration of one loaded world and its episodes."""

from dataclasses import dataclass, replace
from hashlib import sha256
from importlib.metadata import version
from math import hypot
from pathlib import Path
from platform import python_version
from subprocess import CalledProcessError, check_output
from typing import Protocol
from uuid import uuid4

from omegaconf import DictConfig, ListConfig, OmegaConf

from narrow_nav.core.actions import HumanMotion, MotionBatch, RobotAction
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.metadata import EpisodeConfig, EpisodeMetadata
from narrow_nav.core.observations import Observation, SensorSlot
from narrow_nav.core.protocols import (
    AdapterSnapshot,
    SensorSeed,
    SimulationStepResult,
    SimulatorAdapter,
)
from narrow_nav.core.seed import derive_seed
from narrow_nav.core.states import CleanDepthFrame, GroundTruthState
from narrow_nav.core.types import NS_PER_SECOND, Twist2D, Velocity2D
from narrow_nav.core.validation import positive, require_type, uint64, unique

from .backend import DepthStreamMetadata, SimulationError, SimulationMetadata
from .collision_monitor import Clearance, measure_clearance
from .configuration import drive_from_config, timing_from_config
from .depth_config import DEPTH_FRAME_ID, DEPTH_MODEL_VERSION, DEPTH_SENSOR_ID
from .robot_controller import DriveProfile
from .state_extractor import executed_motion
from .timing import Timing
from .world_builder import MJCF_ROOT, BuiltWorld, build_world
from .world_geometry import WorldCollision

_UNAVAILABLE_SENSORS = ("depth", "imu", "lidar", "odometry")
_STOP = Twist2D(0, 0, 0)


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

    def __init__(self, adapter: RuntimeAdapter, *, mjcf_root: Path = MJCF_ROOT) -> None:
        self._adapter = adapter
        self._mjcf_root = mjcf_root.resolve()
        self._run_id = uuid4().hex
        self._ordinal = 0
        self._closed = False
        self._loaded: _Loaded | None = None
        self._metadata: SimulationMetadata | None = None
        self._truth: GroundTruthState | None = None
        self._observation: Observation | None = None
        self._clean_depth: tuple[CleanDepthFrame, ...] = ()
        self._last_limited = _STOP

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
        OmegaConf.set_readonly(resolved, True)
        self._loaded = _Loaded(
            resolved, episode, built, timing, drive, collision, slots, goal_tolerance
        )
        self._metadata = self._truth = self._observation = None
        self._clean_depth = ()
        self._last_limited = _STOP

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
        profile = loaded.world.depth_camera
        depth_stream = (
            DepthStreamMetadata(
                profile,
                derive_seed(seed, "sensor", "sensor.depth"),
                DEPTH_MODEL_VERSION,
                DEPTH_SENSOR_ID,
                DEPTH_FRAME_ID,
            )
            if profile is not None
            else None
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
            unavailable_sensors=tuple(
                sensor
                for sensor in _UNAVAILABLE_SENSORS
                if sensor != DEPTH_SENSOR_ID or depth_stream is None
            ),
            depth_stream=depth_stream,
        )
        self._ordinal += 1
        self._metadata = metadata
        self._truth = self._observation = None
        self._clean_depth = ()
        self._last_limited = _STOP
        try:
            seeds = (
                (SensorSeed(DEPTH_SENSOR_ID, depth_stream.seed),)
                if depth_stream
                else ()
            )
            snapshot = self._adapter.reset(
                loaded.world, header, metadata.engine_seed, seeds
            )
            truth = snapshot.ground_truth
            if self._goal_reached(loaded, truth):
                raise SimulationError("initial state already satisfies the goal")
            observation = self._make_observation(loaded, snapshot, -1)
        except Exception as exc:
            self._fail("reset_failed", 0, exc)
            raise
        self._truth, self._observation = truth, observation
        self._clean_depth = snapshot.clean_depth
        self._metadata = replace(metadata, last_valid_tick=0)
        return observation

    def _make_observation(
        self, loaded: _Loaded, snapshot: AdapterSnapshot, start_time_ns: int
    ) -> Observation:
        stamp = snapshot.ground_truth.stamp
        profile = loaded.world.depth_camera
        if profile is None:
            return Observation(stamp, loaded.sensor_slots)
        period = profile.period_ns
        indices = range(
            start_time_ns // period + 1, stamp.timestamp.time_ns // period + 1
        )
        samples, clean_frames = snapshot.sensor_samples, snapshot.clean_depth
        if len(samples) != len(indices) or len(clean_frames) != len(indices):
            raise ContractError("depth capture count does not match its schedule")
        shape = (profile.calibration.height, profile.calibration.width)
        source = "d435i_noisy" if profile.noise.enabled else "d435i_ideal"
        for index, sample, clean in zip(indices, samples, clean_frames, strict=True):
            if (
                sample.sensor_id != DEPTH_SENSOR_ID
                or sample.source != source
                or sample.frame_id != clean.frame_id
                or sample.frame_id != DEPTH_FRAME_ID
                or sample.sample_index != clean.sample_index
                or sample.sample_index != index
                or sample.capture_time_ns != clean.capture_time_ns
                or sample.capture_time_ns != index * period
                or sample.available_time_ns != sample.capture_time_ns
                or sample.payload is None
                or sample.payload.shape != shape
                or sample.payload.valid_mask is None
                or clean.payload.shape != shape
                or sample.payload is clean.payload
            ):
                raise ContractError("invalid D435i capture pair")
        if not samples:
            return Observation(stamp, self.get_observation().sensors)
        return Observation(
            stamp,
            tuple(
                SensorSlot(DEPTH_SENSOR_ID, "valid", samples[-1])
                if slot.sensor_id == DEPTH_SENSOR_ID
                else slot
                for slot in loaded.sensor_slots
            ),
        )

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
        humans = tuple(
            HumanMotion(human.entity_id, Velocity2D(0, 0), start.stamp.timestamp)
            for human in start.humans
        )
        requested = MotionBatch(start.stamp, robot_action, humans)
        limited = MotionBatch(
            start.stamp, replace(robot_action, velocity=velocity), humans
        )
        try:
            advance = self._adapter.advance(limited)
            truth = advance.snapshot.ground_truth
            observation = self._make_observation(
                loaded, advance.snapshot, start.stamp.timestamp.time_ns
            )
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
                sensor_samples=advance.snapshot.sensor_samples,
                contacts=advance.contacts,
                terminated=reason in ("robot_contact", "goal_reached"),
                truncated=reason == "time_budget",
                end_reason=reason,
            )
        except Exception as exc:
            self._fail("step_failed", tick, exc)
            raise
        self._last_limited = velocity
        self._truth, self._observation = truth, observation
        self._clean_depth = advance.snapshot.clean_depth
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

    def get_clean_depth(self) -> tuple[CleanDepthFrame, ...]:
        """Privileged frames captured at reset or during the last successful step."""
        self.get_ground_truth()
        return self._clean_depth

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
            self._clean_depth = ()
            self._closed = True
