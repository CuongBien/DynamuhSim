"""Typed ports and results shared by future adapters, controllers and orchestration."""

from dataclasses import dataclass
from typing import Literal, Protocol, TypeVar

from .actions import HumanMotionRequest, HumanMotionResult, MotionBatch, RobotAction
from .errors import ContractError
from .metadata import EpisodeMetadata
from .observations import Observation, SensorObservation
from .states import GroundTruthState, HumanContext
from .types import ROBOT_BASE_FRAME, WORLD_FRAME, Point2D
from .validation import (
    finite,
    frame_is,
    identifier,
    integer,
    require_type,
    uint64,
    unique,
)

SceneT_contra = TypeVar("SceneT_contra", contravariant=True)
PolicyConfigT_contra = TypeVar("PolicyConfigT_contra", contravariant=True)
BackendConfigT_contra = TypeVar("BackendConfigT_contra", contravariant=True)
ContactKind = Literal["robot_wall", "robot_human", "human_wall", "human_human"]


@dataclass(frozen=True, slots=True)
class ContactEvent:
    time_ns: int
    first_id: str
    second_id: str
    kind: ContactKind | None = None

    def __post_init__(self) -> None:
        integer(self.time_ns, "contact time_ns", minimum=1)
        identifier(self.first_id, "first_id")
        identifier(self.second_id, "second_id")
        if self.first_id >= self.second_id:
            raise ContractError("contact IDs must be distinct and sorted")
        if self.kind not in (
            None,
            "robot_wall",
            "robot_human",
            "human_wall",
            "human_human",
        ):
            raise ContractError("unknown contact kind")


@dataclass(frozen=True, slots=True)
class AdapterSnapshot:
    ground_truth: GroundTruthState
    sensor_samples: tuple[SensorObservation, ...] = ()
    frame_ids: tuple[str, ...] = (WORLD_FRAME, ROBOT_BASE_FRAME)

    def __post_init__(self) -> None:
        require_type(self.ground_truth, GroundTruthState, "ground_truth")
        require_type(self.frame_ids, tuple, "frame_ids")
        frames = tuple(identifier(frame, "frame_id") for frame in self.frame_ids)
        unique(frames, "frame_ids")
        if not {WORLD_FRAME, ROBOT_BASE_FRAME}.issubset(frames):
            raise ContractError("frame_ids must include world and robot_base")
        require_type(self.sensor_samples, tuple, "sensor_samples")
        for sample in self.sensor_samples:
            require_type(sample, SensorObservation, "sensor_sample")
            if sample.frame_id not in frames:
                raise ContractError(f"unregistered sensor frame {sample.frame_id!r}")
            if sample.capture_time_ns > self.ground_truth.stamp.timestamp.time_ns:
                raise ContractError(
                    "sensor sample cannot be captured after the snapshot"
                )
        unique(
            ((sample.sensor_id, sample.sample_index) for sample in self.sensor_samples),
            "samples",
        )


@dataclass(frozen=True, slots=True)
class AdapterStepResult:
    snapshot: AdapterSnapshot
    robot_delta_yaw: float
    contacts: tuple[ContactEvent, ...] = ()

    def __post_init__(self) -> None:
        require_type(self.snapshot, AdapterSnapshot, "snapshot")
        finite(self.robot_delta_yaw, "robot_delta_yaw")
        require_type(self.contacts, tuple, "contacts")
        for contact in self.contacts:
            require_type(contact, ContactEvent, "contact")


@dataclass(frozen=True, slots=True)
class SimulationStepResult:
    requested: MotionBatch
    limited: MotionBatch
    executed: MotionBatch
    observation: Observation
    ground_truth: GroundTruthState
    sensor_samples: tuple[SensorObservation, ...] = ()
    contacts: tuple[ContactEvent, ...] = ()
    terminated: bool = False
    truncated: bool = False
    end_reason: str | None = None

    def __post_init__(self) -> None:
        require_type(self.requested, MotionBatch, "requested")
        for batch in (self.limited, self.executed):
            require_type(batch, MotionBatch, "motion batch")
            if (
                batch.stamp != self.requested.stamp
                or batch.entity_ids != self.requested.entity_ids
            ):
                raise ContractError("motion stages must share metadata and entity IDs")

        require_type(self.observation, Observation, "observation")
        require_type(self.ground_truth, GroundTruthState, "ground_truth")
        start_stamp, end_stamp = self.requested.stamp, self.ground_truth.stamp
        if (
            self.observation.stamp != end_stamp
            or start_stamp.episode_id != end_stamp.episode_id
            or start_stamp.episode_seed != end_stamp.episode_seed
        ):
            raise ContractError(
                "step records must share episode identity and end snapshot metadata"
            )
        start, end = start_stamp.timestamp, end_stamp.timestamp
        dt_ns = end.time_ns - start.time_ns
        if (
            end.tick != start.tick + 1
            or dt_ns <= 0
            or start.time_ns != start.tick * dt_ns
        ):
            raise ContractError(
                "step must advance exactly one control tick at the same cadence"
            )
        if self.requested.entity_ids != self.ground_truth.entity_ids:
            raise ContractError(
                "motion stages and ground truth must describe the same entities"
            )

        require_type(self.sensor_samples, tuple, "sensor_samples")
        for sample in self.sensor_samples:
            require_type(sample, SensorObservation, "sensor_sample")
            if not start.time_ns < sample.available_time_ns <= end.time_ns:
                raise ContractError(
                    "published sensor sample must become available in this step"
                )
        unique(
            ((sample.sensor_id, sample.sample_index) for sample in self.sensor_samples),
            "samples",
        )

        require_type(self.contacts, tuple, "contacts")
        for contact in self.contacts:
            require_type(contact, ContactEvent, "contact")
            if not start.time_ns < contact.time_ns <= end.time_ns:
                raise ContractError("contact must occur in this step")

        require_type(self.terminated, bool, "terminated")
        require_type(self.truncated, bool, "truncated")
        if self.terminated and self.truncated:
            raise ContractError("step cannot be both terminated and truncated")
        if self.terminated or self.truncated:
            identifier(self.end_reason, "end_reason")
        elif self.end_reason is not None:
            raise ContractError("a running step cannot have an end_reason")

    @property
    def human_motion_results(self) -> tuple[HumanMotionResult, ...]:
        return tuple(
            HumanMotionResult(requested, limited, executed)
            for requested, limited, executed in zip(
                self.requested.humans,
                self.limited.humans,
                self.executed.humans,
                strict=True,
            )
        )


@dataclass(frozen=True, slots=True)
class SensorSeed:
    sensor_id: str
    seed: int

    def __post_init__(self) -> None:
        identifier(self.sensor_id, "sensor_id")
        uint64(self.seed, "seed")


@dataclass(frozen=True, slots=True)
class RobotMission:
    entity_id: str
    goal: Point2D

    def __post_init__(self) -> None:
        identifier(self.entity_id, "entity_id")
        frame_is(require_type(self.goal, Point2D, "goal").frame_id, WORLD_FRAME, "goal")


class SimulatorAdapter(Protocol[SceneT_contra]):
    def reset(
        self,
        scene: SceneT_contra,
        metadata: EpisodeMetadata,
        engine_seed: int,
        sensor_seeds: tuple[SensorSeed, ...],
    ) -> AdapterSnapshot: ...

    def advance(self, limited: MotionBatch) -> AdapterStepResult: ...


class RobotPolicy(Protocol[PolicyConfigT_contra]):
    def reset(
        self, mission: RobotMission, config: PolicyConfigT_contra, seed: int
    ) -> None: ...

    def propose(self, observation: Observation) -> RobotAction: ...


class HumanControllerBackend(Protocol[BackendConfigT_contra]):
    def reset(
        self, config: BackendConfigT_contra, seed: int, entity_ids: tuple[str, ...]
    ) -> None: ...

    def propose(self, context: HumanContext) -> tuple[HumanMotionRequest, ...]: ...
