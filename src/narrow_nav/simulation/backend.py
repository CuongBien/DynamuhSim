"""Public runtime protocol and episode metadata."""

from dataclasses import dataclass
from typing import Literal, Protocol

from omegaconf import DictConfig

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.metadata import EpisodeMetadata
from narrow_nav.core.observations import Observation
from narrow_nav.core.protocols import SimulationStepResult
from narrow_nav.core.states import CleanDepthFrame, GroundTruthState

from .depth_config import DepthCameraConfig
from .timing import ClockState, Timing

EpisodeStatus = Literal["running", "terminated", "truncated", "failed"]


class SimulationError(RuntimeError):
    """The runtime cannot perform the operation or complete a physics tick."""


@dataclass(frozen=True, slots=True)
class DepthStreamMetadata:
    profile: DepthCameraConfig
    seed: int
    model_version: str
    sensor_id: str
    frame_id: str
    seed_scope: str = "sensor.depth"
    rng: str = "numpy.PCG64"
    unit: str = "m"
    invalid_value: float = 0.0
    latency_ns: int = 0


@dataclass(frozen=True, slots=True)
class SimulationMetadata:
    episode: EpisodeMetadata
    resolved_config: str
    model_sha256: str
    mjcf_source_sha256: tuple[tuple[str, str], ...]
    engine_version: str
    python_version: str
    project_version: str
    engine_seed: int
    timing: Timing
    clocks: ClockState
    unavailable_sensors: tuple[str, ...]
    human_motion_mode: Literal["stationary"] = "stationary"
    status: EpisodeStatus = "running"
    end_reason: str | None = None
    last_valid_tick: int | None = None
    failure: str | None = None
    failure_tick: int | None = None
    depth_stream: DepthStreamMetadata | None = None


class SimulationBackend(Protocol):
    def load(self, config: DictConfig) -> None: ...

    def reset(
        self, seed: int | None = None, scenario: str | None = None
    ) -> Observation: ...

    def step(self, robot_action: RobotAction) -> SimulationStepResult: ...

    def stop(self) -> SimulationStepResult: ...

    def get_observation(self) -> Observation: ...

    def get_ground_truth(self) -> GroundTruthState: ...

    def get_clean_depth(self) -> tuple[CleanDepthFrame, ...]: ...

    def get_metadata(self) -> SimulationMetadata: ...

    def close(self) -> None: ...
