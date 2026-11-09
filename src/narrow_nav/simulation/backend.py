"""Public runtime protocol and episode metadata."""

from dataclasses import dataclass
from typing import Literal, Protocol

from omegaconf import DictConfig

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.metadata import EpisodeMetadata
from narrow_nav.core.observations import Observation
from narrow_nav.core.protocols import ContactKind, SimulationStepResult
from narrow_nav.core.states import GroundTruthState

from .timing import ClockState, Timing

EpisodeStatus = Literal["running", "terminated", "truncated", "failed"]


class SimulationError(RuntimeError):
    """The runtime cannot perform the operation or complete a physics tick."""


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
    human_motion_mode: Literal["stationary", "kinematic"] = "stationary"
    status: EpisodeStatus = "running"
    end_reason: str | None = None
    last_valid_tick: int | None = None
    failure: str | None = None
    failure_tick: int | None = None
    human_backend_id: str | None = None
    human_backend_version: str | None = None
    human_backend_seed: int | None = None
    not_observable_solver: tuple[ContactKind, ...] = ()
    failure_traceback: str | None = None


class SimulationBackend(Protocol):
    def load(self, config: DictConfig) -> None: ...

    def reset(
        self, seed: int | None = None, scenario: str | None = None
    ) -> Observation: ...

    def step(self, robot_action: RobotAction) -> SimulationStepResult: ...

    def stop(self) -> SimulationStepResult: ...

    def get_observation(self) -> Observation: ...

    def get_ground_truth(self) -> GroundTruthState: ...

    def get_metadata(self) -> SimulationMetadata: ...

    def close(self) -> None: ...
