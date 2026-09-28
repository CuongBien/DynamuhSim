"""Data contracts shared by the episode runner and future runtime managers."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any


class EpisodeState(str, Enum):
    PREPARE = "prepare"
    RESET = "reset"
    LOAD_SCENARIO = "load_scenario"
    WAIT_READY = "wait_ready"
    START = "start"
    RUNNING = "running"
    FINALIZE = "finalize"


class EpisodeStatus(str, Enum):
    SUCCESS = "success"
    COLLISION = "collision"
    TIMEOUT = "timeout"
    STUCK = "stuck"
    NAV_FAILURE = "nav_failure"
    SIM_FAILURE = "sim_failure"
    INVALID_EPISODE = "invalid_episode"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    heading: float


@dataclass(frozen=True)
class EpisodeContext:
    episode_dir: Path
    episode_id: str
    scenario_family: str
    seed: int
    metadata: dict[str, Any]
    scenario: dict[str, Any]
    humans: dict[str, Any]
    start_pose: Pose2D
    goal_node: str
    goal_pose: Pose2D


@dataclass(frozen=True)
class EpisodeTermination:
    status: EpisodeStatus
    reason: str


@dataclass(frozen=True)
class MonitorResult(EpisodeTermination):
    duration_sec: float = 0.0
    navigation_status: str | None = None
    navigation_status_code: int | None = None
    navigation_result_code: int | None = None
    navigation_error_msg: str = ""
    collision_occurred: bool = False
    collision_kind: str | None = None
    collision_pair: tuple[str, str] | None = None
    stuck_displacement_m: float | None = None

    @property
    def termination_reason(self) -> str:
        return self.reason


@dataclass(frozen=True)
class EpisodeResult:
    episode_id: str
    status: EpisodeStatus
    termination_reason: str
    duration_sec: float
    transitions: tuple[EpisodeState, ...]
    context: EpisodeContext | None = None
    monitor_result: MonitorResult | None = None


class EpisodeHooks:
    """Override these methods to attach a recorder in Step 3B."""

    def on_episode_prepare(self, context: EpisodeContext) -> None:
        pass

    def on_episode_start(self, context: EpisodeContext) -> None:
        pass

    def on_episode_end(self, context: EpisodeContext | None, result: EpisodeResult) -> None:
        pass
