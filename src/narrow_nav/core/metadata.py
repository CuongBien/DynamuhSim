"""Episode identity, timing and reproducibility metadata."""

import re
from dataclasses import dataclass

from .errors import ContractError
from .types import SCHEMA_VERSION, Timestamp
from .validation import episode_id, identifier, integer, require_type, run_id, uint64


def _validate_version(value: int) -> None:
    if integer(value, "schema_version") != SCHEMA_VERSION:
        raise ContractError(f"schema_version must be {SCHEMA_VERSION}")


@dataclass(frozen=True, slots=True)
class EpisodeConfig:
    scenario_id: str
    episode_seed: int
    control_dt_ns: int
    physics_dt_ns: int
    duration_ns: int
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        _validate_version(self.schema_version)
        identifier(self.scenario_id, "scenario_id")
        uint64(self.episode_seed)
        integer(self.control_dt_ns, "control_dt_ns", minimum=1)
        integer(self.physics_dt_ns, "physics_dt_ns", minimum=1)
        integer(self.duration_ns, "duration_ns", minimum=1)
        if self.control_dt_ns % self.physics_dt_ns:
            raise ContractError("control_dt_ns must be divisible by physics_dt_ns")
        if self.duration_ns % self.control_dt_ns:
            raise ContractError("duration_ns must be divisible by control_dt_ns")

    @property
    def max_ticks(self) -> int:
        return self.duration_ns // self.control_dt_ns

    def timestamp(self, tick: int) -> Timestamp:
        integer(tick, "tick")
        if tick > self.max_ticks:
            raise ContractError("tick exceeds the episode budget")
        return Timestamp(tick, tick * self.control_dt_ns)


@dataclass(frozen=True, slots=True)
class SnapshotMetadata:
    episode_id: str
    episode_seed: int
    timestamp: Timestamp
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        _validate_version(self.schema_version)
        episode_id(self.episode_id)
        uint64(self.episode_seed)
        require_type(self.timestamp, Timestamp, "timestamp")


@dataclass(frozen=True, slots=True)
class EpisodeMetadata:
    run_id: str
    episode_ordinal: int
    config: EpisodeConfig
    config_sha256: str
    git_commit: str
    git_dirty: bool

    def __post_init__(self) -> None:
        run_id(self.run_id)
        integer(self.episode_ordinal, "episode_ordinal")
        require_type(self.config, EpisodeConfig, "config")
        require_type(self.git_dirty, bool, "git_dirty")
        if not isinstance(self.config_sha256, str) or not re.fullmatch(
            r"[0-9a-f]{64}", self.config_sha256
        ):
            raise ContractError("config_sha256 must be a SHA-256 hex digest")
        if not isinstance(self.git_commit, str) or not re.fullmatch(
            r"(?:[0-9a-f]{40}|[0-9a-f]{64})", self.git_commit
        ):
            raise ContractError("git_commit must be a full Git object ID")

    @property
    def episode_id(self) -> str:
        return f"{self.run_id}:{self.episode_ordinal}"

    def stamp(self, tick: int) -> SnapshotMetadata:
        return SnapshotMetadata(
            self.episode_id,
            self.config.episode_seed,
            self.config.timestamp(tick),
            self.config.schema_version,
        )
