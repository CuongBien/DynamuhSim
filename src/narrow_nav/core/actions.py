"""Robot and human motion at the requested, limited and executed stages."""

from dataclasses import dataclass

from .errors import ContractError
from .metadata import SnapshotMetadata
from .types import ROBOT_BASE_FRAME, WORLD_FRAME, Timestamp, Twist2D, Velocity2D
from .validation import frame_is, identifier, ordered_ids, require_type, unique


@dataclass(frozen=True, slots=True)
class RobotAction:
    entity_id: str
    velocity: Twist2D
    timestamp: Timestamp

    def __post_init__(self) -> None:
        identifier(self.entity_id, "entity_id")
        frame_is(
            require_type(self.velocity, Twist2D, "velocity").frame_id,
            ROBOT_BASE_FRAME,
            "velocity",
        )
        require_type(self.timestamp, Timestamp, "timestamp")


@dataclass(frozen=True, slots=True)
class HumanMotion:
    entity_id: str
    velocity: Velocity2D
    timestamp: Timestamp

    def __post_init__(self) -> None:
        identifier(self.entity_id, "entity_id")
        frame_is(
            require_type(self.velocity, Velocity2D, "velocity").frame_id,
            WORLD_FRAME,
            "velocity",
        )
        require_type(self.timestamp, Timestamp, "timestamp")


HumanMotionRequest = HumanMotion


def validate_human_requests(
    requests: tuple[HumanMotionRequest, ...],
    timestamp: Timestamp,
    *,
    entity_ids: tuple[str, ...] | None = None,
) -> None:
    """Check a batch against its tick and, at the backend boundary, its roster."""
    require_type(requests, tuple, "human motions")
    for request in requests:
        require_type(request, HumanMotion, "human motion")
        if request.timestamp != timestamp:
            raise ContractError("motion timestamps must match batch stamp")
    ids = tuple(request.entity_id for request in requests)
    ordered_ids(ids, "humans")
    if entity_ids is not None and ids != entity_ids:
        raise ContractError("human motions must match the current human IDs")


@dataclass(frozen=True, slots=True)
class HumanMotionResult:
    requested: HumanMotionRequest
    limited: HumanMotion
    executed: HumanMotion

    def __post_init__(self) -> None:
        requested = require_type(self.requested, HumanMotion, "requested")
        for motion in (self.limited, self.executed):
            require_type(motion, HumanMotion, "motion")
            if (
                motion.entity_id != requested.entity_id
                or motion.timestamp != requested.timestamp
            ):
                raise ContractError(
                    "human motion stages must refer to the same entity and tick"
                )


@dataclass(frozen=True, slots=True)
class MotionBatch:
    stamp: SnapshotMetadata
    robot: RobotAction
    humans: tuple[HumanMotion, ...] = ()

    def __post_init__(self) -> None:
        require_type(self.stamp, SnapshotMetadata, "stamp")
        require_type(self.robot, RobotAction, "robot")
        validate_human_requests(self.humans, self.stamp.timestamp)
        unique(self.entity_ids, "entity IDs")
        if self.robot.timestamp != self.stamp.timestamp:
            raise ContractError("motion timestamps must match batch stamp")

    @property
    def entity_ids(self) -> tuple[str, ...]:
        return (self.robot.entity_id, *(human.entity_id for human in self.humans))
