"""Physical state and the context reserved for human behaviour models."""

from dataclasses import asdict, dataclass
from typing import Any, Self

from .errors import ContractError
from .geometry import body_velocity_to_world
from .metadata import SnapshotMetadata
from .observations import SensorPayload
from .types import (
    ROBOT_BASE_FRAME,
    WORLD_FRAME,
    Point2D,
    Pose2D,
    Timestamp,
    Twist2D,
    Velocity2D,
)
from .validation import (
    frame_is,
    identifier,
    integer,
    ordered_ids,
    positive,
    require_type,
    unique,
)


@dataclass(frozen=True, slots=True)
class RobotState:
    entity_id: str
    pose: Pose2D
    velocity: Twist2D
    timestamp: Timestamp

    def __post_init__(self) -> None:
        identifier(self.entity_id, "entity_id")
        frame_is(require_type(self.pose, Pose2D, "pose").frame_id, WORLD_FRAME, "pose")
        frame_is(
            require_type(self.velocity, Twist2D, "velocity").frame_id,
            ROBOT_BASE_FRAME,
            "velocity",
        )
        require_type(self.timestamp, Timestamp, "timestamp")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(
            entity_id=data["entity_id"],
            pose=Pose2D(**data["pose"]),
            velocity=Twist2D(**data["velocity"]),
            timestamp=Timestamp(**data["timestamp"]),
        )


@dataclass(frozen=True, slots=True)
class HumanState:
    entity_id: str
    pose: Pose2D
    velocity: Velocity2D
    goal: Point2D
    timestamp: Timestamp

    def __post_init__(self) -> None:
        identifier(self.entity_id, "entity_id")
        frame_is(require_type(self.pose, Pose2D, "pose").frame_id, WORLD_FRAME, "pose")
        frame_is(
            require_type(self.velocity, Velocity2D, "velocity").frame_id,
            WORLD_FRAME,
            "velocity",
        )
        frame_is(require_type(self.goal, Point2D, "goal").frame_id, WORLD_FRAME, "goal")
        require_type(self.timestamp, Timestamp, "timestamp")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(
            entity_id=data["entity_id"],
            pose=Pose2D(**data["pose"]),
            velocity=Velocity2D(**data["velocity"]),
            goal=Point2D(**data["goal"]),
            timestamp=Timestamp(**data["timestamp"]),
        )


@dataclass(frozen=True, slots=True)
class GroundTruthState:
    stamp: SnapshotMetadata
    robot: RobotState
    humans: tuple[HumanState, ...] = ()

    def __post_init__(self) -> None:
        require_type(self.stamp, SnapshotMetadata, "stamp")
        require_type(self.robot, RobotState, "robot")
        _validate_humans(self.humans, self.stamp)
        unique(self.entity_ids, "entity IDs")
        if self.robot.timestamp != self.stamp.timestamp:
            raise ContractError("robot timestamp must match ground truth stamp")

    @property
    def entity_ids(self) -> tuple[str, ...]:
        return (self.robot.entity_id, *(human.entity_id for human in self.humans))


@dataclass(frozen=True, slots=True)
class CleanDepthFrame:
    """Privileged axial metric depth in the optical frame at capture time."""

    sample_index: int
    capture_time_ns: int
    frame_id: str
    payload: SensorPayload

    def __post_init__(self) -> None:
        integer(self.sample_index, "sample_index")
        integer(self.capture_time_ns, "capture_time_ns")
        identifier(self.frame_id, "frame_id")
        payload = require_type(self.payload, SensorPayload, "clean depth payload")
        if len(payload.shape) != 2 or payload.valid_mask is None:
            raise ContractError("clean depth requires a two-dimensional image and mask")


def _validate_humans(humans: tuple[HumanState, ...], stamp: SnapshotMetadata) -> None:
    require_type(humans, tuple, "humans")
    for human in humans:
        require_type(human, HumanState, "human")
        if human.timestamp != stamp.timestamp:
            raise ContractError("human timestamp must match snapshot stamp")
    ordered_ids(tuple(human.entity_id for human in humans), "humans")


@dataclass(frozen=True, slots=True)
class Obstacle:
    entity_id: str
    polygons: tuple[tuple[Point2D, ...], ...]

    def __post_init__(self) -> None:
        identifier(self.entity_id, "entity_id")
        require_type(self.polygons, tuple, "polygons")
        if not self.polygons:
            raise ContractError("obstacle must have at least one polygon")
        for polygon in self.polygons:
            require_type(polygon, tuple, "polygon")
            if len(polygon) < 3:
                raise ContractError("polygon must have at least three vertices")
            for vertex in polygon:
                frame_is(
                    require_type(vertex, Point2D, "vertex").frame_id,
                    WORLD_FRAME,
                    "vertex",
                )
            unique(polygon, "polygon vertices")


@dataclass(frozen=True, slots=True)
class HumanContext:
    """Physical projection for synthetic human behaviour, separate from robot observations."""

    stamp: SnapshotMetadata
    robot_entity_id: str
    robot_pose: Pose2D
    robot_world_velocity: Velocity2D
    robot_radius_m: float
    human_radius_m: float
    humans: tuple[HumanState, ...] = ()
    obstacles: tuple[Obstacle, ...] = ()

    def __post_init__(self) -> None:
        require_type(self.stamp, SnapshotMetadata, "stamp")
        identifier(self.robot_entity_id, "robot_entity_id")
        pose = require_type(self.robot_pose, Pose2D, "robot_pose")
        frame_is(pose.frame_id, WORLD_FRAME, "robot_pose")
        velocity = require_type(
            self.robot_world_velocity, Velocity2D, "robot_world_velocity"
        )
        frame_is(velocity.frame_id, WORLD_FRAME, "robot_world_velocity")
        positive(self.robot_radius_m, "robot_radius_m")
        positive(self.human_radius_m, "human_radius_m")
        _validate_humans(self.humans, self.stamp)
        require_type(self.obstacles, tuple, "obstacles")
        ids = [self.robot_entity_id, *(human.entity_id for human in self.humans)]
        ids.extend(
            require_type(obstacle, Obstacle, "obstacle").entity_id
            for obstacle in self.obstacles
        )
        unique(ids, "context entity IDs")

    @classmethod
    def from_ground_truth(
        cls,
        truth: GroundTruthState,
        *,
        robot_radius_m: float,
        human_radius_m: float,
        obstacles: tuple[Obstacle, ...] = (),
    ) -> Self:
        robot = truth.robot
        return cls(
            truth.stamp,
            robot.entity_id,
            robot.pose,
            body_velocity_to_world(robot.velocity, robot.pose),
            robot_radius_m,
            human_radius_m,
            truth.humans,
            obstacles,
        )
