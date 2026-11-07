import json
from math import inf, nan

import pytest

from narrow_nav.core.errors import ContractError
from narrow_nav.core.metadata import SnapshotMetadata
from narrow_nav.core.states import GroundTruthState, HumanState, RobotState
from narrow_nav.core.types import Point2D, Pose2D, Timestamp, Twist2D, Velocity2D


def test_states_round_trip_through_json():
    timestamp = Timestamp(1, 50_000_000)
    robot = RobotState("robot", Pose2D(1, 2, 0), Twist2D(0.4, 0, 0.2), timestamp)
    human = HumanState(
        "human", Pose2D(3, 2, 0), Velocity2D(0.2, 0), Point2D(4, 2), timestamp
    )
    for state in (robot, human):
        assert type(state).from_dict(json.loads(json.dumps(state.to_dict()))) == state


@pytest.mark.parametrize("number", [nan, inf, -inf])
def test_non_finite_state_is_rejected(number):
    with pytest.raises(ContractError, match="finite"):
        Pose2D(number, 0, 0)


def test_state_rejects_bad_frames_timestamps_and_duplicate_ids():
    timestamp = Timestamp(0, 0)
    with pytest.raises(ContractError, match="robot_base"):
        RobotState("robot", Pose2D(0, 0, 0), Twist2D(0, 0, 0, "world"), timestamp)
    with pytest.raises(ContractError, match="time_ns"):
        Timestamp(0, 1)

    robot = RobotState("robot", Pose2D(0, 0, 0), Twist2D(0, 0, 0), timestamp)
    human = HumanState(
        "robot", Pose2D(1, 0, 0), Velocity2D(0, 0), Point2D(2, 0), timestamp
    )
    with pytest.raises(ContractError, match="duplicate"):
        GroundTruthState(SnapshotMetadata("run:0", 42, timestamp), robot, (human,))
