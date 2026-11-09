from dataclasses import replace

import pytest

from narrow_nav.core.actions import HumanMotionRequest, MotionBatch, RobotAction
from narrow_nav.core.errors import ContractError
from narrow_nav.core.metadata import SnapshotMetadata
from narrow_nav.core.observations import (
    Observation,
    SensorObservation,
    SensorPayload,
    SensorSlot,
)
from narrow_nav.core.protocols import SimulationStepResult
from narrow_nav.core.states import GroundTruthState, HumanState, RobotState
from narrow_nav.core.types import Point2D, Pose2D, Timestamp, Twist2D, Velocity2D


def test_step_keeps_motion_stages_and_checks_alignment():
    start = SnapshotMetadata("run:0", 42, Timestamp(0, 0))
    end = replace(start, timestamp=Timestamp(1, 10))
    robot = RobotAction("robot", Twist2D(1, 0, 0), start.timestamp)
    human = HumanMotionRequest("human", Velocity2D(1, 0), start.timestamp)
    requested = MotionBatch(start, robot, (human,))
    limited = replace(requested, humans=(replace(human, velocity=Velocity2D(0.5, 0)),))
    executed = replace(limited, robot=replace(robot, velocity=Twist2D(0.3, 0.1, 0)))
    truth = GroundTruthState(
        end,
        RobotState("robot", Pose2D(0, 0, 0), executed.robot.velocity, end.timestamp),
        (
            HumanState(
                "human", Pose2D(1, 0, 0), Velocity2D(0, 0), Point2D(2, 0), end.timestamp
            ),
        ),
    )
    result = SimulationStepResult(requested, limited, executed, Observation(end), truth)
    assert result.clearance is None and result.human_motion_blocked_substeps == 0
    assert result.human_motion_results[0].limited.velocity.linear_x == 0.5
    with pytest.raises(ContractError, match="metadata"):
        replace(result, observation=Observation(start))


def test_observation_rejects_sensor_sample_before_availability():
    sample = SensorObservation(
        "lidar",
        "lidar",
        "lidar_frame",
        0,
        0,
        10,
        payload=SensorPayload((2,), (1.0, 2.0)),
    )
    slot = SensorSlot("lidar", "valid", sample)
    stamp = SnapshotMetadata("run:0", 42, Timestamp(0, 0))
    with pytest.raises(ContractError, match="future"):
        Observation(stamp, (slot,))
    assert Observation(replace(stamp, timestamp=Timestamp(1, 10)), (slot,)).sensors == (
        slot,
    )
