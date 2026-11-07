import pytest

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.types import Twist2D
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine


@pytest.mark.parametrize(
    ("expected", "overrides"),
    [
        ("robot_wall", ("scenarios.robot.initial_pose=[4.2,-0.2,0]",)),
        (
            "robot_human",
            (
                "scenarios.robot.initial_pose=[-1,0,0]",
                "scenarios.humans.0.initial_pose=[0,0,0]",
            ),
        ),
    ],
)
def test_native_contact_classification_and_clearance_are_separate(expected, overrides):
    engine = MuJoCoEngine()
    engine.load(compose_config(overrides=overrides))
    engine.reset()
    assert engine.get_clearance().robot_wall_m > 0
    assert engine.get_clearance().robot_human_m > 0
    for _ in range(65):
        robot = engine.get_ground_truth().robot
        result = engine.step(
            RobotAction(robot.entity_id, Twist2D(0.65, 0, 0), robot.timestamp)
        )
        if result.terminated:
            break
        assert not result.contacts
    assert result.terminated and result.end_reason == "robot_contact"
    assert {event.kind for event in result.contacts} == {expected}
    time_start = result.requested.stamp.timestamp.time_ns
    time_end = result.ground_truth.stamp.timestamp.time_ns
    assert all(time_start < event.time_ns <= time_end for event in result.contacts)
    assert len(
        {(event.time_ns, event.first_id, event.second_id) for event in result.contacts}
    ) == len(result.contacts)
    engine.reset()
    assert not engine.stop().contacts
    engine.close()
