import pytest

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.types import Twist2D
from narrow_nav.humans.registry import create_human_backend
from narrow_nav.simulation import SimulationError
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine


@pytest.fixture
def engine():
    runtime = MuJoCoEngine(human_backend=create_human_backend("scripted"))
    yield runtime
    runtime.close()


@pytest.mark.parametrize(
    ("contact_kind", "overrides"),
    [
        ("robot_wall", ("scenarios.robot.initial_pose=[4.2,-0.2,0]",)),
        (
            "robot_human",
            (
                "scenarios.robot.initial_pose=[-1,0,0]",
                "scenarios.humans.0.initial_pose=[0.1,0,0]",
                "scenarios.humans.0.goal=[-3,0]",
            ),
        ),
    ],
)
def test_native_contact_keeps_substep_events_and_separate_clearance(
    engine, contact_kind, overrides
):
    engine.load(compose_config(overrides=overrides))
    engine.reset()
    for _ in range(65):
        robot = engine.get_ground_truth().robot
        step = engine.step(
            RobotAction(robot.entity_id, Twist2D(0.65, 0, 0), robot.timestamp)
        )
        if step.terminated:
            break
    assert step.terminated and step.end_reason == "robot_contact"
    assert {event.kind for event in step.contacts} == {contact_kind}
    lo = step.requested.stamp.timestamp.time_ns
    hi = step.ground_truth.stamp.timestamp.time_ns
    assert any(event.time_ns < hi for event in step.contacts)
    assert all(
        lo < event.time_ns <= hi and event.time_ns % 2_000_000 == 0
        for event in step.contacts
    )
    assert step.clearance == engine.get_clearance()
    assert engine.get_metadata().not_observable_solver == ("human_wall", "human_human")


def test_blocked_wall_substep_keeps_three_measurably_different_motion_stages(engine):
    engine.load(
        compose_config(
            overrides=(
                "worlds=blind_corner",
                "scenarios=blind_corner",
                "scenarios.human_profile.max_speed_mps=0.45",
            )
        )
    )
    engine.reset()
    for _ in range(150):
        step = engine.stop()
        assert step.clearance.human_wall_m >= 0
        if step.human_motion_blocked_substeps:
            break
    assert 0 < step.human_motion_blocked_substeps <= 25
    assert not step.contacts
    (motion,) = step.human_motion_results
    assert (
        abs(motion.requested.velocity.linear_x)
        > abs(motion.limited.velocity.linear_x)
        > abs(motion.executed.velocity.linear_x)
    )
    assert motion.limited.velocity.linear_x - motion.executed.velocity.linear_x < -0.005


def test_two_humans_do_not_pass_through_each_other_or_emit_solver_contact(engine):
    engine.load(
        compose_config(
            overrides=(
                "scenarios.robot.initial_pose=[-4,-0.6,0]",
                (
                    "scenarios.humans=[{id:human_a,initial_pose:[-1,0,0],goal:[2,0]},"
                    "{id:human_b,initial_pose:[1,0,0],goal:[-2,0]}]"
                ),
            )
        )
    )
    engine.reset()
    for _ in range(60):
        step = engine.stop()
        assert step.clearance.human_human_m >= 0
        if step.human_motion_blocked_substeps:
            break
    assert step.human_motion_blocked_substeps
    assert step.clearance.human_human_m < 0.005
    assert not step.contacts


def test_active_root_teleport_fails_and_preserves_last_snapshot(engine):
    engine.load(compose_config())
    engine.reset()
    good = engine.stop()
    model, data = engine._adapter._native()
    mocap_id = model.body_mocapid[model.body("human_01").id]
    data.mocap_pos[mocap_id, 0] += 0.5
    with pytest.raises(SimulationError, match="kinematic human mocap pose"):
        engine.stop()
    assert engine.get_ground_truth() == good.ground_truth
    assert engine.get_metadata().last_valid_tick == 1


def test_transient_native_contact_survives_a_contact_free_tick_end(engine):
    engine.load(
        compose_config(
            overrides=(
                "scenarios.robot.initial_pose=[0,0,0]",
                "scenarios.robot.goal=[3,0]",
                "experiments.terminate_on_contact=false",
                "scenarios.humans.0.initial_pose=[0.4995,0.62,0]",
                "scenarios.humans.0.goal=[0.4995,-0.62]",
            )
        )
    )
    engine.reset()
    for _ in range(35):
        step = engine.stop()
        if step.contacts and not engine._adapter._contact_pairs(engine._adapter._data):
            break
    assert step.contacts
    assert {event.kind for event in step.contacts} == {"robot_human"}
    assert not engine._adapter._contact_pairs(engine._adapter._data)
    assert (
        max(event.time_ns for event in step.contacts)
        < step.ground_truth.stamp.timestamp.time_ns
    )
