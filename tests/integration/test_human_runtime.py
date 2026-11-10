from dataclasses import replace
from math import atan2, hypot

import pytest

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.seed import derive_seed
from narrow_nav.core.types import Twist2D, Velocity2D
from narrow_nav.humans.registry import create_human_backend
from narrow_nav.humans.scripted import ScriptedHumanBackend
from narrow_nav.simulation import SimulationError
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine

_FIVE_METRES = (
    "worlds.width_m=3",
    "scenarios.robot.initial_pose=[-4,-1.1,0]",
    "scenarios.robot.goal=[4,-1.1]",
    "scenarios.humans.0.initial_pose=[-2.4,-0.7,0]",
    "scenarios.humans.0.goal=[2.4,0.7]",
)


def test_scripted_walks_five_metre_route_stops_and_replays():
    config = compose_config(overrides=_FIVE_METRES)
    engine = MuJoCoEngine(human_backend=create_human_backend(config.humans.backend))
    engine.load(config)
    episodes, traces = [], []
    try:
        for _ in range(2):
            observation = engine.reset(seed=71)
            metadata = engine.get_metadata()
            episodes.append(metadata.episode.episode_id)
            assert metadata.human_backend_id == "scripted"
            assert metadata.human_backend_version == "1"
            assert metadata.human_backend_seed == derive_seed(71, "human_backend")
            assert metadata.human_motion_mode == "kinematic"
            assert all(
                slot.sample is None and slot.status == "missing"
                for slot in observation.sensors
            )
            (human,) = engine.get_ground_truth().humans
            assert hypot(
                human.goal.x - human.pose.x, human.goal.y - human.pose.y
            ) == pytest.approx(5)
            trace = []
            for _ in range(160):
                before = human.pose
                step = engine.stop()
                (human,) = step.ground_truth.humans
                (motion,) = step.human_motion_results
                assert motion.executed.velocity.linear_x == pytest.approx(
                    (human.pose.x - before.x) / 0.05
                )
                assert motion.executed.velocity.linear_y == pytest.approx(
                    (human.pose.y - before.y) / 0.05
                )
                assert step.clearance == engine.get_clearance()
                assert step.clearance.human_wall_m > 0
                assert not step.human_motion_blocked_substeps
                trace.append((human.pose, human.velocity, motion.executed.velocity))
            assert trace[0][0].x > -2.4 and trace[0][0].y > -0.7
            assert trace[0][0].yaw == pytest.approx(atan2(1.4, 4.8))
            assert (
                hypot(human.goal.x - human.pose.x, human.goal.y - human.pose.y) <= 0.1
            )
            assert all(
                pose == human.pose and velocity == Velocity2D(0, 0)
                for pose, velocity, _ in trace[-10:]
            )
            assert (
                motion.requested.velocity
                == motion.limited.velocity
                == motion.executed.velocity
                == Velocity2D(0, 0)
            )
            traces.append(trace)
        assert episodes[0] != episodes[1]
        assert traces[0] == traces[1]
    finally:
        engine.close()


@pytest.mark.parametrize(
    ("human_period", "first_speed", "second_speed"),
    [
        (50_000_000, 0.075, 0.15),
        (100_000_000, 0.15, 0.15),
    ],
)
def test_scripted_schedule_holds_motion_for_two_sorted_humans(
    human_period, first_speed, second_speed
):
    config = compose_config(
        overrides=(
            f"simulation.human_control_dt_ns={human_period}",
            (
                "scenarios.humans=[{id:human_a,initial_pose:[3,0.4,0],goal:[-3,0.4]},"
                "{id:human_b,initial_pose:[3,-0.4,0],goal:[-3,-0.4]}]"
            ),
        )
    )
    engine = MuJoCoEngine(human_backend=create_human_backend("scripted"))
    engine.load(config)
    try:
        engine.reset()
        for index, speed in enumerate((first_speed, second_speed)):
            robot = engine.get_ground_truth().robot
            step = engine.step(
                RobotAction(robot.entity_id, Twist2D(0.8, 0, 0), robot.timestamp)
            )
            assert step.requested.entity_ids == ("robot", "human_a", "human_b")
            assert step.requested.stamp.timestamp.tick == index
            for motion in step.human_motion_results:
                assert motion.requested.velocity.linear_x == pytest.approx(-0.8)
                assert motion.limited.velocity.linear_x == pytest.approx(-speed)
                assert motion.executed.velocity.linear_x == pytest.approx(-speed)
            assert step.executed.robot.velocity != step.limited.robot.velocity
    finally:
        engine.close()


def test_scripted_zero_speed_keeps_native_pose_and_yaw():
    engine = MuJoCoEngine(human_backend=create_human_backend("scripted"))
    engine.load(compose_config(overrides=("humans.speed_mps=0",)))
    try:
        engine.reset()
        (before,) = engine.get_ground_truth().humans
        for _ in range(10):
            (after,) = engine.stop().ground_truth.humans
            assert after.pose == before.pose and after.velocity == Velocity2D(0, 0)
    finally:
        engine.close()


def test_quaternius_uses_executed_motion_without_changing_physics():
    traces = []
    for visual in ("quaternius", "proxy"):
        engine = MuJoCoEngine(human_backend=create_human_backend("scripted"))
        engine.load(
            compose_config(
                overrides=(*_FIVE_METRES, f"simulation.human_visuals={visual}")
            )
        )
        try:
            engine.reset()
            trace = []
            for _ in range(6):
                step = engine.stop()
                truth = step.ground_truth
                trace.append((truth.robot, truth.humans, step.clearance, step.contacts))
            model, data = engine._adapter._native()
            assert bool(model.nskin) == (visual == "quaternius")
            human = truth.humans[0]
            assert tuple(data.xpos[model.body(human.entity_id).id, :2]) == (
                human.pose.x,
                human.pose.y,
            )
            if visual == "quaternius":
                assert (
                    engine._adapter._visuals._animator._actors[
                        human.entity_id
                    ].clip_name
                    == "walk"
                )
            traces.append(trace)
        finally:
            engine.close()
    assert traces[0] == traces[1]


class _InvalidVectorBackend(ScriptedHumanBackend):
    def propose(self, context):
        motions = super().propose(context)
        if context.stamp.timestamp.tick == 2:
            return (replace(motions[0], velocity=Velocity2D(float("nan"), 0)),)
        return motions


def test_invalid_backend_callback_fails_before_physics_and_preserves_prefix():
    engine = MuJoCoEngine(human_backend=_InvalidVectorBackend())
    engine.load(compose_config())
    try:
        engine.reset()
        engine.stop()
        good = engine.stop()
        with pytest.raises(ContractError, match="finite"):
            engine.stop()
        metadata = engine.get_metadata()
        assert (
            metadata.status,
            metadata.end_reason,
            metadata.last_valid_tick,
            metadata.failure_tick,
        ) == ("failed", "human_backend_failed", 2, 3)
        assert (
            "propose" in metadata.failure_traceback
            and "ContractError" in metadata.failure_traceback
        )
        assert engine.get_ground_truth() == good.ground_truth
        assert engine.get_observation() == good.observation
        assert engine._adapter._data.time == pytest.approx(0.1)
        with pytest.raises(SimulationError, match="reset"):
            engine.stop()
        assert engine.reset().stamp.episode_id != metadata.episode.episode_id
        assert engine.stop().ground_truth.stamp.timestamp.tick == 1
    finally:
        engine.close()


def test_active_backend_requires_matching_config_and_aligned_clock():
    with pytest.raises(ConfigurationError, match="unsupported"):
        create_human_backend("unregistered")
    engine = MuJoCoEngine(human_backend=create_human_backend("scripted"))
    try:
        with pytest.raises(ConfigurationError, match="injected backend"):
            engine.load(compose_config(overrides=("humans.backend=unknown",)))
        with pytest.raises(ConfigurationError, match="align"):
            engine.load(
                compose_config(overrides=("simulation.human_control_dt_ns=66000000",))
            )
    finally:
        engine.close()
