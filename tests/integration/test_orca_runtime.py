import json
import os
import subprocess
import sys
from math import hypot
from pathlib import Path

import pytest

from narrow_nav.core.seed import derive_seed
from narrow_nav.core.types import Velocity2D
from narrow_nav.humans.registry import create_human_backend
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine

_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def native_rvo2():
    pytest.importorskip("rvo2")


def test_orca_controls_native_mocap_reaches_goal_and_replays():
    config = compose_config(
        overrides=(
            "humans=orca",
            "worlds.width_m=3",
            "scenarios.robot.initial_pose=[-4,-1.1,0]",
            "scenarios.robot.goal=[4,-1.1]",
            "scenarios.humans.0.initial_pose=[-2.4,-0.7,0]",
            "scenarios.humans.0.goal=[2.4,0.7]",
        )
    )
    engine = MuJoCoEngine(human_backend=create_human_backend(config.humans.backend))
    traces, episodes = [], []
    try:
        engine.load(config)
        for _ in range(2):
            observation = engine.reset(seed=71)
            metadata = engine.get_metadata()
            episodes.append(metadata.episode.episode_id)
            assert metadata.human_backend_id == "orca"
            assert metadata.human_backend_seed == derive_seed(71, "human_backend")
            assert metadata.human_backend_version.startswith("1+pyrvo2.")
            assert all(slot.status == "missing" for slot in observation.sensors)
            trace = []
            (human,) = engine.get_ground_truth().humans
            for _ in range(160):
                before = human
                step = engine.stop()
                (human,) = step.ground_truth.humans
                (motion,) = step.human_motion_results
                assert motion.executed.velocity.linear_x == pytest.approx(
                    (human.pose.x - before.pose.x) / 0.05
                )
                assert motion.executed.velocity.linear_y == pytest.approx(
                    (human.pose.y - before.pose.y) / 0.05
                )
                assert motion.requested.velocity.frame_id == "world"
                assert not step.contacts and step.clearance.human_wall_m > 0
                assert not step.human_motion_blocked_substeps
                trace.append(
                    (human.pose, motion.requested.velocity, motion.executed.velocity)
                )
            assert trace[0][0].x > -2.4 and trace[0][0].y > -0.7
            assert (
                hypot(human.pose.x - human.goal.x, human.pose.y - human.goal.y) <= 0.1
            )
            assert all(executed == Velocity2D(0, 0) for _, _, executed in trace[-10:])
            traces.append(trace)
        assert episodes[0] != episodes[1]
        assert traces[0] == traces[1]
    finally:
        engine.close()


def _evaluate(*arguments):
    env = {
        key: value
        for key, value in os.environ.items()
        if key not in ("DISPLAY", "WAYLAND_DISPLAY", "PYTHONPATH")
    }
    run = subprocess.run(
        [
            sys.executable,
            "-B",
            "experiments/social_navigation/evaluate_orca.py",
            *arguments,
        ],
        cwd=_ROOT,
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )
    return json.loads(run.stdout)


@pytest.mark.parametrize(
    ("policy", "command_xy"),
    [
        ("advance", (0.35, 0.0)),
        ("stop", (0.0, 0.0)),
        ("yield", (0.0, -0.22)),
        ("reverse", (-0.25, 0.0)),
    ],
)
def test_real_orca_evaluator_keeps_robot_policy_external(policy, command_xy):
    result = _evaluate("--ticks", "95", "--policy", policy)
    assert result["episode"]["human_backend_id"] == "orca"
    assert len(result["trace"]) == 95
    for index, expected in ((0, (0.35, 0.0)), (80, command_xy), (90, (0.35, 0.0))):
        action = result["trace"][index]["requested"]["robot"]["velocity"]
        assert (action["linear_x"], action["linear_y"]) == expected
    actual = result["trace"][80]["executed"]["robot"]["velocity"]
    if policy == "yield":
        assert actual["linear_y"] < -0.1
    elif policy == "reverse":
        assert actual["linear_x"] < -0.1
    elif policy == "stop":
        assert abs(actual["linear_x"]) < 0.02
    else:
        assert actual["linear_x"] > 0.2
    assert result["measurements"]["humans"]["human_01"]["speed_mps"]["max"] > 0


def test_external_advance_contact_is_retained_as_a_native_failure():
    result = _evaluate(
        "--ticks",
        "180",
        "robots=mobile_robot",
        "worlds.width_m=0.75",
        "scenarios.robot.initial_pose=[2.9,0,0]",
        "scenarios.robot.goal=[4.65,0]",
        "scenarios.humans.0.initial_pose=[4.2,0,0]",
        "scenarios.humans.0.goal=[-2,0]",
    )
    assert result["episode"]["status"] == "terminated"
    assert result["stop_reason"] == "robot_contact"
    assert result["measurements"]["native_contact_pair_samples"]["robot_human"] > 0
    assert result["measurements"]["humans"]["human_01"]["time_to_goal_s"] is None


def test_symmetric_pair_is_retained_as_a_deadlock_candidate():
    result = _evaluate(
        "--ticks",
        "260",
        "worlds.length_m=20",
        "worlds.width_m=6",
        "scenarios.robot.initial_pose=[-8,-2,0]",
        "scenarios.robot.goal=[8,-2]",
        (
            "scenarios.humans=[{id:human_a,initial_pose:[-2,0,0],goal:[2,0]},"
            "{id:human_b,initial_pose:[2,0,0],goal:[-2,0]}]"
        ),
    )
    measurements = result["measurements"]
    assert result["stop_reason"] == "fixture_tick_limit"
    assert measurements["min_clearance_m"]["human_human_m"] >= 0
    assert not measurements["native_contact_pair_samples"]
    assert all(
        human["deadlock_candidate"] and human["time_to_goal_s"] is None
        for human in measurements["humans"].values()
    )
