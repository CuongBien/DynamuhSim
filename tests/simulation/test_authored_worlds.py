from dataclasses import astuple
from hashlib import sha256

import pytest

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.types import Twist2D
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import MJCF_ROOT, build_world

_WORLD_SCENARIOS = (
    ("corridor_070", "corridor_070", 1),
    ("corridor_090", "corridor_090", 1),
    ("corridor_120", "corridor_120", 1),
    ("arena_obstacle", "arena_obstacle", 1),
    ("arena_dataset", "arena_dataset", 0),
    ("arena_rvo2", "arena_rvo2", 4),
    ("hospital_easy", "hospital_easy", 3),
    ("hospital_medium", "hospital_medium", 7),
    ("hospital_hard", "hospital_hard", 14),
    ("school_arena", "school_arena", 8),
    ("school_arena_1", "school_arena_1", 0),
    ("school_arena_hunav", "school_arena_hunav", 8),
    ("school_floor", "school_floor", 10),
    ("school_floor", "school_floor_baseline", 0),
)


@pytest.mark.parametrize(("world", "scenario", "human_count"), _WORLD_SCENARIOS)
def test_authored_world_runs_headless(
    world, scenario, human_count, tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    config = compose_config(
        MJCF_ROOT.parents[1] / "configs",
        [f"worlds={world}", f"scenarios={scenario}", "robots=mecanum_robot"],
    )
    built = build_world(config)
    source = f"worlds/{config.worlds.geometry_id}/main.xml"
    path = MJCF_ROOT / source
    reference = inspect_world(path)
    compiled = inspect_world(built)
    for expected in (reference, built.scene.collision):
        for shapes, actual in (
            (expected.floors, compiled.floors),
            (expected.walls, compiled.walls),
        ):
            for source_shape, compiled_shape in zip(shapes, actual, strict=True):
                assert type(source_shape) is type(compiled_shape)
                assert astuple(source_shape) == pytest.approx(
                    astuple(compiled_shape), abs=1e-9
                )

    engine = MuJoCoEngine()
    try:
        engine.load(config)
        engine.reset()
        truth = engine.get_ground_truth()
        assert len(truth.humans) == human_count
        assert engine.get_clearance().robot_wall_m > 0
        action = RobotAction(
            truth.robot.entity_id, Twist2D(0.35, 0, 0), truth.robot.timestamp
        )
        result = engine.step(action)
        assert result.ground_truth.robot.timestamp.tick == 1
        assert not result.contacts
        assert (
            source,
            sha256(path.read_bytes()).hexdigest(),
        ) in engine.get_metadata().mjcf_source_sha256
    finally:
        engine.close()


@pytest.mark.parametrize(
    ("world", "width"),
    [("corridor_070", 0.7), ("corridor_090", 0.9), ("corridor_120", 1.2)],
)
def test_corridor_width_and_recess_from_compiled_collision(world, width):
    config = compose_config(overrides=[f"worlds={world}", f"scenarios={world}"])
    geometry = inspect_world(build_world(config))
    assert geometry.clear_width(0) == pytest.approx(width, abs=1e-9)
    assert geometry.clear_width(2) == pytest.approx(width + 0.6, abs=1e-9)
    assert geometry.contains(2, width / 2 + 0.3, 0.25)
    assert not geometry.contains(0, width / 2 + 0.3, 0.25)


def test_arena_cylinder_uses_round_clearance_and_native_contact():
    config = compose_config(
        overrides=[
            "worlds=arena_dataset",
            "scenarios=arena_dataset",
            "scenarios.robot.initial_pose=[-9.3,-2.2,0]",
        ]
    )
    geometry = inspect_world(build_world(config))
    assert geometry.contains(-8.45, -2.65, 0.2)
    assert not geometry.contains(-8.45, -2.2, 0.2)
    engine = MuJoCoEngine()
    try:
        engine.load(config)
        engine.reset()
        assert engine.get_clearance().robot_wall_m == pytest.approx(0.65)
        for _ in range(65):
            robot = engine.get_ground_truth().robot
            result = engine.step(
                RobotAction(robot.entity_id, Twist2D(0.65, 0, 0), robot.timestamp)
            )
            if result.terminated:
                break
        assert result.end_reason == "robot_contact"
        assert {(c.kind, c.first_id, c.second_id) for c in result.contacts} == {
            ("robot_wall", "robot", "static_cylinder_b")
        }
    finally:
        engine.close()


def test_authored_world_validates_spawn_and_confines_geometry_path():
    base = ("worlds=corridor_070", "scenarios=corridor_070")
    with pytest.raises(ConfigurationError, match="outside collision-free space"):
        compose_config(overrides=[*base, "scenarios.robot.initial_pose=[-13,0.2,0]"])
    with pytest.raises(ConfigurationError, match="worlds.geometry_id"):
        compose_config(overrides=[*base, "worlds.geometry_id=../templates"])
