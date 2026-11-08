from collections import Counter, deque
from dataclasses import astuple
from hashlib import sha256
from itertools import pairwise
from math import pi

import mujoco
import numpy as np
import pytest

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.types import Twist2D
from narrow_nav.simulation.configuration import compose_config, validate_scene_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import MJCF_ROOT, build_world
from narrow_nav.simulation.world_geometry import Rectangle

_WORLD_SCENARIOS = (
    ("corridor_070", "corridor_070", 1),
    ("corridor_090", "corridor_090", 1),
    ("corridor_120", "corridor_120", 1),
    ("arena_obstacle", "arena_obstacle", 1),
    ("arena_dataset", "arena_dataset", 0),
    ("arena_rvo2", "arena_rvo2", 4),
    ("hospital_easy", "hospital_easy", 6),
    ("hospital_medium", "hospital_medium", 12),
    ("hospital_hard", "hospital_hard", 20),
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
    "world", ["school_arena", "school_arena_1", "school_arena_hunav", "school_floor"]
)
def test_school_yield_bays_fit_robot_and_keep_classroom_entrances(world):
    path = MJCF_ROOT / "worlds" / world / "main.xml"
    model = mujoco.MjModel.from_xml_path(str(path))
    collision = inspect_world(path)
    bays = [
        model.geom(i).name.removesuffix("_back")
        for i in range(model.ngeom)
        if model.geom(i).name.startswith("yield_bay_")
        and model.geom(i).name.endswith("_back")
    ]
    straight = world == "school_arena_1"
    expected = (
        {"a": 8} if straight else {"a": 6, "b": 1, "c": 1, "d": 1, "e": 1, "f": 3}
    )
    assert Counter(bay.removeprefix("yield_bay_")[0] for bay in bays) == expected
    centers = {"a": [], "f": []}
    for bay in bays:
        low, high, back = (
            model.geom(f"{bay}_{end}") for end in ("low", "high", "back")
        )
        along = int(back.size[1] > back.size[0])
        cross = 1 - along
        side = 1 if back.pos[cross] > low.pos[cross] else -1
        corridor = bay.removeprefix("yield_bay_")[0]
        throat = (
            side * 0.54
            if straight
            else {
                "a": -8.54 if side < 0 else -7.46,
                "b": 15.54,
                "c": -4.54,
                "d": 6.46,
                "e": 0.54,
                "f": 14.46,
            }[corridor]
        )
        assert (high.pos[along] - high.size[along]) - (
            low.pos[along] + low.size[along]
        ) == pytest.approx(0.90)
        assert side * (back.pos[cross] - throat) - back.size[cross] == pytest.approx(
            0.95
        )
        if corridor in centers:
            centers[corridor].append(back.pos[along])

        point = list(back.pos[:2])
        for step in range(19):
            point[cross] = throat + side * (-0.32 + 0.05 * step)
            # A radius of 0.25 m covers both stock robot profiles.
            assert collision.contains(*point, radius_m=0.25), (world, bay, point)
        assert not collision.contains(*back.pos[:2])
        assert not collision.contains(*low.pos[:2])
        assert not collision.contains(*high.pos[:2])
    for group in centers.values():
        positions = sorted(group)
        assert all(b - a <= 5.0 for a, b in pairwise(positions))

    # The first classroom retains both pedestrian entrances alongside the bay.
    doors = (-14.90, -13.10) if straight else (-13.53, -11.57)
    for door_x in doors:
        y_values = (
            (0.25, 0.60, 0.90, 1.30) if straight else (-8.25, -8.60, -8.90, -9.25)
        )
        assert all(collision.contains(door_x, y, 0.25) for y in y_values)


def test_mecanum_stops_inside_a_school_yield_bay_under_wheel_control():
    config = compose_config(
        overrides=[
            "worlds=school_floor",
            "scenarios=school_floor_baseline",
            "robots=mecanum_robot",
            f"scenarios.robot.initial_pose=[-12.55,-8.0,{-pi / 2}]",
        ]
    )
    engine = MuJoCoEngine()
    try:
        engine.load(config)
        engine.reset()
        for _ in range(100):
            robot = engine.get_ground_truth().robot
            result = engine.step(
                RobotAction(robot.entity_id, Twist2D(0.30, 0, 0), robot.timestamp)
            )
            assert not result.contacts
            if result.ground_truth.robot.pose.y <= -9.05:
                break
        assert result.ground_truth.robot.pose.y <= -9.05
        for _ in range(12):
            result = engine.stop()
            assert not result.contacts
        robot = result.ground_truth.robot
        assert abs(robot.velocity.linear_x) < 0.02
        assert (
            -9.49 + config.robots.radius_m
            < robot.pose.y
            < -8.54 - config.robots.radius_m
        )
    finally:
        engine.close()


def _reachable_cells(free, start):
    seen = np.zeros_like(free)
    assert free[start]
    seen[start] = True
    pending = deque([start])
    width, height = free.shape
    while pending:
        x, y = pending.popleft()
        for i, j in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if 0 <= i < width and 0 <= j < height and free[i, j] and not seen[i, j]:
                seen[i, j] = True
                pending.append((i, j))
    return seen


@pytest.mark.parametrize(
    "world", ["school_arena", "school_arena_1", "school_arena_hunav", "school_floor"]
)
@pytest.mark.parametrize("radius", [0.18, 0.25])
def test_school_route_connects_inside_the_perimeter(world, radius):
    config = compose_config(overrides=[f"worlds={world}", f"scenarios={world}"])
    collision = inspect_world(MJCF_ROOT / "worlds" / world / "main.xml")
    # A 0.1 m diagnostic grid extends outside every school boundary.
    xs = np.linspace(-18, 22, 401)
    ys = np.linspace(-15, 16, 311)
    x, y = xs[:, None], ys[None, :]
    free = np.ones((xs.size, ys.size), dtype=bool)
    for wall in collision.walls:
        if isinstance(wall, Rectangle):
            dx = np.maximum(np.maximum(wall.x_min - x, x - wall.x_max), 0)
            dy = np.maximum(np.maximum(wall.y_min - y, y - wall.y_max), 0)
            free &= np.hypot(dx, dy) > radius + 1e-9
        else:
            free &= np.hypot(x - wall.x, y - wall.y) > wall.radius_m + radius + 1e-9

    def cell(px, py):
        return int(np.argmin(abs(xs - px))), int(np.argmin(abs(ys - py)))

    start = cell(*config.scenarios.robot.initial_pose[:2])
    goal = cell(*config.scenarios.robot.goal)
    seen = _reachable_cells(free, start)
    assert seen[goal]
    assert not any((seen[0].any(), seen[-1].any(), seen[:, 0].any(), seen[:, -1].any()))
    if world != "school_arena_1":
        # Both ends of the flat aisle beside the east stair wall are open.
        assert all(collision.contains(15, y, radius) for y in np.linspace(-4.1, 1, 103))
        assert seen[cell(15, -2)]


@pytest.mark.parametrize("preset", [None, "w070", "w090", "w120"])
def test_school_mecanum_crosses_the_aisle_beside_the_stairs(preset):
    config = compose_config(
        overrides=[
            "worlds=school_floor",
            "scenarios=school_floor_baseline",
            "robots=mecanum_robot",
            f"scenarios.robot.initial_pose=[15,-4.1,{pi / 2}]",
            "scenarios.robot.goal=[15,1.0]",
            *([f"corridor_width={preset}"] if preset else []),
        ]
    )
    _, scene = validate_scene_config(config)
    engine = MuJoCoEngine()
    try:
        engine.load(config)
        engine.reset()
        for _ in range(280):
            robot = engine.get_ground_truth().robot
            result = engine.step(
                RobotAction(robot.entity_id, Twist2D(0.40, 0, 0), robot.timestamp)
            )
            assert not result.contacts
            if result.terminated:
                break
        assert result.end_reason == "goal_reached"
        assert result.ground_truth.robot.pose.y > scene.robot.goal.y - 0.10
    finally:
        engine.close()


@pytest.mark.parametrize(
    ("world", "width"),
    [("corridor_070", 0.7), ("corridor_090", 0.9), ("corridor_120", 1.2)],
)
def test_corridor_has_a_fixed_width_and_closed_straight_walls(world, width):
    config = compose_config(overrides=[f"worlds={world}", f"scenarios={world}"])
    geometry = inspect_world(build_world(config))
    for x in (-14, 0, 2, 14):
        assert geometry.clear_width(x) == pytest.approx(width, abs=1e-9)
        assert not geometry.contains(x, width / 2 + 0.03)
    assert not geometry.contains(-15.02, 0)
    assert not geometry.contains(15.02, 0)


def test_hospital_has_open_doors_waiting_seats_and_separate_yield_bays():
    path = MJCF_ROOT / "worlds" / "hospital_medium" / "main.xml"
    model = mujoco.MjModel.from_xml_path(str(path))
    collision = inspect_world(path)
    assert collision.clear_width(0) == pytest.approx(3.0, abs=1e-9)
    assert collision.clear_width(-26.9) == pytest.approx(2.475, abs=1e-9)
    assert collision.clear_width(-27.3) == pytest.approx(2.46, abs=1e-9)
    assert all(collision.contains(x, 0, 0.25) for x in np.linspace(-27.7, 27.7, 223))

    centers = []
    for side, sign in (("n", 1), ("s", -1)):
        for x in np.linspace(-25.2, 25.2, 10):
            assert all(
                collision.contains(x, sign * y, 0.25) for y in (1.2, 1.6, 2.0, 2.4)
            )
        for index in range(1, 7):
            seat = model.geom(f"waiting_bench_{side}_{index:02d}_seat")
            assert abs(seat.pos[1]) + seat.size[1] < 1.5
        for index in range(1, 6):
            bay = f"yield_bay_{side}_{index:02d}"
            low, high, back = (
                model.geom(f"{bay}_{end}") for end in ("low", "high", "back")
            )
            assert high.pos[0] - high.size[0] - low.pos[0] - low.size[
                0
            ] == pytest.approx(1.30)
            assert sign * back.pos[1] - back.size[1] - 1.5 == pytest.approx(1.20)
            centers.append((back.pos[0], side))
            assert all(
                collision.contains(back.pos[0], sign * y, 0.25)
                for y in np.linspace(0, 2.35, 24)
            )
    ordered = sorted(centers)
    assert all(
        b - a == pytest.approx(5.6) and side_a != side_b
        for (a, side_a), (b, side_b) in pairwise(ordered)
    )


@pytest.mark.parametrize(
    ("world", "x", "goal_y", "width_override"),
    [
        ("hospital_easy", -23.5, 2.1, []),
        ("blind_corner", 1.2, 1.40, ["corridor_width=w070"]),
    ],
)
def test_mecanum_can_drive_into_a_yield_bay(world, x, goal_y, width_override):
    config = compose_config(
        overrides=[
            f"worlds={world}",
            f"scenarios={world}",
            "robots=mecanum_robot",
            "simulation.human_visuals=proxy",
            f"scenarios.robot.initial_pose=[{x},0,{pi / 2}]",
            f"scenarios.robot.goal=[{x},{goal_y}]",
            *width_override,
        ]
    )
    _, scene = validate_scene_config(config)
    assert scene.robot.pose.x == scene.robot.goal.x == x
    engine = MuJoCoEngine()
    try:
        engine.load(config)
        engine.reset()
        for _ in range(160):
            robot = engine.get_ground_truth().robot
            result = engine.step(
                RobotAction(robot.entity_id, Twist2D(0.35, 0, 0), robot.timestamp)
            )
            assert not result.contacts
            if result.terminated:
                break
        assert result.end_reason == "goal_reached"
        assert result.ground_truth.robot.pose.y >= scene.robot.goal.y - 0.1
    finally:
        engine.close()


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
