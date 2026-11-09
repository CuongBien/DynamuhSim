import mujoco
import numpy as np
import pytest

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.world_builder import build_world


@pytest.mark.parametrize("width", [1.0, 1.4, 1.8])
def test_clear_width_comes_from_compiled_walls(width):
    built = build_world(
        compose_config(overrides=[f"worlds.width_m={width}", "worlds.length_m=12"])
    )
    collision = inspect_world(built)
    assert collision.clear_width(0) == pytest.approx(width, abs=1e-9, rel=0)
    assert collision.contains(5.5, 0, 0.25)
    assert not collision.contains(5.9, 0, 0.25)
    assert not collision.contains(0, width / 2 + 0.03)


def test_bay_width_and_position_change_collision_geometry():
    built = build_world(
        compose_config(
            overrides=[
                "worlds=passing_bay",
                "worlds.passing_bay_width_m=2.6",
                "worlds.passing_bay_center_x_m=1.5",
                "worlds.passing_bay_length_m=2",
            ]
        )
    )
    collision = inspect_world(built)
    assert collision.clear_width(0) == pytest.approx(1.4, abs=1e-9, rel=0)
    assert collision.clear_width(1.5) == pytest.approx(2.6, abs=1e-9, rel=0)
    assert collision.contains(1.5, 1.4, 0.25)
    assert not collision.contains(0, 1.4, 0.25)


@pytest.mark.parametrize(
    ("world", "scenario", "cap", "side", "wall_top", "overrides"),
    [
        (
            "blind_corner",
            "blind_corner",
            "yield_bay_main_cap",
            1,
            2.5,
            ("corridor_width=w070",),
        ),
        (
            "school_floor",
            "school_floor_baseline",
            "yield_bay_a_01_cap",
            -1,
            2.6,
            ("corridor_width=w070",),
        ),
        (
            "hospital_medium",
            "hospital_medium",
            "yield_bay_n_01_cap",
            1,
            2.5,
            (),
        ),
    ],
)
@pytest.mark.parametrize("opening_m", (None, 1.05))
def test_yield_bay_blocks_upper_rays_and_keeps_floor_open(
    world, scenario, cap, side, wall_top, overrides, opening_m
):
    config = [
        f"worlds={world}",
        f"scenarios={scenario}",
        "simulation.human_visuals=proxy",
        *overrides,
    ]
    if opening_m is not None:
        config.append(f"yield_bay_opening_height_m={opening_m}")
    expected = 0.8 if opening_m is None else opening_m
    built = build_world(compose_config(overrides=config))
    model = mujoco.MjModel.from_xml_string(built.mjcf)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    block = model.geom(cap)
    assert block.pos[2] - block.size[2] == pytest.approx(expected)
    assert block.pos[2] + block.size[2] == pytest.approx(wall_top)
    assert block.contype != 0
    assert built.scene.collision.contains(*block.pos[:2], radius_m=0.25)
    assert inspect_world(built).contains(*block.pos[:2], radius_m=0.25)

    geom_id = np.empty(1, dtype=np.int32)
    wall_group = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)
    direction = np.array([0, side, 0], dtype=float)
    # Start on the corridor side of the mouth after applying the width preset.
    mouth_distance = 0.25
    x = block.pos[0]
    y = block.pos[1] - side * (block.size[1] + mouth_distance)
    below, above = (
        mujoco.mj_ray(
            model, data, np.array([x, y, z]), direction, wall_group, True, -1, geom_id
        )
        for z in (expected - 0.05, expected + 0.05)
    )
    assert below > above == pytest.approx(mouth_distance)
    assert geom_id[0] == block.id


def test_blind_corner_wall_blocks_line_of_sight():
    world = build_world(
        compose_config(overrides=["worlds=blind_corner", "scenarios=blind_corner"])
    )
    model = mujoco.MjModel.from_xml_string(world.mjcf)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    origin = np.array([-3.0, 0, 0.3])
    target = np.array([4.2, 2.8, 0.3])
    direction = target - origin
    distance = np.linalg.norm(direction)
    wall_group = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)
    geom_id = np.empty(1, dtype=np.int32)
    hit = mujoco.mj_ray(
        model, data, origin, direction / distance, wall_group, True, -1, geom_id
    )
    assert 0 < hit < distance
    assert model.geom(geom_id[0]).name.startswith("wall_")
    collision = inspect_world(world)
    assert collision.contains(4.2, 2.8, 0.25)
    assert collision.contains(4.2, 0.8, 0.25)
    assert not collision.contains(0, 2.8, 0.25)


@pytest.mark.parametrize(
    ("overrides", "error"),
    [
        (["worlds.width_m=0"], "worlds.width_m"),
        (["worlds.width_m=0.8"], "collision-free"),
        (["scenarios.robot.id=world"], "reserved"),
        (["worlds.wall_height_m=-1"], "worlds.wall_height_m"),
        (["yield_bay_opening_height_m=0"], "yield_bay_opening_height_m"),
        (
            [
                "worlds=passing_bay",
                "scenarios=passing_bay",
                "yield_bay_opening_height_m=2.5",
            ],
            "below the wall top",
        ),
        (
            [
                "worlds=school_floor",
                "scenarios=school_floor_baseline",
                "yield_bay_opening_height_m=2.6",
            ],
            "below the wall top",
        ),
        (["worlds=blind_corner", "worlds.turn_length_m=0"], "turn_length_m"),
        (
            ["worlds=passing_bay", "worlds.passing_bay_width_m=1.3"],
            "passing_bay_width_m",
        ),
        (["worlds=passing_bay", "worlds.passing_bay_center_x_m=4.8"], "wall returns"),
        (["robots.radius_m=0.16"], "robot footprint"),
        (["robots.mass_kg=0.4"], "total wheel mass"),
    ],
)
def test_invalid_geometry_rejected(overrides, error):
    with pytest.raises(ConfigurationError, match=error):
        build_world(compose_config(overrides=overrides))
