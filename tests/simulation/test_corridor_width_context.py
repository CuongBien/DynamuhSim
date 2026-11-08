import mujoco
import numpy as np
import pytest

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.simulation.configuration import _world_spec, compose_config
from narrow_nav.simulation.world_builder import MJCF_ROOT, build_world

# Source-layout probes at solid wall faces, away from doors, bays and furniture.
# The fourth value is any widening beyond the nominal hallway width.
_SCHOOL = (
    (-14.7, -8, 1, 0),
    (15, -2, 0, 0),
    (9, -4, 1, 0),
    (7, -0.85, 0, 0),
    (9, 0, 1, 0),
)
_PROBES = {
    "blind_corner": ((-1, 0, 1, 0), (4.2, 2.8, 0, 0), (1.2, 0, 1, 1.10)),
    "passing_bay": ((-2, 0, 1, 0), (0, 0, 1, 1.0)),
    "school_arena": _SCHOOL,
    "school_arena_hunav": _SCHOOL,
    "school_floor": _SCHOOL,
    "school_arena_1": ((-15.8, 0, 1, 0),),
}


@pytest.mark.parametrize(
    ("world", "scenario"),
    [*((world, world) for world in _PROBES), ("school_floor", "school_floor_baseline")],
)
@pytest.mark.parametrize(
    ("preset", "width"), [("w070", 0.70), ("w090", 0.90), ("w120", 1.20)]
)
def test_width_context_compiles_and_matches_wall_faces(world, scenario, preset, width):
    overrides = [f"worlds={world}", f"scenarios={scenario}", f"corridor_width={preset}"]
    # Compose with both footprints. Compile once with the stock visual assets.
    compose_config(overrides=overrides)
    config = compose_config(overrides=[*overrides, "robots=mecanum_robot"])
    built = build_world(config)
    model = mujoco.MjModel.from_xml_string(built.mjcf, dict(built.visual_assets))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    _, resize = _world_spec(config.worlds, MJCF_ROOT, width)
    wall_group = np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)
    geom_id = np.empty(1, dtype=np.int32)
    for x, y, axis, widening in _PROBES[world]:
        origin = np.array([*resize.actor_point(x, y, 0), 0.2])
        direction = np.eye(3)[axis]
        hits = [
            mujoco.mj_ray(
                model, data, origin, sign * direction, wall_group, True, -1, geom_id
            )
            for sign in (-1, 1)
        ]
        assert min(hits) > 0, (origin, hits)
        assert sum(hits) == pytest.approx(width + widening, abs=1e-6), origin


@pytest.mark.parametrize(
    ("world", "scenario"),
    [
        ("arena_dataset", "arena_dataset"),
        ("corridor", "head_on"),
        *(
            (name, name)
            for name in (
                "corridor_070",
                "corridor_090",
                "corridor_120",
                "hospital_easy",
                "hospital_medium",
                "hospital_hard",
            )
        ),
    ],
)
def test_fixed_worlds_reject_corridor_width_context(world, scenario):
    with pytest.raises(ConfigurationError, match="no corridor-width context"):
        compose_config(
            overrides=[
                f"worlds={world}",
                f"scenarios={scenario}",
                "corridor_width=w070",
            ]
        )
