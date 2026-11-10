from math import cos, pi, sin

import mujoco
import numpy as np
import pytest

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.types import Point2D
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.mujoco.geometry import inspect_mjcf
from narrow_nav.simulation.world_builder import build_world

jps = pytest.importorskip("jupedsim")

from shapely.geometry import Point

from narrow_nav.humans.jupedsim_bridge.clock import BridgeClock
from narrow_nav.humans.jupedsim_bridge.geometry import build_walkable
from narrow_nav.humans.jupedsim_bridge.model_factory import create_simulation

_MJCF = """
<mujoco>
  <compiler angle="radian"/>
  <worldbody>
    <geom name="floor" type="box" pos="0 0 -.1" size="3 2 .1" group="0"/>
    <geom name="south" type="box" pos="0 -1.2 .8" size=".1 .8 .8" group="1"/>
    <geom name="north" type="box" pos="0 1.2 .8" size=".1 .8 .8" group="1"/>
    <geom name="angled" type="box" pos="1.5 .5 .75"
          size=".75 .12 .75" euler="0 0 .63" group="1"/>
    <geom name="round" type="cylinder" pos="-1.8 -.8 .5" size=".3 .5" group="1"/>
    <geom name="decoration" type="box" pos="2 -1 .5" size=".2 .2 .2"
          contype="0" conaffinity="0" group="1"/>
  </worldbody>
</mujoco>
"""


def test_projection_against_native_rays_including_yaw_circle_door_and_outside():
    collision = inspect_mjcf(_MJCF, allow_rotated_walls=True)
    exported = collision.planar_world()
    projected = build_walkable(exported)
    model = mujoco.MjModel.from_xml_string(_MJCF)
    # mj_ray sees visual geoms too. Put the contact-disabled decoration outside its mask.
    model.geom_group[model.geom("decoration").id] = 2
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geom_id = np.empty(1, dtype=np.int32)
    samples = [
        (float(x), float(y))
        for x in np.linspace(-3.25, 3.25, 64)
        for y in np.linspace(-2.2, 2.2, 55)
    ]
    samples += [(0, 0), (0, 1), (2, -1), (1.5 + 0.6 * cos(0.63), 0.5 + 0.6 * sin(0.63))]
    for x, y in samples:
        # A downward ray hits the upper face of an occupied geom or the floor.
        # Use collision groups and remove the render-only geom from this query.
        groups = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)
        mujoco.mj_ray(
            model,
            data,
            np.array([x, y, 3.0]),
            np.array([0.0, 0.0, -1.0]),
            groups,
            True,
            -1,
            geom_id,
        )
        native_walkable = int(geom_id[0]) == model.geom("floor").id
        assert projected.polygon.covers(Point(x, y)) == native_walkable, (x, y)
    assert exported.frame_id == "world"
    assert exported.projection_error_m == pytest.approx(0.3 * (1 / cos(pi / 64) - 1))
    assert projected.opening_width(
        Point2D(0, 0), axis=(0, 1), half_span_m=2
    ) == pytest.approx(0.8)
    assert len(projected.polygon.interiors) == 2
    projected.validate_route(Point2D(-2, 0), Point2D(2, 0), 0.25)


@pytest.mark.parametrize(
    "xml",
    [_MJCF.replace("0 0 -.1", "0 0 .1"), _MJCF.replace("0 0 .63", ".1 0 .63")],
)
def test_projection_rejects_floor_height_and_tilt_it_cannot_represent(xml):
    with pytest.raises(ConfigurationError, match="Z=0|upright"):
        inspect_mjcf(xml, allow_rotated_walls=True)


@pytest.mark.parametrize(
    ("world", "scenario"),
    [
        ("corridor", "head_on"),
        ("blind_corner", "blind_corner"),
        ("arena_obstacle", "arena_obstacle"),
        ("hospital_medium", "hospital_medium"),
        ("school_floor", "school_floor"),
    ],
)
def test_world_projection_from_resolved_mjcf(world, scenario):
    config = compose_config(
        overrides=[
            f"worlds={world}",
            f"scenarios={scenario}",
            "robots=mecanum_robot",
            "simulation.human_visuals=proxy",
        ]
    )
    built = build_world(config)
    compiled = inspect_world(built, allow_rotated_walls=True)
    geometry = build_walkable(compiled.planar_world())
    assert geometry.polygon.is_valid and geometry.polygon.area > 0
    endpoints = []
    for actor in (built.scene.robot, *built.scene.humans):
        spawn = Point2D(actor.pose.x, actor.pose.y)
        geometry.validate_route(spawn, actor.goal, actor.radius_m)
        endpoints.extend((spawn, actor.goal))
    native = geometry.native_polygon(tuple(endpoints))
    simulation = create_simulation(
        jps.CollisionFreeSpeedModel(), native, BridgeClock(100_000_000)
    )
    assert len(simulation.get_geometry().holes()) == len(native.interiors)
    assert simulation.iteration_count() == 0
    print(
        world,
        f"area={geometry.polygon.area:.9f}",
        f"components={len(geometry.components)}",
        f"native_holes={len(native.interiors)}",
        f"projection_error_m={geometry.projection_error_m:.9f}",
        f"geometry_hash={geometry.geometry_hash}",
    )


def test_intersection_fixture_keeps_both_mjcf_floor_legs():
    # Technical four-way fixture. It supplies no research Card dimensions.
    xml = """
    <mujoco><worldbody>
      <geom name="east_west" type="box" pos="0 0 -.05" size="3 .75 .05" group="0"/>
      <geom name="north_south" type="box" pos="0 0 -.05" size=".75 3 .05" group="0"/>
      <geom name="north_east" type="box" pos="1 1 .5" size=".25 .25 .5" group="1"/>
    </worldbody></mujoco>
    """
    geometry = build_walkable(inspect_mjcf(xml).planar_world())
    native = geometry.native_polygon()
    assert native.area == pytest.approx(15.75)
    assert not native.covers(Point(1, 1))
    geometry.validate_route(Point2D(-2.5, 0), Point2D(2.5, 0), 0.25)
    geometry.validate_route(Point2D(0, -2.5), Point2D(0, 2.5), 0.25)
    assert geometry.opening_width(
        Point2D(2, 0), axis=(0, 1), half_span_m=2
    ) == pytest.approx(1.5)
