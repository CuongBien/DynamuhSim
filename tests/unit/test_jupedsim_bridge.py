from dataclasses import replace
from math import pi

import pytest

from narrow_nav.core.errors import ContractError
from narrow_nav.core.metadata import SnapshotMetadata
from narrow_nav.core.planar_world import PlanarWorld
from narrow_nav.core.states import HumanContext, Obstacle
from narrow_nav.core.types import Point2D, Pose2D, Timestamp, Velocity2D

pytest.importorskip("jupedsim")

from shapely.geometry import Point

from narrow_nav.humans.jupedsim_bridge.clock import BridgeClock
from narrow_nav.humans.jupedsim_bridge.geometry import (
    BridgeCapabilityError,
    build_walkable,
)
from narrow_nav.humans.jupedsim_bridge.robot_proxy import (
    RobotFootprintSample,
    require_dynamic_robot_support,
)


def _ring(*points):
    return tuple(Point2D(*xy) for xy in points)


def _box(x0, y0, x1, y1):
    return _ring((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def test_corridor_and_door_radius_are_checked_without_inflating_native_polygon():
    floor = _box(-3, -1.5, 3, 1.5)
    walls = (
        Obstacle("south", (_box(-0.1, -1.5, 0.1, -0.4),)),
        Obstacle("north", (_box(-0.1, 0.4, 0.1, 1.5),)),
    )
    area = build_walkable(PlanarWorld((floor,), walls, 0))
    assert area.polygon.area == pytest.approx(18 - 2 * 0.2 * 1.1)
    assert len(area.polygon.interiors) == 0
    assert area.opening_width(
        Point2D(0, 0), axis=(0, 2), half_span_m=2
    ) == pytest.approx(0.8)
    assert area.opening_width(
        Point2D(-1, 0), axis=(0, 1), half_span_m=2
    ) == pytest.approx(3)
    with pytest.raises(ContractError, match="both sides"):
        area.opening_width(Point2D(0, 0), axis=(0, 1), half_span_m=0.3)
    area.validate_route(Point2D(-2, 0), Point2D(2, 0), 0.39)
    for radius in (0.4, 0.41):
        with pytest.raises(ContractError, match="disconnected"):
            area.validate_route(Point2D(-2, 0), Point2D(2, 0), radius)
    with pytest.raises(ContractError, match="spawn footprint"):
        area.validate_route(Point2D(-3, 0), Point2D(2, 0), 0.25)
    with pytest.raises(ContractError, match="goal footprint"):
        area.validate_route(Point2D(-2, 0), Point2D(0, 0.3), 0.25)
    with pytest.raises(ContractError, match="world"):
        area.validate_route(Point2D(-2, 0, "robot_base"), Point2D(2, 0), 0.25)
    with pytest.raises(ContractError, match="radius"):
        area.validate_route(Point2D(-2, 0), Point2D(2, 0), 0)


def test_invalid_rings_disconnected_floor_holes_and_hash():
    floor = _box(-3, -1.5, 3, 1.5)
    island = Obstacle("island", (_box(-1, -0.5, 1, 0.5),))
    area = build_walkable(PlanarWorld((floor,), (island,), 0))
    assert len(area.polygon.interiors) == 1
    assert area.polygon.area == pytest.approx(16)
    assert not area.polygon.covers(Point(0, 0))
    equivalent = build_walkable(PlanarWorld((floor[::-1],), (island,), 0))
    assert area.geometry_hash == equivalent.geometry_hash
    assert (
        area.geometry_hash != build_walkable(PlanarWorld((floor,), (), 0)).geometry_hash
    )
    bowtie = _ring((0, 0), (1, 1), (0, 1), (1, 0))
    with pytest.raises(ContractError, match="Self-intersection"):
        build_walkable(PlanarWorld((bowtie,), (), 0))
    separated = build_walkable(PlanarWorld((floor, _box(4, 0, 5, 1)), (), 0))
    assert len(separated.components) == 2
    assert separated.native_polygon((Point2D(4.5, 0.5),)).area == 1
    for endpoints in ((), (Point2D(0, 0), Point2D(4.5, 0.5))):
        with pytest.raises(BridgeCapabilityError, match="one connected Polygon"):
            separated.native_polygon(endpoints)


def test_robot_footprint_reports_measured_error_without_adding_human_radius():
    geometry = build_walkable(PlanarWorld((_box(-3, -2, 3, 2),), (), 0))
    stamp = SnapshotMetadata("probe:0", 1, Timestamp(2, 100_000_000))
    context = HumanContext(
        stamp, "robot", Pose2D(1, -0.5, pi / 2), Velocity2D(0, 0.4), 0.25, 0.3
    )
    robot = RobotFootprintSample.from_context(context)
    polygon = robot.carve(geometry.native_polygon())
    assert robot.centre == Point2D(1, -0.5)
    assert robot.velocity == Velocity2D(0, 0.4)
    assert robot.time_ns == 100_000_000
    assert not polygon.covers(Point(1, -0.5))
    assert polygon.contains(Point(1.251, -0.5))
    held = replace(robot, centre=Point2D(1, -0.46))
    assert robot.error_bound_m(held) == pytest.approx(0.040301499, abs=1e-8)
    with pytest.raises(BridgeCapabilityError, match="external robot") as exc:
        require_dynamic_robot_support()
    assert exc.value.status == "BLOCKED_CAPABILITY"


def test_clock_grid_and_no_fractional_iterations():
    clock = BridgeClock(100_000_000)
    assert clock.internal_dt_s == 0.01
    assert clock.iterations_per_callback == 10
    assert [clock.iterations_at(k * 100_000_000) for k in (0, 1, 10, 100)] == [
        0,
        10,
        100,
        1000,
    ]
    with pytest.raises(ContractError, match="callback grid"):
        clock.iterations_at(50_000_000)
    with pytest.raises(ContractError, match="must divide"):
        BridgeClock(100_000_000, 30_000_000)
