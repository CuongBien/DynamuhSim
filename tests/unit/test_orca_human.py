from dataclasses import replace
from math import hypot

import pytest

from narrow_nav.core.errors import ContractError
from narrow_nav.core.metadata import SnapshotMetadata
from narrow_nav.core.protocols import HumanBackendConfig, HumanControllerBackend
from narrow_nav.core.states import HumanContext, HumanState, Obstacle
from narrow_nav.core.types import Point2D, Pose2D, Timestamp, Velocity2D
from narrow_nav.humans.registry import create_human_backend

pytestmark = pytest.mark.usefixtures("native_rvo2")


@pytest.fixture
def native_rvo2():
    return pytest.importorskip("rvo2")


def _backend(context, **parameters) -> HumanControllerBackend[HumanBackendConfig]:
    backend = create_human_backend("orca")
    config = HumanBackendConfig(
        {
            "speed_mps": 0.8,
            "neighbor_dist_m": 4.0,
            "max_neighbors": 10,
            "time_horizon_s": 2.0,
            "obstacle_horizon_s": 2.0,
            "robot_profile": "reciprocal_external_observed",
            **parameters,
        },
        100_000_000,
        0.1,
        1.5,
        1.4,
    )
    backend.reset(config, 71, tuple(human.entity_id for human in context.humans))
    return backend


def _context(routes, obstacles=()):
    stamp = SnapshotMetadata("fixture:0", 71, Timestamp(0, 0))
    humans = tuple(
        HumanState(
            f"human_{i}",
            Pose2D(*start, 0),
            Velocity2D(0, 0),
            Point2D(*goal),
            stamp.timestamp,
        )
        for i, (start, goal) in enumerate(routes)
    )
    return HumanContext(
        stamp,
        "robot",
        Pose2D(20, 20, 0),
        Velocity2D(0, 0),
        0.18,
        0.25,
        humans,
        obstacles,
    )


def _wall(name, x0, x1, y0, y1):
    return Obstacle(
        name, ((Point2D(x0, y0), Point2D(x0, y1), Point2D(x1, y1), Point2D(x1, y0)),)
    )


def _rollout(context, ticks):
    """Exercise the native solver with a simple kinematic test caller."""
    backend = _backend(context)
    trace = []
    for tick in range(1, ticks + 1):
        stamp = replace(context.stamp, timestamp=Timestamp(tick, tick * 100_000_000))
        requests = backend.propose(context)
        assert tuple(m.entity_id for m in requests) == tuple(
            h.entity_id for h in context.humans
        )
        humans = tuple(
            replace(
                human,
                pose=Pose2D(
                    human.pose.x + 0.1 * motion.velocity.linear_x,
                    human.pose.y + 0.1 * motion.velocity.linear_y,
                    0,
                ),
                velocity=motion.velocity,
                timestamp=stamp.timestamp,
            )
            for human, motion in zip(context.humans, requests, strict=True)
        )
        context = replace(context, stamp=stamp, humans=humans)
        trace.append(humans)
    return trace


def test_native_binding_zero_neighbours_and_goal_stop(native_rvo2):
    sim = native_rvo2.PyRVOSimulator(0.1, 4, 0, 2, 2, 0.25, 1.4)
    index = sim.addAgent((0, 0))
    sim.setAgentPrefVelocity(index, (0.8, 0))
    sim.doStep()
    assert sim.getAgentNumAgentNeighbors(index) == 0
    assert sim.getAgentPosition(index) == pytest.approx((0.08, 0))
    context = _context((((0, 0), (2, 0)), ((0, 2), (0, 2))))
    motion, arrived = _backend(context, max_neighbors=0).propose(context)
    assert motion.velocity.linear_x == pytest.approx(0.8)
    assert motion.velocity.frame_id == "world"
    assert arrived.velocity == Velocity2D(0, 0)


@pytest.mark.parametrize(
    "routes",
    [
        (((-2, 0.08), (2, 0.08)), ((2, -0.08), (-2, -0.08))),
        (((-2, 0), (2, 0)), ((0, -1.9), (0, 2.1))),
    ],
)
def test_reciprocal_head_on_and_crossing_clearance_and_replay(routes):
    context = _context(routes)
    trace = _rollout(context, 95)
    assert trace == _rollout(context, 95)
    assert (
        min(hypot(a.pose.x - b.pose.x, a.pose.y - b.pose.y) for a, b in trace)
        >= 0.5 - 1e-6
    )
    assert all(
        hypot(h.pose.x - h.goal.x, h.pose.y - h.goal.y) <= 0.1 for h in trace[-1]
    )
    assert all(h.velocity == Velocity2D(0, 0) for h in trace[-1])


def test_doorway_with_one_disc_and_solid_wall_avoidance():
    walls = (
        _wall("lower", -0.15, 0.15, -3, -0.35),
        _wall("upper", -0.15, 0.15, 0.35, 3),
    )
    context = _context((((-1, 0), (1, 0)),), walls)
    assert _rollout(context, 55)[-1][0].pose.x > 0.9
    solid = _context((((-1, 0), (1, 0)),), (_wall("wall", -0.15, 0.15, -3, 3),))
    trace = _rollout(solid, 100)
    assert all(humans[0].pose.x <= -0.4 + 1e-6 for humans in trace)
    assert abs(trace[-1][0].velocity.linear_x) < 0.02  # Isolated stall at a wall.


def test_symmetric_head_on_retains_deadlock():
    context = _context((((-2, 0), (2, 0)), ((2, 0), (-2, 0))))
    a, b = _rollout(context, 150)[-1]
    assert b.pose.x - a.pose.x == pytest.approx(0.5, abs=0.02)
    assert all(abs(h.velocity.linear_x) < 0.02 for h in (a, b))
    assert a.pose.x < 0 < b.pose.x


def test_refresh_overwrites_internal_predictions_and_robot_velocity():
    context = _context((((-1, 0), (3, 0)),))
    context = replace(context, robot_pose=Pose2D(0.3, 0, 0))
    backend = _backend(context)
    stopped = backend.propose(context)
    human = replace(
        context.humans[0], pose=Pose2D(-1.4, 0, 0), velocity=Velocity2D(0.1, 0)
    )
    measured = replace(
        context, humans=(human,), robot_world_velocity=Velocity2D(-0.45, 0)
    )
    assert backend.propose(measured) == _backend(measured).propose(measured)
    assert (
        _backend(replace(context, robot_world_velocity=Velocity2D(-0.45, 0))).propose(
            replace(context, robot_world_velocity=Velocity2D(-0.45, 0))
        )
        != stopped
    )
    with pytest.raises(ContractError, match="roster"):
        backend.propose(replace(measured, humans=()))
