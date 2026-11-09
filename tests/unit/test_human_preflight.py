from dataclasses import replace
from math import hypot, inf, pi
from random import Random

import pytest

from narrow_nav.core.actions import HumanMotionRequest, validate_human_requests
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.metadata import SnapshotMetadata
from narrow_nav.core.protocols import HumanControllerBackend
from narrow_nav.core.seed import derive_seed
from narrow_nav.core.states import (
    GroundTruthState,
    HumanContext,
    HumanState,
    Obstacle,
    RobotState,
)
from narrow_nav.core.types import Point2D, Pose2D, Timestamp, Twist2D, Velocity2D
from narrow_nav.simulation.human_motion import HumanMotionLimits, heading_from_velocity
from narrow_nav.simulation.timing import Timing


@pytest.fixture
def context():
    time = Timestamp(2, 100_000_000)
    robot = RobotState("robot", Pose2D(5, 6, pi / 2), Twist2D(2, 1, 0), time)
    humans = (
        HumanState("human_a", Pose2D(1, 0, 0), Velocity2D(0, 0), Point2D(1, 0), time),
        HumanState("human_b", Pose2D(2, 0, 0), Velocity2D(0, 0), Point2D(3, 0), time),
    )
    truth = GroundTruthState(SnapshotMetadata("preflight:0", 42, time), robot, humans)
    obstacle = Obstacle("wall", ((Point2D(0, 1), Point2D(1, 1), Point2D(0, 2)),))
    return HumanContext.from_ground_truth(
        truth, robot_radius_m=0.18, human_radius_m=0.25, obstacles=(obstacle,)
    )


def test_context_projects_velocity_and_checks_obstacles(context):
    assert context.robot_pose == Pose2D(5, 6, pi / 2)
    assert context.robot_world_velocity.frame_id == "world"
    assert (
        context.robot_world_velocity.linear_x,
        context.robot_world_velocity.linear_y,
    ) == pytest.approx((-1, 2))
    obstacle = context.obstacles[0]
    (vertices,) = obstacle.polygons
    with pytest.raises(ContractError, match="duplicate"):
        replace(obstacle, polygons=((*vertices, vertices[0]),))
    with pytest.raises(ContractError, match="world"):
        replace(
            obstacle,
            polygons=((replace(vertices[0], frame_id="robot_base"), *vertices[1:]),),
        )
    with pytest.raises(ContractError, match="positive"):
        replace(context, human_radius_m=0)


def test_backend_port_accepts_ordered_requests_and_preserves_goal_actor(context):
    class FixtureBackend:
        def reset(self, config: float, seed: int, entity_ids: tuple[str, ...]) -> None:
            self.speed = config
            self.rng = Random(seed)
            self.ids = entity_ids

        def propose(self, context: HumanContext) -> tuple[HumanMotionRequest, ...]:
            return tuple(
                HumanMotionRequest(
                    entity_id, Velocity2D(speed, 0), context.stamp.timestamp
                )
                for entity_id, speed in zip(
                    self.ids, (0, self.speed * self.rng.random()), strict=True
                )
            )

    backend: HumanControllerBackend[float] = FixtureBackend()
    ids = tuple(human.entity_id for human in context.humans)
    stamp = context.stamp.timestamp
    seed = derive_seed(context.stamp.episode_seed, "human_backend")
    assert seed != derive_seed(context.stamp.episode_seed, "robot_policy")
    backend.reset(0.6, seed, ids)
    requests = backend.propose(context)
    validate_human_requests(requests, stamp, entity_ids=ids)
    assert requests[0].velocity == Velocity2D(0, 0)
    assert requests[1].velocity == Velocity2D(0.6 * Random(seed).random(), 0)
    other_seed = derive_seed(43, "human_backend")
    backend.reset(0.1, other_seed, ids)
    assert backend.propose(context)[1].velocity == Velocity2D(
        0.1 * Random(other_seed).random(), 0
    )
    backend.reset(0.6, seed, ids)
    assert backend.propose(context) == requests
    validate_human_requests((), stamp, entity_ids=())


def test_backend_requests_reject_wrong_roster_tick_frame_and_values(context):
    stamp = context.stamp.timestamp
    ids = ("human_a", "human_b")
    first, second = (HumanMotionRequest(key, Velocity2D(0, 0), stamp) for key in ids)
    for invalid, error in (
        ((second, first), "sorted"),
        ((first, first), "duplicate"),
        ((first,), "current human IDs"),
        ((first, replace(second, entity_id="stranger")), "current human IDs"),
        ((first, replace(second, timestamp=Timestamp(1, 50_000_000))), "timestamps"),
        ([first, second], "tuple"),
    ):
        with pytest.raises(ContractError, match=error):
            validate_human_requests(invalid, stamp, entity_ids=ids)
    with pytest.raises(ContractError, match="world"):
        replace(first, velocity=Velocity2D(0, 0, "robot_base"))
    with pytest.raises(ContractError, match="finite"):
        Velocity2D(inf, 0)


def test_human_limiter_caps_speed_and_acceleration_on_human_period():
    limits = HumanMotionLimits(1.4, 1.5)
    dt_ns = 100_000_000
    prior = Velocity2D(0, 0)
    request = Velocity2D(1e308, 1e308)
    for _ in range(15):
        limited = limits.limit(request, prior, dt_ns)
        assert hypot(limited.linear_x, limited.linear_y) <= 1.4 + 1e-12
        assert (
            hypot(limited.linear_x - prior.linear_x, limited.linear_y - prior.linear_y)
            <= 0.15 + 1e-12
        )
        prior = limited
    assert hypot(prior.linear_x, prior.linear_y) == pytest.approx(1.4)
    assert heading_from_velocity(prior, -pi) == pytest.approx(pi / 4)
    assert heading_from_velocity(Velocity2D(0, 0), -pi) == -pi
    stopping = limits.limit(Velocity2D(0, 0), prior, dt_ns)
    assert hypot(stopping.linear_x, stopping.linear_y) == pytest.approx(1.25)
    assert limits.limit(Velocity2D(0, 0), Velocity2D(0, 0), dt_ns) == Velocity2D(0, 0)
    with pytest.raises(ContractError, match="speed cap"):
        limits.limit(request, Velocity2D(2, 0), dt_ns)
    with pytest.raises(ContractError, match="human_dt_ns"):
        limits.limit(request, prior, 0)


def test_boundary_schedule_and_bridge_divisibility():
    timing = Timing(2_000_000, 50_000_000, 100_000_000, 50_000_000)
    assert [tick for tick in range(6) if timing.human_control_due(tick)] == [0, 2, 4]
    assert timing.human_substeps(10_000_000) == 10
    assert timing.human_substeps(100_000_000) == 1
    with pytest.raises(ConfigurationError, match="must divide"):
        timing.human_substeps(30_000_000)
    with pytest.raises(ContractError, match="internal_dt_ns"):
        timing.human_substeps(0)
    with pytest.raises(ContractError, match="robot_tick"):
        timing.clocks(-1)
    asynchronous = replace(timing, human_control_dt_ns=66_000_000)
    assert asynchronous.clocks(2).last_human_control_time_ns == 66_000_000
    with pytest.raises(ConfigurationError, match="align"):
        asynchronous.human_control_due(2)
