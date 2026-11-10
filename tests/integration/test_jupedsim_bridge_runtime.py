"""Real CFM v1 probes for the shared bridge. No research backend is registered."""

import json
from dataclasses import replace
from importlib.metadata import version
from math import cos, dist, hypot, pi

import pytest

jps = pytest.importorskip("jupedsim")
from shapely.geometry import LineString, Point

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ContractError
from narrow_nav.core.states import HumanContext
from narrow_nav.core.types import Point2D, Twist2D, Velocity2D
from narrow_nav.humans.jupedsim_bridge.clock import BridgeClock
from narrow_nav.humans.jupedsim_bridge.geometry import build_walkable
from narrow_nav.humans.jupedsim_bridge.model_factory import create_simulation
from narrow_nav.humans.jupedsim_bridge.robot_proxy import (
    UNSUPPORTED_CAPABILITIES,
    RobotFootprintSample,
)
from narrow_nav.humans.jupedsim_bridge.state_sync import StateSync
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.mujoco.geometry import inspect_mjcf
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import build_world

_MAX_SYNC_ERROR_M = 0.25
_CFM = "CollisionFreeSpeedModel"


def _add_agent(sim, position, goal, radius, *, speed=0.55):
    stage = sim.add_waypoint_stage(goal, 0.1)
    journey = sim.add_journey(jps.JourneyDescription([stage]))
    return sim.add_agent(
        jps.CollisionFreeSpeedModelAgentParameters(
            position=position,
            radius=radius,
            desired_speed=speed,
            journey_id=journey,
            stage_id=stage,
        )
    )


class _BridgeProbe:
    """Test-only composition of the shared bridge with a static robot footprint."""

    backend_id = "jupedsim_bridge_probe"
    backend_version = version("jupedsim")

    def __init__(self, geometry):
        self.geometry = geometry

    def reset(self, config, seed, entity_ids):
        self.clock = BridgeClock(config.control_dt_ns)
        self.sync = self.sim = None
        self.samples = []
        self.robot = None
        self.entity_ids = entity_ids

    def propose(self, context):
        robot = RobotFootprintSample.from_context(context)
        if self.sync is None:
            assert self.entity_ids == tuple(h.entity_id for h in context.humans)
            self.robot = robot
            endpoints = (robot.centre,) + tuple(
                point
                for human in context.humans
                for point in (Point2D(human.pose.x, human.pose.y), human.goal)
            )
            polygon = robot.carve(self.geometry.native_polygon(endpoints))
            self.sim = create_simulation(
                jps.CollisionFreeSpeedModel(), polygon, self.clock
            )
            ids = {}
            for human in context.humans:
                self.geometry.validate_route(
                    Point2D(human.pose.x, human.pose.y),
                    human.goal,
                    context.human_radius_m,
                )
                ids[human.entity_id] = _add_agent(
                    self.sim,
                    (human.pose.x, human.pose.y),
                    (human.goal.x, human.goal.y),
                    context.human_radius_m,
                )
            self.sync = StateSync(
                self.sim,
                self.clock,
                ids,
                context,
                max_position_error_m=_MAX_SYNC_ERROR_M,
            )
        assert hypot(robot.velocity.linear_x, robot.velocity.linear_y) <= 1e-3
        error = self.robot.error_bound_m(robot)
        assert error <= 1e-3
        motions = self.sync.propose(context)
        self.samples.append(
            (context.stamp.timestamp.time_ns, self.sync.last_sync, error)
        )
        return motions


def _context(engine, scene):
    return HumanContext.from_ground_truth(
        engine.get_ground_truth(),
        robot_radius_m=scene.robot.radius_m,
        human_radius_m=scene.humans[0].radius_m if scene.humans else 0.25,
    )


def test_cfm_v1_shadow_against_ten_seconds_of_mujoco_gt_pause_reset_and_replay():
    config = compose_config(
        overrides=(
            "worlds.width_m=3",
            "simulation.human_visuals=proxy",
            "experiments.duration_ns=10000000000",
            "humans.backend=jupedsim_bridge_probe",
            "scenarios.robot.initial_pose=[0,0.55,0]",
            "scenarios.robot.goal=[4,0.55]",
            "scenarios.humans.0.initial_pose=[-4,-0.7,0]",
            "scenarios.humans.0.goal=[4,-0.35]",
        )
    )
    built = build_world(config)
    geometry = build_walkable(inspect_world(built).planar_world())
    backend = _BridgeProbe(geometry)
    engine = MuJoCoEngine(human_backend=backend)
    engine.load(config)
    episodes, traces = [], []
    try:
        for _ in range(2):
            engine.reset(seed=11)
            episodes.append(engine.get_metadata().episode.episode_id)
            assert backend.sync is None
            trace = []
            for tick in range(1, 201):
                start = engine.get_ground_truth()
                step = engine.stop()
                truth = step.ground_truth
                assert truth.stamp.timestamp.time_ns == tick * 50_000_000
                assert step.clearance.human_wall_m > 0
                assert step.clearance.robot_human_m > 0
                assert not step.contacts and not step.human_motion_blocked_substeps
                (old,), (new,) = start.humans, truth.humans
                (motion,) = step.human_motion_results
                assert motion.executed.velocity.linear_x == pytest.approx(
                    (new.pose.x - old.pose.x) / 0.05
                )
                assert motion.executed.velocity.linear_y == pytest.approx(
                    (new.pose.y - old.pose.y) / 0.05
                )
                assert (
                    dist((new.pose.x, new.pose.y), (old.pose.x, old.pose.y))
                    <= 1.4 * 0.05 + 1e-9
                )
                assert backend.sim.iteration_count() == ((tick + 1) // 2) * 10
                if tick == 1:
                    assert motion.requested.velocity != motion.limited.velocity
                if tick % 2 == 0:
                    assert trace[-1][2] == motion.requested.velocity
                trace.append(
                    (
                        new.pose,
                        truth.robot.pose,
                        motion.requested.velocity,
                        motion.executed.velocity,
                    )
                )
                if tick == 100:
                    paused = _context(engine, built.scene)
                    before = backend.sim.iteration_count()
                    assert backend.sync.compare(paused)
                    assert backend.sync.compare(paused)
                    assert backend.sim.iteration_count() == before == 500
            end = _context(engine, built.scene)
            final = backend.sync.compare(end)
            assert step.truncated and end.stamp.timestamp.time_ns == 10_000_000_000
            assert backend.sim.iteration_count() == 1000
            assert backend.sim.elapsed_time() == pytest.approx(10, abs=1e-9)
            assert engine.get_metadata().clocks.physics_steps == 5000
            assert (
                engine.get_metadata().clocks.last_observation_time_ns == 10_000_000_000
            )
            assert len(backend.samples) == 100
            assert len({stamp for stamp, _, _ in backend.samples}) == 100
            assert trace[-1][0].x - trace[0][0].x > 5
            assert abs(trace[-1][0].y - trace[0][0].y) > 0.2
            traces.append(trace)
        assert episodes[0] != episodes[1]
        assert traces[0] == traces[1]
        print(
            json.dumps(
                {
                    "mode": "native_cfm_v1_static_robot_geometry_probe",
                    "model": _CFM,
                    "commit": engine.get_metadata().episode.git_commit,
                    "episode_ids": episodes,
                    "episode_seed": 11,
                    "geometry_hash": geometry.geometry_hash,
                    "projection_error": geometry.projection_error_m,
                    "internal_dt": backend.clock.internal_dt_s,
                    "unsupported_capabilities": UNSUPPORTED_CAPABILITIES,
                    "robot_proxy_error": max(error for _, _, error in backend.samples),
                    "state_sync_error": max(
                        item.position_error_m
                        for _, items, _ in backend.samples
                        for item in items
                    ),
                    "state_sync_limit_m": _MAX_SYNC_ERROR_M,
                    "paired_xy_every_second": [
                        {
                            "time_ns": stamp,
                            "predicted_xy_m": samples[0].predicted_xy_m,
                            "executed_xy_m": samples[0].executed_xy_m,
                        }
                        for stamp, samples, _ in backend.samples[::10]
                    ],
                    "final_state_sync": {
                        "predicted_xy_m": final[0].predicted_xy_m,
                        "executed_xy_m": final[0].executed_xy_m,
                        "error_m": final[0].position_error_m,
                    },
                    "duration_s": backend.sim.elapsed_time(),
                    "physics_steps": engine.get_metadata().clocks.physics_steps,
                    "native_iterations": backend.sim.iteration_count(),
                    "replay_equal": traces[0] == traces[1],
                }
            )
        )
        # Current native position cannot be restored by the public API after drift.
        (human,) = end.humans
        invalid = replace(
            end, humans=(replace(human, pose=replace(human.pose, x=human.pose.x + 1)),)
        )
        with pytest.raises(ContractError, match="state_sync_error"):
            backend.sync.propose(invalid)
        assert backend.sync.last_sync[0].position_error_m > _MAX_SYNC_ERROR_M
        assert backend.sim.iteration_count() == 1000
        with pytest.raises(ContractError, match="iteration count"):
            backend.sync.propose(paused)
    finally:
        engine.close()


@pytest.mark.parametrize("obstacle", ("wall", "robot_static"))
def test_native_cfm_avoids_static_obstacle(obstacle):
    # Use an authored MJCF shape in the wall case and the same shared projection for both.
    wall = '<geom name="barrier" type="box" pos="0 -.6 .5" size=".1 .7 .5" group="1"/>'
    xml = f"""
    <mujoco><worldbody>
      <geom name="floor" type="box" pos="0 0 -.05" size="3 1.5 .05" group="0"/>
      <geom name="edge" type="box" pos="-3.1 0 .5" size=".1 1.5 .5" group="1"/>
      {wall if obstacle == "wall" else ""}
    </worldbody></mujoco>"""
    geometry = build_walkable(inspect_mjcf(xml).planar_world())
    polygon = geometry.native_polygon()
    if obstacle == "robot_static":
        sample = RobotFootprintSample(Point2D(0, -0.55), Velocity2D(0, 0), 0.25, 0)
        polygon = sample.carve(polygon)
    sim = create_simulation(
        jps.CollisionFreeSpeedModel(), polygon, BridgeClock(100_000_000)
    )
    start, goal = (-2, -0.48), (2, -0.48)
    assert not polygon.covers(LineString([start, goal]))
    agent = sim.agent(_add_agent(sim, start, goal, 0.20, speed=0.8))
    trajectory = [start]
    for _ in range(1000):
        sim.iterate()
        trajectory.append(agent.position)
        if dist(agent.position, goal) <= 0.12:
            break
    assert all(polygon.contains(Point(*xy)) for xy in trajectory)
    assert min(polygon.boundary.distance(Point(*xy)) for xy in trajectory) > 0.19
    assert dist(agent.position, goal) <= 0.12
    print(
        obstacle,
        f"native_iterations={sim.iteration_count()}",
        f"clearance_m={min(polygon.boundary.distance(Point(*xy)) for xy in trajectory) - 0.2:.9f}",
    )


def test_native_switch_geometry_tracks_sampled_mujoco_robot_but_blocks_overlap():
    config = compose_config(
        overrides=(
            "worlds.width_m=3",
            "simulation.human_visuals=proxy",
            "scenarios.humans=[]",
            f"scenarios.robot.initial_pose=[-1,-0.85,{pi / 2}]",
            "scenarios.robot.goal=[4,0.85]",
        )
    )
    built = build_world(config)
    geometry = build_walkable(inspect_world(built).planar_world())
    polygon = geometry.native_polygon()
    clock = BridgeClock(100_000_000)
    sim = create_simulation(jps.CollisionFreeSpeedModel(), polygon, clock)
    start, goal = (-1.8, -0.52), (3, -0.52)
    agent = sim.agent(_add_agent(sim, start, goal, 0.25, speed=0.8))
    reference = create_simulation(jps.CollisionFreeSpeedModel(), polygon, clock)
    baseline = reference.agent(_add_agent(reference, start, goal, 0.25, speed=0.8))
    engine = MuJoCoEngine()
    engine.load(config)
    engine.reset(seed=11)
    error = []
    samples = []
    robot_trace = []
    try:
        for tick in range(20):
            context = _context(engine, built.scene)
            observed = RobotFootprintSample.from_context(context)
            if tick % 2 == 0:
                sampled = observed
                samples.append(sampled)
                before = sim.iteration_count()
                assert before == clock.iterations_at(observed.time_ns)
                sim.switch_geometry(sampled.carve(polygon))
                assert sim.iteration_count() == before
                (hole,) = sim.get_geometry().holes()
                assert dist(
                    hole[0], (sampled.centre.x, sampled.centre.y)
                ) == pytest.approx(sampled.radius_m / cos(pi / 64))
                sim.iterate(clock.iterations_per_callback)
                reference.iterate(clock.iterations_per_callback)
            step = engine.step(
                RobotAction(
                    context.robot_entity_id,
                    Twist2D(0.35, 0, 0),
                    context.stamp.timestamp,
                )
            )
            robot_trace.append(step.ground_truth.robot.pose)
            error.append(
                sampled.error_bound_m(
                    RobotFootprintSample.from_context(_context(engine, built.scene))
                )
            )
        assert samples[-1].centre.y - samples[0].centre.y > 0.15
        assert samples[-1].velocity.linear_y > 0.25
        assert abs(samples[-1].velocity.linear_x) < 1e-4
        assert 0.015 < max(error) < 0.04
        assert sim.iteration_count() == 100
        assert sim.agent_count() == 1
        position_difference = dist(agent.position, baseline.position)
        assert position_difference > 0.05
        with pytest.raises(AttributeError, match="no setter"):
            agent.position = (1, 0)
        with pytest.raises(AttributeError, match="no setter"):
            agent.orientation = (1, 0)
        assert not hasattr(type(agent.model), "velocity")
        with pytest.raises(RuntimeError, match="agents would be outside"):
            sim.switch_geometry(
                replace(sampled, centre=Point2D(*agent.position)).carve(polygon)
            )
        with pytest.raises(RuntimeError, match="stages would be outside"):
            sim.switch_geometry(replace(sampled, centre=Point2D(*goal)).carve(polygon))
        assert sim.iteration_count() == 100
        engine.reset(seed=11)
        for pose in robot_trace:
            robot = engine.get_ground_truth().robot
            result = engine.step(
                RobotAction(robot.entity_id, Twist2D(0.35, 0, 0), robot.timestamp)
            )
            assert result.ground_truth.robot.pose == pose
        print(
            json.dumps(
                {
                    "mode": "moving_robot_sampled_hole_probe",
                    "status": "BLOCKED_CAPABILITY",
                    "geometry_hash": geometry.geometry_hash,
                    "projection_error": geometry.projection_error_m,
                    "internal_dt": clock.internal_dt_s,
                    "unsupported_capabilities": UNSUPPORTED_CAPABILITIES,
                    "robot_proxy_error": max(error),
                    "robot_displacement_m": dist(
                        (samples[-1].centre.x, samples[-1].centre.y),
                        (samples[0].centre.x, samples[0].centre.y),
                    ),
                    "measured_last_robot_velocity_y_mps": samples[-1].velocity.linear_y,
                    "native_iterations": sim.iteration_count(),
                    "agent_and_stage_overlap_switch_rejected": True,
                    "position_difference_vs_no_robot_hole_m": position_difference,
                    "robot_pose_replay_equal_without_proxy": True,
                }
            )
        )
    finally:
        engine.close()
