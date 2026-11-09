"""Native AVM diagnostics for the TASK 04 coupling gate."""

import json
from importlib.metadata import version
from math import dist

import pytest

jps = pytest.importorskip("jupedsim")
from shapely.geometry import Point

from narrow_nav.core.states import HumanContext
from narrow_nav.humans.jupedsim_bridge.clock import BridgeClock
from narrow_nav.humans.jupedsim_bridge.geometry import build_walkable
from narrow_nav.humans.jupedsim_bridge.model_factory import create_simulation
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import build_world

BUILD = version("jupedsim")
pytestmark = pytest.mark.skipif(
    BUILD not in ("1.4.2", "1.4.2+task04s", "1.4.2+task04u"),
    reason="audited JuPedSim 1.4.2 builds",
)
CLOCK = BridgeClock(100_000_000)
SEED = 11


def _add_agent(sim, start, goal, radius):
    stage = sim.add_waypoint_stage(goal, 0.1)
    journey = sim.add_journey(jps.JourneyDescription([stage]))
    return sim.add_agent(
        jps.AnticipationVelocityModelAgentParameters(
            position=start, radius=radius, journey_id=journey, stage_id=stage
        )
    )


def _native_avm(polygon):
    return create_simulation(
        jps.AnticipationVelocityModel(rng_seed=SEED), polygon, CLOCK
    )


@pytest.mark.parametrize(
    ("fixture", "second_route"),
    [("crossing", ((0, -2), (0, 2))), ("head_on", ((2, 0.15), (-2, 0.15)))],
)
def test_native_avm_anticipation_changes_precontact_motion(fixture, second_route):
    world = build_world(compose_config(overrides=("worlds.width_m=6",)))
    geometry = build_walkable(inspect_world(world).planar_world())
    polygon = geometry.native_polygon()
    radius = jps.AnticipationVelocityModelAgentParameters().radius

    def branch(anticipation_time):
        sim = _native_avm(polygon)
        ids = (
            _add_agent(sim, (-2, 0), (2, 0), radius),
            _add_agent(sim, *second_route, radius),
        )
        sim.iterate(100)
        before = tuple(sim.agent(i).position for i in ids)
        for agent_id in ids:
            model = sim.agent(agent_id).model
            assert type(model) is jps.AnticipationVelocityModelState
            model.anticipation_time = anticipation_time
        sim.iterate()
        assert sim.iteration_count() == 101
        return before, tuple(sim.agent(i).position for i in ids)

    before, predicted = branch(1.0)
    baseline_before, no_anticipation = branch(0.0)
    assert (before, predicted) == branch(1.0)
    assert baseline_before == before
    clearance = min(
        dist(*points) - 2 * radius for points in (before, predicted, no_anticipation)
    )
    response_delta = max(
        dist(a, b) for a, b in zip(predicted, no_anticipation, strict=True)
    )
    peak_speed = max(
        dist(a, b) / CLOCK.internal_dt_s
        for after in (predicted, no_anticipation)
        for a, b in zip(before, after, strict=True)
    )
    assert clearance > 0
    assert response_delta > 1e-6
    assert 0 < peak_speed <= 1.2 + 1e-9
    assert min(polygon.boundary.distance(Point(p)) - radius for p in predicted) > 0
    print(
        json.dumps(
            {
                "scope": "native_avm_2d_only",
                "fixture": fixture,
                "distribution": BUILD,
                "seed": SEED,
                "internal_dt_s": CLOCK.internal_dt_s,
                "branch_time_s": 1.0,
                "geometry_hash": geometry.geometry_hash,
                "start_xy_m": before,
                "anticipation_1s_end_xy_m": predicted,
                "anticipation_0s_end_xy_m": no_anticipation,
                "min_pair_clearance_at_branch_m": clearance,
                "max_step_response_delta_m": response_delta,
                "max_branch_speed_mps": peak_speed,
            }
        )
    )


def test_legacy_api_cannot_accept_mujoco_robot_or_rebase_avm_pedestrian():
    config = compose_config(overrides=("simulation.human_visuals=proxy",))
    scene = build_world(config)
    geometry = build_walkable(inspect_world(scene).planar_world())
    engine = MuJoCoEngine()
    try:
        engine.load(config)
        engine.reset(seed=SEED)
        context = HumanContext.from_ground_truth(
            engine.get_ground_truth(),
            robot_radius_m=scene.scene.robot.radius_m,
            human_radius_m=scene.scene.humans[0].radius_m,
        )
    finally:
        engine.close()

    sim = _native_avm(geometry.native_polygon())
    (human,) = context.humans
    xy = human.pose.x, human.pose.y
    agent_id = _add_agent(sim, xy, (human.goal.x, human.goal.y), context.human_radius_m)
    agent = sim.agent(agent_id)
    with pytest.raises(AttributeError):
        agent.position = (xy[0] - 0.01, xy[1])
    assert not hasattr(agent.model, "velocity")
    native = agent.model._obj
    assert not hasattr(native, "velocity")

    errors = {}
    if BUILD in ("1.4.2+task04s", "1.4.2+task04u"):
        match = "experimental SFM coupling requires SFM with dt=10 ms"
        with pytest.raises(RuntimeError, match=match) as registration:
            sim.register_external_sfm_actor(context.robot_radius_m)
        with pytest.raises(RuntimeError, match=match) as rebase:
            sim.apply_sfm_coupled_state(
                0,
                CLOCK.iterations_per_callback,
                external=[],
                pedestrians=[(agent_id, (xy[0] - 0.01, xy[1]), (0.1, 0.0))],
            )
        errors = {
            "external_robot": str(registration.value),
            "human_rebase": str(rebase.value),
        }
    else:
        for method in ("register_external_sfm_actor", "apply_sfm_coupled_state"):
            with pytest.raises(AttributeError) as missing:
                getattr(sim, method)
            errors[method] = str(missing.value)
    assert sim.iteration_count() == 0
    assert sim.agent_count() == 1
    assert agent.position == xy
    print(
        json.dumps(
            {
                "scope": "legacy_avm_robot_coupling_gate",
                "status": "BLOCKED_CAPABILITY",
                "distribution": BUILD,
                "mujoco_sample_time_ns": context.stamp.timestamp.time_ns,
                "mujoco_robot_radius_m": context.robot_radius_m,
                "avm_iterations": sim.iteration_count(),
                "api_errors": errors,
            }
        )
    )
