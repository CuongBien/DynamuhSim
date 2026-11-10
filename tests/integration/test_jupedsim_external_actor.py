"""Native TASK 04S tests, explicitly selected using the experimental interpreter."""

from importlib.metadata import version
from math import dist, nan

import pytest

jps = pytest.importorskip("jupedsim")
if not hasattr(jps.Simulation, "apply_sfm_coupled_state"):
    pytest.skip("requires TASK 04S isolated JuPedSim build", allow_module_level=True)

from experiments.social_navigation.probe_jupedsim_coupling import ACTIONS, _add_agent
from experiments.social_navigation.probe_jupedsim_external_actor import (
    BRANCH_NS,
    identity,
    measure_neighbor_response,
    require_experimental_build,
    run_fixture,
)


def _simulation(model=None, dt=0.01):
    return jps.Simulation(
        model=jps.SocialForceModel() if model is None else model,
        geometry=[(-4, -2), (4, -2), (4, 2), (-4, 2)],
        dt=dt,
    )


def _batch(sim, time_ns, robot, human, rxy, rv, *, hxy=(0, 0), hv=(0, 0), steps=1):
    sim.apply_sfm_coupled_state(
        time_ns,
        steps,
        external=[(robot, rxy, rv)],
        pedestrians=[(human, hxy, hv)],
    )


def test_external_actor_is_read_only_for_sfm_and_expires_on_integer_boundary():
    if version("jupedsim") == "1.4.2+task04s":
        assert require_experimental_build()["distribution"] == "1.4.2+task04s"
    sim = _simulation()
    human = _add_agent(sim, (0, 0), (0, 0), 0, 0.25, goal=(2, 0))
    robot = sim.register_external_sfm_actor(0.18)
    assert robot != human
    with pytest.raises(RuntimeError, match="missing or expired"):
        sim.iterate()
    assert sim.iteration_count() == 0

    supplied = [
        ((1, 0), (0, 0)),
        ((1, 0.2), (0, 0.5)),
        ((0.9, 0.2), (-0.5, 0)),
    ]
    for index, (xy, vxy) in enumerate(supplied):
        time_ns = index * 100_000_000
        _batch(sim, time_ns, robot, human, xy, vxy, steps=10)
        for offset in range(11):
            assert sim.external_sfm_actor(robot) == (
                robot,
                xy,
                vxy,
                0.18,
                time_ns,
                True,
            )
            assert sim.iteration_count() == index * 10 + offset
            assert [agent.id for agent in sim.agents()] == [human]
            if offset < 10:
                sim.iterate()
        with pytest.raises(RuntimeError, match="missing or expired"):
            sim.iterate()
    assert sim.iteration_count() == 30


def test_atomic_rebase_preserves_handles_parameters_and_multistage_journey():
    sim = _simulation()
    first = sim.add_waypoint_stage((0, 0), 0.15)
    second = sim.add_waypoint_stage((2, 0), 0.15)
    journey = jps.JourneyDescription([first, second])
    journey.set_transition_for_stage(
        first, jps.Transition.create_fixed_transition(second)
    )
    route = sim.add_journey(journey)
    native = sim.add_agent(
        jps.SocialForceModelAgentParameters(
            position=(-2, 0),
            orientation=(1, 0),
            mass=70,
            reaction_time=0.6,
            radius=0.25,
            journey_id=route,
            stage_id=first,
        )
    )
    other = _add_agent(sim, (-2, -1), (0, 0), 0, 0.25, goal=(2, -1))
    agent = sim.agent(native)
    preserved = (
        identity(agent),
        agent.target,
        agent.orientation,
        agent.model.mass,
        agent.model.reaction_time,
    )
    robot = sim.register_external_sfm_actor(0.18)
    external = [(robot, (2.5, 1), (0, 0))]
    good = [(native, (-0.1, 0), (0.25, 0)), (other, (-2, -1), (0, 0))]
    # An invalid second update must not commit the valid first update.
    with pytest.raises(RuntimeError, match="invalid SFM synchronization"):
        sim.apply_sfm_coupled_state(
            0,
            1,
            external=external,
            pedestrians=[
                good[0],
                (other, (-2, -1), (nan, 0)),
            ],
        )
    assert agent.position == (-2, 0)
    assert list(sim.agents_in_range((-2, 0), 0.01)) == [native]
    assert sim.iteration_count() == 0
    sim.apply_sfm_coupled_state(0, 1, external=external, pedestrians=good)
    assert agent.position == (-0.1, 0) and agent.model.velocity == (0.25, 0)
    assert list(sim.agents_in_range((-0.1, 0), 0.01)) == [native]
    assert list(sim.agents_in_range((-2, 0), 0.01)) == []
    assert (
        identity(agent),
        agent.target,
        agent.orientation,
        agent.model.mass,
        agent.model.reaction_time,
    ) == preserved
    sim.iterate()
    assert identity(agent) == (native, route, second)


def test_native_rejects_bad_timestamp_roster_footprint_and_model():
    with pytest.raises(RuntimeError, match="dt=10 ms"):
        _simulation(dt=0.02).register_external_sfm_actor(0.18)
    with pytest.raises(RuntimeError, match="requires SFM"):
        _simulation(jps.CollisionFreeSpeedModel()).register_external_sfm_actor(0.18)
    sim = _simulation()
    human = _add_agent(sim, (0, 0), (0, 0), 0, 0.25, goal=(2, 0))
    with pytest.raises(RuntimeError, match="positive radius"):
        sim.register_external_sfm_actor(nan)
    robot = sim.register_external_sfm_actor(0.18)
    for bad_ns in (1, 10_000_000):
        with pytest.raises(RuntimeError, match="off-grid"):
            _batch(sim, bad_ns, robot, human, (1, 0), (0, 0))
    with pytest.raises(RuntimeError, match="full roster"):
        sim.apply_sfm_coupled_state(0, 1, external=[], pedestrians=[])
    with pytest.raises(RuntimeError, match="unknown or duplicate"):
        _batch(sim, 0, human, human, (1, 0), (0, 0))
    with pytest.raises(RuntimeError, match="invalid SFM synchronization"):
        _batch(sim, 0, robot, human, (1, 0), (nan, 0))
    with pytest.raises(RuntimeError, match="footprint"):
        _batch(sim, 0, robot, human, (3.95, 0), (0, 0))
    with pytest.raises(RuntimeError, match="coincident"):
        _batch(sim, 0, robot, human, (0, 0), (0, 0))
    _batch(sim, 0, robot, human, (1, 0), (0, 0))
    with pytest.raises(RuntimeError, match="duplicate"):
        _batch(sim, 0, robot, human, (1, 0), (0, 0))
    sim.iterate()
    with pytest.raises(RuntimeError, match="stale"):
        _batch(sim, 0, robot, human, (1, 0), (0, 0))
    assert sim.external_sfm_actor(robot) == (robot, (1, 0), (0, 0), 0.18, 0, True)
    assert sim.iteration_count() == 1


def test_neighbor_spatial_effect_and_tangential_overlap_separated():
    responses = {
        name: result["human_velocity_after_10ms"]
        for name, result in measure_neighbor_response().items()
    }
    separated = responses["separated_near_stop"]
    assert separated != responses["separated_far_stop"]
    assert separated == responses["separated_near_yield"]
    assert responses["overlap_stop"][1] == 0
    assert responses["overlap_yield"][1] > 0.9


def test_paired_mujoco_actions_resync_causality_and_replay():
    cases = {policy: run_fixture("head_on", policy, 11) for policy in ACTIONS}
    advance = cases["advance"]
    split = BRANCH_NS // 50_000_000
    for policy, case in cases.items():
        assert case["status"] == "fixture_completed"
        assert case["native_iterations"] == 300
        assert len(case["trace"]) == 60
        assert case["metadata"]["timing"] == {
            "physics_dt_ns": 2_000_000,
            "robot_control_dt_ns": 50_000_000,
            "human_control_dt_ns": 100_000_000,
            "observation_dt_ns": 50_000_000,
        }
        assert case["metadata"]["clocks"]["physics_steps"] == 1500
        assert case["internal_dt_ns"] == 10_000_000
        assert case["robot_sync_error_m"] == {
            "count": 330,
            "max": 0,
            "mean": 0,
            "p99": 0,
        }
        assert len({sample["robot_input"][0] for sample in case["samples"]}) == 1
        assert {sample["robot_domain_id"] for sample in case["samples"]} == {"robot"}
        assert (
            len(
                {
                    tuple(sample["pedestrian_native_identities"])
                    for sample in case["samples"]
                }
            )
            == 1
        )
        for index, sample in enumerate(case["samples"]):
            assert sample["time_ns"] == index * 100_000_000
            assert sample["iteration_before"] == index * 10
            assert sample["iteration_after"] == (index + 1) * 10
            first = case["trace"][index * 2]
            second = case["trace"][index * 2 + 1]
            assert (
                first["requested"]["humans"][0]["velocity"]
                == second["requested"]["humans"][0]["velocity"]
            )
            assert (
                first["limited"]["humans"][0]["velocity"]
                == second["limited"]["humans"][0]["velocity"]
            )
            xy = sample["pedestrian_rebase_input"][0][1]
            predicted = sample["pedestrian_predicted_xy"][0]
            request = first["requested"]["humans"][0]["velocity"]
            for axis, field, current, target in zip(
                ("x", "y"), ("linear_x", "linear_y"), xy, predicted, strict=True
            ):
                assert request[field] * 0.1 == pytest.approx(
                    target - current, abs=1e-15
                )
                end = second["ground_truth"]["humans"][0]["pose"][axis]
                executed = sum(
                    tick["executed"]["humans"][0]["velocity"][field] * 0.05
                    for tick in (first, second)
                )
                assert executed == pytest.approx(end - current, abs=1e-15)
            if index:
                previous = case["trace"][index * 2 - 1]
                assert sample["robot_input"][2] == previous["robot_world_velocity"]
                previous_pose = previous["ground_truth"]["robot"]["pose"]
                assert sample["robot_input"][1] == (
                    previous_pose["x"],
                    previous_pose["y"],
                )
                assert (
                    sample["time_ns"]
                    == previous["ground_truth"]["stamp"]["timestamp"]["time_ns"]
                )
                person = previous["ground_truth"]["humans"][0]
                assert sample["pedestrian_rebase_input"][0][1] == (
                    person["pose"]["x"],
                    person["pose"]["y"],
                )
                assert sample["pedestrian_rebase_input"][0][2] == (
                    person["velocity"]["linear_x"],
                    person["velocity"]["linear_y"],
                )
        for index in range(split + 1):
            # The robot command at split may differ; the human proposal sees the same start snapshot.
            assert (
                case["trace"][index]["requested"]["humans"]
                == advance["trace"][index]["requested"]["humans"]
            )
        robot = case["trace"][split]["requested"]["robot"]["velocity"]
        assert (robot["linear_x"], robot["linear_y"]) == ACTIONS[policy]
    assert cases["advance"]["trace"][-1]["robot_world_velocity"][0] > 0.3
    assert abs(cases["stop"]["trace"][-1]["robot_world_velocity"][0]) < 0.02
    assert cases["yield"]["trace"][-1]["robot_world_velocity"][1] < -0.18
    assert cases["reverse"]["trace"][-1]["robot_world_velocity"][0] < -0.2
    for idx in range(split + 1, len(advance["trace"])):
        a = advance["trace"][idx]["requested"]["humans"][0]["velocity"]
        b = cases["yield"]["trace"][idx]["requested"]["humans"][0]["velocity"]
        if dist((a["linear_x"], a["linear_y"]), (b["linear_x"], b["linear_y"])) > 0.1:
            break
    else:
        pytest.fail("no measured change in the human proposal between robot actions")
    for seed in (11, 12):
        replay = run_fixture("head_on", "advance", seed)
        assert [step["ground_truth"]["humans"] for step in replay["trace"]] == [
            step["ground_truth"]["humans"] for step in advance["trace"]
        ]
        assert (
            replay["metadata"]["episode"]["run_id"]
            != advance["metadata"]["episode"]["run_id"]
        )
        if seed != 11:
            assert (
                replay["metadata"]["human_backend_seed"]
                != advance["metadata"]["human_backend_seed"]
            )


def test_mujoco_negative_clearance_and_substep_contact_stay_authoritative():
    case = run_fixture("narrow_wall", "advance", 11)
    assert case["status"] == "interaction_failure"
    assert case["native_iterations"] == 300
    assert case["robot_sync_error_m"]["max"] == 0
    assert any(
        kind == "robot_human_m" and value < 0
        for _, kind, value in case["negative_clearances"]
    )
    assert any(
        sample["pedestrian_pre_rebase_error_m"][0] > 0.001 for sample in case["samples"]
    )
    assert any(
        tick["human_motion_blocked_substeps"]
        and tick["limited"]["humans"][0]["velocity"]
        != tick["executed"]["humans"][0]["velocity"]
        for tick in case["trace"]
    )
    contacts = [event for tick in case["trace"] for event in tick["contacts"]]
    assert contacts and all(event["kind"] == "robot_human" for event in contacts)
    assert all(event["time_ns"] % 2_000_000 == 0 for event in contacts)
    assert set(case["metadata"]["not_observable_solver"]) == {
        "human_wall",
        "human_human",
    }
