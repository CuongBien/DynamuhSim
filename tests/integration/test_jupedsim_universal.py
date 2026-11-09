"""Native and MuJoCo acceptance of the TASK 04U shared bridge."""

import json
from dataclasses import asdict, replace
from importlib.metadata import version
from math import dist, isfinite, pi
from pathlib import Path

import pytest

jps = pytest.importorskip("jupedsim")
from omegaconf import OmegaConf
from shapely.geometry import box

from experiments.social_navigation.jupedsim_04u_parity import (
    MODELS,
    add_agent,
    simulation,
)
from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.geometry import body_velocity_to_world
from narrow_nav.core.states import HumanContext
from narrow_nav.core.types import ROBOT_BASE_FRAME, Twist2D, Velocity2D
from narrow_nav.humans.registry import create_human_backend
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import MJCF_ROOT

BUILD = version("jupedsim")
requires_04u = pytest.mark.skipif(
    BUILD != "1.4.2+task04u", reason="requires isolated 04U wheel"
)
STEP_NS = 50_000_000
DEFAULT_SCENE = (
    "worlds.width_m=3",
    "scenarios.robot.initial_pose=[-1.3,-0.2,0]",
    "scenarios.robot.goal=[4,-0.2]",
    "scenarios.humans.0.initial_pose=[0.1,0.25,-3.141592653589793]",
    "scenarios.humans.0.goal=[-4,0.25]",
)
HUMAN_PAIR = (
    "worlds.width_m=6",
    "scenarios.robot.initial_pose=[-4.5,-2.5,0]",
    "scenarios.robot.goal=[4,-2.5]",
)
PAIR_HUMANS = {
    "head_on": "scenarios.humans=[{id:human_a,initial_pose:[-2,0,0],goal:[2,0]},"
    "{id:human_b,initial_pose:[2,0.2,-3.141592653589793],goal:[-2,0.2]}]",
    "crossing": "scenarios.humans=[{id:human_a,initial_pose:[-2,0,0],goal:[2,0]},"
    "{id:human_b,initial_pose:[0,-2,1.5707963267948966],goal:[0,2]}]",
}


@pytest.mark.skipif(BUILD == "1.4.2+task04u", reason="stock and 04S rejection only")
@pytest.mark.parametrize("backend", ["avm_jupedsim", "csm_jupedsim"])
def test_older_build_rejects_04u_backends(backend):
    with pytest.raises(ConfigurationError, match=r"1.4.2\+task04u coupling APIs"):
        create_human_backend(backend)


@requires_04u
@pytest.mark.parametrize("backend", MODELS)
def test_native_external_response_and_transaction(backend):
    def response(robot_xy, robot_velocity):
        sim = simulation(backend, box(-6, -3, 6, 3))
        person = add_agent(sim, backend, (0, 0), (4, 0))
        actor = sim.register_external_actor(0.18, 1.0)
        agent = sim.agent(person)
        identity = agent.id, agent.journey_id, agent.stage_id
        before = agent.position, agent.orientation, sim.coupled_human_velocity(person)
        human_sample = (person, (0.05, 0), (0.4, 0), 0.2)
        robot_sample = (actor, robot_xy, robot_velocity, pi - 0.2)
        invalid = (
            (1, 10, robot_sample, human_sample, "off-grid"),
            (0, 11, robot_sample, human_sample, "off-grid"),
            (0, 10, robot_sample, (actor, *human_sample[1:]), "unknown or duplicate"),
            (0, 10, (*robot_sample[:2], (float("nan"), 0), 0), human_sample, "invalid"),
            (0, 10, (*robot_sample[:3], float("inf")), human_sample, "invalid"),
            (0, 10, (actor, (5.99, 0), (0, 0), 0), human_sample, "footprint"),
            (0, 10, (actor, human_sample[1], (0, 0), 0), human_sample, "coincident"),
        )
        for time_ns, steps, robot, human, match in invalid:
            with pytest.raises(RuntimeError, match=match):
                sim.apply_coupled_state(
                    time_ns, steps, external=[robot], pedestrians=[human]
                )
            assert (
                agent.position,
                agent.orientation,
                sim.coupled_human_velocity(person),
            ) == before
            assert sim.iteration_count() == 0
        with pytest.raises(RuntimeError, match="expired"):
            sim.iterate()
        sim.apply_coupled_state(
            0, 10, external=[robot_sample], pedestrians=[human_sample]
        )
        assert sim.iteration_count() == 0
        assert (agent.id, agent.journey_id, agent.stage_id) == identity
        assert agent.position == human_sample[1]
        if backend != "sfm_jupedsim":
            assert agent.orientation != before[1]
        assert sim.coupled_human_velocity(person) == (
            None if backend == "csm_jupedsim" else human_sample[2]
        )
        held = sim.external_actor(actor)
        assert held[:3] == robot_sample[:3]
        assert held[3] == pytest.approx(robot_sample[3])
        assert held[4:] == (0.18, 0)
        assert sim.agent_count() == 1
        assert actor not in list(sim.agents_in_range(robot_xy, 6))
        for step in range(10):
            if step == 5:
                with pytest.raises(RuntimeError, match="off-grid"):
                    sim.apply_coupled_state(
                        50_000_000,
                        5,
                        external=[robot_sample],
                        pedestrians=[human_sample],
                    )
            sim.iterate()
            assert sim.external_actor(actor) == held
        delta = agent.position
        with pytest.raises(RuntimeError, match="expired"):
            sim.iterate()
        moved = (actor, (4.5, 1.5), (0, 0), 0.0)
        sim.apply_coupled_state(
            100_000_000, 10, external=[moved], pedestrians=[human_sample]
        )
        assert sim.external_actor(actor)[1] == moved[1]
        assert sim.external_actor(actor)[5] == 100_000_000
        assert type(agent.model).__name__ == f"{MODELS[backend][0]}State"
        sim.iterate(10)
        return delta, agent.position

    far, far_next = response((4.5, 1.5), (0, 0))
    robot_xy = (1.2, 0.15) if backend == "avm_jupedsim" else (0.85, 0.15)
    stationary, stationary_next = response(robot_xy, (0, 0))
    approaching, approaching_next = response(robot_xy, (-0.7, 0))
    assert stationary_next == pytest.approx(far_next, abs=1e-12)
    assert approaching_next == pytest.approx(far_next, abs=1e-12)
    assert dist(far, stationary) > 1e-5
    if backend == "avm_jupedsim":
        assert dist(stationary, approaching) > 1e-5
    else:
        assert stationary == approaching
    print(
        json.dumps(
            {
                "backend": backend,
                "native_robot_presence_delta_m": dist(far, stationary),
                "native_robot_velocity_delta_m": dist(stationary, approaching),
            }
        )
    )


def _xy(value):
    return value.x, value.y


def _v(value):
    return value.linear_x, value.linear_y


def _run(backend_id, overrides, duration, command, mjcf_root=MJCF_ROOT):
    backend = create_human_backend(backend_id)
    config = compose_config(
        overrides=(
            f"humans={backend_id}",
            "robots=mecanum_robot",
            "simulation.human_visuals=proxy",
            f"experiments.duration_ns={duration}",
            *overrides,
        ),
    )
    if mjcf_root != MJCF_ROOT:
        resolved = OmegaConf.to_container(config, resolve=True)
        resolved["worlds"] = {"id": "bridge_doorway", "geometry_id": "bridge_doorway"}
        resolved["scenarios"]["world_id"] = "bridge_doorway"
        config = OmegaConf.create(resolved)
    engine = MuJoCoEngine(mjcf_root=mjcf_root, human_backend=backend)
    rows, sync, contacts, gt_10hz = [], [], [], []
    goal_seen = set()
    try:
        engine.load(config)
        engine.reset(seed=11)
        before = engine.get_ground_truth()
        initial = before
        gt_10hz.append(asdict(before))
        while True:
            stamp = before.stamp.timestamp
            step = engine.step(
                RobotAction(
                    before.robot.entity_id, Twist2D(command(stamp.time_ns), 0, 0), stamp
                )
            )
            after = step.ground_truth
            assert after.stamp.timestamp.time_ns - stamp.time_ns == STEP_NS
            sample = backend.last_sync
            if stamp.time_ns % 100_000_000 == 0:
                assert sample.time_ns == stamp.time_ns
                if stamp.time_ns == 0:
                    context = HumanContext.from_ground_truth(
                        before, robot_radius_m=0.18, human_radius_m=0.25
                    )
                    with pytest.raises(ContractError, match="world"):
                        replace(
                            context,
                            robot_world_velocity=Velocity2D(0, 0, ROBOT_BASE_FRAME),
                        )
                assert sample.robot_id == before.robot.entity_id
                assert sample.robot_position == _xy(before.robot.pose)
                assert sample.robot_velocity == _v(
                    body_velocity_to_world(before.robot.velocity, before.robot.pose)
                )
                assert sample.iteration_before * 10_000_000 == stamp.time_ns
                assert sample.iteration_after == sample.iteration_before + 10
                for p, old, motion in zip(
                    sample.pedestrians,
                    before.humans,
                    step.human_motion_results,
                    strict=True,
                ):
                    assert p.position == _xy(old.pose) and p.velocity == _v(
                        old.velocity
                    )
                    assert _v(motion.requested.velocity) == pytest.approx(
                        tuple(
                            (end - start) / 0.1
                            for start, end in zip(p.position, p.prediction, strict=True)
                        )
                    )
                sync.append(asdict(sample))
            else:
                assert rows[-1]["requested"] == [
                    _v(m.requested.velocity) for m in step.human_motion_results
                ]
                assert rows[-1]["limited"] == [
                    _v(m.limited.velocity) for m in step.human_motion_results
                ]
            for old, human, motion in zip(
                before.humans, after.humans, step.human_motion_results, strict=True
            ):
                assert motion.executed.entity_id == human.entity_id == old.entity_id
                assert dist(_xy(old.pose), _xy(human.pose)) <= 1.4 * 0.05 + 1e-9
                assert _v(motion.executed.velocity) == pytest.approx(
                    (
                        (human.pose.x - old.pose.x) / 0.05,
                        (human.pose.y - old.pose.y) / 0.05,
                    )
                )
                assert all(isfinite(v) for v in _v(motion.requested.velocity))
                if dist(_xy(human.pose), _xy(human.goal)) <= 0.1:
                    goal_seen.add(human.entity_id)
            if after.stamp.timestamp.time_ns % 100_000_000 == 0:
                gt_10hz.append(asdict(after))
            rows.append(
                {
                    "time_ns": after.stamp.timestamp.time_ns,
                    "robot": _xy(after.robot.pose),
                    "robot_velocity": _v(
                        body_velocity_to_world(after.robot.velocity, after.robot.pose)
                    ),
                    "robot_hold_lag_m": dist(
                        sample.robot_position, _xy(after.robot.pose)
                    ),
                    "humans": [_xy(h.pose) for h in after.humans],
                    "human_velocity": [_v(h.velocity) for h in after.humans],
                    "requested": [
                        _v(m.requested.velocity) for m in step.human_motion_results
                    ],
                    "limited": [
                        _v(m.limited.velocity) for m in step.human_motion_results
                    ],
                    "executed": [
                        _v(m.executed.velocity) for m in step.human_motion_results
                    ],
                    "clearance": asdict(step.clearance),
                    "guard_substeps": step.human_motion_blocked_substeps,
                }
            )
            contacts.extend(asdict(contact) for contact in step.contacts)
            before = after
            if step.terminated or step.truncated:
                break
        meta = engine.get_metadata()
        assert meta.human_backend_id == backend_id
        assert meta.human_backend_version.endswith(backend.build.binary_sha256)
        assert meta.clocks.physics_steps == len(rows) * 25
        assert meta.not_observable_solver == ("human_wall", "human_human")
        return {
            "model": backend.model_class,
            "build": asdict(backend.build),
            "metadata": asdict(meta),
            "geometry_hash": backend.geometry_hash,
            "rows": rows,
            "sync": sync,
            "contacts": contacts,
            "goals_seen": sorted(goal_seen),
            "gt_10hz": gt_10hz,
            "human_movement_m": max(
                dist(_xy(h.pose), xy)
                for h, xy in zip(initial.humans, rows[-1]["humans"], strict=True)
            ),
        }
    finally:
        engine.close()


def _write(tmp_path, name, trace):
    clearance = {
        key: min(
            (
                row["clearance"][key]
                for row in trace["rows"]
                if row["clearance"][key] is not None
            ),
            default=None,
        )
        for key in trace["rows"][0]["clearance"]
    }
    safe = not trace["contacts"] and all(
        value is None or value >= 0 for value in clearance.values()
    )
    if name in PAIR_HUMANS:
        reached = len(trace["goals_seen"]) == 2
    elif name == "doorway":
        reached = trace["metadata"]["end_reason"] == "goal_reached" and bool(
            trace["goals_seen"]
        )
    else:
        reached = False
    if name in ("advance", "stop", "advance_replay"):
        scenario = "DIAGNOSTIC_WINDOW"
    else:
        scenario = "SCENARIO_SUCCESS" if safe and reached else "SCENARIO_FAILURE"
    trace["scenario_status"] = scenario
    (tmp_path / f"{name}.json").write_text(json.dumps(trace, allow_nan=False) + "\n")
    print(
        json.dumps(
            {
                "fixture": name,
                "scenario_status": scenario,
                "backend": trace["metadata"]["human_backend_id"],
                "end_reason": trace["metadata"]["end_reason"],
                "clearance_m": clearance,
                "contacts": len(trace["contacts"]),
                "human_movement_m": trace["human_movement_m"],
                "human_goals_seen": trace["goals_seen"],
                "samples": len(trace["sync"]),
                "max_robot_hold_lag_m": max(
                    row["robot_hold_lag_m"] for row in trace["rows"]
                ),
                "max_pre_rebase_m": max(
                    p["pre_rebase_error_m"]
                    for s in trace["sync"]
                    for p in s["pedestrians"]
                ),
            }
        )
    )


@requires_04u
@pytest.mark.parametrize("backend", MODELS)
def test_mujoco_advance_stop_and_replay(backend, tmp_path):
    advance = _run(backend, DEFAULT_SCENE, 3_000_000_000, lambda _: 0.35)
    stop = _run(
        backend, DEFAULT_SCENE, 3_000_000_000, lambda t: 0.35 if t < 700_000_000 else 0
    )
    replay = _run(backend, DEFAULT_SCENE, 3_000_000_000, lambda _: 0.35)
    assert (
        advance["rows"] == replay["rows"] and advance["contacts"] == replay["contacts"]
    )
    assert advance["rows"][:14] == stop["rows"][:14]
    # The first human callback after branching reads only the new actual robot sample.
    divergence = max(
        dist(a["humans"][0], b["humans"][0])
        for a, b in zip(advance["rows"], stop["rows"], strict=False)
    )
    assert divergence > 1e-4
    assert advance["human_movement_m"] > 0.2 and stop["human_movement_m"] > 0.2
    print(
        json.dumps({"backend": backend, "paired_advance_stop_max_delta_m": divergence})
    )
    for name, trace in (
        ("advance", advance),
        ("stop", stop),
        ("advance_replay", replay),
    ):
        _write(tmp_path, name, trace)


@requires_04u
@pytest.mark.parametrize("backend", MODELS)
@pytest.mark.parametrize("fixture", PAIR_HUMANS)
def test_mujoco_human_pair_with_stationary_robot(backend, fixture, tmp_path):
    trace = _run(
        backend, (*HUMAN_PAIR, PAIR_HUMANS[fixture]), 8_000_000_000, lambda _: 0
    )
    assert trace["human_movement_m"] > 0.2
    assert min(row["clearance"]["human_human_m"] for row in trace["rows"]) >= 0
    assert (
        max(dist(row["robot"], trace["rows"][0]["robot"]) for row in trace["rows"])
        < 1e-3
    )
    _write(tmp_path, fixture, trace)


@requires_04u
@pytest.mark.parametrize("backend", MODELS)
def test_mujoco_doorway_with_waiting_space(backend, tmp_path):
    fixture = Path(__file__).resolve().parents[1] / "fixtures/jupedsim_04u/doorway.xml"
    world = tmp_path / "worlds/bridge_doorway"
    world.mkdir(parents=True)
    (world / "main.xml").write_bytes(fixture.read_bytes())
    for directory in ("templates", "models"):
        (tmp_path / directory).symlink_to(
            MJCF_ROOT / directory, target_is_directory=True
        )
    overrides = (
        "scenarios.robot.initial_pose=[-1.7,-0.05,0]",
        "scenarios.robot.goal=[0.9,-0.05]",
        "scenarios.humans.0.initial_pose=[1.5,0.05,-3.141592653589793]",
        "scenarios.humans.0.goal=[-3,0.05]",
    )
    trace = _run(
        backend,
        overrides,
        12_000_000_000,
        lambda t: 0 if t < 2_000_000_000 else 0.35,
        tmp_path,
    )
    assert trace["human_movement_m"] > 0.2
    _write(tmp_path, "doorway", trace)


@requires_04u
@pytest.mark.parametrize("backend", MODELS)
def test_mujoco_negative_corridor_retains_model_outcome(backend, tmp_path):
    overrides = (
        "worlds.width_m=0.8",
        "scenarios.robot.initial_pose=[-0.7,0,0]",
        "scenarios.robot.goal=[2,0]",
        "scenarios.humans.0.initial_pose=[0.5,0,-3.141592653589793]",
        "scenarios.humans.0.goal=[-2,0]",
    )
    trace = _run(backend, overrides, 4_000_000_000, lambda _: 0.35)
    _write(tmp_path, "negative_corridor", trace)
    if backend == "avm_jupedsim":
        assert trace["metadata"]["end_reason"] == "robot_contact"
        assert any(event["kind"] == "robot_human" for event in trace["contacts"])
        assert min(row["clearance"]["robot_human_m"] for row in trace["rows"]) < 0
    else:
        assert trace["metadata"]["end_reason"] == "time_budget"
        assert not trace["contacts"]
        assert not trace["goals_seen"]


@requires_04u
def test_csm_v2_solver_contact_with_zero_desired_speed(tmp_path):
    overrides = (
        "humans.desired_speed=0",
        "scenarios.robot.initial_pose=[-1,0,0]",
        "scenarios.robot.goal=[2,0]",
        "scenarios.humans.0.initial_pose=[0.25,0,0]",
        "scenarios.humans.0.goal=[-2,0]",
    )
    trace = _run("csm_jupedsim", overrides, 4_000_000_000, lambda _: 0.35)
    _write(tmp_path, "csm_contact", trace)
    assert trace["metadata"]["end_reason"] == "robot_contact"
    assert trace["human_movement_m"] == 0
    assert all(event["kind"] == "robot_human" for event in trace["contacts"])
    assert trace["contacts"]
    assert min(row["clearance"]["robot_human_m"] for row in trace["rows"]) < 0
