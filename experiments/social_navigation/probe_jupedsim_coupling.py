"""Bounded native SFM diagnostics for TASK 04R. This profile is never registered."""

import argparse
import json
from dataclasses import asdict, replace
from importlib.metadata import version
from math import cos, dist, sin
from pathlib import Path
from time import perf_counter

import jupedsim as jps
import jupedsim.native as jps_native
from jupedsim.models.social_force import SocialForceModelState

from narrow_nav.core.actions import HumanMotion, RobotAction
from narrow_nav.core.errors import ContractError
from narrow_nav.core.geometry import body_velocity_to_world
from narrow_nav.core.types import NS_PER_SECOND, Point2D, Twist2D, Velocity2D
from narrow_nav.humans.jupedsim_bridge.clock import BridgeClock
from narrow_nav.humans.jupedsim_bridge.geometry import build_walkable
from narrow_nav.humans.jupedsim_bridge.model_factory import create_simulation
from narrow_nav.humans.jupedsim_bridge.robot_proxy import RobotFootprintSample
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import build_world

ROOT = Path(__file__).resolve().parents[2]
CLOCK = BridgeClock(100_000_000)
GATE_STATUS = "BLOCKED_UPSTREAM_API"
PROFILE = "sfm_reconstruction_probe"
ACTIONS = {
    "advance": (0.35, 0.0),
    "stop": (0.0, 0.0),
    "yield": (0.0, -0.22),
    "reverse": (-0.25, 0.0),
}
FIXTURES = {
    "head_on": (),
    "straight": (
        "scenarios.robot.initial_pose=[-1.3,-1,0]",
        "scenarios.robot.goal=[4,-1]",
    ),
    "crossing": (
        "scenarios.robot.initial_pose=[0,-1.05,1.5707963267948966]",
        "scenarios.robot.goal=[0,1.1]",
        "scenarios.humans.0.initial_pose=[1.25,-0.22,-3.141592653589793]",
        "scenarios.humans.0.goal=[-4,-0.22]",
    ),
    "narrow_wall": (
        "worlds.width_m=0.85",
        "scenarios.robot.initial_pose=[3.8,0,0]",
        "scenarios.robot.goal=[2,0]",
        "scenarios.humans.0.initial_pose=[4.72,0,0]",
        "scenarios.humans.0.goal=[4.74,0]",
    ),
    "waypoint": (
        "scenarios.robot.initial_pose=[-0.5,0,0]",
        "scenarios.robot.goal=[4,0]",
        "scenarios.humans.0.initial_pose=[-2,-0.65,0]",
        "scenarios.humans.0.goal=[-0.08,0]",
    ),
}
COMMON_OVERRIDES = (
    "robots=mecanum_robot",
    "simulation.human_visuals=proxy",
    "worlds.width_m=3",
    "experiments.duration_ns=3000000000",
    "experiments.terminate_on_contact=false",
    f"humans.backend={PROFILE}",
    "scenarios.robot.initial_pose=[-1.3,-0.2,0]",
    "scenarios.robot.goal=[4,-0.2]",
    "scenarios.humans.0.initial_pose=[0.1,0.25,-3.141592653589793]",
    "scenarios.humans.0.goal=[-4,0.25]",
)


def _position(pose):
    return pose.x, pose.y


def _velocity(velocity):
    return velocity.linear_x, velocity.linear_y


def _motion(pose, velocity, dt):
    return tuple(p + v * dt for p, v in zip(pose, velocity, strict=True))


def _add_agent(sim, position, velocity, yaw, radius, *, goal=None):
    if goal is None:
        stage = sim.add_direct_steering_stage()
        target = position
    else:
        stage = sim.add_waypoint_stage(goal, 0.1)
        target = goal
    journey = sim.add_journey(jps.JourneyDescription([stage]))
    identifier = sim.add_agent(
        jps.SocialForceModelAgentParameters(
            position=position,
            orientation=(cos(yaw), sin(yaw)),
            velocity=velocity,
            radius=radius,
            desired_speed=0.0 if goal is None else 0.8,
            journey_id=journey,
            stage_id=stage,
        )
    )
    if goal is None:
        sim.agent(identifier).target = target
    return identifier


def _make_sim(polygon, clock=CLOCK):
    return create_simulation(jps.SocialForceModel(), polygon, clock)


def _runtime_error(operation):
    try:
        operation()
    except RuntimeError as error:
        return str(error)
    return None


def _api_geometry():
    built = build_world(compose_config(ROOT / "configs", COMMON_OVERRIDES))
    return build_walkable(inspect_world(built).planar_world())


def _isolated_reconstruction(polygon):
    start, goal = (-2, 0), (2, 0)
    continuous = _make_sim(polygon)
    original_id = _add_agent(continuous, start, (0, 0), 0, 0.25, goal=goal)
    original = continuous.agent(original_id)
    position, velocity = start, (0, 0)
    errors, ids = [], []
    for _ in range(10):
        rebuilt = _make_sim(polygon)
        new_id = _add_agent(rebuilt, position, velocity, 0, 0.25, goal=goal)
        ids.append(new_id)
        rebuilt.iterate(CLOCK.iterations_per_callback)
        continuous.iterate(CLOCK.iterations_per_callback)
        agent = rebuilt.agent(new_id)
        position, velocity = agent.position, agent.model.velocity
        errors.append(
            (dist(position, original.position), dist(velocity, original.model.velocity))
        )
    return {
        "max_xy_error_m": max(error[0] for error in errors),
        "max_velocity_error_mps": max(error[1] for error in errors),
        "continuous_iterations": continuous.iteration_count(),
        "distinct_rebuild_ids": len(set(ids)),
    }


def probe_public_api():
    geometry = _api_geometry()
    polygon = geometry.native_polygon()
    sim = _make_sim(polygon)
    first = _add_agent(sim, (-1, 0), (0, 0), 0, 0.25, goal=(2, 0))
    agent = sim.agent(first)
    surfaces = {}
    for field, value in (("position", (-2, 0)), ("orientation", (0, 1))):
        try:
            setattr(agent, field, value)
        except AttributeError as error:
            surfaces[field] = str(error)
        else:
            raise AssertionError(f"unexpected writable Agent.{field}")
    agent.velocity = (9, 9)
    wrapper_velocity = agent.model.velocity
    agent.model.velocity = (0.2, -0.1)
    native_velocity = sim.agent(first).model.velocity
    del agent

    assert sim.mark_agent_for_removal(first)
    old_visible = sim.agent(first).id == first
    duplicate_error = _runtime_error(
        lambda: _add_agent(sim, (-1, 0), (0, 0), 0, 0.25, goal=(2, 0))
    )
    before = sim.iteration_count()
    sim.iterate()
    after = sim.iteration_count()
    old_after_error = _runtime_error(lambda: sim.agent(first))
    native_reported_removed = tuple(sim.removed_agents())
    replacement = _add_agent(sim, (-1, 0), (0, 0), 0, 0.25, goal=(2, 0))

    def pair(distance, robot_velocity, steps=1):
        paired = _make_sim(polygon)
        human = _add_agent(paired, (0, 0), (0, 0), 0, 0.25, goal=(2, 0))
        robot_position = (distance, 0)
        robot = _add_agent(paired, robot_position, robot_velocity, 0, 0.18)
        proxy = paired.agent(robot)
        person = paired.agent(human)
        assert isinstance(person.model, SocialForceModelState)
        paired.iterate(steps)
        return {
            "human_xy": person.position,
            "human_velocity": person.model.velocity,
            "robot_start_xy": robot_position,
            "robot_shadow_xy": proxy.position,
            "robot_shadow_velocity": proxy.model.velocity,
            "robot_measured_velocity": robot_velocity,
            "robot_shadow_error_vs_linear_m": dist(
                proxy.position,
                _motion(robot_position, robot_velocity, steps * CLOCK.internal_dt_s),
            ),
            "native_iterations": paired.iteration_count(),
        }

    overlap_cases = {
        str(distance): _runtime_error(
            lambda distance=distance: pair(distance, (0, 0), steps=0)
        )
        for distance in (0, 0.1, 0.35)
    }
    pairs = {
        "separated_stop": pair(0.6, (0, 0)),
        "separated_yield": pair(0.6, (0, 0.4)),
        "overlap_stop": pair(0.35, (0, 0)),
        "overlap_yield": pair(0.35, (0, 0.4)),
        "overlap_reverse": pair(0.35, (-0.4, 0)),
        "autonomous_drift": pair(0.6, (0.35, 0), steps=10),
    }

    hole_sim = _make_sim(polygon)
    _add_agent(hole_sim, (-1, 0), (0, 0), 0, 0.25, goal=(2, 0))
    sample = RobotFootprintSample(Point2D(0, 0), Velocity2D(0, 0), 0.18, 0)
    hole_sim.switch_geometry(sample.carve(polygon))
    static_holes = len(hole_sim.get_geometry().holes())
    hole_errors = {}
    for name, point in (("agent", Point2D(-1, 0)), ("waypoint", Point2D(2, 0))):
        candidate = replace(sample, centre=point)
        hole_errors[name] = _runtime_error(
            lambda candidate=candidate: hole_sim.switch_geometry(
                candidate.carve(polygon)
            )
        )
    reentry = replace(sample, centre=Point2D(0, 0.5))
    hole_sim.switch_geometry(reentry.carve(polygon))
    return {
        "jupedsim": version("jupedsim"),
        "build": str(jps.get_build_info()),
        "model": jps.SocialForceModel.__name__,
        "geometry_hash": geometry.geometry_hash,
        "projection_error_m": geometry.projection_error_m,
        "position_orientation_write_errors": surfaces,
        "native_pose_setters": {
            field: getattr(jps_native.Agent, field).fset is not None
            for field in ("position", "orientation")
        },
        "wrapper_velocity_assignment_native_readback": wrapper_velocity,
        "model_velocity_setter_readback": native_velocity,
        "removal": {
            "old_visible_before_iterate": old_visible,
            "duplicate_error": duplicate_error,
            "before": before,
            "after": after,
            "old_lookup_after_iterate_error": old_after_error,
            "native_removed_agents_readback": native_reported_removed,
            "native_id_changed": replacement != first,
        },
        "insertion_overlap_errors": overlap_cases,
        "pairs": pairs,
        "isolated_reconstruction": _isolated_reconstruction(polygon),
        "geometry_switch": {
            "static_holes": static_holes,
            "errors": hole_errors,
            "reentry_holes": len(hole_sim.get_geometry().holes()),
            "native_iterations": hole_sim.iteration_count(),
        },
    }


class ObservedOverlap(ContractError):
    """The diagnostic retains the measured overlap and stops before advancing JPS."""


class ReconstructionProbe:
    """Fresh native SFM per callback with a reciprocating agent as a proxy."""

    backend_id = PROFILE
    backend_version = f"probe+jupedsim.{version('jupedsim')}"

    def __init__(self, geometry, endpoints):
        self.polygon = geometry.native_polygon(endpoints)

    def reset(self, config, seed, entity_ids):
        self.clock = BridgeClock(config.control_dt_ns)
        self.entity_ids = entity_ids
        self.samples = []
        self.iterations = 0

    def propose(self, context):
        stamp = context.stamp.timestamp
        ids = tuple(human.entity_id for human in context.humans)
        if (
            self.iterations != self.clock.iterations_at(stamp.time_ns)
            or ids != self.entity_ids
        ):
            raise ContractError("unexpected reconstruction callback boundary or roster")
        robot = RobotFootprintSample.from_context(context)
        rxy = (robot.centre.x, robot.centre.y)
        rv = _velocity(robot.velocity)
        sample = {
            "time_ns": stamp.time_ns,
            "robot_measured_xy": rxy,
            "robot_measured_yaw": context.robot_pose.yaw,
            "robot_measured_velocity_xy": rv,
            "native_iterations_before": self.iterations,
            "goal_covered_by_robot": [
                human.entity_id
                for human in context.humans
                if dist(rxy, (human.goal.x, human.goal.y)) <= robot.radius_m
            ],
            "human_robot_surface_clearance_m": {
                human.entity_id: dist(rxy, _position(human.pose))
                - robot.radius_m
                - context.human_radius_m
                for human in context.humans
            },
        }
        self.samples.append(sample)
        if any(
            clearance < 0
            for clearance in sample["human_robot_surface_clearance_m"].values()
        ):
            sample["error"] = "observed_robot_human_footprint_overlap"
            raise ObservedOverlap(sample["error"])
        sim = _make_sim(self.polygon, self.clock)
        human_ids = {
            human.entity_id: _add_agent(
                sim,
                _position(human.pose),
                _velocity(human.velocity),
                human.pose.yaw,
                context.human_radius_m,
                goal=(human.goal.x, human.goal.y),
            )
            for human in context.humans
        }
        robot_id = _add_agent(sim, rxy, rv, context.robot_pose.yaw, robot.radius_m)
        sample["agent_ids"] = human_ids
        sample["robot_inserted_pose_velocity"] = (
            sim.agent(robot_id).position,
            sim.agent(robot_id).model.velocity,
        )
        sample["local_iteration_before"] = sim.iteration_count()
        sim.iterate(self.clock.iterations_per_callback)
        count = sim.iteration_count()
        sample["local_iteration_after"] = count
        if count != self.clock.iterations_per_callback:
            raise ContractError("unexpected native reconstruction iteration count")
        self.iterations += count
        dt = self.clock.human_dt_ns / NS_PER_SECOND
        sample["native_iterations_after"] = self.iterations
        sample["robot_shadow_xy"] = sim.agent(robot_id).position
        sample["robot_shadow_velocity_xy"] = sim.agent(robot_id).model.velocity
        sample["robot_shadow_vs_constant_velocity_m"] = dist(
            sample["robot_shadow_xy"], _motion(rxy, rv, dt)
        )
        sample["human_predictions"] = {
            human.entity_id: {
                "initial_xy": _position(human.pose),
                "initial_velocity": _velocity(human.velocity),
                "predicted_xy": sim.agent(human_ids[human.entity_id]).position,
            }
            for human in context.humans
        }
        requests = []
        for entity_id, data in sample["human_predictions"].items():
            x, y = data["initial_xy"]
            predicted_x, predicted_y = data["predicted_xy"]
            velocity = Velocity2D((predicted_x - x) / dt, (predicted_y - y) / dt)
            requests.append(HumanMotion(entity_id, velocity, stamp))
        return tuple(requests)


def _compare_windows(samples, trace):
    ticks = {record["end_time_ns"]: record for record in trace}
    for sample in samples:
        end = sample["time_ns"] + CLOCK.human_dt_ns
        if "human_predictions" not in sample or end not in ticks:
            continue
        tick = ticks[end]
        sample["robot_shadow_vs_mujoco_m"] = dist(
            sample["robot_shadow_xy"], tick["robot_xy"]
        )
        for entity_id, data in sample["human_predictions"].items():
            actor = tick["humans"][entity_id]
            executed = actor["end_xy"]
            data["executed_xy"] = executed
            data["state_sync_error_m"] = dist(data["predicted_xy"], executed)


def probe_runtime(fixture, policy, seed):
    config = compose_config(ROOT / "configs", (*COMMON_OVERRIDES, *FIXTURES[fixture]))
    built = build_world(config)
    geometry = build_walkable(inspect_world(built).planar_world())
    endpoints = tuple(
        point
        for actor in (built.scene.robot, *built.scene.humans)
        for point in (Point2D(*_position(actor.pose)), actor.goal)
    )
    probe = ReconstructionProbe(geometry, endpoints)
    engine = MuJoCoEngine(human_backend=probe)
    trace = []
    started = perf_counter()
    failure = None
    try:
        engine.load(config)
        engine.reset(seed=seed)
        for _ in range(built.episode.max_ticks):
            before = engine.get_ground_truth()
            stamp = before.stamp.timestamp
            # One common approach, then the assigned policy. Policy takes no GT input.
            vx, vy = ACTIONS["advance" if stamp.time_ns < 700_000_000 else policy]
            try:
                step = engine.step(
                    RobotAction(before.robot.entity_id, Twist2D(vx, vy, 0), stamp)
                )
            except ObservedOverlap as error:
                failure = str(error)
                break
            truth = step.ground_truth
            trace.append(
                {
                    "start_time_ns": stamp.time_ns,
                    "end_time_ns": truth.stamp.timestamp.time_ns,
                    "robot_xy": _position(truth.robot.pose),
                    "robot_world_velocity": _velocity(
                        body_velocity_to_world(truth.robot.velocity, truth.robot.pose)
                    ),
                    "robot_requested_body": _velocity(step.requested.robot.velocity),
                    "humans": {
                        old.entity_id: {
                            "start_xy": _position(old.pose),
                            "end_xy": _position(new.pose),
                            "requested": _velocity(motion.requested.velocity),
                            "limited": _velocity(motion.limited.velocity),
                            "executed": _velocity(motion.executed.velocity),
                        }
                        for old, new, motion in zip(
                            before.humans,
                            truth.humans,
                            step.human_motion_results,
                            strict=True,
                        )
                    },
                    "contacts": [asdict(contact) for contact in step.contacts],
                    "clearance": asdict(step.clearance),
                    "guard_blocked_substeps": step.human_motion_blocked_substeps,
                }
            )
            if step.terminated or step.truncated:
                break
        metadata = engine.get_metadata()
        _compare_windows(probe.samples, trace)
        return {
            "fixture": fixture,
            "policy": policy,
            "branch_time_ns": 700_000_000,
            "geometry_hash": geometry.geometry_hash,
            "projection_error_m": geometry.projection_error_m,
            "model": jps.SocialForceModel.__name__,
            "coupling_profile": PROFILE,
            "status": GATE_STATUS,
            "failure": failure,
            "native_iterations": probe.iterations,
            "human_dt_ns": probe.clock.human_dt_ns,
            "internal_dt_ns": probe.clock.internal_dt_ns,
            "robot_ticks": len(trace),
            "wall_time_s": perf_counter() - started,
            "metadata": asdict(metadata),
            "samples": probe.samples,
            "trace": trace,
        }
    finally:
        engine.close()


def probe_contact_reference():
    """Measure native robot-human contacts on the narrow fixture with a stationary human."""
    config = compose_config(
        ROOT / "configs",
        (*COMMON_OVERRIDES, *FIXTURES["narrow_wall"], "humans.backend=scripted"),
    )
    engine = MuJoCoEngine()
    contacts, minimum_clearance = [], None
    try:
        engine.load(config)
        engine.reset(seed=11)
        for _ in range(engine.get_metadata().episode.config.max_ticks):
            state = engine.get_ground_truth()
            step = engine.step(
                RobotAction(
                    state.robot.entity_id, Twist2D(0.35, 0, 0), state.stamp.timestamp
                )
            )
            contacts.extend(asdict(event) for event in step.contacts)
            clearance = step.clearance.robot_human_m
            minimum_clearance = (
                clearance
                if minimum_clearance is None
                else min(minimum_clearance, clearance)
            )
            if step.terminated or step.truncated:
                break
        return {
            "min_robot_human_clearance_m": minimum_clearance,
            "native_contacts": contacts,
            "metadata": asdict(engine.get_metadata()),
        }
    finally:
        engine.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--require-gate", action="store_true", help="Exit 2 on a blocked gate."
    )
    parser.add_argument("--fixture", choices=FIXTURES)
    parser.add_argument("--policy", choices=ACTIONS, default="advance")
    parser.add_argument("--seed", type=int, default=11)
    args = parser.parse_args()
    if args.fixture:
        cases = [(args.fixture, args.policy, args.seed)]
    else:
        cases = [("head_on", action, 11) for action in ACTIONS]
        cases.extend(
            [
                ("head_on", "advance", 11),
                ("head_on", "advance", 12),
                ("straight", "advance", 11),
                ("crossing", "advance", 11),
                ("narrow_wall", "advance", 11),
                ("waypoint", "advance", 11),
            ]
        )
    print(
        json.dumps(
            {
                "gate_status": GATE_STATUS,
                "api": probe_public_api(),
                "runtime": [probe_runtime(*case) for case in cases],
                "stationary_contact_reference": probe_contact_reference()
                if not args.fixture
                else None,
            },
            allow_nan=False,
        )
    )
    return 2 if args.require_gate else 0


if __name__ == "__main__":
    raise SystemExit(main())
