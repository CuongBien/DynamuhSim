"""TASK 04S experimental SFM boundary probe. No backend is registered."""

import argparse
import json
import sys
from dataclasses import asdict
from hashlib import sha256
from importlib.metadata import version
from math import ceil, dist
from pathlib import Path
from statistics import mean

import jupedsim as jps
import jupedsim.native as jps_native

from narrow_nav.core.actions import HumanMotion, RobotAction
from narrow_nav.core.errors import ContractError
from narrow_nav.core.geometry import body_velocity_to_world
from narrow_nav.core.types import NS_PER_SECOND, Point2D, Twist2D, Velocity2D
from narrow_nav.humans.jupedsim_bridge.clock import BridgeClock
from narrow_nav.humans.jupedsim_bridge.geometry import build_walkable
from narrow_nav.humans.jupedsim_bridge.model_factory import create_simulation
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.world_builder import build_world

from .probe_jupedsim_coupling import (
    ACTIONS,
    COMMON_OVERRIDES,
    FIXTURES,
    ROOT,
    _add_agent,
)

PROFILE = "sfm_external_hold_probe"
BRANCH_NS = 700_000_000


def require_experimental_build():
    package = Path(jps.__file__).resolve()
    binary = Path(jps_native.py_jps.__file__).resolve()
    prefix = Path(sys.prefix).resolve()
    if (
        version("jupedsim") != "1.4.2+task04s"
        or not package.is_relative_to(prefix)
        or not binary.is_relative_to(prefix)
        or not hasattr(jps_native.Simulation, "apply_sfm_coupled_state")
    ):
        raise RuntimeError("TASK 04S requires the isolated experimental JuPedSim build")
    return {
        "package": str(package),
        "native_binary": str(binary),
        "native_sha256": sha256(binary.read_bytes()).hexdigest(),
        "distribution": version("jupedsim"),
        "build": str(jps.get_build_info()),
    }


def position(pose):
    return pose.x, pose.y


def velocity(vector):
    return vector.linear_x, vector.linear_y


def identity(agent):
    return agent.id, agent.journey_id, agent.stage_id


def error_stats(errors):
    ordered = sorted(errors)
    if not ordered:
        return None
    return {
        "count": len(ordered),
        "max": ordered[-1],
        "mean": mean(ordered),
        "p99": ordered[ceil(0.99 * len(ordered)) - 1],
    }


class ExternalActorProbe:
    backend_id = PROFILE
    backend_version = "jupedsim.1.4.2+task04s"

    def __init__(self, polygon):
        self.polygon = polygon

    def reset(self, config, seed, entity_ids):
        self.clock = BridgeClock(config.control_dt_ns)
        self.sim = create_simulation(jps.SocialForceModel(), self.polygon, self.clock)
        self.entity_ids = entity_ids
        self.ids = {}
        self.robot_id = None
        self.robot_domain_id = None
        self.samples = []

    def propose(self, context):
        stamp = context.stamp.timestamp
        before = self.sim.iteration_count()
        if (
            before != self.clock.iterations_at(stamp.time_ns)
            or tuple(human.entity_id for human in context.humans) != self.entity_ids
        ):
            raise ContractError("SFM probe clock or pedestrian roster changed")
        if self.robot_id is None:
            for human in context.humans:
                self.ids[human.entity_id] = _add_agent(
                    self.sim,
                    position(human.pose),
                    velocity(human.velocity),
                    human.pose.yaw,
                    context.human_radius_m,
                    goal=(human.goal.x, human.goal.y),
                )
            self.robot_id = self.sim.register_external_sfm_actor(context.robot_radius_m)
            self.robot_domain_id = context.robot_entity_id
        if context.robot_entity_id != self.robot_domain_id:
            raise ContractError("external robot domain ID changed")
        rxy = position(context.robot_pose)
        rv = velocity(context.robot_world_velocity)
        humans = [
            (self.ids[human.entity_id], position(human.pose), velocity(human.velocity))
            for human in context.humans
        ]
        identities = [identity(self.sim.agent(native_id)) for native_id, _, _ in humans]
        pre_rebase_errors = [
            dist(self.sim.agent(native_id).position, xy) for native_id, xy, _ in humans
        ]
        steps = self.clock.iterations_per_callback
        self.sim.apply_sfm_coupled_state(
            stamp.time_ns,
            steps,
            external=[(self.robot_id, rxy, rv)],
            pedestrians=humans,
        )
        for state, original_identity in zip(humans, identities, strict=True):
            native_id, xy, vxy = state
            agent = self.sim.agent(native_id)
            if (agent.position, agent.model.velocity, identity(agent)) != (
                xy,
                vxy,
                original_identity,
            ):
                raise ContractError(
                    "SFM rebase changed identity or failed exact readback"
                )
        readback_errors = []
        robot_sample = (
            self.robot_id,
            rxy,
            rv,
            context.robot_radius_m,
            stamp.time_ns,
            True,
        )
        for offset in range(steps + 1):
            readback = self.sim.external_sfm_actor(self.robot_id)
            readback_errors.append(dist(readback[1], rxy))
            if (
                readback != robot_sample
                or self.sim.iteration_count() != before + offset
            ):
                raise ContractError(
                    "external robot state drifted or native clock changed"
                )
            if offset < steps:
                self.sim.iterate()
        requests = []
        for entity_id, (native_id, (x, y), _) in zip(
            self.entity_ids, humans, strict=True
        ):
            px, py = self.sim.agent(native_id).position
            dt = self.clock.human_dt_ns / NS_PER_SECOND
            requests.append(
                HumanMotion(entity_id, Velocity2D((px - x) / dt, (py - y) / dt), stamp)
            )
        self.samples.append(
            {
                "time_ns": stamp.time_ns,
                "iteration_before": before,
                "iteration_after": self.sim.iteration_count(),
                "robot_input": robot_sample,
                "robot_domain_id": self.robot_domain_id,
                "robot_readback_position_errors_m": readback_errors,
                "pedestrian_domain_ids": self.entity_ids,
                "pedestrian_native_identities": identities,
                "pedestrian_rebase_input": humans,
                "pedestrian_pre_rebase_error_m": pre_rebase_errors,
                "pedestrian_predicted_xy": [
                    self.sim.agent(native_id).position for native_id, _, _ in humans
                ],
            }
        )
        return tuple(requests)


def run_fixture(fixture, policy, seed):
    config = compose_config(
        ROOT / "configs",
        (*COMMON_OVERRIDES, *FIXTURES[fixture], f"humans.backend={PROFILE}"),
    )
    built = build_world(config)
    geometry = build_walkable(inspect_world(built).planar_world())
    endpoints = tuple(
        point
        for actor in (built.scene.robot, *built.scene.humans)
        for point in (Point2D(*position(actor.pose)), actor.goal)
    )
    probe = ExternalActorProbe(geometry.native_polygon(endpoints))
    engine = MuJoCoEngine(human_backend=probe)
    trace, negative_clearances = [], []
    failure = None
    try:
        engine.load(config)
        engine.reset(seed=seed)
        for _ in range(built.episode.max_ticks):
            before = engine.get_ground_truth()
            stamp = before.stamp.timestamp
            action = ACTIONS["advance" if stamp.time_ns < BRANCH_NS else policy]
            try:
                step = engine.step(
                    RobotAction(before.robot.entity_id, Twist2D(*action, 0), stamp)
                )
            except (ContractError, RuntimeError) as error:
                failure = str(error)
                break
            truth = step.ground_truth
            record = asdict(step)
            record["robot_world_velocity"] = velocity(
                body_velocity_to_world(truth.robot.velocity, truth.robot.pose)
            )
            trace.append(record)
            for kind, clearance in asdict(step.clearance).items():
                if clearance is not None and clearance < 0:
                    negative_clearances.append(
                        (truth.stamp.timestamp.time_ns, kind, clearance)
                    )
            if step.terminated or step.truncated:
                break
        metadata = engine.get_metadata()
        lag = []
        for record in trace:
            start = record["requested"]["stamp"]["timestamp"]["time_ns"]
            sample = probe.samples[start // probe.clock.human_dt_ns]
            pose = record["ground_truth"]["robot"]["pose"]
            lag.append(dist(sample["robot_input"][1], (pose["x"], pose["y"])))
        return {
            "profile": PROFILE,
            "fixture": fixture,
            "policy": policy,
            "branch_time_ns": BRANCH_NS,
            "geometry_hash": geometry.geometry_hash,
            "internal_dt_ns": probe.clock.internal_dt_ns,
            "native_iterations": probe.sim.iteration_count(),
            "metadata": asdict(metadata),
            "status": "probe_error"
            if failure
            else (
                "interaction_failure"
                if negative_clearances or any(tick["contacts"] for tick in trace)
                else "fixture_completed"
            ),
            "error": failure,
            "negative_clearances": negative_clearances,
            "robot_sync_error_m": error_stats(
                error
                for sample in probe.samples
                for error in sample["robot_readback_position_errors_m"]
            ),
            "robot_hold_lag_at_executed_ticks_m": error_stats(lag),
            "samples": probe.samples,
            "trace": trace,
        }
    finally:
        engine.close()


def measure_neighbor_response():
    """Isolate native position and velocity contributions in one 10 ms step."""
    cases = {
        "separated_near_stop": ((0.6, 0), (0, 0)),
        "separated_far_stop": ((1.2, 0), (0, 0)),
        "separated_near_yield": ((0.6, 0), (0, 0.4)),
        "overlap_stop": ((0.35, 0), (0, 0)),
        "overlap_yield": ((0.35, 0), (0, 0.4)),
    }
    responses = {}
    for name, (xy, vxy) in cases.items():
        sim = jps.Simulation(
            model=jps.SocialForceModel(),
            dt=0.01,
            geometry=[(-4, -2), (4, -2), (4, 2), (-4, 2)],
        )
        human = _add_agent(sim, (0, 0), (0, 0), 0, 0.25, goal=(2, 0))
        robot = sim.register_external_sfm_actor(0.18)
        sim.apply_sfm_coupled_state(
            0, 1, external=[(robot, xy, vxy)], pedestrians=[(human, (0, 0), (0, 0))]
        )
        sim.iterate()
        responses[name] = {
            "robot_xy": xy,
            "robot_velocity_xy": vxy,
            "human_velocity_after_10ms": sim.agent(human).model.velocity,
        }
    return responses


def parity_trace():
    """Public API only. Run unchanged in separate stock and experimental processes."""
    clock = BridgeClock(100_000_000)
    sim = create_simulation(
        jps.SocialForceModel(), [(-4, -2), (4, -2), (4, 2), (-4, 2)], clock
    )
    ids = [
        _add_agent(sim, xy, vxy, yaw, 0.25, goal=goal)
        for xy, vxy, yaw, goal in [
            ((-2, -0.2), (0.1, 0), 0, (3, -0.4)),
            ((2, 0.2), (-0.15, 0), 3.14, (-3, 0.4)),
        ]
    ]
    trace = []
    for _ in range(300):
        sim.iterate()
        trace.append(
            [
                (
                    identity(agent),
                    agent.position,
                    agent.model.velocity,
                    agent.orientation,
                )
                for agent in map(sim.agent, ids)
            ]
        )
    return {"iterations": sim.iteration_count(), "trace": trace}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parity", action="store_true")
    args = parser.parse_args()
    if args.parity:
        output = parity_trace()
    else:
        output = {
            "provenance": require_experimental_build(),
            "one_step_responses": measure_neighbor_response(),
            "cases": [
                run_fixture(fixture, policy, seed)
                for fixture, policy, seed in [
                    *(("head_on", policy, 11) for policy in ACTIONS),
                    ("head_on", "advance", 11),
                    ("head_on", "advance", 12),
                    ("straight", "stop", 11),
                    ("crossing", "advance", 11),
                    ("narrow_wall", "advance", 11),
                ]
            ],
        }
    print(json.dumps(output, allow_nan=False))


if __name__ == "__main__":
    main()
