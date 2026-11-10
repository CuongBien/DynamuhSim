"""Offline force isolation and time-series plots for the bounded SFM suite."""

import argparse
import json
from math import dist
from pathlib import Path

import jupedsim as jps
from omegaconf import OmegaConf

from narrow_nav.core.geometry import body_velocity_to_world
from narrow_nav.core.states import RobotState
from narrow_nav.humans.jupedsim_bridge.clock import BridgeClock
from narrow_nav.humans.jupedsim_bridge.geometry import build_walkable
from narrow_nav.humans.jupedsim_bridge.model_factory import create_simulation
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.world_builder import build_world

from .probe_jupedsim_coupling import _add_agent


def _read(path):
    return json.loads(path.read_text())


def _state(snapshot):
    robot = RobotState.from_dict(snapshot["robot"])
    person = snapshot["humans"][0]
    rv = body_velocity_to_world(robot.velocity, robot.pose)
    return {
        "robot_xy": (robot.pose.x, robot.pose.y),
        "robot_v": (rv.linear_x, rv.linear_y),
        "human_xy": (person["pose"]["x"], person["pose"]["y"]),
        "human_yaw": person["pose"]["yaw"],
        "human_v": (person["velocity"]["linear_x"], person["velocity"]["linear_y"]),
        "goal_xy": (person["goal"]["x"], person["goal"]["y"]),
    }


def isolate_native_robot_force(state, scene, polygon):
    """Difference of two real SFM steps at the same state. No force is reimplemented."""
    clock = BridgeClock(100_000_000)
    velocities = {}
    for external in (False, True):
        sim = create_simulation(jps.SocialForceModel(), polygon, clock)
        human = _add_agent(
            sim,
            state["human_xy"],
            state["human_v"],
            state["human_yaw"],
            scene.humans[0].radius_m,
            goal=state["goal_xy"],
        )
        if external:
            actor = sim.register_external_sfm_actor(scene.robot.radius_m)
            sim.apply_sfm_coupled_state(
                0,
                1,
                external=[(actor, state["robot_xy"], state["robot_v"])],
                pedestrians=[(human, state["human_xy"], state["human_v"])],
            )
        sim.iterate()
        agent = sim.agent(human)
        velocities[external] = agent.model.velocity
        mass = agent.model.mass
    return tuple(
        mass * (a - b) / clock.internal_dt_s
        for a, b in zip(velocities[True], velocities[False], strict=True)
    )


def analyze_reference(reference):
    narrow = next(
        case for case in reference["cases"] if case["fixture"] == "narrow_wall"
    )
    trace = narrow["trace"]
    index = next(
        i for i, tick in enumerate(trace) if tick["clearance"]["robot_human_m"] < 0
    )
    first = trace[index]
    first_ns = first["ground_truth"]["stamp"]["timestamp"]["time_ns"]
    sample = max(
        (s for s in narrow["samples"] if s["time_ns"] < first_ns),
        key=lambda s: s["time_ns"],
    )
    physics = narrow["metadata"]["timing"]["physics_dt_ns"]
    contacts = [event for tick in trace for event in tick["contacts"]]
    world = build_world(OmegaConf.create(narrow["metadata"]["resolved_config"]))
    polygon = build_walkable(inspect_world(world).planar_world()).native_polygon()
    states = []
    for tick in trace[index - 2 : index + 1]:
        truth = tick["ground_truth"]
        state = _state(truth)
        state.update(
            time_ns=truth["stamp"]["timestamp"]["time_ns"],
            clearance_m=tick["clearance"]["robot_human_m"],
            center_distance_m=dist(state["robot_xy"], state["human_xy"]),
            relative_velocity_human_minus_robot_mps=tuple(
                h - r for h, r in zip(state["human_v"], state["robot_v"], strict=True)
            ),
            native_robot_force_n=isolate_native_robot_force(
                state, world.scene, polygon
            ),
            requested=tick["requested"]["humans"][0]["velocity"],
            limited=tick["limited"]["humans"][0]["velocity"],
            executed=tick["executed"]["humans"][0]["velocity"],
            guard_blocked_substeps=tick["human_motion_blocked_substeps"],
            contact_samples=len(tick["contacts"]),
        )
        states.append(state)
    return {
        "first_sampled_overlap_ns": first_ns,
        "sampling_interval_ns": (states[-2]["time_ns"], first_ns),
        "first_native_contact_ns": contacts[0]["time_ns"],
        "contact_sample_count": len(contacts),
        "solver_contact_span_ns": (contacts[0]["time_ns"], contacts[-1]["time_ns"]),
        "physics_dt_ns": physics,
        "min_clearance_m": min(tick["clearance"]["robot_human_m"] for tick in trace),
        "guard_blocked_substeps": sum(
            tick["human_motion_blocked_substeps"] for tick in trace
        ),
        "first_guard_tick_ns": next(
            tick["ground_truth"]["stamp"]["timestamp"]["time_ns"]
            for tick in trace
            if tick["human_motion_blocked_substeps"]
        ),
        "max_hold_lag_m": narrow["robot_hold_lag_at_executed_ticks_m"]["max"],
        "callback_time_ns": sample["time_ns"],
        "held_robot_pose": sample["robot_input"][1],
        "robot_displacement_since_sample_at_overlap_m": dist(
            sample["robot_input"][1], states[-1]["robot_xy"]
        ),
        "states": states,
    }


def plot(output):
    import matplotlib.pyplot as plt

    def trace(name):
        return _read(output / f"{name}.json")["trace"][1:]

    def series(rows, key):
        return [r["time_s"] for r in rows], [r[key] for r in rows]

    figure, axes = plt.subplots(
        3, 1, sharex=True, figsize=(10, 8), layout="constrained"
    )
    for period in (100, 50):
        rows = trace(f"wall_04s_{period}")
        axes[0].plot(*series(rows, "robot_human_m"), label=f"{period} ms")
    axes[0].axhline(0, color="black", linewidth=0.8)
    axes[0].set_ylabel("Footprint clearance (m)")
    axes[0].legend()
    rows = trace("wall_04s_100")
    for stage in ("requested", "limited", "executed"):
        axes[1].step(
            [r["time_s"] for r in rows],
            [r[f"human_{stage}"][0] for r in rows],
            where="pre",
            label=stage,
        )
    axes[1].set_ylabel("Human world vx (m/s)")
    axes[1].legend()
    axes[2].step(*series(rows, "guard_blocked_substeps"), where="pre", label="Guard")
    axes[2].step(*series(rows, "contact_samples"), where="pre", label="Solver contact")
    axes[2].set_ylabel("Substep samples per tick")
    axes[2].set_xlabel("End of robot tick (s)")
    axes[2].legend()
    for axis in axes:
        axis.axvline(0.9, color="gray", linestyle=":")
    figure.savefig(output / "narrow-timeline.png", dpi=155)
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(10, 6), layout="constrained")
    for policy in ("advance", "stop", "yield", "reverse"):
        axes[0].plot(*series(trace(f"easy_{policy}"), "robot_human_m"), label=policy)
    axes[0].axvspan(4, 6, alpha=0.1, color="gray")
    axes[0].set_ylabel("Footprint clearance (m)")
    axes[0].legend()
    rows = trace("free")
    axes[1].plot(*series(rows, "human_speed_mps"), label="Human speed (m/s)")
    axes[1].plot(
        *series(rows, "human_goal_distance_m"), label="Human goal distance (m)"
    )
    axes[1].legend()
    axes[1].set_xlabel("Time (s)")
    figure.savefig(output / "paired-and-free-timeline.png", dpi=155)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    analysis = analyze_reference(_read(args.reference))
    (args.output / "wall-04s-analysis.json").write_text(
        json.dumps(analysis, indent=2, allow_nan=False) + "\n"
    )
    plot(args.output)


if __name__ == "__main__":
    main()
