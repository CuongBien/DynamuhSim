"""Run one technical ORCA fixture in MuJoCo and write measured JSON to stdout."""

from argparse import ArgumentParser
from dataclasses import asdict
from json import dumps
from math import hypot
from statistics import fmean

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.protocols import SimulationStepResult
from narrow_nav.core.states import GroundTruthState
from narrow_nav.core.types import Twist2D
from narrow_nav.humans.registry import create_human_backend
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine

_ADVANCE = Twist2D(0.35, 0, 0)
_WINDOW = (3_000_000_000, 4_500_000_000)
_POLICIES = {
    "advance": _ADVANCE,
    "stop": Twist2D(0, 0, 0),
    "yield": Twist2D(0, -0.22, 0),
    "reverse": Twist2D(-0.25, 0, 0),
}
_ENCOUNTER_M = 1.0
_STALL_MPS = 0.02
_DEADLOCK_S = 1.0


def robot_command(policy: str, time_ns: int) -> Twist2D:
    return _POLICIES[policy] if _WINDOW[0] <= time_ns < _WINDOW[1] else _ADVANCE


def _norm_summary(values):
    return {"mean": fmean(values) if values else None, "max": max(values, default=None)}


def _human_metrics(index, initial, steps, goal_tolerance_m, dt_s):
    actor = initial.humans[index]
    previous_vx, previous_vy = actor.velocity.linear_x, actor.velocity.linear_y
    previous_accel = None
    speeds, accelerations, jerks = [], [], []
    goal_time_s = None
    slow_ticks = conflict_ticks = longest_conflict = encounters = 0
    in_encounter = (
        hypot(actor.pose.x - initial.robot.pose.x, actor.pose.y - initial.robot.pose.y)
        <= _ENCOUNTER_M
    )
    encounters += int(in_encounter)
    if (
        hypot(actor.pose.x - actor.goal.x, actor.pose.y - actor.goal.y)
        <= goal_tolerance_m
    ):
        goal_time_s = 0.0
    for step in steps:
        truth = step.ground_truth
        human = truth.humans[index]
        velocity = step.executed.humans[index].velocity
        vx, vy = velocity.linear_x, velocity.linear_y
        speed = hypot(vx, vy)
        ax, ay = (vx - previous_vx) / dt_s, (vy - previous_vy) / dt_s
        speeds.append(speed)
        accelerations.append(hypot(ax, ay))
        if previous_accel is not None:
            jerks.append(hypot(ax - previous_accel[0], ay - previous_accel[1]) / dt_s)
        previous_vx, previous_vy, previous_accel = vx, vy, (ax, ay)
        nearby_robot = (
            hypot(human.pose.x - truth.robot.pose.x, human.pose.y - truth.robot.pose.y)
            <= _ENCOUNTER_M
        )
        encounters += int(nearby_robot and not in_encounter)
        in_encounter = nearby_robot
        distance = hypot(human.pose.x - human.goal.x, human.pose.y - human.goal.y)
        if distance <= goal_tolerance_m and goal_time_s is None:
            goal_time_s = truth.stamp.timestamp.time_s
        stalled = distance > goal_tolerance_m and speed < _STALL_MPS
        slow_ticks += int(stalled)
        nearby_human = any(
            other.entity_id != human.entity_id
            and hypot(human.pose.x - other.pose.x, human.pose.y - other.pose.y)
            <= _ENCOUNTER_M
            for other in truth.humans
        )
        conflict_ticks = (
            conflict_ticks + 1 if stalled and (nearby_human or nearby_robot) else 0
        )
        longest_conflict = max(longest_conflict, conflict_ticks)
    return {
        "speed_mps": _norm_summary(speeds),
        "acceleration_mps2": _norm_summary(accelerations),
        "jerk_mps3": _norm_summary(jerks),
        "time_to_goal_s": goal_time_s,
        "robot_encounters": encounters,
        "stall_s": slow_ticks * dt_s,
        "longest_interaction_stall_s": longest_conflict * dt_s,
        "deadlock_candidate": longest_conflict * dt_s >= _DEADLOCK_S,
    }


def measure(
    initial: GroundTruthState,
    steps: list[SimulationStepResult],
    goal_tolerance_m: float,
    dt_s: float,
):
    clearance = {
        field: min(
            (
                value
                for step in steps
                if (value := getattr(step.clearance, field)) is not None
            ),
            default=None,
        )
        for field in ("robot_wall_m", "robot_human_m", "human_wall_m", "human_human_m")
    }
    contacts: dict[str, int] = {}
    for step in steps:
        for event in step.contacts:
            kind = event.kind or "unclassified"
            contacts[kind] = contacts.get(kind, 0) + 1
    return {
        "min_clearance_m": clearance,
        "native_contact_pair_samples": contacts,
        "guard_blocked_substeps": sum(
            step.human_motion_blocked_substeps for step in steps
        ),
        "humans": {
            human.entity_id: _human_metrics(i, initial, steps, goal_tolerance_m, dt_s)
            for i, human in enumerate(initial.humans)
        },
    }


def main() -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=_POLICIES, default="advance")
    parser.add_argument("--ticks", type=int, default=160)
    parser.add_argument("overrides", nargs="*")
    args = parser.parse_args()
    if args.ticks < 1:
        parser.error("--ticks must be positive")
    config = compose_config(
        overrides=("humans=orca", "robots=mecanum_robot", *args.overrides)
    )
    engine = MuJoCoEngine(human_backend=create_human_backend("orca"))
    steps = []
    initial = None
    try:
        engine.load(config)
        try:
            engine.reset()
            initial = engine.get_ground_truth()
            for _ in range(args.ticks):
                robot = engine.get_ground_truth().robot
                command = robot_command(args.policy, robot.timestamp.time_ns)
                step = engine.step(
                    RobotAction(robot.entity_id, command, robot.timestamp)
                )
                steps.append(step)
                if step.terminated or step.truncated:
                    break
        except (ValueError, RuntimeError):
            if engine.get_metadata().status != "failed":
                raise
        metadata = engine.get_metadata()
        dt_s = metadata.timing.robot_control_dt_ns / 1e9
        print(
            dumps(
                {
                    "policy": args.policy,
                    "policy_window_ns": _WINDOW,
                    "encounter_center_distance_m": _ENCOUNTER_M,
                    "stall_speed_mps": _STALL_MPS,
                    "deadlock_window_s": _DEADLOCK_S,
                    "episode": asdict(metadata),
                    "stop_reason": metadata.end_reason or "fixture_tick_limit",
                    "measurements": None
                    if initial is None
                    else measure(initial, steps, config.humans.goal_tolerance_m, dt_s),
                    "initial_ground_truth": None
                    if initial is None
                    else asdict(initial),
                    "trace": [
                        {
                            "ground_truth": asdict(step.ground_truth),
                            "requested": asdict(step.requested),
                            "limited": asdict(step.limited),
                            "executed": asdict(step.executed),
                            "clearance": asdict(step.clearance),
                            "contacts": [asdict(event) for event in step.contacts],
                            "human_motion_blocked_substeps": step.human_motion_blocked_substeps,
                        }
                        for step in steps
                    ],
                },
                indent=2,
                allow_nan=False,
            )
        )
        return int(metadata.status == "failed")
    finally:
        engine.close()


if __name__ == "__main__":
    raise SystemExit(main())
