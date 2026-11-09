"""Run a headless physics smoke check and print measurements as JSON."""

from argparse import ArgumentParser
from dataclasses import asdict
from json import dumps
from time import perf_counter

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.types import Twist2D
from narrow_nav.humans.registry import create_human_backend

from .configuration import compose_config
from .mujoco_engine import MuJoCoEngine


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--ticks", type=int, default=40)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--config-dir", default="configs")
    parser.add_argument("overrides", nargs="*")
    args = parser.parse_args()
    if args.ticks < 1:
        parser.error("--ticks must be positive")
    config = compose_config(args.config_dir, args.overrides)
    command = Twist2D(args.vx, args.vy, args.wz)
    engine = MuJoCoEngine(human_backend=create_human_backend(config.humans.backend))
    try:
        engine.load(config)
        engine.reset()
        start = engine.get_ground_truth()
        begin = perf_counter()
        blocked = 0
        for _ in range(args.ticks):
            robot = engine.get_ground_truth().robot
            result = engine.step(RobotAction(robot.entity_id, command, robot.timestamp))
            blocked += result.human_motion_blocked_substeps
            if result.terminated or result.truncated:
                break
        elapsed = perf_counter() - begin
        truth = engine.get_ground_truth()
        end = truth.robot
        metadata = engine.get_metadata()
        print(
            dumps(
                {
                    "robot_profile": config.robots.id,
                    "mujoco": metadata.engine_version,
                    "episode_seed": metadata.episode.config.episode_seed,
                    "physics_steps": metadata.clocks.physics_steps,
                    "robot_ticks": end.timestamp.tick,
                    "simulated_seconds": end.timestamp.time_s,
                    "step_wall_seconds": elapsed,
                    "sim_seconds_per_wall_second": end.timestamp.time_s / elapsed,
                    "displacement_world_m": [
                        end.pose.x - start.robot.pose.x,
                        end.pose.y - start.robot.pose.y,
                    ],
                    "yaw_rad": end.pose.yaw,
                    "clearance": asdict(engine.get_clearance()),
                    "human_backend_id": metadata.human_backend_id,
                    "human_backend_version": metadata.human_backend_version,
                    "human_backend_seed": metadata.human_backend_seed,
                    "human_displacement_world_m": {
                        before.entity_id: [
                            after.pose.x - before.pose.x,
                            after.pose.y - before.pose.y,
                        ]
                        for before, after in zip(
                            start.humans, truth.humans, strict=True
                        )
                    },
                    "human_motion_blocked_substeps": blocked,
                    "not_observable_solver": metadata.not_observable_solver,
                    "status": metadata.status,
                    "unavailable_sensors": metadata.unavailable_sensors,
                },
                indent=2,
            )
        )
    finally:
        engine.close()


if __name__ == "__main__":
    main()
