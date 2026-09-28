#!/usr/bin/env python3
"""Check reset/load/readiness/goal dispatch across generated episodes."""
from __future__ import annotations

import argparse
import logging
import shlex
import subprocess
import sys
import time
from typing import Callable
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dataset_runner import (EpisodeLoader, HuNavManager, ReadinessChecker,
                            SimulatorManager)
from dataset_runner.nav2_manager import GoalState, Nav2Manager
from dataset_runner.hunav_manager import (
    DockerHuNavRuntime, HuNavConfig, HuNavError, ROS_ENV,
)


class HuNavPreflight:
    """Optional whole-container reset, confined to the configured HuNav container."""

    # Verified for the dedicated container used by runner.yaml. Refuse to
    # restart another image even if a container name is changed accidentally.
    DEDICATED_IMAGE = "gz_fortress_hunavsim"

    def __init__(self, config: HuNavConfig, runtime: DockerHuNavRuntime, *,
                 run_command: Callable = subprocess.run,
                 clock: Callable[[], float] = time.monotonic,
                 pause: Callable[[float], None] = time.sleep,
                 logger: logging.Logger | None = None):
        self.config = config
        self.runtime = runtime
        self._run_command = run_command
        self._clock = clock
        self._pause = pause
        self.logger = logger or logging.getLogger(__name__)

    def _command(self, command: list[str], timeout: float, code: str):
        try:
            result = self._run_command(command, capture_output=True, text=True,
                                       timeout=timeout, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise HuNavError(code, f"{command[:3]}: {exc}") from exc
        if result.returncode != 0:
            raise HuNavError(code, f"{command[:3]} exit={result.returncode}: "
                             f"{(result.stderr or result.stdout).strip()[-300:]}")
        return result

    def _identity(self) -> tuple[str, str]:
        result = self._command(
            ["docker", "inspect", "-f", "{{.Id}} {{.Config.Image}}",
             self.config.container_name], self.config.command_timeout_sec,
            "container_unavailable",
        )
        parts = result.stdout.strip().split(maxsplit=1)
        if len(parts) != 2:
            raise HuNavError("container_unavailable", "invalid Docker container identity")
        return parts[0], parts[1]

    def _ros_environment_ready(self) -> bool:
        command = ROS_ENV + (
            "ros2 pkg prefix hunav_agent_manager >/dev/null && "
            "python3 -c 'import hunav_msgs'"
        )
        try:
            result = self._run_command(
                ["docker", "exec", self.config.container_name, "bash", "-lc", command],
                capture_output=True, text=True, timeout=self.config.command_timeout_sec,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0

    def _wait_ready(self) -> None:
        deadline = self._clock() + self.config.startup_timeout_sec
        while True:
            if self.runtime.container_running() and self._ros_environment_ready():
                return
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise HuNavError(
                    "container_readiness_timeout",
                    f"{self.config.container_name} not running with usable HuNav ROS "
                    f"environment after {self.config.startup_timeout_sec:g}s",
                )
            self._pause(min(self.config.poll_interval_sec, remaining))

    def check(self, *, reset_container: bool = False) -> None:
        self.logger.info("[HUNAV PREFLIGHT] container=%s", self.config.container_name)
        container_id, image = self._identity()
        running = self.runtime.container_running()
        external = self.runtime.external_nodes() if running else []
        if external:
            self.logger.warning("[HUNAV PREFLIGHT] external runtime detected: %s", external)
        if external and not reset_container:
            raise HuNavError(
                "external_runtime",
                f"HuNav nodes already running outside manager in "
                f"{self.config.container_name}. Stop them yourself, or rerun "
                "this smoke command with --reset-hunav-container to restart only "
                "the configured HuNav container.",
            )
        if not running and not reset_container:
            raise HuNavError(
                "container_unavailable",
                f"{self.config.container_name} is stopped; start it or use "
                "--reset-hunav-container",
            )
        if reset_container and (external or not running):
            if image != self.DEDICATED_IMAGE:
                raise HuNavError(
                    "unsafe_container",
                    f"refusing to restart image {image!r}; expected "
                    f"{self.DEDICATED_IMAGE!r}",
                )
            self.logger.info("[HUNAV CLEANUP] docker restart %s", self.config.container_name)
            self._command(
                ["docker", "restart", "--time", "10", self.config.container_name],
                self.config.shutdown_timeout_sec + self.config.startup_timeout_sec,
                "cleanup_failed",
            )
            new_id, _ = self._identity()
            if new_id != container_id:
                raise HuNavError("cleanup_failed", "configured container identity changed")
        self._wait_ready()
        remaining = self.runtime.external_nodes()
        if remaining:
            raise HuNavError("cleanup_failed", f"HuNav runtime still external: {remaining}")
        self.logger.info("[HUNAV CLEAN] container running; ROS environment usable")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episodes", type=Path, nargs="+")
    parser.add_argument("--observe-sec", type=float, default=2.0)
    parser.add_argument("--reset-hunav-container", action="store_true",
                        help="Explicitly restart the configured dedicated HuNav container")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    loader = EpisodeLoader()
    log_dir = Path("/tmp/pbl6_runtime_smoke_logs")
    hunav_config = HuNavConfig.from_yaml()
    simulator = SimulatorManager(log_dir=log_dir)
    hunav = HuNavManager(config=hunav_config,
                         runtime=DockerHuNavRuntime(hunav_config, log_dir=log_dir))
    nav2 = Nav2Manager()
    readiness = ReadinessChecker()
    try:
        HuNavPreflight(hunav_config, hunav.runtime).check(
            reset_container=args.reset_hunav_container)
    except HuNavError as exc:
        command = shlex.join([sys.executable, str(Path(__file__).resolve()),
                              *sys.argv[1:], "--reset-hunav-container"])
        hint = f"\nRun with explicit container reset:\n  {command}\n" if (
            exc.code == "external_runtime" and not args.reset_hunav_container
        ) else "\n"
        parser.exit(1, f"HUNAV PREFLIGHT FAILED: {exc}{hint}")
    for episode_dir in args.episodes:
        context = loader.load(episode_dir)
        print(f"EPISODE {context.episode_id} {context.scenario_family}", flush=True)
        try:
            nav2.cancel_goal()
            hunav.reset(context)
            simulator.reset(context)
            print("RESET_PASS", flush=True)
            logging.info("[LOAD_SCENARIO] %s/humans.yaml", context.episode_dir)
            hunav.load_scenario(context)
            simulator.wait_ready(context)
            print("SIM_READY", flush=True)
            hunav.wait_ready(context)
            print(f"HUNAV_READY expected={context.metadata['humans']['total']} "
                  f"actual={hunav.actual_count}", flush=True)
            readiness.wait_ready(context)
            print("ODOM_SCAN_RGB_TF_POSE_READY", flush=True)
            nav2.wait_ready(context)
            print("NAV2_READY", flush=True)
            nav2.send_goal(context)
            print(f"GOAL_ACCEPTED x={context.goal_pose.x} y={context.goal_pose.y} "
                  f"yaw={context.goal_pose.heading}", flush=True)
            deadline = time.monotonic() + args.observe_sec
            while time.monotonic() < deadline:
                result = nav2.poll(context)
                if result.state in (GoalState.SUCCEEDED, GoalState.ABORTED,
                                    GoalState.CANCELED):
                    break
                nav2.backend.tick(min(nav2.config.poll_interval_sec,
                                      max(0.0, deadline - time.monotonic())))
            print(f"GOAL_STATE {result.state.value} code={result.error_code}", flush=True)
        finally:
            try:
                nav2.finalize(context)
            finally:
                try:
                    readiness.finalize(context)
                finally:
                    try:
                        hunav.finalize(context)
                    finally:
                        simulator.finalize(context)
            print(f"CLEANUP {context.episode_id} sim_pid={simulator.pgid} "
                  f"hunav_owned={len(hunav.owned)} nav_goal={nav2.handle}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
