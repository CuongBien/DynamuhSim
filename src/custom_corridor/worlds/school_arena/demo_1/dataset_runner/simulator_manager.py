"""Own and reset the Gazebo/Nav2 launch process for one generated episode.

Only the launch session started by this manager may be signalled. HuNav, Nav2
readiness and episode monitoring belong to their respective later managers.
"""
from __future__ import annotations

import logging
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import yaml

from .episode_types import EpisodeContext


class SimulatorError(RuntimeError):
    """Infrastructure failure with a stable machine-readable reason code."""

    def __init__(self, code: str, detail: str):
        self.code = code
        super().__init__(f"{code}: {detail}")


@dataclass(frozen=True)
class SimulatorConfig:
    startup_timeout_sec: float = 30.0
    shutdown_timeout_sec: float = 10.0
    kill_timeout_sec: float = 2.0
    poll_interval_sec: float = 0.2
    probe_interval_sec: float = 1.0
    probe_timeout_sec: float = 5.0
    probe_attempts: int = 2
    gui: bool = False
    rviz: bool = False

    def __post_init__(self):
        for name in (
            "startup_timeout_sec", "shutdown_timeout_sec", "kill_timeout_sec",
            "poll_interval_sec", "probe_interval_sec", "probe_timeout_sec",
        ):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"simulator.{name} must be positive")
        if (isinstance(self.probe_attempts, bool) or not isinstance(self.probe_attempts, int)
                or self.probe_attempts < 1):
            raise ValueError("simulator.probe_attempts must be a positive integer")
        if not isinstance(self.gui, bool) or not isinstance(self.rviz, bool):
            raise ValueError("simulator.gui and simulator.rviz must be booleans")

    @classmethod
    def from_yaml(cls, path: Path | None = None) -> "SimulatorConfig":
        path = Path(path) if path is not None else Path(__file__).parent / "config/runner.yaml"
        try:
            doc = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f"Cannot read simulator config {path}: {exc}") from exc
        if not isinstance(doc, dict) or not isinstance(doc.get("simulator"), dict):
            raise ValueError(f"Missing simulator config mapping: {path}")
        return cls(**doc["simulator"])


def _group_alive(pgid: int) -> bool:
    """True while an owned Linux process group has a live member.

    killpg(..., 0) also reports zombie descendants, which cannot respond to
    signals and may linger briefly after the launch leader exits.
    """
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            fields = (Path(entry.path) / "stat").read_text().rsplit(")", 1)[1].split()
            state, process_group = fields[0], int(fields[2])
        except (OSError, IndexError, ValueError):
            continue
        if process_group == pgid and state not in ("Z", "X"):
            return True
    return False


class SimulatorManager:
    """Launch an episode in its own process session and reap it on finalize."""

    def __init__(
        self, *, config: SimulatorConfig | None = None,
        launch_file: Path | None = None, log_dir: Path | None = None,
        popen: Callable = subprocess.Popen,
        signal_group: Callable[[int, int], None] = os.killpg,
        group_alive: Callable[[int], bool] = _group_alive,
        ready_probe: Callable[[EpisodeContext], bool] | None = None,
        clock: Callable[[], float] = time.monotonic,
        pause: Callable[[float], None] = time.sleep,
        logger: logging.Logger | None = None,
    ) -> None:
        self.config = config or SimulatorConfig.from_yaml()
        self.launch_file = Path(launch_file) if launch_file is not None else (
            Path(__file__).resolve().parents[1] / "school_hunav_demo.launch.py"
        )
        self.log_dir = Path(log_dir) if log_dir is not None else None
        self._popen = popen
        self._signal_group = signal_group
        self._group_alive = group_alive
        self._ready_probe = ready_probe or self._gazebo_ready
        self._clock = clock
        self._pause = pause
        self.logger = logger or logging.getLogger(__name__)
        self.process = None
        self.pgid: int | None = None
        self.episode_id: str | None = None
        self._launch_time: float | None = None
        # This matches the explicit GZ_PARTITION in school_hunav_demo.launch.py.
        self._env = os.environ.copy()
        self._env["GZ_PARTITION"] = "school_hunav"

    def _gazebo_ready(self, context: EpisodeContext) -> bool:
        # Gazebo transport discovery can return an empty first list even when
        # the world is running; a second immediate query sees the discovered
        # services. Check exact service names to avoid substring matches.
        for _ in range(self.config.probe_attempts):
            try:
                result = subprocess.run(
                    ["gz", "service", "-l"], capture_output=True, text=True,
                    timeout=self.config.probe_timeout_sec, env=self._env, check=False,
                )
            except subprocess.TimeoutExpired:
                continue
            except OSError as exc:
                raise SimulatorError("launch_failed", f"Gazebo service probe unavailable: {exc}") from exc
            if result.returncode == 0 and "/world/school_arena/control" in result.stdout.splitlines():
                return True
        return False

    def _wait_for_group_exit(self, pgid: int, timeout: float) -> bool:
        deadline = self._clock() + timeout
        while self._group_alive(pgid):
            remaining = deadline - self._clock()
            if remaining <= 0:
                return False
            self._pause(min(self.config.poll_interval_sec, remaining))
        return True

    def _stop_owned(self) -> None:
        process, pgid = self.process, self.pgid
        if process is None:
            return
        assert pgid is not None
        try:
            if self._group_alive(pgid):
                self.logger.info("Stopping owned simulator group %s with SIGINT", pgid)
                try:
                    self._signal_group(pgid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                if not self._wait_for_group_exit(pgid, self.config.shutdown_timeout_sec):
                    self.logger.warning("Owned simulator group %s exceeded shutdown timeout; SIGKILL", pgid)
                    try:
                        self._signal_group(pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    if not self._wait_for_group_exit(pgid, self.config.kill_timeout_sec):
                        raise SimulatorError("cleanup_failed", f"process group {pgid} survived SIGKILL")
            try:
                process.wait(timeout=self.config.kill_timeout_sec)
            except subprocess.TimeoutExpired as exc:
                raise SimulatorError("cleanup_failed", f"launch PID {process.pid} did not exit") from exc
        except SimulatorError:
            raise
        except OSError as exc:
            raise SimulatorError("cleanup_failed", f"cannot stop process group {pgid}: {exc}") from exc
        else:
            self.logger.info("Owned simulator group %s exited", pgid)
            self.process = None
            self.pgid = None
            self.episode_id = None
            self._launch_time = None

    def _command(self, context: EpisodeContext) -> list[str]:
        world = context.episode_dir / "school_floor.world"
        nav_params = context.episode_dir / "nav2_school.yaml"
        for path in (self.launch_file, world, nav_params):
            if not path.is_file():
                raise SimulatorError("launch_failed", f"Missing launch input: {path}")
        pose = context.start_pose
        return [
            "ros2", "launch", str(self.launch_file),
            f"episode_world:={world}", f"nav_params_file:={nav_params}",
            f"robot_x:={pose.x}", f"robot_y:={pose.y}",
            f"robot_yaw:={pose.heading}",
            f"gui:={str(self.config.gui).lower()}",
            f"rviz:={str(self.config.rviz).lower()}",
        ]

    def reset(self, context: EpisodeContext) -> None:
        self._stop_owned()
        command = self._command(context)
        if self._ready_probe(context):
            raise SimulatorError(
                "launch_failed", "Gazebo school_arena world already visible before launch; "
                "refusing to share an external runtime"
            )
        log_dir = self.log_dir or context.episode_dir.parent / "logs"
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"{context.episode_id}.simulator.log"
            with log_path.open("ab", buffering=0) as log_stream:
                self.logger.info("Launching %s; output=%s", command, log_path)
                process = self._popen(
                    command, stdin=subprocess.DEVNULL, stdout=log_stream,
                    stderr=subprocess.STDOUT, start_new_session=True, env=self._env,
                )
        except OSError as exc:
            raise SimulatorError("launch_failed", f"cannot start launch: {exc}") from exc
        self.process = process
        # start_new_session makes the launch PID the process group ID.
        self.pgid = process.pid
        self.episode_id = context.episode_id
        self._launch_time = self._clock()
        code = process.poll()
        if code is not None:
            self._stop_owned()
            raise SimulatorError("runtime_exited", f"launch exited immediately with code {code}; see {log_path}")

    def wait_ready(self, context: EpisodeContext) -> None:
        if self.process is None or self.episode_id != context.episode_id:
            raise SimulatorError("launch_failed", "no owned launch for this episode")
        deadline = self._launch_time + self.config.startup_timeout_sec
        next_probe = self._clock()
        while True:
            code = self.process.poll()
            if code is not None:
                self._stop_owned()
                raise SimulatorError("runtime_exited", f"launch exited during startup with code {code}")
            now = self._clock()
            if now >= next_probe:
                if self._ready_probe(context):
                    self.logger.info("Gazebo school_arena world service ready for %s", context.episode_id)
                    return
                next_probe = self._clock() + self.config.probe_interval_sec
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise SimulatorError(
                    "reset_timeout", f"Gazebo world service not ready after "
                    f"{self.config.startup_timeout_sec:g}s for {context.episode_id}"
                )
            self._pause(min(self.config.poll_interval_sec, remaining))

    def finalize(self, context: EpisodeContext) -> None:
        if self.process is not None and self.episode_id != context.episode_id:
            raise SimulatorError(
                "cleanup_failed", f"owned launch belongs to {self.episode_id}, "
                f"not {context.episode_id}"
            )
        self._stop_owned()
