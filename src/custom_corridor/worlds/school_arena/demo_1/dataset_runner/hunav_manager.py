"""Load generated HuNav agents into the configured Humble Docker runtime."""
from __future__ import annotations

import hashlib
import json
import logging
import math
import os
import re
import shlex
import signal
import subprocess
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import yaml

from .episode_types import EpisodeContext

SCENARIOS = "/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios"
RUNTIME = "/tmp/school_arena"
PROBE = f"{RUNTIME}/pbl6_hunav_probe.py"
ROS_ENV = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/hunav_gz_fortress_ws/install/setup.bash && "
    "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp "
    "FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=0 && "
)


class HuNavError(RuntimeError):
    def __init__(self, code: str, detail: str):
        self.code = code
        super().__init__(f"{code}: {detail}")


@dataclass(frozen=True)
class HuNavConfig:
    container_name: str = "hunavsim_gz_fortress"
    startup_timeout_sec: float = 30.0
    service_timeout_sec: float = 15.0
    shutdown_timeout_sec: float = 10.0
    kill_timeout_sec: float = 2.0
    install_timeout_sec: float = 30.0
    command_timeout_sec: float = 8.0
    poll_interval_sec: float = 0.2

    def __post_init__(self):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", self.container_name):
            raise ValueError("Invalid HuNav container name")
        for name in (
            "startup_timeout_sec", "service_timeout_sec", "shutdown_timeout_sec",
            "kill_timeout_sec", "install_timeout_sec", "command_timeout_sec",
            "poll_interval_sec",
        ):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"hunav.{name} must be positive and finite")

    @classmethod
    def from_yaml(cls, path: Path | None = None) -> "HuNavConfig":
        path = Path(path) if path is not None else Path(__file__).parent / "config/runner.yaml"
        try:
            doc = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f"Cannot read HuNav config {path}: {exc}") from exc
        if not isinstance(doc, dict) or not isinstance(doc.get("hunav"), dict):
            raise ValueError(f"Missing hunav config mapping: {path}")
        return cls(**doc["hunav"])


@dataclass
class OwnedHuNavProcess:
    role: str
    token: str
    pgid: int | None
    host_process: Any


class DockerHuNavRuntime:
    """Docker commands and signals limited to tagged, owned exec sessions."""

    def __init__(
        self, config: HuNavConfig, *, run_command: Callable = subprocess.run,
        popen: Callable = subprocess.Popen, clock: Callable[[], float] = time.monotonic,
        pause: Callable[[float], None] = time.sleep,
        log_dir: Path | None = None,
    ) -> None:
        self.config = config
        self._run = run_command
        self._popen = popen
        self._clock = clock
        self._pause = pause
        self.log_dir = Path(log_dir) if log_dir is not None else None
        self.demo_dir = Path(__file__).resolve().parents[1]

    def _command(self, args: list[str], timeout: float, code: str) -> subprocess.CompletedProcess:
        try:
            result = self._run(args, capture_output=True, text=True, timeout=timeout,
                               check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise HuNavError(code, f"command failed: {args[:4]}: {exc}") from exc
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()[-500:]
            raise HuNavError(code, f"command exit={result.returncode}: {detail}")
        return result

    def container_running(self) -> bool:
        try:
            result = self._command(
                ["docker", "inspect", "-f", "{{.State.Running}}", self.config.container_name],
                self.config.command_timeout_sec, "container_unavailable",
            )
        except HuNavError:
            return False
        return result.stdout.strip() == "true"

    def _docker(self, args: list[str], *, timeout: float | None = None,
                code: str = "load_failed") -> subprocess.CompletedProcess:
        return self._command(
            ["docker", "exec", self.config.container_name, *args],
            timeout or self.config.command_timeout_sec, code,
        )

    def install(self, context: EpisodeContext, humans_file: Path) -> None:
        script = self.demo_dir / "scenario_generator/install_episode.sh"
        self._command(
            [str(script), str(context.episode_dir), self.config.container_name],
            self.config.install_timeout_sec, "load_failed",
        )
        self._command(
            ["docker", "cp", str(Path(__file__).parent / "hunav_container_probe.py"),
             f"{self.config.container_name}:{PROBE}"],
            self.config.command_timeout_sec, "load_failed",
        )
        installed = f"{SCENARIOS}/{context.episode_id}.yaml"
        digest = hashlib.sha256(humans_file.read_bytes()).hexdigest()
        result = self._docker(["sha256sum", installed], code="load_failed")
        words = result.stdout.split()
        if not words or words[0] != digest:
            raise HuNavError("load_failed", f"installed HuNav YAML differs from {humans_file}")

    def external_nodes(self) -> list[str]:
        # The Humble ROS graph CLI can fail under cross-distro DDS discovery.
        # Before launching any owned HuNav process, inspect only container
        # commands that could duplicate the three documented runtime roles.
        result = self._docker(["ps", "-eo", "pid,args"], code="container_unavailable")
        markers = ("hunav_loader", "hunav_agent_manager", "hunav_gz8_school_bridge.py")
        return [line.strip() for line in result.stdout.splitlines()[1:]
                if any(marker in line for marker in markers)]

    def _status(self, token: str) -> list[dict]:
        result = self._docker(["python3", PROBE, "status", token], code="reset_failed")
        try:
            return json.loads(result.stdout)["processes"]
        except (ValueError, KeyError, TypeError) as exc:
            raise HuNavError("reset_failed", "invalid container process status") from exc

    def _signal(self, owned: OwnedHuNavProcess, sig: int) -> None:
        # The helper verifies the unique token in /proc before signalling PGID.
        self._docker(
            ["python3", PROBE, "signal", owned.token, str(owned.pgid), str(sig)],
            code="reset_failed",
        )

    def launch(self, role: str, command: list[str], context: EpisodeContext) -> OwnedHuNavProcess:
        token = uuid.uuid4().hex
        log_dir = self.log_dir or context.episode_dir.parent / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"{context.episode_id}.hunav_{role}.log"
        shell_command = ROS_ENV + "exec " + shlex.join(command)
        try:
            with log_path.open("ab", buffering=0) as stream:
                process = self._popen(
                    ["docker", "exec", "-e", f"PBL6_HUNAV_TOKEN={token}",
                     self.config.container_name, "bash", "-lc", shell_command],
                    stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
        except OSError as exc:
            raise HuNavError("load_failed", f"cannot launch {role}: {exc}") from exc
        # Return the handle immediately so the manager owns it even if the
        # Docker process exits or ROS fails before service discovery.
        return OwnedHuNavProcess(role, token, None, process)

    def _ros_helper(self, mode: str, argument: str, timeout: float) -> dict:
        command = (
            ROS_ENV + "exec python3 " + shlex.quote(PROBE) + " " + mode + " "
            + shlex.quote(argument) + " " + shlex.quote(str(timeout))
        )
        result = self._docker(
            ["bash", "-lc", command], timeout=timeout + self.config.command_timeout_sec,
            code="hunav_not_ready",
        )
        try:
            return json.loads(result.stdout.splitlines()[-1])
        except (ValueError, IndexError) as exc:
            raise HuNavError("hunav_not_ready", f"invalid HuNav probe output: {result.stdout[-300:]}") from exc

    def wait_service(self, service: str) -> bool:
        try:
            result = self._ros_helper("wait_service", service, self.config.service_timeout_sec)
        except HuNavError as exc:
            raise HuNavError("hunav_service_unavailable", f"{service}: {exc}") from exc
        return result.get("status") == "ready"

    def observe(self, names: list[str]) -> dict:
        return self._ros_helper("observe", json.dumps(names), self.config.startup_timeout_sec)

    def stop(self, owned: OwnedHuNavProcess) -> None:
        if not self.container_running():
            if owned.host_process.poll() is None:
                raise HuNavError("reset_failed", "container unavailable while owned exec is active")
            owned.host_process.wait(timeout=self.config.kill_timeout_sec)
            return
        members = self._status(owned.token)
        discovery_deadline = self._clock() + self.config.command_timeout_sec
        while not members and owned.host_process.poll() is None:
            remaining = discovery_deadline - self._clock()
            if remaining <= 0:
                raise HuNavError("reset_failed", f"cannot discover owned {owned.role} process")
            self._pause(min(self.config.poll_interval_sec, remaining))
            members = self._status(owned.token)
        if members:
            pgids = {item["pgid"] for item in members}
            if len(pgids) != 1:
                raise HuNavError("reset_failed", f"owned {owned.role} has multiple process groups")
            owned.pgid = pgids.pop()
            self._signal(owned, signal.SIGINT)
            deadline = self._clock() + self.config.shutdown_timeout_sec
            while self._status(owned.token):
                remaining = deadline - self._clock()
                if remaining <= 0:
                    break
                self._pause(min(self.config.poll_interval_sec, remaining))
            if self._status(owned.token):
                self._signal(owned, signal.SIGKILL)
                deadline = self._clock() + self.config.kill_timeout_sec
                while self._status(owned.token):
                    remaining = deadline - self._clock()
                    if remaining <= 0:
                        raise HuNavError("reset_failed", f"owned {owned.role} group survived SIGKILL")
                    self._pause(min(self.config.poll_interval_sec, remaining))
        try:
            owned.host_process.wait(timeout=self.config.kill_timeout_sec)
        except subprocess.TimeoutExpired as exc:
            raise HuNavError("reset_failed", f"docker exec client for {owned.role} did not exit") from exc


class HuNavManager:
    """Install exactly one episode YAML, start HuNav, verify live agent names."""

    def __init__(self, *, config: HuNavConfig | None = None,
                 runtime: DockerHuNavRuntime | None = None,
                 logger: logging.Logger | None = None) -> None:
        self.config = config or HuNavConfig.from_yaml()
        self.runtime = runtime or DockerHuNavRuntime(self.config)
        self.logger = logger or logging.getLogger(__name__)
        self.owned: list[OwnedHuNavProcess] = []
        self.episode_id: str | None = None
        self.expected_names: list[str] | None = None
        self.actual_count: int | None = None

    def reset(self, context: EpisodeContext) -> None:
        for process in reversed(self.owned):
            self.runtime.stop(process)
            self.owned.remove(process)
        self.episode_id = None
        self.expected_names = None
        self.actual_count = None

    def load_scenario(self, context: EpisodeContext) -> None:
        self.reset(context)
        humans_file = context.episode_dir / "humans.yaml"
        if not humans_file.is_file():
            raise HuNavError("humans_file_missing", str(humans_file))
        try:
            doc = yaml.safe_load(humans_file.read_text(encoding="utf-8"))
            names = doc["hunav_loader"]["ros__parameters"].get("agents", [])
        except (OSError, yaml.YAMLError, KeyError, TypeError, AttributeError) as exc:
            raise HuNavError("load_failed", f"invalid {humans_file}: {exc}") from exc
        if (not isinstance(names, list) or any(not isinstance(name, str) for name in names)
                or len(names) != context.metadata["humans"]["total"]):
            raise HuNavError("load_failed", "episode human count/names invalid")
        if not re.fullmatch(r"ep_[0-9]{6}", context.episode_id):
            raise HuNavError("load_failed", "unsafe episode ID for container paths")
        if not self.runtime.container_running():
            raise HuNavError("container_unavailable", self.config.container_name)
        external = self.runtime.external_nodes()
        if external:
            raise HuNavError("load_failed", f"HuNav nodes already running outside manager: {external}")
        self.runtime.install(context, humans_file)
        installed = f"{SCENARIOS}/{context.episode_id}.yaml"
        world = f"{RUNTIME}/{context.episode_id}.world"
        self.episode_id = context.episode_id
        self.expected_names = list(names)
        commands = (
            ("loader", ["ros2", "run", "hunav_agent_manager", "hunav_loader",
                        "--ros-args", "--params-file", installed], "/get_parameters"),
            ("manager", ["ros2", "run", "hunav_agent_manager", "hunav_agent_manager",
                         "--ros-args", "-p", "use_sim_time:=false", "-p", "publish_tf:=false",
                         "-p", "publish_sfm_forces:=false"], "/compute_agents"),
            ("bridge", ["python3", f"{RUNTIME}/hunav_gz8_school_bridge.py",
                        "--scenario", installed, "--sdf", world, "--ros-args",
                        "-p", "world_name:=school_arena", "-p", "robot_name:=robot",
                        "-p", "update_hz:=10.0", "-p", "human_z:=0.0",
                        "-p", "route_half_width:=0.45"], None),
        )
        for role, command, service in commands:
            self.owned.append(self.runtime.launch(role, command, context))
            self._check_processes()
            if service and not self.runtime.wait_service(service):
                self._check_processes()
                raise HuNavError("hunav_service_unavailable", f"{service} unavailable after {self.config.service_timeout_sec:g}s")
            self._check_processes()
        self.logger.info("HuNav loaded %s expected agents for %s", len(names), context.episode_id)

    def _check_processes(self) -> None:
        for process in self.owned:
            code = process.host_process.poll()
            if code is not None:
                raise HuNavError("hunav_runtime_exited", f"{process.role} exited with code {code}")

    def wait_ready(self, context: EpisodeContext) -> None:
        if self.episode_id != context.episode_id or self.expected_names is None:
            raise HuNavError("hunav_not_ready", "episode HuNav scenario was not loaded")
        self._check_processes()
        observation = self.runtime.observe(self.expected_names)
        self._check_processes()
        status = observation.get("status")
        if status == "human_count_mismatch":
            raise HuNavError("human_count_mismatch", f"expected {self.expected_names}; actual {observation.get('names')}")
        if status != "ready":
            missing = {"/get_parameters", "/compute_agents"} - set(observation.get("services") or [])
            code = "hunav_service_unavailable" if missing else "hunav_not_ready"
            raise HuNavError(code, f"HuNav readiness timed out: {observation}")
        actual_names = observation.get("names")
        if (not isinstance(actual_names, list) or sorted(actual_names) != sorted(self.expected_names)
                or observation.get("count") != len(self.expected_names)):
            raise HuNavError("human_count_mismatch", f"expected {self.expected_names}; actual {actual_names}")
        self.actual_count = len(actual_names)
        self.logger.info("HuNav ready: expected=%s actual=%s", len(self.expected_names), self.actual_count)

    def finalize(self, context: EpisodeContext) -> None:
        if self.owned and self.episode_id != context.episode_id:
            raise HuNavError("reset_failed", f"owned HuNav belongs to {self.episode_id}")
        self.reset(context)
