"""Shared process ownership and Step1/2/3 trial lifecycle.

Experiment configuration and result schemas stay in the suite entrypoints and
monitor profiles. This module owns scenario preparation, startup and teardown.
"""
from functools import wraps
import uuid
import threading
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

from .sim_env import build_sim_env, create_corridor_scenario_json, patch_corridor_world_sdf

# Also support suite scripts invoked directly from an uninstalled checkout.
PACKAGE_ROOT = Path(__file__).resolve().parents[3] / "python"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
from dynamuhsim.runtime.nav2_trial import stop_ros_launch

MODES = {"1", "2a", "2b", "3a", "3b", "3c"}
STEP1_DESCRIPTION = "Step 1: Sensing Horizon (8m Lidar, 7m Obstacle Range, 10x10m Costmap)"


def prepare_trial(root, mode, corridor, persona, seed, speed, domain):
    """Create the same scenario, world, commands and environment as the old suites."""
    if mode not in MODES:
        raise ValueError("Unknown MPPI profile")
    if corridor not in {"090", "120"}:
        raise ValueError("Unknown corridor")
    root = Path(root).resolve()
    configs = root / "experiments/gazebo_benchmark/configs"
    generated = configs / "generated"
    generated.mkdir(parents=True, exist_ok=True)
    suffix = f"a3_3_step{mode}_{corridor}_{persona}_{seed}"
    scenario = generated / f"scenario_{suffix}.json"
    world = generated / f"world_{suffix}.sdf"
    create_corridor_scenario_json(str(scenario), seed, persona, v_human=speed)
    patch_corridor_world_sdf(
        str(root / f"src/custom_corridor/worlds/corridor_{corridor}.sdf"),
        str(world), seed, str(scenario), rtf=1.0,
        proxy_z="0.80" if corridor == "090" else "0.875",
    )
    env = build_sim_env(str(root), domain, f"{suffix}_{domain}")
    nav_env = env.copy()
    profiler = (Path(f"/tmp/controller_profiler_step{mode}_{corridor}_{persona}_{seed}.txt")
                if mode.startswith("3") else None)
    library = root / "experiments/gazebo_benchmark/core/profiler/libcontroller_profiler.so"
    if profiler is not None and library.exists():
        nav_env.update(
            LD_PRELOAD=f"{library}:{os.environ.get('LD_PRELOAD', '')}",
            CONTROLLER_PROFILER_OUT=str(profiler),
        )
    sim = [
        "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
        f"width:={'0.90' if corridor == '090' else '1.20'}",
        f"custom_world:={world}",
        f"custom_robot_model:={configs / 'robot_models/model_mecanum_step1.sdf'}",
        "obstacle:=human", "gui:=false", "rviz:=false", "robot_model:=mecanum",
        "x:=-13.0", "y:=0.0", "yaw:=0.0",
    ]
    nav = [
        "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py", "controller:=mppi",
        f"params_file:={configs / f'nav2_mppi/nav2_mppi_step{mode}.yaml'}",
        f"map:=corridor_{corridor}", "robot_model:=mecanum",
    ]
    return dict(
        sim=sim, nav=nav, env=env, nav_env=nav_env, scenario=scenario, world=world,
        nav_log=Path(f"/tmp/nav2_{suffix}_{domain}.log"), profiler=profiler,
        corridor_half_width=.45 if corridor == "090" else .60,
    )


def _terminate_group(process):
    """Reap the owned session, including children left by an exited leader."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        process.wait(timeout=3.)
        return
    deadline = time.monotonic() + 1.
    while time.monotonic() < deadline:
        process.poll()  # Reap the leader so it does not keep an empty group alive.
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(.05)
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=3.)


def stop_owned(processes, *, timeout=None):
    """Stop Nav2 before Gazebo, attempting every owned group even after a fault."""
    errors = []
    for process in reversed(processes):
        try:
            try:
                if timeout is None:
                    stop_ros_launch(process, _terminate_group)
                else:
                    stop_ros_launch(process, _terminate_group, timeout=timeout)
            finally:
                # ROS launch may have exited before all of its children did.
                _terminate_group(process)
        except Exception as error:
            errors.append(error)
    if errors:
        raise errors[0]


TOKEN_KEY = "DYNAMUHSIM_PROCESS_SCOPES"


def _scope_terminated(signum, frame):
    raise SystemExit(128 + signum)


class ProcessScope:
    """Own launches and tagged descendants, including detached worker sessions."""
    def __init__(self, shutdown_timeout=15.):
        self.token = uuid.uuid4().hex
        self.processes = []
        self.workers = []
        self.shutdown_timeout = shutdown_timeout

    def environment(self, env=None):
        env = dict(os.environ if env is None else env)
        inherited = env.get(TOKEN_KEY, os.environ.get(TOKEN_KEY, ""))
        env[TOKEN_KEY] = (inherited + " " + self.token).strip()
        return env

    def popen(self, command, **kwargs):
        kwargs["env"] = self.environment(kwargs.get("env"))
        if kwargs.get("preexec_fn") is os.setsid:
            del kwargs["preexec_fn"]
        if "preexec_fn" in kwargs or kwargs.get("start_new_session") is False:
            raise ValueError("Owned processes require a private session")
        kwargs["start_new_session"] = True
        process = subprocess.Popen(command, **kwargs)
        self.processes.append(process)
        return process

    def start_worker(self, worker):
        # Register before start so a partial startup failure is still owned.
        self.workers.append(worker)
        worker.start()
        return worker

    def stop_workers(self):
        workers, self.workers = self.workers, []
        errors = []
        for worker in reversed(workers):
            try:
                if worker.pid is None:
                    continue
                if worker.is_alive():
                    worker.terminate()
                worker.join(timeout=min(2., self.shutdown_timeout))
                if worker.is_alive():
                    worker.kill()
                    worker.join(timeout=min(2., self.shutdown_timeout))
                if worker.is_alive():
                    raise RuntimeError("Owned telemetry worker survived shutdown")
            except Exception as error:
                errors.append(error)
        if errors:
            raise errors[0]

    def run(self, command, *, check=False, timeout=None, **kwargs):
        # A completed worker can leave detached children even after exit code 0.
        with ProcessScope(self.shutdown_timeout) as worker:
            kwargs["env"] = self.environment(kwargs.get("env"))
            process = worker.popen(command, **kwargs)
            code = process.wait(timeout=timeout)
            if check and code:
                raise subprocess.CalledProcessError(code, command)
            return subprocess.CompletedProcess(command, code)

    def members(self):
        members = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                state = (entry / "stat").read_text().rsplit(")", 1)[1].split()[0]
                if state in ("Z", "X"):
                    continue
                for value in (entry / "environ").read_bytes().split(b"\0"):
                    if value.startswith((TOKEN_KEY + "=").encode()):
                        if self.token.encode() in value.split(b"=", 1)[1].split():
                            members.append(int(entry.name))
                        break
            except (OSError, IndexError):
                continue
        return members

    def stop(self):
        processes, self.processes = self.processes, []
        try:
            try:
                self.stop_workers()
            finally:
                stop_owned(processes, timeout=self.shutdown_timeout)
        finally:
            # Use per-PID signals here: a leftover may share a group with an
            # untagged process. Only the direct handles above own whole groups.
            for sig in (signal.SIGTERM, signal.SIGKILL):
                for pid in self.members():
                    try:
                        os.kill(pid, sig)
                    except ProcessLookupError:
                        pass
                deadline = time.monotonic() + 1.
                while self.members() and time.monotonic() < deadline:
                    time.sleep(.02)
            if self.members():
                raise RuntimeError("Owned benchmark descendants survived shutdown")
            for process in processes:
                try:
                    process.wait(timeout=3.)
                finally:
                    for name in ("stdin", "stdout", "stderr"):
                        stream = getattr(process, name, None)
                        if stream is not None:
                            stream.close()

    def __enter__(self):
        if threading.current_thread() is threading.main_thread():
            previous = signal.getsignal(signal.SIGTERM)
            if previous in (signal.SIG_DFL, _scope_terminated):
                self._sigterm = previous
                signal.signal(signal.SIGTERM, _scope_terminated)
        return self

    def __exit__(self, kind, error, traceback):
        try:
            try:
                self.stop()
            except Exception as cleanup_error:
                if error is None:
                    raise
                if hasattr(error, "add_note"):
                    error.add_note(f"Process cleanup also failed: {cleanup_error}")
        finally:
            if hasattr(self, "_sigterm"):
                signal.signal(signal.SIGTERM, self._sigterm)
        return False


def scoped_processes(function):
    @wraps(function)
    def run(*args, **kwargs):
        with ProcessScope() as processes:
            return function(*args, **kwargs, _processes=processes)
    return run


def run_trial(root, mode, corridor, persona, seed, speed, domain, output, *, monitor_factory, ros=None):
    plan = prepare_trial(root, mode, corridor, persona, seed, speed, domain)
    if ros is None:
        import rclpy as ros
    if plan["profiler"] is not None:
        plan["profiler"].unlink(missing_ok=True)
    processes = []
    monitor = None
    initialized = False
    previous = {key: os.environ.get(key) for key in ("ROS_DOMAIN_ID", "GZ_PARTITION")}
    result = {}
    with plan["nav_log"].open("w") as log:
        try:
            processes.append(subprocess.Popen(
                plan["sim"], env=plan["env"], stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, start_new_session=True,
            ))
            time.sleep(4.5)
            processes.append(subprocess.Popen(
                plan["nav"], env=plan["nav_env"], stdout=log,
                stderr=subprocess.STDOUT, text=True, start_new_session=True,
            ))
            start = time.time()
            ready = False
            while time.time() - start < 60.:
                if plan["nav_log"].read_text().count("Managed nodes are active") >= 2:
                    ready = True
                    break
                time.sleep(.4)
            if not ready:
                result = dict(error="nav2_activation_timeout", goal_reached=False)
            else:
                time.sleep(1.)
                os.environ.update({key: plan["env"][key] for key in previous})
                ros.init()
                initialized = True
                monitor = monitor_factory(
                    target_gx=-7., target_gy=0.,
                    max_duration_s=40. if mode.startswith("3") else 25.,
                    corridor_half_width=plan["corridor_half_width"],
                )
                result = monitor.run_navigation()
        finally:
            try:
                try:
                    if monitor is not None:
                        monitor.destroy_node()
                finally:
                    if initialized:
                        ros.shutdown()
            finally:
                try:
                    stop_owned(processes)
                finally:
                    for key, value in previous.items():
                        if value is None:
                            os.environ.pop(key, None)
                        else:
                            os.environ[key] = value
    if plan["profiler"] is not None:
        mean = p95 = None
        if plan["profiler"].exists():
            try:
                durations = [float(line.strip()) for line in plan["profiler"].read_text().splitlines() if line.strip()]
                if durations:
                    mean = round(float(np.mean(durations)), 2)
                    p95 = round(float(np.percentile(durations, 95)), 2)
            except (OSError, ValueError):
                pass
        result.update(controller_calc_ms_mean=mean, controller_calc_ms_p95=p95)
    result.update(
        seed=seed, corridor=f"corridor_{corridor}",
        corridor_width_m=.90 if corridor == "090" else 1.20,
        persona=persona, v_human=speed, rtf_setting=1.0,
        step=STEP1_DESCRIPTION if mode == "1" else f"Step {mode}",
    )
    with open(output, "w") as stream:
        json.dump(result, stream, indent=2)
    return result
