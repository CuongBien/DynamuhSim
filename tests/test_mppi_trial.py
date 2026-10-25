"""Trial inputs/results replayed from eed95ae; shutdown ownership tested separately."""
import ast
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.gazebo_benchmark.core import mppi_trial as trial
from experiments.gazebo_benchmark.core import sim_env

MODES = ("1", "2a", "2b", "3a", "3b", "3c")
SEED = 914101
DOMAIN = 189
ENV = dict(GZ_SIM_RESOURCE_PATH="/inherited/models", GZ_SIM_SYSTEM_PLUGIN_PATH="/inherited/plugins",
           LD_PRELOAD="/inherited/profiler.so", ROS_DOMAIN_ID="188", GZ_PARTITION="unrelated")


def fixture_root(root):
    worlds = root / "src/custom_corridor/worlds"
    worlds.mkdir(parents=True)
    for corridor in ("090", "120"):
        shutil.copyfile(ROOT / f"src/custom_corridor/worlds/corridor_{corridor}.sdf", worlds / f"corridor_{corridor}.sdf")


def digest(value, root):
    text = json.dumps(value, sort_keys=True, separators=(",", ":")).replace(str(root), "<ROOT>")
    return hashlib.sha256(text.encode()).hexdigest()


def exercise(root, mode, corridor="090", persona="H1", *, ready=True, profiler="valid", library=True,
             fault=None, parent_env=ENV):
    """Fake only ROS/process/time boundaries; run actual scenario/world/result code."""
    lib = root / "experiments/gazebo_benchmark/core/profiler/libcontroller_profiler.so"
    lib.parent.mkdir(parents=True, exist_ok=True)
    if library:
        lib.touch()
    else:
        lib.unlink(missing_ok=True)
    nav_log = Path(f"/tmp/nav2_a3_3_step{mode}_{corridor}_{persona}_{SEED}_{DOMAIN}.log")
    profiler_out = Path(f"/tmp/controller_profiler_step{mode}_{corridor}_{persona}_{SEED}.txt")
    if mode.startswith("3"):
        profiler_out.write_text("9999\n")  # Must not contaminate this trial.
    launches, events, processes = [], [], []
    clock = NS(now=0.)
    def sleep(seconds):
        clock.now += seconds
    def popen(cmd, **kwargs):
        if fault == "launch" and len(launches) == 1:
            raise RuntimeError("launch failed")
        launches.append(dict(command=cmd, env=kwargs["env"], session=bool(kwargs.get("start_new_session") or kwargs.get("preexec_fn"))))
        process = NS(pid=700000 + len(launches))
        processes.append(process)
        if len(launches) == 2:
            if ready:
                kwargs["stdout"].write("Managed nodes are active\nManaged nodes are active\n")
                kwargs["stdout"].flush()
            if mode.startswith("3") and library and profiler != "missing":
                profiler_out.write_text("1\n2\n3\n10\n" if profiler == "valid" else "bad data\n")
        return process
    def init():
        events.append("init")
        if fault == "init":
            raise RuntimeError("init failed")
    def shutdown():
        events.append("shutdown")
        if fault == "shutdown":
            raise RuntimeError("shutdown failed")
    ros = NS(init=init, shutdown=shutdown)
    def monitor(**kwargs):
        events.append(dict(monitor=kwargs))
        if fault == "construct":
            raise RuntimeError("construct failed")
        def run():
            events.append("run")
            if fault == "monitor":
                raise RuntimeError("monitor failed")
            return dict(goal_reached=True, time_to_goal=8.25, dmin_edge_to_edge=.1234)
        def destroy():
            events.append("destroy")
            if fault == "destroy":
                raise RuntimeError("destroy failed")
        return NS(run_navigation=run, destroy_node=destroy)
    def stop(owned):
        events.append(dict(stopped=[p.pid for p in owned]))
    fake_process = NS(Popen=popen, run=lambda *args, **kwargs: None, DEVNULL=subprocess.DEVNULL, STDOUT=subprocess.STDOUT)
    output = root / "result.json"
    try:
        with patch.dict(os.environ, parent_env, clear=True):
            try:
                with patch.object(trial, "subprocess", fake_process), patch.object(trial, "time", NS(time=lambda: clock.now, sleep=sleep)), patch.object(trial, "stop_owned", stop):
                    trial.run_trial(root, mode, corridor, persona, SEED, 1.217, DOMAIN, output, monitor_factory=monitor, ros=ros)
            finally:
                restored = dict(os.environ)
        suffix = f"a3_3_step{mode}_{corridor}_{persona}_{SEED}"
        generated = root / "experiments/gazebo_benchmark/configs/generated"
        return dict(launches=launches, scenario=(generated / f"scenario_{suffix}.json").read_text(),
                    world=(generated / f"world_{suffix}.sdf").read_text(), result=output.read_text(),
                    monitor=[event["monitor"] for event in events if isinstance(event, dict) and "monitor" in event],
                    events=events, restored=restored)
    except Exception as error:
        error.events = events
        error.restored = restored
        raise
    finally:
        nav_log.unlink(missing_ok=True)
        if mode.startswith("3"):
            profiler_out.unlink(missing_ok=True)


def parity_payload(value):
    # Teardown and parent environment restoration deliberately changed.
    return {key: value[key] for key in ("launches", "scenario", "world", "result", "monitor")}


def cases():
    for mode in MODES:
        for corridor in ("090", "120"):
            for persona in ("H1", "H3"):
                for ready in (True, False):
                    yield f"{mode}:{corridor}:{persona}:{ready}", dict(mode=mode, corridor=corridor, persona=persona, ready=ready)
    for profiler in ("missing", "invalid"):
        for mode in ("3a", "3b", "3c"):
            yield f"{mode}:profiler:{profiler}", dict(mode=mode, profiler=profiler)
    for mode in ("3a", "3b", "3c"):
        yield f"{mode}:no_library", dict(mode=mode, library=False)


class MppiTrialTests(unittest.TestCase):
    def test_launch_inputs_world_scenario_and_result_match_original_six_profiles(self):
        golden = json.loads((ROOT / "tests/fixtures/mppi_trial.json").read_text())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fixture_root(root)
            for key, kwargs in cases():
                with self.subTest(case=key):
                    value = exercise(root, **kwargs)
                    self.assertEqual(digest(parity_payload(value), root), golden["cases"][key])
                    self.assertEqual(value["restored"], ENV)
                    self.assertEqual(value["events"][-1], dict(stopped=[700001, 700002]))

    def test_launch_monitor_and_constructor_faults_release_owned_resources(self):
        for fault, expected in [("launch", []), ("init", []), ("construct", ["shutdown"]), ("monitor", ["destroy", "shutdown"]), ("destroy", ["destroy", "shutdown"]), ("shutdown", ["destroy", "shutdown"])]:
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                fixture_root(root)
                with self.assertRaisesRegex(RuntimeError, f"{fault} failed") as raised:
                    exercise(root, "3a", fault=fault)
                self.assertEqual(raised.exception.restored, ENV)
                events = raised.exception.events
                self.assertEqual([e for e in events if e in ("destroy", "shutdown")], expected)
                self.assertEqual(events[-1], dict(stopped=[700001] if fault == "launch" else [700001, 700002]))
                self.assertFalse((root / "result.json").exists())

    def test_parent_environment_remains_unset_when_originally_absent(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fixture_root(root)
            self.assertEqual(exercise(root, "1", parent_env={})["restored"], {})

    def test_runner_wrappers_select_profiles_and_batch_forwards_domain(self):
        for step, name in [(1, "step1_vision"), (2, "step2"), (3, "step3")]:
            path = ROOT / f"experiments/gazebo_benchmark/suites/mppi_ablation/benchmark_a3_3_{name}.py"
            tree = ast.parse(path.read_text())
            wrapper = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run_single_trial")
            calls = []
            namespace = dict(ROOT_DIR=str(ROOT), run_trial=lambda *args, **kwargs: calls.append((args, kwargs)),
                             **{f"Step{step}InteractionMonitor": f"monitor{step}"})
            exec(compile(ast.Module(body=[wrapper], type_ignores=[]), str(path), "exec"), namespace)
            modes = [mode for mode in MODES if mode.startswith(str(step))]
            for mode in modes:
                args = ("120", "H3", 301, 1.217, 189, "/tmp/result.json")
                namespace["run_single_trial"](*args if step == 1 else (mode, *args))
                self.assertEqual(calls[-1], ((str(ROOT), mode, *args), dict(monitor_factory=f"monitor{step}")))
            self.assertNotIn("cleanup_all", path.read_text())
            self.assertNotIn("pkill", path.read_text())
            domain_lists = [n for n in ast.walk(tree) if isinstance(n, ast.List) and any(isinstance(v, ast.Constant) and v.value == "--domain_id" for v in n.elts)]
            self.assertEqual(len(domain_lists), 1)
            values = domain_lists[0].elts
            index = next(i for i, value in enumerate(values) if isinstance(value, ast.Constant) and value.value == "--domain_id")
            self.assertEqual(ast.unparse(values[index + 1]), "str(args.domain_id)")

    def test_existing_sim_env_callers_keep_default_speed_and_proxy_height(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fixture_root(root)
            for persona, speed in [("H1", 1.05), ("H3", 1.15)]:
                scenario = root / "scenario.json"
                sim_env.create_corridor_scenario_json(str(scenario), 101, persona)
                self.assertEqual(json.loads(scenario.read_text())["humans"][0]["speed"], speed)
            world = root / "world.sdf"
            sim_env.patch_corridor_world_sdf(str(root / "src/custom_corridor/worlds/corridor_090.sdf"), str(world), 101, str(scenario))
            self.assertIn("-7.0 0.0 0.875 0 0 3.14159", world.read_text())

    def test_stop_owned_attempts_every_group_even_when_one_shutdown_fails(self):
        stopped = []
        with patch.object(trial, "stop_ros_launch", side_effect=RuntimeError("shutdown fault")), patch.object(trial, "_terminate_group", side_effect=stopped.append):
            with self.assertRaisesRegex(RuntimeError, "shutdown fault"):
                trial.stop_owned(["sim", "nav"])
        self.assertEqual(stopped, ["nav", "sim"])

    @unittest.skipUnless(sys.platform == "linux", "POSIX session ownership")
    def test_shutdown_stops_owned_hung_leader_and_orphans_but_keeps_unrelated_session(self):
        for exited in (False, True):
            with self.subTest(leader_exited=exited), tempfile.TemporaryDirectory() as directory:
                pidfile = Path(directory) / "child.pid"
                child_code = ("import signal,time,pathlib,os,sys; "
                              "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
                              "signal.signal(signal.SIGINT,signal.SIG_IGN); "
                              "pathlib.Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(120)")
                parent_code = ("import subprocess,sys,signal,time; "
                               "signal.signal(signal.SIGINT,signal.SIG_IGN); "
                               "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
                               "subprocess.Popen([sys.executable,'-c',sys.argv[2],sys.argv[1]]); "
                               + ("pass" if exited else "time.sleep(120)"))
                unrelated = subprocess.Popen([sys.executable, "-c", "import time;time.sleep(120)"], start_new_session=True)
                owned = subprocess.Popen([sys.executable, "-c", parent_code, str(pidfile), child_code], start_new_session=True)
                try:
                    deadline = time.monotonic() + 5
                    while not pidfile.exists() and time.monotonic() < deadline:
                        time.sleep(.02)
                    child_pid = int(pidfile.read_text())
                    if exited:
                        owned.wait(timeout=5)
                    graceful_stop = trial.stop_ros_launch
                    with patch.object(trial, "stop_ros_launch", side_effect=lambda p, stop: graceful_stop(p, stop, timeout=.05)):
                        trial.stop_owned([owned])
                    self.assertIsNone(unrelated.poll())
                    self.assertIsNotNone(owned.poll())
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline:
                        stat = Path(f"/proc/{child_pid}/stat")
                        if not stat.exists() or stat.read_text().split()[2] == "Z":
                            break
                        time.sleep(.02)
                    else:
                        self.fail("Owned orphan still running")
                finally:
                    # Test-owned PIDs only, even on assertion failure.
                    for process in (owned, unrelated):
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        process.wait(timeout=5)


@unittest.skipUnless(sys.platform == "linux", "Linux process-token ownership")
class ProcessScopeTests(unittest.TestCase):
    def test_fork_worker_cleanup_keeps_foreign_worker_in_the_same_group(self):
        context = multiprocessing.get_context('fork')
        ready = context.Event()
        def stubborn_worker():
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            ready.set()
            time.sleep(120)
        unrelated = context.Process(target=time.sleep, args=(120,))
        owned = context.Process(target=stubborn_worker)
        unrelated.start()
        try:
            with self.assertRaisesRegex(ValueError, 'measurement failed'):
                with trial.ProcessScope(shutdown_timeout=.05) as scope:
                    scope.start_worker(owned)
                    self.assertTrue(ready.wait(timeout=5))
                    self.assertEqual(os.getpgid(owned.pid), os.getpgrp())
                    raise ValueError('measurement failed')
            self.assertFalse(owned.is_alive())
            self.assertEqual(owned.exitcode, -signal.SIGKILL)
            self.assertTrue(unrelated.is_alive())
        finally:
            for worker in (owned, unrelated):
                if worker.pid is not None:
                    if worker.is_alive(): worker.kill()
                    worker.join(timeout=5)

    def test_worker_start_failure_is_cleaned_before_or_after_fork(self):
        context = multiprocessing.get_context('fork')
        for started in (False, True):
            with self.subTest(started=started):
                ready = context.Event()
                def idle_worker():
                    ready.set()
                    time.sleep(120)
                worker = context.Process(target=idle_worker)
                real_start = worker.start
                def broken_start():
                    if started:
                        real_start()
                        self.assertTrue(ready.wait(timeout=5))
                    raise RuntimeError('worker startup failed')
                try:
                    with patch.object(worker, 'start', side_effect=broken_start), self.assertRaisesRegex(RuntimeError, 'worker startup failed'):
                        with trial.ProcessScope(shutdown_timeout=.05) as scope:
                            scope.start_worker(worker)
                    if started:
                        self.assertFalse(worker.is_alive())
                        self.assertIsNotNone(worker.exitcode)
                finally:
                    if worker.pid is not None:
                        if worker.is_alive(): worker.kill()
                        worker.join(timeout=5)

    def test_environment_is_copied_and_private_session_is_required(self):
        env = {trial.TOKEN_KEY: "ancestor", "ROS_DOMAIN_ID": "77"}
        scope = trial.ProcessScope(shutdown_timeout=.05)
        tagged = scope.environment(env)
        self.assertEqual(env, {trial.TOKEN_KEY: "ancestor", "ROS_DOMAIN_ID": "77"})
        self.assertEqual(tagged[trial.TOKEN_KEY].split(), ["ancestor", scope.token])
        with self.assertRaises(ValueError):
            scope.popen(["unused"], start_new_session=False)

    def test_body_error_survives_cleanup_error(self):
        with patch.object(trial, "stop_owned", side_effect=OSError("cleanup failed")):
            with self.assertRaisesRegex(ValueError, "body failed"):
                with trial.ProcessScope():
                    raise ValueError("body failed")
            with self.assertRaisesRegex(OSError, "cleanup failed"):
                with trial.ProcessScope():
                    pass

    def exercise_worker(self, mode):
        with tempfile.TemporaryDirectory() as directory:
            pidfile = Path(directory) / "detached.pid"
            child_code = ("import signal,time,pathlib,os,sys; "
                          "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
                          "signal.signal(signal.SIGINT,signal.SIG_IGN); "
                          "pathlib.Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(120)")
            worker_code = ("import subprocess,sys,time,pathlib,signal; "
                           "subprocess.Popen([sys.executable,'-c',sys.argv[2],sys.argv[1]],start_new_session=True); "
                           "path=pathlib.Path(sys.argv[1]); "
                           "exec('while not path.exists(): time.sleep(.01)'); "
                           + ("time.sleep(120)" if mode == "timeout" else "sys.exit(" + ("7" if mode == "failure" else "0") + ")"))
            unrelated = subprocess.Popen([sys.executable, "-c", "import time;time.sleep(120)"], start_new_session=True)
            scope = trial.ProcessScope(shutdown_timeout=.05)
            try:
                with scope:
                    command = [sys.executable, "-c", worker_code, str(pidfile), child_code]
                    if mode == "failure":
                        with self.assertRaises(subprocess.CalledProcessError) as error:
                            scope.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        self.assertEqual(error.exception.returncode, 7)
                    elif mode == "timeout":
                        with self.assertRaises(subprocess.TimeoutExpired):
                            scope.run(command, timeout=1., stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    else:
                        self.assertEqual(scope.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode, 0)
                    self.assertFalse(scope.members(), "Completed worker left a tagged descendant")
                    self.assertIsNone(unrelated.poll())
                if pidfile.exists():
                    stat = Path('/proc') / pidfile.read_text() / 'stat'
                    self.assertTrue(not stat.exists() or stat.read_text().rsplit(')', 1)[1].split()[0] in ('Z', 'X'))
            finally:
                # Test-owned PIDs only if the assertion or implementation failed.
                scope.stop()
                if pidfile.exists():
                    try: os.kill(int(pidfile.read_text()), signal.SIGKILL)
                    except ProcessLookupError: pass
                unrelated.kill()
                unrelated.wait(timeout=5)

    def test_successful_worker_reaps_detached_descendant(self):
        self.exercise_worker("success")

    def test_failed_worker_keeps_return_code_and_reaps_descendant(self):
        self.exercise_worker("failure")

    def test_worker_timeout_reaps_detached_descendant(self):
        self.exercise_worker("timeout")

    def test_sigterm_unwinds_scope_and_restores_parent_handler(self):
        previous = signal.getsignal(signal.SIGTERM)
        with trial.ProcessScope():
            pass
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous)
        with tempfile.TemporaryDirectory() as directory:
            pidfile = Path(directory) / "child.pid"
            owner_code = chr(10).join([
                "import sys,time,pathlib",
                "sys.path.insert(0,sys.argv[1])",
                "from experiments.gazebo_benchmark.core.mppi_trial import ProcessScope",
                "with ProcessScope(shutdown_timeout=.05) as scope:",
                "    p=scope.popen([sys.executable,'-c','import time;time.sleep(120)'])",
                "    pathlib.Path(sys.argv[2]).write_text(str(p.pid))",
                "    time.sleep(120)",
            ])
            owner = subprocess.Popen([sys.executable, '-c', owner_code, str(ROOT), str(pidfile)],
                                     start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                deadline = time.monotonic() + 5
                while not pidfile.exists() and owner.poll() is None and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(pidfile.exists())
                owner.terminate()
                self.assertEqual(owner.wait(timeout=5), 143)
                stat = Path('/proc') / pidfile.read_text() / 'stat'
                self.assertTrue(not stat.exists() or stat.read_text().rsplit(')', 1)[1].split()[0] in ('Z', 'X'))
            finally:
                if owner.poll() is None: owner.kill()
                owner.wait(timeout=5)
                if pidfile.exists():
                    try: os.killpg(int(pidfile.read_text()), signal.SIGKILL)
                    except ProcessLookupError: pass

    def test_decorator_cleans_first_launch_when_second_launch_fails(self):
        handles = []
        @trial.scoped_processes
        def broken(*, _processes=None):
            handles.append(_processes.popen([sys.executable, "-c", "import time;time.sleep(120)"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            _processes.popen(['/definitely/missing/benchmark-command'])
        try:
            with self.assertRaises(FileNotFoundError):
                broken()
            self.assertIsNotNone(handles[0].poll())
        finally:
            for process in handles:
                if process.poll() is None: process.kill()
                process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
