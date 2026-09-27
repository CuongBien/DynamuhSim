import signal
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

DEMO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(DEMO))

from dataset_runner import EpisodeContext, Pose2D
from dataset_runner.simulator_manager import (
    SimulatorConfig, SimulatorError, SimulatorManager,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def pause(self, seconds):
        self.now += seconds


class FakeProcess:
    def __init__(self, pid, *, alive=True, exit_code=None):
        self.pid = pid
        self.alive = alive
        self.exit_code = exit_code

    def poll(self):
        return None if self.alive else self.exit_code

    def wait(self, timeout):
        if self.alive:
            raise subprocess.TimeoutExpired("ros2 launch", timeout)
        return self.exit_code


class FakeProcesses:
    def __init__(self, *, ignore_sigint=False, ignore_sigkill=False):
        self.events = []
        self.processes = {}
        self.ignore_sigint = ignore_sigint
        self.ignore_sigkill = ignore_sigkill
        self.next_pid = 1000
        self.next_process = None

    def popen(self, command, **kwargs):
        process = self.next_process or FakeProcess(self.next_pid)
        self.next_process = None
        self.next_pid += 1
        self.processes[process.pid] = process
        self.events.append(("launch", process.pid, command, kwargs))
        return process

    def alive(self, pgid):
        return self.processes[pgid].alive

    def signal(self, pgid, sig):
        self.events.append(("signal", pgid, sig))
        if ((sig == signal.SIGINT and not self.ignore_sigint)
                or (sig == signal.SIGKILL and not self.ignore_sigkill)):
            self.processes[pgid].alive = False
            self.processes[pgid].exit_code = -sig


class SimulatorManagerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.episode = root / "ep_000001"
        self.episode.mkdir()
        for name in ("school_floor.world", "nav2_school.yaml"):
            (self.episode / name).write_text("fixture\n", encoding="utf-8")
        self.launch = root / "school_hunav_demo.launch.py"
        self.launch.write_text("fixture\n", encoding="utf-8")
        self.context = EpisodeContext(
            episode_dir=self.episode, episode_id="ep_000001",
            scenario_family="empty", seed=17, metadata={}, scenario={}, humans={},
            start_pose=Pose2D(3.25, -8.15, 1.57), goal_node="goal",
            goal_pose=Pose2D(5.0, -8.15, 0.0),
        )
        self.clock = FakeClock()
        self.processes = FakeProcesses()
        self.probe_calls = 0
        self.ready_after = 2

    def probe(self, context):
        self.probe_calls += 1
        return self.probe_calls >= self.ready_after

    def manager(self):
        return SimulatorManager(
            config=SimulatorConfig(startup_timeout_sec=1.0, shutdown_timeout_sec=0.5,
                                   kill_timeout_sec=0.5, poll_interval_sec=0.1),
            launch_file=self.launch, log_dir=self.episode.parent / "logs",
            popen=self.processes.popen, signal_group=self.processes.signal,
            group_alive=self.processes.alive, ready_probe=self.probe,
            clock=self.clock.monotonic, pause=self.clock.pause,
        )

    def test_launch_uses_episode_files_pose_session_and_log(self):
        manager = self.manager()
        manager.reset(self.context)
        event = self.processes.events[0]
        command, kwargs = event[2], event[3]
        self.assertEqual(command[:3], ["ros2", "launch", str(self.launch)])
        for arg in (
            f"episode_world:={self.episode / 'school_floor.world'}",
            f"nav_params_file:={self.episode / 'nav2_school.yaml'}",
            "robot_x:=3.25", "robot_y:=-8.15", "robot_yaw:=1.57",
            "gui:=false", "rviz:=false",
        ):
            self.assertIn(arg, command)
        self.assertTrue(kwargs["start_new_session"])
        self.assertEqual(kwargs["env"]["GZ_PARTITION"], "school_hunav")
        self.assertTrue((self.episode.parent / "logs/ep_000001.simulator.log").is_file())
        manager.wait_ready(self.context)
        manager.finalize(self.context)
        self.assertIsNone(manager.process)

    def test_previous_owned_process_cleaned_before_relaunch(self):
        manager = self.manager()
        manager.reset(self.context)
        self.ready_after = 999  # no external runtime at the second preflight
        manager.reset(self.context)
        events = [(event[0], event[1]) for event in self.processes.events]
        self.assertEqual(events[:3], [("launch", 1000), ("signal", 1000),
                                      ("launch", 1001)])
        self.assertEqual(manager.pgid, 1001)
        manager.finalize(self.context)

    def test_immediate_launch_exit_is_reported(self):
        self.processes.next_process = FakeProcess(1000, alive=False, exit_code=2)
        manager = self.manager()
        with self.assertRaises(SimulatorError) as error:
            manager.reset(self.context)
        self.assertEqual(error.exception.code, "runtime_exited")
        self.assertIsNone(manager.process)

    def test_exit_during_wait_ready_is_reported(self):
        manager = self.manager()
        manager.reset(self.context)
        manager.process.alive = False
        manager.process.exit_code = 3
        with self.assertRaises(SimulatorError) as error:
            manager.wait_ready(self.context)
        self.assertEqual(error.exception.code, "runtime_exited")
        self.assertIsNone(manager.process)

    def test_graceful_shutdown_and_idempotent_finalize(self):
        manager = self.manager()
        manager.reset(self.context)
        manager.finalize(self.context)
        manager.finalize(self.context)
        signals = [event[2] for event in self.processes.events if event[0] == "signal"]
        self.assertEqual(signals, [signal.SIGINT])

    def test_force_kill_only_after_grace_timeout(self):
        self.processes.ignore_sigint = True
        manager = self.manager()
        manager.reset(self.context)
        manager.finalize(self.context)
        signals = [event[2] for event in self.processes.events if event[0] == "signal"]
        self.assertEqual(signals, [signal.SIGINT, signal.SIGKILL])
        self.assertGreaterEqual(self.clock.now, 0.5)
        self.assertIsNone(manager.process)

    def test_cleanup_failure_keeps_owned_handle_and_blocks_relaunch(self):
        self.processes.ignore_sigint = True
        self.processes.ignore_sigkill = True
        manager = self.manager()
        manager.reset(self.context)
        with self.assertRaises(SimulatorError) as error:
            manager.reset(self.context)
        self.assertEqual(error.exception.code, "cleanup_failed")
        self.assertEqual(len([e for e in self.processes.events if e[0] == "launch"]), 1)
        self.assertIsNotNone(manager.process)
        # Let test teardown stop the fake owned process cleanly.
        self.processes.ignore_sigkill = False
        manager.finalize(self.context)

    def test_gazebo_probe_retries_initial_empty_discovery(self):
        manager = self.manager()
        empty = subprocess.CompletedProcess(["gz", "service", "-l"], 0, "", "")
        ready = subprocess.CompletedProcess(
            ["gz", "service", "-l"], 0,
            "/world/school_arena/control\n/world/school_arena/control/state\n", "",
        )
        with patch("dataset_runner.simulator_manager.subprocess.run",
                   side_effect=[empty, ready]) as run:
            self.assertTrue(manager._gazebo_ready(self.context))
        self.assertEqual(run.call_count, 2)

    def test_external_runtime_is_never_signalled(self):
        self.ready_after = 1  # service exists before this manager launches
        manager = self.manager()
        with self.assertRaises(SimulatorError) as error:
            manager.reset(self.context)
        self.assertEqual(error.exception.code, "launch_failed")
        manager.finalize(self.context)
        self.assertEqual(self.processes.events, [])

    def test_readiness_timeout_has_distinct_code(self):
        self.ready_after = 999
        manager = self.manager()
        manager.reset(self.context)
        with self.assertRaises(SimulatorError) as error:
            manager.wait_ready(self.context)
        self.assertEqual(error.exception.code, "reset_timeout")
        manager.finalize(self.context)


if __name__ == "__main__":
    unittest.main()
