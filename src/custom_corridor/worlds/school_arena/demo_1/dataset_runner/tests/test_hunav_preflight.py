import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

DEMO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(DEMO))

from dataset_runner.hunav_manager import HuNavConfig, HuNavError
from dataset_runner.scripts.smoke_runtime_integration import HuNavPreflight


class FakeRuntime:
    def __init__(self):
        self.running = True
        self.external = []

    def container_running(self):
        return self.running

    def external_nodes(self):
        return list(self.external)


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.config = HuNavConfig(startup_timeout_sec=0.03,
                                  command_timeout_sec=0.01,
                                  poll_interval_sec=0.01)
        self.runtime = FakeRuntime()
        self.clock = [0.0]
        self.commands = []
        self.image = "gz_fortress_hunavsim"
        self.ros_ready = True
        self.restart_fails = False
        self.external_after_restart = []

    def run_command(self, command, **kwargs):
        self.commands.append(command)
        if command[:2] == ["docker", "inspect"]:
            return SimpleNamespace(returncode=0,
                                   stdout=f"container-id {self.image}\n", stderr="")
        if command[:2] == ["docker", "restart"]:
            if self.restart_fails:
                return SimpleNamespace(returncode=1, stdout="", stderr="restart denied")
            self.runtime.running = True
            self.runtime.external = list(self.external_after_restart)
            return SimpleNamespace(returncode=0, stdout=self.config.container_name,
                                   stderr="")
        if command[:2] == ["docker", "exec"]:
            return SimpleNamespace(returncode=0 if self.ros_ready else 1,
                                   stdout="", stderr="ROS not ready")
        raise AssertionError(f"Unexpected command: {command}")

    def preflight(self):
        return HuNavPreflight(self.config, self.runtime,
                             run_command=self.run_command,
                             clock=lambda: self.clock[0],
                             pause=lambda seconds: self.clock.__setitem__(
                                 0, self.clock[0] + seconds))

    def assert_code(self, code, fn):
        with self.assertRaises(HuNavError) as cm:
            fn()
        self.assertEqual(cm.exception.code, code)

    def test_no_external_continues_without_restart(self):
        self.preflight().check()
        self.assertFalse(any(cmd[:2] == ["docker", "restart"]
                             for cmd in self.commands))
        self.assertTrue(any(cmd[:2] == ["docker", "exec"]
                            for cmd in self.commands))

    def test_external_default_fails_without_takeover(self):
        self.runtime.external = ["389 hunav_loader", "439 hunav_agent_manager"]
        self.assert_code("external_runtime", lambda: self.preflight().check())
        self.assertFalse(any(cmd[:2] == ["docker", "restart"]
                             for cmd in self.commands))

    def test_explicit_restart_only_configured_container_then_continues(self):
        self.runtime.external = ["389 hunav_loader"]
        self.preflight().check(reset_container=True)
        restarts = [cmd for cmd in self.commands if cmd[:2] == ["docker", "restart"]]
        self.assertEqual(restarts,
                         [["docker", "restart", "--time", "10", self.config.container_name]])
        self.assertEqual(self.runtime.external, [])
        self.assertTrue(self.runtime.running)

    def test_restart_failure_has_clear_code(self):
        self.runtime.external = ["389 hunav_loader"]
        self.restart_fails = True
        self.assert_code("cleanup_failed",
                         lambda: self.preflight().check(reset_container=True))

    def test_container_readiness_timeout_has_clear_code(self):
        self.ros_ready = False
        self.assert_code("container_readiness_timeout",
                         lambda: self.preflight().check())
        self.assertGreaterEqual(self.clock[0], self.config.startup_timeout_sec)

    def test_wrong_image_is_never_restarted(self):
        self.runtime.external = ["389 hunav_loader"]
        self.image = "unrelated-image"
        self.assert_code("unsafe_container",
                         lambda: self.preflight().check(reset_container=True))
        self.assertFalse(any(cmd[:2] == ["docker", "restart"]
                             for cmd in self.commands))

    def test_stopped_configured_container_requires_opt_in(self):
        self.runtime.running = False
        self.assert_code("container_unavailable", lambda: self.preflight().check())
        self.preflight().check(reset_container=True)
        self.assertTrue(self.runtime.running)


if __name__ == "__main__":
    unittest.main()
