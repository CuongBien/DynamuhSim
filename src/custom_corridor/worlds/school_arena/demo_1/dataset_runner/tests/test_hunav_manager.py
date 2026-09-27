import os
import signal
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

DEMO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(DEMO))
from dataset_runner import EpisodeContext, Pose2D
from dataset_runner.hunav_manager import (
    DockerHuNavRuntime, HuNavConfig, HuNavError, HuNavManager, OwnedHuNavProcess,
)
from dataset_runner.hunav_container_probe import owned_processes


class FakeHostProcess:
    def __init__(self):
        self.returncode = None

    def poll(self):
        return self.returncode


class FakeRuntime:
    def __init__(self):
        self.events = []
        self.running = True
        self.external = []
        self.installer_error = None
        self.services = True
        self.observation = {"status": "ready", "count": 2,
                            "names": ["event_agent_1", "background_agent_1"],
                            "services": ["/get_parameters", "/compute_agents"]}
        self.processes = []

    def container_running(self):
        return self.running

    def external_nodes(self):
        return self.external

    def install(self, context, humans_file):
        self.events.append(("install", humans_file))
        if self.installer_error:
            raise HuNavError("load_failed", self.installer_error)

    def launch(self, role, command, context):
        self.events.append(("launch", role, command))
        process = OwnedHuNavProcess(role, role, 100 + len(self.processes), FakeHostProcess())
        self.processes.append(process)
        return process

    def wait_service(self, service):
        self.events.append(("wait_service", service))
        return self.services

    def observe(self, names):
        self.events.append(("observe", names))
        return self.observation

    def stop(self, process):
        self.events.append(("stop", process.role))
        process.host_process.returncode = 0


class HuNavManagerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.episode = Path(self.temp.name) / "ep_000002"
        self.episode.mkdir()
        self.humans_file = self.episode / "humans.yaml"
        self.humans_file.write_text(yaml.safe_dump({
            "hunav_loader": {"ros__parameters": {
                "agents": ["event_agent_1", "background_agent_1"]}}
        }), encoding="utf-8")
        self.context = EpisodeContext(
            episode_dir=self.episode, episode_id="ep_000002",
            scenario_family="head_on", seed=101, metadata={"humans": {"total": 2}},
            scenario={}, humans={}, start_pose=Pose2D(0.0, -7.85, 0.0),
            goal_node="corridor_a_west", goal_pose=Pose2D(-14.0, -8.15, 0.0),
        )
        self.runtime = FakeRuntime()
        self.manager = HuNavManager(config=HuNavConfig(), runtime=self.runtime)

    def test_loads_exact_episode_humans_and_waits_for_services(self):
        self.manager.load_scenario(self.context)
        self.assertEqual(self.runtime.events[0], ("install", self.humans_file))
        launches = [e for e in self.runtime.events if e[0] == "launch"]
        self.assertEqual([e[1] for e in launches], ["loader", "manager", "bridge"])
        self.assertIn(f"/scenarios/{self.context.episode_id}.yaml", " ".join(launches[0][2]))
        self.assertEqual([e[1] for e in self.runtime.events if e[0] == "wait_service"],
                         ["/get_parameters", "/compute_agents"])
        self.assertEqual(self.manager.expected_names,
                         ["event_agent_1", "background_agent_1"])
        self.manager.finalize(self.context)

    def test_missing_humans_file(self):
        self.humans_file.unlink()
        with self.assertRaises(HuNavError) as error:
            self.manager.load_scenario(self.context)
        self.assertEqual(error.exception.code, "humans_file_missing")
        self.assertEqual(self.runtime.events, [])

    def test_container_unavailable(self):
        self.runtime.running = False
        with self.assertRaises(HuNavError) as error:
            self.manager.load_scenario(self.context)
        self.assertEqual(error.exception.code, "container_unavailable")

    def test_actual_installer_nonzero_exit_is_load_failed(self):
        def failed_command(args, **kwargs):
            return subprocess.CompletedProcess(args, 1, "", "docker cp failed")

        runtime = DockerHuNavRuntime(HuNavConfig(), run_command=failed_command)
        with self.assertRaises(HuNavError) as error:
            runtime.install(self.context, self.humans_file)
        self.assertEqual(error.exception.code, "load_failed")
        self.assertIn("docker cp failed", str(error.exception))

    def test_installer_failure(self):
        self.runtime.installer_error = "docker cp failed"
        with self.assertRaises(HuNavError) as error:
            self.manager.load_scenario(self.context)
        self.assertEqual(error.exception.code, "load_failed")
        self.assertFalse(any(e[0] == "launch" for e in self.runtime.events))

    def test_hunav_ready_and_count_verified(self):
        self.manager.load_scenario(self.context)
        self.manager.wait_ready(self.context)
        self.assertEqual(self.manager.actual_count, 2)
        self.assertIn(("observe", ["event_agent_1", "background_agent_1"]),
                      self.runtime.events)
        self.manager.finalize(self.context)

    def test_readiness_timeout(self):
        self.manager.load_scenario(self.context)
        self.runtime.observation = {"status": "timeout", "names": None,
                                    "services": ["/get_parameters", "/compute_agents"]}
        with self.assertRaises(HuNavError) as error:
            self.manager.wait_ready(self.context)
        self.assertEqual(error.exception.code, "hunav_not_ready")
        self.manager.finalize(self.context)

    def test_human_count_mismatch(self):
        self.manager.load_scenario(self.context)
        self.runtime.observation = {"status": "human_count_mismatch",
                                    "names": ["event_agent_1"]}
        with self.assertRaises(HuNavError) as error:
            self.manager.wait_ready(self.context)
        self.assertEqual(error.exception.code, "human_count_mismatch")
        self.manager.finalize(self.context)

    def test_reset_restarts_only_owned_processes_and_finalize_is_idempotent(self):
        self.manager.load_scenario(self.context)
        self.manager.reset(self.context)
        self.assertEqual([e[1] for e in self.runtime.events if e[0] == "stop"],
                         ["bridge", "manager", "loader"])
        self.assertEqual(self.manager.owned, [])
        self.manager.load_scenario(self.context)
        self.manager.finalize(self.context)
        stopped = len([e for e in self.runtime.events if e[0] == "stop"])
        self.manager.finalize(self.context)
        self.assertEqual(len([e for e in self.runtime.events if e[0] == "stop"]), stopped)

    def test_unowned_nodes_are_not_killed(self):
        self.runtime.external = ["/hunav_agent_manager"]
        with self.assertRaises(HuNavError) as error:
            self.manager.load_scenario(self.context)
        self.assertEqual(error.exception.code, "load_failed")
        self.assertFalse(any(e[0] in ("launch", "stop") for e in self.runtime.events))

    def test_runtime_exit_is_reported(self):
        self.manager.load_scenario(self.context)
        self.manager.owned[1].host_process.returncode = 3
        with self.assertRaises(HuNavError) as error:
            self.manager.wait_ready(self.context)
        self.assertEqual(error.exception.code, "hunav_runtime_exited")
        self.manager.finalize(self.context)


class ProcessTokenTests(unittest.TestCase):
    def test_token_identifies_only_owned_process_group(self):
        token = "pbl6_test_" + os.urandom(8).hex()
        env = os.environ.copy()
        env["PBL6_HUNAV_TOKEN"] = token
        owned = subprocess.Popen(["sleep", "20"], env=env, start_new_session=True)
        external = subprocess.Popen(["sleep", "20"], start_new_session=True)
        try:
            members = owned_processes(token)
            self.assertIn(owned.pid, [item["pid"] for item in members])
            self.assertNotIn(external.pid, [item["pid"] for item in members])
            os.killpg(owned.pid, signal.SIGINT)
            owned.wait(timeout=5)
            self.assertIsNone(external.poll())
        finally:
            if owned.poll() is None:
                owned.kill()
            if external.poll() is None:
                external.terminate()
            owned.wait(timeout=5)
            external.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
