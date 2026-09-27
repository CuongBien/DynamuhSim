import copy
import logging
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

DEMO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(DEMO))

from dataset_runner.nav2_manager import Nav2Error

from dataset_runner import (
    EpisodeHooks, EpisodeLoader, EpisodeRunner, EpisodeState, EpisodeStatus,
    EpisodeTermination, InvalidStateTransition,
)


class Fixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.episode = root / "ep_000001"
        self.episode.mkdir()
        self.graph = root / "graph.yaml"
        self.graph.write_text(yaml.safe_dump({
            "frame_id": "map",
            "nodes": [
                {"id": "start", "x": 1.0, "y": 2.0},
                {"id": "goal", "x": 4.0, "y": 2.0},
            ],
        }), encoding="utf-8")
        robot = {
            "start_node": "start", "goal_node": "goal",
            "start_pose": {"x": 1.0, "y": 2.0, "heading": 0.0},
        }
        self.metadata = {
            "episode_id": "ep_000001", "scenario_family": "head_on", "seed": 42,
            "density": "low", "robot": copy.deepcopy(robot),
            "humans": {"total": 1},
        }
        self.scenario = {
            "scenario_family": "head_on", "seed": 42,
            "robot": {**copy.deepcopy(robot), "route": ["start", "goal"], "heading": 0.0},
            "humans": [{"name": "event_agent_1"}],
        }
        self.humans = {"hunav_loader": {"ros__parameters": {
            "agents": ["event_agent_1"],
        }}}
        self.write()

    def write(self):
        for name, doc in (
            ("metadata.yaml", self.metadata), ("scenario.yaml", self.scenario),
            ("humans.yaml", self.humans),
        ):
            (self.episode / name).write_text(yaml.safe_dump(doc), encoding="utf-8")

    def loader(self):
        return EpisodeLoader(self.graph)


class LoaderTests(Fixture):
    def test_valid_episode_resolves_existing_start_and_goal(self):
        context = self.loader().load(self.episode)
        self.assertEqual((context.episode_id, context.scenario_family, context.seed),
                         ("ep_000001", "head_on", 42))
        self.assertEqual((context.start_pose.x, context.start_pose.y,
                          context.start_pose.heading), (1.0, 2.0, 0.0))
        self.assertEqual(context.goal_node, "goal")
        self.assertEqual((context.goal_pose.x, context.goal_pose.y,
                          context.goal_pose.heading), (4.0, 2.0, 0.0))

    def test_missing_each_yaml_returns_invalid_episode(self):
        for filename in ("metadata.yaml", "scenario.yaml", "humans.yaml"):
            with self.subTest(filename=filename):
                path = self.episode / filename
                contents = path.read_bytes()
                path.unlink()
                try:
                    result = EpisodeRunner(loader=self.loader()).run(self.episode)
                    self.assertEqual(result.status, EpisodeStatus.INVALID_EPISODE)
                    self.assertEqual(result.transitions,
                                     (EpisodeState.PREPARE, EpisodeState.FINALIZE))
                    self.assertIn(filename, result.termination_reason)
                finally:
                    path.write_bytes(contents)

    def test_unknown_goal_node_is_invalid(self):
        self.metadata["robot"]["goal_node"] = "absent"
        self.scenario["robot"].update(goal_node="absent", route=["start", "absent"])
        self.write()
        result = EpisodeRunner(loader=self.loader()).run(self.episode)
        self.assertEqual(result.status, EpisodeStatus.INVALID_EPISODE)
        self.assertIn("Unknown robot goal node", result.termination_reason)


class FakeSimulator:
    def __init__(self, events):
        self.events = events

    def reset(self, context):
        self.events.append("reset")

    def wait_ready(self, context):
        self.events.append("sim_ready")

    def finalize(self, context):
        self.events.append("sim_finalize")


class FakeHuNav:
    def __init__(self, events):
        self.events = events

    def load_scenario(self, context):
        self.events.append("load_hunav")

    def wait_ready(self, context):
        self.events.append("hunav_ready")

    def finalize(self, context):
        self.events.append("hunav_finalize")


class FakeNav2:
    def __init__(self, events):
        self.events = events

    def wait_ready(self, context):
        self.events.append("nav_ready")

    def send_goal(self, context):
        self.events.append("send_goal")

    def finalize(self, context):
        self.events.append("nav_finalize")


class FakeMonitor:
    def __init__(self, events):
        self.events = events

    def run(self, context):
        self.events.append("monitor")
        return EpisodeTermination(EpisodeStatus.SUCCESS, "nav_goal_reached")


class RecordingHooks(EpisodeHooks):
    def __init__(self, events):
        self.events = events

    def on_episode_prepare(self, context):
        self.events.append("hook_prepare")

    def on_episode_start(self, context):
        self.events.append("hook_start")

    def on_episode_end(self, context, result):
        self.events.append("hook_end")


class RunnerTests(Fixture):
    def make_runner(self, events):
        return EpisodeRunner(
            loader=self.loader(), simulator_manager=FakeSimulator(events),
            hunav_manager=FakeHuNav(events), nav2_manager=FakeNav2(events),
            monitor=FakeMonitor(events), hooks=RecordingHooks(events),
        )

    def test_state_order_and_hooks(self):
        events = []
        runner = self.make_runner(events)
        with self.assertLogs("dataset_runner.episode_runner", logging.INFO) as logs:
            result = runner.run(self.episode)
        self.assertEqual(result.status, EpisodeStatus.SUCCESS)
        self.assertEqual(result.termination_reason, "nav_goal_reached")
        self.assertEqual(result.transitions, tuple(EpisodeState))
        self.assertEqual(events, [
            "hook_prepare", "reset", "load_hunav", "sim_ready", "hunav_ready",
            "nav_ready", "send_goal", "hook_start", "monitor", "nav_finalize",
            "hunav_finalize", "sim_finalize", "hook_end",
        ])
        for state in EpisodeState:
            self.assertTrue(any(f"[{state.name}]" in line for line in logs.output))

    def test_invalid_transition_is_guarded(self):
        runner = self.make_runner([])
        with self.assertRaises(InvalidStateTransition):
            runner.transition(EpisodeState.RUNNING)
        runner.transition(EpisodeState.PREPARE)
        with self.assertRaises(InvalidStateTransition):
            runner.transition(EpisodeState.START)
        runner.transition(EpisodeState.FINALIZE)
        with self.assertRaises(InvalidStateTransition):
            runner.transition(EpisodeState.RESET)

    def test_nav2_failure_codes_map_to_episode_status(self):
        for code, expected in (("goal_rejected", EpisodeStatus.NAV_FAILURE),
                               ("nav2_runtime_lost", EpisodeStatus.SIM_FAILURE)):
            with self.subTest(code=code):
                runner = self.make_runner([])
                def fail(context):
                    raise Nav2Error(code, "test failure")
                if code == "goal_rejected":
                    runner.nav2_manager.send_goal = fail
                else:
                    runner.nav2_manager.wait_ready = fail
                with self.assertLogs("dataset_runner.episode_runner", logging.ERROR):
                    result = runner.run(self.episode)
                self.assertEqual(result.status, expected)
                self.assertIn(code, result.termination_reason)
                self.assertEqual(result.transitions[-1], EpisodeState.FINALIZE)

    def test_missing_managers_cannot_report_success(self):
        result = EpisodeRunner(loader=self.loader()).run(self.episode)
        self.assertEqual(result.status, EpisodeStatus.SIM_FAILURE)
        self.assertEqual(result.transitions,
                         (EpisodeState.PREPARE, EpisodeState.FINALIZE))
        self.assertIn("Runtime managers not configured", result.termination_reason)


if __name__ == "__main__":
    unittest.main()
