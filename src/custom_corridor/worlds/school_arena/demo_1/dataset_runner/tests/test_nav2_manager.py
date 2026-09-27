import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

DEMO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(DEMO))

from dataset_runner.episode_types import Pose2D
from dataset_runner.nav2_manager import (
    GoalObservation, GoalState, Nav2Config, Nav2Error, Nav2Manager, REQUIRED_NODES,
)


class FakeBackend:
    def __init__(self):
        self.states = {name: "active" for name in REQUIRED_NODES}
        self.ready = True
        self.sent = []
        self.accepted = True
        self.result = GoalObservation(GoalState.EXECUTING, 2)
        self.cancel_accepted = True
        self.cancel_count = 0
        self.closed = False
        self.ticks = 0

    def lifecycle_state(self, name, timeout):
        return self.states[name]

    def action_ready(self):
        return self.ready

    def tick(self, seconds):
        self.ticks += 1

    def send_goal(self, pose, timeout):
        self.sent.append(pose)
        return SimpleNamespace(accepted=self.accepted)

    def goal_result(self, handle):
        return self.result

    def cancel_goal(self, handle, timeout):
        self.cancel_count += 1
        if self.cancel_accepted:
            self.result = GoalObservation(GoalState.CANCELED, 5)
        return self.cancel_accepted

    def close(self):
        self.closed = True


class Nav2ManagerTests(unittest.TestCase):
    def setUp(self):
        self.backend = FakeBackend()
        self.now = 0.0
        self.config = Nav2Config(startup_timeout_sec=0.03, service_timeout_sec=0.001,
                                 goal_response_timeout_sec=0.01, cancel_timeout_sec=0.01,
                                 poll_interval_sec=0.01)
        self.context = SimpleNamespace(episode_id="ep_000001",
                                       goal_pose=Pose2D(4.0, -2.0, math.pi / 2))
        self.manager = Nav2Manager(config=self.config, backend_factory=lambda: self.backend,
                                   clock=lambda: self.now)
        def advance(seconds):
            self.now += seconds
            self.backend.ticks += 1
        self.backend.tick = advance

    def assert_code(self, code, fn):
        with self.assertRaises(Nav2Error) as cm:
            fn()
        self.assertEqual(cm.exception.code, code)

    def test_ready_requires_active_lifecycle_and_action(self):
        self.manager.wait_ready(self.context)
        self.assertEqual(self.backend.ticks, 0)

    def test_inactive_lifecycle_timeout(self):
        self.backend.states["amcl"] = "inactive"
        self.assert_code("lifecycle_not_active", lambda: self.manager.wait_ready(self.context))

    def test_missing_lifecycle_timeout(self):
        self.backend.states["map_server"] = None
        self.assert_code("lifecycle_not_active", lambda: self.manager.wait_ready(self.context))

    def test_action_server_unavailable(self):
        self.backend.ready = False
        self.assert_code("action_server_unavailable", lambda: self.manager.wait_ready(self.context))

    def test_goal_uses_context_pose_and_accepts(self):
        observation = self.manager.send_goal(self.context)
        self.assertEqual(observation.state, GoalState.ACCEPTED)
        self.assertIs(self.backend.sent[0], self.context.goal_pose)

    def test_goal_rejected(self):
        self.backend.accepted = False
        self.assert_code("goal_rejected", lambda: self.manager.send_goal(self.context))
        self.assertEqual(self.manager.poll(self.context).state, GoalState.REJECTED)

    def test_goal_send_failed(self):
        def fail(pose, timeout):
            raise RuntimeError("failed")
        self.backend.send_goal = fail
        self.assert_code("goal_send_failed", lambda: self.manager.send_goal(self.context))

    def test_success_and_abort_preserve_result(self):
        self.manager.send_goal(self.context)
        self.backend.result = GoalObservation(GoalState.SUCCEEDED, 4, 0, "")
        self.assertEqual(self.manager.poll(self.context).state, GoalState.SUCCEEDED)
        self.backend.result = GoalObservation(GoalState.ABORTED, 6, 42, "planner failed")
        result = self.manager.poll(self.context)
        self.assertEqual((result.state, result.status_code, result.error_code, result.error_msg),
                         (GoalState.ABORTED, 6, 42, "planner failed"))
        self.assertEqual(result.failure_code, "goal_aborted")

    def test_cancel_acknowledged_and_idempotent(self):
        self.manager.send_goal(self.context)
        self.manager.cancel_goal(self.context)
        self.manager.cancel_goal(self.context)
        self.assertEqual(self.backend.cancel_count, 1)
        self.assertEqual(self.manager.poll(self.context).state, GoalState.CANCELED)

    def test_cancel_failure_retains_handle(self):
        self.manager.send_goal(self.context)
        self.backend.cancel_accepted = False
        self.assert_code("goal_cancel_failed", lambda: self.manager.cancel_goal(self.context))
        self.assertIsNotNone(self.manager.handle)

    def test_previous_goal_canceled_before_new_goal(self):
        self.manager.send_goal(self.context)
        self.manager.send_goal(self.context)
        self.assertEqual(self.backend.cancel_count, 1)
        self.assertEqual(len(self.backend.sent), 2)

    def test_runtime_lost(self):
        self.manager.send_goal(self.context)
        self.backend.ready = False
        self.assert_code("nav2_runtime_lost", lambda: self.manager.poll(self.context))
        self.assertEqual(self.manager.observation.state, GoalState.SERVER_LOST)

    def test_finalize_is_idempotent(self):
        self.manager.send_goal(self.context)
        self.manager.finalize(self.context)
        self.manager.finalize(self.context)
        self.assertEqual(self.backend.cancel_count, 1)
        self.assertTrue(self.backend.closed)


if __name__ == "__main__":
    unittest.main()
