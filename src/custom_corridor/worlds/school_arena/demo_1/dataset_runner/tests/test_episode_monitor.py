import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

DEMO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(DEMO))

from dataset_runner.episode_monitor import (EpisodeMonitor, MonitorConfig,
                                            RosGazeboMonitorBackend, classify_contact)
from dataset_runner.episode_types import EpisodeStatus, MonitorResult
from dataset_runner.nav2_manager import GoalObservation, GoalState, Nav2Error
from dataset_runner.simulator_manager import SimulatorError
from dataset_runner.hunav_manager import HuNavError
from dataset_runner.tests.test_episode_runner import (
    Fixture, FakeHuNav, FakeNav2, FakeReadiness, FakeSimulator,
)
from dataset_runner import EpisodeRunner


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Backend:
    def __init__(self, clock, *, contact_at=None, motion=0.0):
        self.clock = clock
        self.contact_at = contact_at
        self.motion = motion
        self.closed = False
        self.samples = []
        self.emitted = False

    def tick(self, seconds):
        self.clock.now += seconds
        self.samples.append((self.clock.now, self.motion * self.clock.now, 0.0))

    def observations(self):
        pair = ()
        if self.contact_at is not None and not self.emitted and self.clock.now >= self.contact_at:
            pair = (("robot::base_link::base_collision", "event_agent_1::link::collision"),)
            self.emitted = True
        return pair, tuple(self.samples)

    def close(self):
        self.closed = True


class Nav:
    def __init__(self, observations):
        self.observations = list(observations)
        self.calls = 0

    def poll(self, context):
        self.calls += 1
        if len(self.observations) > 1:
            item = self.observations.pop(0)
        else:
            item = self.observations[0]
        if isinstance(item, Exception):
            raise item
        return item


class Health:
    def __init__(self, error=None):
        self.error = error

    def check_health(self, context):
        if self.error:
            raise self.error


class MonitorTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.backend = Backend(self.clock)
        self.config = MonitorConfig(2.0, 0.1, True, 0.5, 0.05, 0.3,
                                    "/odom", ("/robot/contacts/base_link",))
        self.context = SimpleNamespace(humans={"hunav_loader": {"ros__parameters": {
            "agents": ["event_agent_1"]}}})

    def run_monitor(self, observations, *, simulator=None, hunav=None, backend=None, config=None):
        nav = Nav(observations)
        monitor = EpisodeMonitor(nav2_manager=nav, simulator_manager=simulator or Health(),
                                 hunav_manager=hunav or Health(), config=config or self.config,
                                 backend_factory=lambda: backend or self.backend, clock=self.clock)
        result = monitor.run(self.context)
        self.assertTrue((backend or self.backend).closed)
        return result, nav

    def test_succeeded_only_after_action_result(self):
        result, nav = self.run_monitor([GoalObservation(GoalState.ACCEPTED),
                                        GoalObservation(GoalState.EXECUTING),
                                        GoalObservation(GoalState.SUCCEEDED, 4, 0)])
        self.assertEqual((result.status, result.reason),
                         (EpisodeStatus.SUCCESS, "nav_goal_reached"))
        self.assertEqual(nav.calls, 3)

    def test_accepted_then_aborted_102_is_nav_failure(self):
        result, nav = self.run_monitor([GoalObservation(GoalState.ACCEPTED),
                                        GoalObservation(GoalState.ABORTED, 6, 102, "failed")])
        self.assertEqual((result.status, result.reason, result.navigation_result_code),
                         (EpisodeStatus.NAV_FAILURE, "nav_goal_aborted", 102))
        self.assertEqual(nav.calls, 2)

    def test_rejected_is_nav_failure(self):
        result, _ = self.run_monitor([GoalObservation(GoalState.REJECTED, 6)])
        self.assertEqual((result.status, result.reason),
                         (EpisodeStatus.NAV_FAILURE, "nav_goal_rejected"))

    def test_unexpected_idle_action_is_nav_failure(self):
        result, _ = self.run_monitor([GoalObservation(GoalState.IDLE)])
        self.assertEqual((result.status, result.reason),
                         (EpisodeStatus.NAV_FAILURE, "nav_goal_missing"))

    def test_timeout_starts_at_monitor_entry(self):
        result, _ = self.run_monitor([GoalObservation(GoalState.EXECUTING)],
                                     config=MonitorConfig(0.4, 0.1, False, 0.5, 0.05, 0.3,
                                                          "/odom", ("/robot/contacts/base_link",)))
        self.assertEqual(result.status, EpisodeStatus.TIMEOUT)
        self.assertAlmostEqual(result.duration_sec, 0.4)

    def test_stuck_requires_complete_window(self):
        result, _ = self.run_monitor([GoalObservation(GoalState.EXECUTING)])
        self.assertEqual(result.status, EpisodeStatus.STUCK)
        self.assertGreaterEqual(result.duration_sec, 0.5)

    def test_insufficient_odom_window_does_not_mark_stuck(self):
        result, _ = self.run_monitor([GoalObservation(GoalState.EXECUTING)],
            config=MonitorConfig(0.4, 0.1, True, 0.5, 0.05, 0.3,
                                 "/odom", ("/robot/contacts/base_link",)))
        self.assertEqual(result.status, EpisodeStatus.TIMEOUT)

    def test_movement_above_threshold_avoids_stuck(self):
        result, _ = self.run_monitor([GoalObservation(GoalState.EXECUTING)],
                                     backend=Backend(self.clock, motion=0.2))
        self.assertEqual(result.status, EpisodeStatus.TIMEOUT)

    def test_near_goal_feedback_avoids_stuck(self):
        result, _ = self.run_monitor([GoalObservation(GoalState.EXECUTING,
                                                      distance_remaining=0.2)])
        self.assertEqual(result.status, EpisodeStatus.TIMEOUT)

    def test_real_contact_precedes_action_result(self):
        result, nav = self.run_monitor([GoalObservation(GoalState.EXECUTING),
                                        GoalObservation(GoalState.SUCCEEDED)],
                                       backend=Backend(self.clock, contact_at=0.1))
        self.assertEqual((result.status, result.collision_kind),
                         (EpisodeStatus.COLLISION, "human"))
        self.assertEqual(nav.calls, 1)
        self.assertTrue(result.collision_occurred)

    def test_gazebo_entity_names_are_extracted_from_real_protobuf(self):
        import threading
        from collections import deque
        from gz.msgs10.contacts_pb2 import Contacts
        message = Contacts()
        contact = message.contact.add()
        contact.collision1.id = 7
        contact.collision1.name = "robot::base_link::base_collision"
        contact.collision2.id = 35
        contact.collision2.name = "probe_wall::link::collision"
        backend = SimpleNamespace(_lock=threading.Lock(), _contacts=deque())
        RosGazeboMonitorBackend._on_contacts(backend, message)
        self.assertEqual(backend._contacts.pop(),
                         ("robot::base_link::base_collision", "probe_wall::link::collision"))

    def test_ground_and_self_contact_ignored_and_static_classified(self):
        self.assertIsNone(classify_contact(("robot::wheel", "ground::link"), set()))
        self.assertIsNone(classify_contact(("robot::wheel", "robot::base"), set()))
        self.assertEqual(classify_contact(("robot::base", "wall::link"), set()),
                         "static_obstacle")

    def test_simulator_exit(self):
        result, _ = self.run_monitor([GoalObservation(GoalState.EXECUTING)],
            simulator=Health(SimulatorError("simulator_process_exited", "gone")))
        self.assertEqual((result.status, result.reason),
                         (EpisodeStatus.SIM_FAILURE, "simulator_process_exited"))

    def test_hunav_exit(self):
        result, _ = self.run_monitor([GoalObservation(GoalState.EXECUTING)],
            hunav=Health(HuNavError("hunav_runtime_exited", "gone")))
        self.assertEqual((result.status, result.reason),
                         (EpisodeStatus.SIM_FAILURE, "hunav_runtime_exited"))

    def test_nav2_server_lost(self):
        result, _ = self.run_monitor([Nav2Error("nav2_runtime_lost", "gone")])
        self.assertEqual((result.status, result.reason),
                         (EpisodeStatus.SIM_FAILURE, "nav2_runtime_lost"))


class RunnerMonitorFinalizationTests(Fixture):
    def test_every_terminal_status_finalizes_goal_and_runtime_once(self):
        for status in (EpisodeStatus.SUCCESS, EpisodeStatus.COLLISION,
                       EpisodeStatus.TIMEOUT, EpisodeStatus.STUCK,
                       EpisodeStatus.NAV_FAILURE, EpisodeStatus.SIM_FAILURE):
            with self.subTest(status=status):
                events = []
                monitor = SimpleNamespace(run=lambda ctx: MonitorResult(status, status.value, 1.0))
                runner = EpisodeRunner(loader=self.loader(), simulator_manager=FakeSimulator(events),
                                       hunav_manager=FakeHuNav(events), nav2_manager=FakeNav2(events),
                                       readiness_checker=FakeReadiness(events), monitor=monitor)
                result = runner.run(self.episode)
                self.assertEqual(result.status, status)
                self.assertEqual(events.count("nav_finalize"), 1)
                self.assertEqual(events.count("hunav_finalize"), 1)
                self.assertEqual(events.count("sim_finalize"), 1)
                self.assertIsInstance(result.monitor_result, MonitorResult)


if __name__ == "__main__":
    unittest.main()
