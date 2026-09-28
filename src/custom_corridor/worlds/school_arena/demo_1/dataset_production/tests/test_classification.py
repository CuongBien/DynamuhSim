import unittest

from dataset_production.classification import classify
from dataset_runner.episode_types import EpisodeResult, EpisodeStatus


class ClassificationTests(unittest.TestCase):
    def test_navigation_outcomes_are_valid_if_recording_and_qa_pass(self):
        for status in (EpisodeStatus.SUCCESS, EpisodeStatus.COLLISION,
                       EpisodeStatus.TIMEOUT, EpisodeStatus.STUCK,
                       EpisodeStatus.NAV_FAILURE):
            result = EpisodeResult("ep_000001", status, "observed", 10.0, ())
            with self.subTest(status=status):
                decision = classify(result, recording_ok=True,
                                    structured_ok=True, qa_pass=True)
                self.assertEqual((decision.status, decision.outcome), ("PASS", status.value))

    def test_infrastructure_and_data_failures_are_isolated(self):
        success = EpisodeResult("ep_000001", EpisodeStatus.SUCCESS, "goal", 10.0, ())
        crash = EpisodeResult("ep_000002", EpisodeStatus.SIM_FAILURE, "Gazebo crash", 2.0, ())
        self.assertEqual(classify(crash, recording_ok=True, structured_ok=True,
                                  qa_pass=True).status, "FAIL_INFRASTRUCTURE")
        for flags in ((False, True, True), (True, False, True), (True, True, False)):
            self.assertEqual(classify(success, recording_ok=flags[0],
                                      structured_ok=flags[1], qa_pass=flags[2]).status,
                             "FAIL_INFRASTRUCTURE")


if __name__ == "__main__":
    unittest.main()
