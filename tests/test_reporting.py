"""Keep report population/window/formatting policies and audit gap semantics."""
import ast
import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.analysis.reporting import recorded_poses, gap_table_row, robot_gap_series
from dynamuhsim.analysis.warehouse import robot_human_gaps


class ReportingTests(unittest.TestCase):
    def test_recording_filter_keeps_legacy_population_and_cutoff_tolerance(self):
        def row(time, count=7):
            return dict(time=time, poses={str(i): dict(x=0.,y=0.) for i in range(count)})
        rows = [row(0.),row(.5),row(.500000005),row(.50000002),row(.25,8),row(.25,6)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gazebo_poses.jsonl.gz"
            content = "".join(json.dumps(r)+"\n" for r in rows).encode()
            path.write_bytes(gzip.compress(content,mtime=0))
            before = path.read_bytes()
            self.assertEqual(recorded_poses(directory),rows[:4])
            self.assertEqual(recorded_poses(directory,assessment_end=.5),rows[:3])
            self.assertEqual(path.read_bytes(),before)

    def test_gap_table_preserves_column_order_units_precision_and_error_labels(self):
        gaps = dict(pair=-.00496,robot=.123456,static=-0.)
        self.assertEqual(gap_table_row(["SocNavGym guarded","crossing_near","Lỗi tại5.5s"],gaps,.987654),
                         "|SocNavGym guarded|crossing_near|Lỗi tại5.5s|-0.0050m|0.1235m|-0.0000m|98.77%|")
        self.assertEqual(gap_table_row(["fault_error01","Pause đúng khi fault"],gaps,1.),
                         "|fault_error01|Pause đúng khi fault|-0.0050m|0.1235m|-0.0000m|100.00%|")

    def test_robot_gap_series_uses_interval_ends_and_catches_between_frame_contact(self):
        poses = [dict(time=t,poses={"mecanum_robot":dict(x=0.,y=0.),"human":dict(x=x,y=0.)})
                 for t,x in [(0.,1.),(.1,-1.),(.2,-1.)]]
        times,gaps = robot_gap_series(poses,{1:"human"})
        self.assertEqual(times,[.1,.2])
        np.testing.assert_array_equal(gaps,[-.61,.39])
        self.assertEqual(robot_gap_series([],{}),([],[]))
        self.assertEqual(robot_gap_series(poses[:1],{1:"human"}),([],[]))

    def test_interval_gaps_match_audited_linear_closest_approach_boundaries(self):
        # Crossing, stationary, past/future closest approach: no extrapolation.
        before = [[[1.,0.]],[[1.,0.]],[[1.,0.]],[[2.,0.]]]
        after = [[[-1.,0.]],[[1.,0.]],[[2.,0.]],[[1.,0.]]]
        robot = [[0.,0.]]*4
        np.testing.assert_array_equal(robot_human_gaps(before,after,robot,robot),[-.61,.39,.39,.39])

    def test_all_original_cohort_and_failure_acceptance_assertions_remain(self):
        fixture = json.loads((ROOT / "tests/fixtures/reporting.json").read_text())
        for name,expected in fixture["policies"].items():
            source = (ROOT / "experiments/social_navigation/benchmark" / name / "summarize.py").read_text()
            assertions = [ast.dump(n,include_attributes=False) for n in ast.walk(ast.parse(source)) if isinstance(n,ast.Assert)]
            actual = dict(assertion_count=len(assertions),assertion_sha256=hashlib.sha256(json.dumps(assertions).encode()).hexdigest())
            self.assertEqual(actual,expected,name)


if __name__ == "__main__":
    unittest.main()
