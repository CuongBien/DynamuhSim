"""Pin hardware query formats and the scaling measurement window."""
import subprocess
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from experiments.gazebo_benchmark.core import profiler


class BenchmarkResourceTests(unittest.TestCase):
    def test_gpu_contracts_preserve_types_columns_and_timeouts(self):
        for output, floats, integers in [
            ("42, 1024\n", (42., 1024.), (42, 1024)),
            ("42.5, 1024.5", (42.5, 1024.5), (0, 0)),
            ("1,2,3", (0., 0.), (1, 2)),
            ("1,2\n3,4", (0., 0.), (0, 0)),
            ("N/A, 20", (0., 0.), (0, 0)),
        ]:
            for query, expected, kwargs in [(profiler.query_gpu, floats, {"text": True}),
                                             (profiler.query_gpu_int, integers, {"text": True, "timeout": 1.})]:
                with self.subTest(output=output, query=query.__name__), patch.object(profiler.subprocess, "check_output", return_value=output) as command:
                    actual = query()
                    self.assertEqual(actual, expected)
                    self.assertEqual(tuple(map(type, actual)), tuple(map(type, expected)))
                    command.assert_called_once_with(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader,nounits"], **kwargs)
        for error in (FileNotFoundError("missing"), subprocess.TimeoutExpired("nvidia-smi", 1.)):
            with patch.object(profiler.subprocess, "check_output", side_effect=error):
                self.assertEqual(profiler.query_gpu(), (0., 0.))
                self.assertEqual(profiler.query_gpu_int(), (0, 0))

    def test_scaling_deadline_sampling_order_and_process_breakdown(self):
        for duration, expected_count in [(0., 0), (.5, 1), (1., 2)]:
            events = []
            clock = NS(now=4.)
            def cpu(**kwargs):
                events.append(("cpu", kwargs))
                return 20.
            def gpu():
                events.append("gpu")
                return 40., 1000.
            def breakdown():
                events.append("breakdown")
                return 1., 2., 3., 4.
            def sleep(seconds):
                events.append(("sleep", seconds))
                clock.now += seconds
            with patch.object(profiler, "time", NS(time=lambda: clock.now, sleep=sleep)):
                actual = profiler.sample_resources(4., duration, cpu, gpu, breakdown)
            self.assertEqual(actual, ([20.] * expected_count, [40.] * expected_count, [1000.] * expected_count, [(1., 2., 3., 4.)] * expected_count))
            self.assertEqual(events, [("cpu", {"interval": None}), "gpu", "breakdown", ("sleep", .5)] * expected_count)
