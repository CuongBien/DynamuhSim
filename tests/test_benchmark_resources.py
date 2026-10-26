"""Pin hardware query formats and the scaling measurement window."""
import ast
import json
import tempfile
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from experiments.gazebo_benchmark.core import profiler
from test_mppi_monitor import FakeNode, FakeParameter


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


def telemetry_classes():
    path = Path(__file__).resolve().parents[1] / 'experiments/gazebo_benchmark/core/profiler/benchmark_profiler.py'
    classes = [n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name != 'BenchmarkProfiler']
    namespace = dict(Node=FakeNode, Clock=NS, LaserScan=NS, Image=NS,
                     rclpy=NS(parameter=NS(Parameter=FakeParameter), Parameter=FakeParameter), time=NS(time=lambda: 12.5))
    exec(compile(ast.Module(body=classes, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


class ScalingCollectorTests(unittest.TestCase):
    def test_profiles_preserve_node_names_topics_qos_parameters_and_latest_fields(self):
        classes = telemetry_classes()
        for name, prefix, qos, depth, latest in [
            ('DomainTelemetryListener', 'telemetry_listener_dom', 50, '/camera/depth_image', False),
            ('DomainClockListener', 'clock_listener_dom', 50, '/camera/depth_image', True),
            ('TelemetryCollector', 'telemetry_collector_', 100, '/camera/depth/image_rect_raw', False),
        ]:
            with self.subTest(profile=name):
                node = classes[name](82)
                sensors = [(depth, 10), ('/scan', 10)] if name == 'TelemetryCollector' else [('/scan', 10), (depth, 10)]
                self.assertEqual(node.name, prefix + '82')
                self.assertEqual(node.subscriptions, [('/clock', qos), *sensors])
                self.assertEqual([p.values for p in getattr(node, 'parameters', [])], [('use_sim_time', 1, True)] if name == 'TelemetryCollector' else [])
                self.assertEqual(hasattr(node, 'latest_sim_time'), latest)
                self.assertEqual((node.samples, node.has_scan, node.has_depth), ([], False, False))
                node._clock_cb(NS(clock=NS(sec=3, nanosec=250_000_000)))
                node._depth_cb(NS()); node._scan_cb(NS())
                self.assertEqual((node.samples, node.has_scan, node.has_depth), ([(12.5, 3.25)], True, True))
                if latest:
                    self.assertEqual((node.latest_wall_time, node.latest_sim_time), (12.5, 3.25))

    def test_duplicate_and_reset_clock_samples_remain_ordered_and_instance_local(self):
        classes = telemetry_classes()
        for name in ('DomainTelemetryListener', 'DomainClockListener', 'TelemetryCollector'):
            with self.subTest(profile=name):
                first, second = classes[name](81), classes[name](82)
                for sec, nanos in [(3, 999_999_999), (4, 0), (4, 0), (0, 0)]:
                    first._clock_cb(NS(clock=NS(sec=sec, nanosec=nanos)))
                self.assertEqual(first.samples, [(12.5, 3.999999999), (12.5, 4.), (12.5, 4.), (12.5, 0.)])
                self.assertEqual(second.samples, [])
                first._scan_cb(NS())
                self.assertFalse(second.has_scan)

    def test_duration_worker_preserves_domain_json_and_ros_shutdown_order(self):
        path = Path(__file__).resolve().parents[1] / 'experiments/gazebo_benchmark/core/profiler/benchmark_profiler.py'
        function = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'collect_telemetry')
        events, environ = [], {}
        node = NS(samples=[(10., 2.)], has_scan=True, has_depth=False, destroy_node=lambda: events.append('destroy'))
        ros = NS(init=lambda: events.append('init'), shutdown=lambda: events.append('shutdown'), ok=lambda: True,
                 spin_once=lambda node, timeout_sec: events.append(('spin', timeout_sec)))
        clock = iter([0., 1., 3.])
        namespace = dict(os=NS(environ=environ), rclpy=ros, time=NS(time=lambda: next(clock)), json=json)
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
        def collector(domain):
            events.append(('node', domain))
            return node
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'worker.json'
            namespace['collect_telemetry'](81, 2., output, collector)
            self.assertEqual(output.read_text(), '{"samples": [[10.0, 2.0]], "has_scan": true, "has_depth": false}')
        self.assertEqual(environ, {'ROS_DOMAIN_ID': '81'})
        self.assertEqual(events, ['init', ('node', 81), ('spin', .01), 'destroy', 'shutdown'])
