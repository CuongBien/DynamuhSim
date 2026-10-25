"""Regression checks for the A3 environment and sim-time collector contracts."""
import importlib.util
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from experiments.gazebo_benchmark.core import sim_env
from test_mppi_monitor import FakeNode as MonitorNode

ROOT = Path(__file__).resolve().parents[1]


class FakeNode(MonitorNode):
    def create_publisher(self, kind, topic, qos):
        self.published = []
        self.publisher_spec = (topic, qos)
        return NS(publish=self.published.append)


def twist():
    return NS(header=NS(), twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.)))


def load_collector():
    transport = NS(Node=lambda: NS(subscribe=lambda *args: None))
    message = NS(StringMsg=NS)
    modules = {'rclpy': NS(), 'rclpy.node': NS(Node=FakeNode),
               'rosgraph_msgs.msg': NS(Clock=NS), 'nav_msgs.msg': NS(Odometry=NS),
               'geometry_msgs.msg': NS(TwistStamped=twist),
               'gz': NS(transport13=transport, msgs=NS(stringmsg_pb2=message)),
               'gz.transport13': transport, 'gz.msgs': NS(stringmsg_pb2=message),
               'gz.msgs.stringmsg_pb2': message}
    spec = importlib.util.spec_from_file_location('_a3_trajectory_test', ROOT / 'experiments/gazebo_benchmark/core/trajectory.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class CorrectnessRuntimeTests(unittest.TestCase):
    def test_inherited_isolation_and_both_model_search_paths(self):
        original = {'ROS_DOMAIN_ID': '17', 'GZ_PARTITION': 'another_session',
                    'GZ_SIM_RESOURCE_PATH': '/extra/models', 'GZ_SIM_SYSTEM_PLUGIN_PATH': '/extra/lib'}
        with patch.dict(os.environ, original, clear=True):
            env = sim_env.build_benchmark_env('/workspace', turtlebot3=True)
            self.assertEqual(env['ROS_DOMAIN_ID'], '17')
            self.assertEqual(env['GZ_PARTITION'], 'another_session')
            self.assertEqual(env['GZ_SIM_RESOURCE_PATH'], '/workspace/install/custom_corridor/share/custom_corridor/models:/workspace/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models:/extra/models')
            self.assertEqual(env['GZ_SIM_SYSTEM_PLUGIN_PATH'], '/workspace/install/custom_corridor_plugins/lib:/extra/lib')
            inherited_renderer = sim_env.build_benchmark_env('/workspace', d3d12=False)
            self.assertNotIn('GALLIUM_DRIVER', inherited_renderer)
            self.assertEqual(dict(os.environ), original)
            isolated = sim_env.build_sim_env('/workspace', 77, 'trial')
            self.assertEqual((isolated['ROS_DOMAIN_ID'], isolated['GZ_PARTITION']), ('77', 'trial'))
            self.assertNotIn('turtlebot3', isolated['GZ_SIM_RESOURCE_PATH'])

    def test_spawn_preserves_coordinates_and_propagates_failures(self):
        with patch.object(sim_env.subprocess, 'run') as run:
            sim_env.spawn_benchmark_robot('robot with spaces.sdf', {'ROS_DOMAIN_ID': '77'}, x='0.0')
            run.assert_called_once_with(['ros2', 'run', 'ros_gz_sim', 'create', '-name', 'burger', '-file', 'robot with spaces.sdf', '-x', '0.0', '-y', '0.0', '-z', '0.01', '-Y', '0.0'], env={'ROS_DOMAIN_ID': '77'}, stdout=sim_env.subprocess.DEVNULL, stderr=sim_env.subprocess.DEVNULL, timeout=10)
            run.side_effect = OSError('missing executable')
            with self.assertRaises(OSError):
                sim_env.spawn_benchmark_robot('robot.sdf', {})

    def test_sim_time_boundaries_and_ground_truth_fallback(self):
        module = load_collector()
        node = module.PreciseTrajectoryRecorder()
        for nanos in [0, 19_999_999, 20_000_000, 99_999_999, 100_000_000]:
            node._clock_cb(NS(clock=NS(sec=0, nanosec=nanos)))
        self.assertEqual(len(node.published), 2)
        self.assertEqual(node.trajectory, [(0.1, 3., 0.)])
        self.assertEqual(node.publisher_spec, ('/cmd_vel', 50))
        self.assertEqual(node.subscriptions, [('/clock', 100)])
        self.assertEqual(node.published[0].twist.linear.x, .15)

    def test_synchronized_odometry_is_only_a_fallback(self):
        node = load_collector().SynchronizedTrajectoryCollector()
        odom = NS(pose=NS(pose=NS(position=NS(x=1., y=2.))))
        node._odom_cb(odom)
        self.assertEqual(node.latest_pos, (4., 2.))
        node._gt_cb(NS(data='{"robot":{"x":7.0,"y":8.0}}'))
        node._odom_cb(odom)
        node._gt_cb(NS(data='malformed'))
        self.assertEqual(node.latest_pos, (7., 8.))
        self.assertEqual(node.name, 'sync_traj_collector')
        self.assertEqual(node.subscriptions, [('/clock', 100), ('/odom', 100)])
