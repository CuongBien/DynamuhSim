"""Gazebo Harmonic/ROS2 Jazzy one-worker adapter for corridor_090 proof of loop.

Run INSIDE a sourced ROS2 Jazzy + built DynamuhSim workspace. This code cannot
be exercised in a plain Python environment without ROS/Gazebo services.
"""
from __future__ import annotations
import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import uuid
import numpy as np
from core import Backend, PHYSICS_DT, Scenario, SensorSnapshot, SimulatorFault

WORLD = 'corridor_0.90'
CONTROL = f'/world/{WORLD}/control'


class GazeboROSBackend:
    def __init__(self, *, ros_domain_id=61, startup_timeout=90., step_timeout=20.):
        self.domain_id = int(ros_domain_id)
        self.startup_timeout = startup_timeout
        self.step_timeout = step_timeout
        self.epoch = None
        self.context = None
        self.node = None
        self.executor = None
        self.thread = None
        self.process = None
        self.log_handle = None
        self.gz_env = None
        self.clock = None
        self.samples = {}
        self.lock = threading.Lock()

    def _ros_callbacks(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, Imu
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from geometry_msgs.msg import TwistStamped

        self.context = Context()
        rclpy.init(context=self.context, domain_id=self.domain_id)
        self.node = rclpy.create_node('dynamuh_gym_adapter', context=self.context)
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.publisher = self.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.TwistStamped = TwistStamped

        def stamp(msg):
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        def on_clock(msg):
            with self.lock:
                self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

        def on_scan(msg):
            d = (stamp(msg), np.asarray(msg.ranges, dtype=np.float32).copy(),
                 float(msg.range_min), float(msg.range_max))
            with self.lock:
                self.samples['scan'] = d

        def on_odom(msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            p, twist = msg.pose.pose.position, msg.twist.twist
            d = np.array([p.x, p.y, yaw,
                          twist.linear.x, twist.linear.y, twist.angular.z], dtype=np.float64)
            with self.lock:
                self.samples['odom'] = (stamp(msg), d)

        def on_imu(msg):
            with self.lock:
                self.samples['imu'] = stamp(msg)

        self.node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)
        self.node.create_subscription(Imu, '/imu', on_imu, qos_profile_sensor_data)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _gz(self, args, timeout):
        try:
            return subprocess.run(['gz', 'service', *args], env=self.gz_env,
                                  capture_output=True, text=True, timeout=timeout)
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            raise SimulatorFault(f'Gazebo transport unavailable/timeout: {exc}') from exc

    def _ensure_alive(self):
        if self.process is None or self.process.poll() is not None:
            raise SimulatorFault('Owned ros2 launch exited; inspect the adapter launch log')

    def _wait_service(self):
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            self._ensure_alive()
            r = self._gz(['-l'], timeout=3)
            if r.returncode == 0 and CONTROL in r.stdout.splitlines():
                return
            time.sleep(.15)  # service discovery polling only, NOT a physics step
        raise SimulatorFault(f'Gazebo service {CONTROL} not found')

    def _world_steps(self, steps):
        if not isinstance(steps, int) or steps <= 0:
            raise ValueError('steps must be a positive integer')
        with self.lock:
            start_clock = self.clock
        timeout_ms = 2500
        for _ in range(3):
            self._ensure_alive()
            r = self._gz(['-s', CONTROL, '--reqtype', 'gz.msgs.WorldControl',
                          '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms),
                          '--req', f'pause: true multi_step: {steps}'],
                         timeout=timeout_ms / 1000. + 2.)
            if r.returncode == 0 and 'data: true' in r.stdout.lower():
                return
            time.sleep(.05)
            with self.lock:
                now_clock = self.clock
            if (start_clock is not None and now_clock is not None
                    and now_clock >= start_clock + steps * PHYSICS_DT - .002):
                return
        raise SimulatorFault(f'WorldControl step rejected: {r.stdout[-300:]} {r.stderr[-300:]}')

    def _snapshot(self, *, min_time, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._ensure_alive()
            with self.lock:
                t = self.clock
                samples = copy.deepcopy(self.samples)
            if (t is not None and t + 1e-7 >= min_time
                    and all(key in samples for key in ('scan','odom','imu'))):
                st, scan, rmin, rmax = samples['scan']
                ot, odom = samples['odom']
                snapshot = SensorSnapshot(
                    epoch=self.epoch, sim_time=float(t), scan_time=float(st),
                    scan=scan, range_min=rmin, range_max=rmax,
                    odom_time=float(ot), odom=odom, imu_time=float(samples['imu']))
                # Require fresh samples for the CURRENT time target, not old cached data.
                from core import require_fresh
                try:
                    require_fresh(snapshot, self.epoch)
                except SimulatorFault:
                    pass
                else:
                    return snapshot
            time.sleep(.005)  # ROS callback delivery, never used as time advance
        raise SimulatorFault('Timed out awaiting fresh /clock, /scan, /odom and /imu')

    def _publish(self, action):
        self._ensure_alive()
        if self.publisher.get_subscription_count() < 1:
            raise SimulatorFault('No /cmd_vel subscriber: bridge not ready')
        msg = self.TwistStamped()
        msg.header.frame_id = 'base_footprint'
        now = self.node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.twist.linear.x = float(action[0])
        msg.twist.linear.y = float(action[1])
        msg.twist.angular.z = float(action[2])
        self.publisher.publish(msg)
        # DDS transport is asynchronous; this delay permits command delivery
        # BEFORE the world is stepped; the sim remains paused meanwhile.
        time.sleep(.04)

    def reset(self, scenario: Scenario, epoch: int) -> SensorSnapshot:
        self.close()
        self.epoch = epoch
        with self.lock:
            self.clock = None
            self.samples.clear()
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(self.domain_id)
        env['GZ_IP'] = '127.0.0.1'
        env['GZ_PARTITION'] = 'dynamuh_gym_' + uuid.uuid4().hex[:12]
        # This variable is for provenance only. Existing RVO2 plugin seed
        # semantics must be audited before claiming deterministic reset.
        env['DYNAMUH_GYM_SEED'] = str(scenario.seed)
        self.gz_env = env
        self._ros_callbacks()
        self.log_handle = tempfile.NamedTemporaryFile(prefix='dynamuh_gym_', suffix='.log', delete=False)
        self.last_log = self.log_handle.name
        cmd = ['ros2', 'launch', 'custom_corridor', 'corridor_tb3.launch.py',
               'width:=0.90', 'robot_model:=mecanum',
               'obstacle:=' + ('none' if scenario.kind == 'empty' else 'human'),
               'paused:=true', 'gui:=false', 'rviz:=false',
               f'x:={scenario.start[0]}', f'y:={scenario.start[1]}',
               f'yaw:={scenario.start[2]}']
        try:
            self.process = subprocess.Popen(cmd, env=env, start_new_session=True,
                                            stdout=self.log_handle, stderr=subprocess.STDOUT)
            self._wait_service()
            # Still paused after startup; bootstrap 0.2 s at a time until
            # Gazebo's create/bridge and sensors are all ready.
            target = 0.0
            for _ in range(20):
                self._world_steps(50)
                target += 50 * PHYSICS_DT
                try:
                    return self._snapshot(min_time=target - .002, timeout=1.5)
                except SimulatorFault:
                    continue
            raise SimulatorFault('Simulation bootstrap failed; check ' + self.last_log)
        except Exception:
            self.close()
            raise

    def advance(self, velocity: np.ndarray, steps: int, epoch: int) -> SensorSnapshot:
        if epoch != self.epoch:
            raise SimulatorFault('advance() called with wrong episode epoch')
        with self.lock:
            old = self.clock
        if old is None:
            raise SimulatorFault('advance before valid /clock')
        self._publish(velocity)
        self._world_steps(steps)
        target = old + steps * PHYSICS_DT
        result = self._snapshot(min_time=target - .002, timeout=self.step_timeout)
        actual = result.sim_time - old
        if abs(actual - steps*PHYSICS_DT) > .00801:
            raise SimulatorFault(f'Gazebo advanced {actual:.4f}s; expected {steps*PHYSICS_DT:.4f}s')
        return result

    def close(self):
        if self.node is not None and self.process is not None and self.process.poll() is None:
            try:
                self._publish(np.zeros(3, dtype=np.float32))
            except Exception:
                pass
        if self.process is not None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try: os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                try: self.process.wait(timeout=3)
                except subprocess.TimeoutExpired: pass
            self.process = None
        if self.executor is not None:
            try: self.executor.shutdown(timeout_sec=2)
            except Exception: pass
        if self.thread is not None:
            self.thread.join(timeout=2)
        if self.node is not None:
            try: self.node.destroy_node()
            except Exception: pass
        if self.context is not None:
            try:
                import rclpy
                if self.context.ok(): rclpy.shutdown(context=self.context)
            except Exception: pass
        if self.log_handle is not None:
            self.log_handle.close()
        self.node = self.executor = self.context = self.thread = None
        self.log_handle = None
        self.publisher = None
