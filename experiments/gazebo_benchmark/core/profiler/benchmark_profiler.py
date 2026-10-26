"""
Real-Time System & Sensor Profiler for Gazebo Harmonic Benchmark.
Samples every 1.0 second:
- RTF: delta sim_time / delta wall_time (median, p5, mean, std)
- Actual topic frequencies: /scan (Hz), /camera/depth_image (Hz)
- CPU: total system CPU % and gz sim process CPU %
- GPU: Compute utilization % and VRAM usage (MB) via nvidia-smi
- RAM: System RAM usage (MB)
"""

import json
import sys
import os
import time
import numpy as np
import psutil

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import TwistStamped

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
from experiments.gazebo_benchmark.core.profiler import query_gpu_int


class DomainTelemetryListener(Node):
    """Wall/sim clock samples and sensor readiness for the scaling suites."""
    node_prefix = 'telemetry_listener_dom'
    clock_qos = 50
    depth_topic = '/camera/depth_image'
    use_sim_time = None
    depth_first = False
    track_latest = False

    def __init__(self, domain_id: int):
        super().__init__(f'{self.node_prefix}{domain_id}')
        if self.use_sim_time is not None:
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, self.use_sim_time)])
        self.domain_id = domain_id
        if self.track_latest:
            self.latest_sim_time = None
            self.latest_wall_time = None
        self.samples = []
        self.has_scan = False
        self.has_depth = False
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, self.clock_qos)
        sensors = [('scan', LaserScan, '/scan', self._scan_cb),
                   ('depth', Image, self.depth_topic, self._depth_cb)]
        if self.depth_first:
            sensors.reverse()
        for name, kind, topic, callback in sensors:
            setattr(self, name + '_sub', self.create_subscription(kind, topic, callback, 10))

    def _clock_cb(self, msg: Clock):
        w_t = time.time()
        s_t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.track_latest:
            self.latest_wall_time = w_t
            self.latest_sim_time = s_t
        self.samples.append((w_t, s_t))

    def _scan_cb(self, msg: LaserScan):
        self.has_scan = True

    def _depth_cb(self, msg: Image):
        self.has_depth = True


class DomainClockListener(DomainTelemetryListener):
    node_prefix = 'clock_listener_dom'
    track_latest = True


class TelemetryCollector(DomainTelemetryListener):
    node_prefix = 'telemetry_collector_'
    clock_qos = 100
    depth_topic = '/camera/depth/image_rect_raw'
    use_sim_time = True
    depth_first = True


def collect_telemetry(domain_id, duration_s, out_file, collector):
    """Run the duration-limited scaling worker and preserve its JSON schema."""
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    rclpy.init()
    node = collector(domain_id)
    t0 = time.time()
    while rclpy.ok() and (time.time() - t0) < duration_s:
        rclpy.spin_once(node, timeout_sec=0.01)

    data = {
        "samples": node.samples,
        "has_scan": node.has_scan,
        "has_depth": node.has_depth
    }
    with open(out_file, "w") as f:
        json.dump(data, f)

    node.destroy_node()
    rclpy.shutdown()


class BenchmarkProfiler(Node):
    def __init__(self, target_duration: float = 120.0, warmup_duration: float = 10.0):
        super().__init__('benchmark_profiler')
        self.target_duration = target_duration
        self.warmup_duration = warmup_duration

        # Subscriptions
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 50)
        self.depth_sub = self.create_subscription(Image, '/camera/depth_image', self._depth_cb, 50)

        # Publisher for fixed robot cmd_vel
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # State tracking
        self.latest_sim_time = 0.0
        self.scan_count = 0
        self.depth_count = 0
        self.start_wall_time = None
        self.is_running = True

        # Time series storage
        self.samples = []

    def _clock_cb(self, msg: Clock):
        self.latest_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _scan_cb(self, msg: LaserScan):
        self.scan_count += 1

    def _depth_cb(self, msg: Image):
        self.depth_count += 1

    def publish_cmd_vel(self, t_elapsed: float):
        """Publishes fixed robot velocity trajectory."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        # Smooth forward motion with gentle sinusoidal oscillation
        twist.twist.linear.x = 0.15
        twist.twist.angular.z = 0.05 * np.sin(0.1 * t_elapsed)
        self.cmd_pub.publish(twist)

    def run_profiling(self) -> dict:
        """
        Executes warm-up and 120s profiling loop.
        """
        self.get_logger().info(f"Waiting for Gazebo sim clock to start...")
        while rclpy.ok() and self.latest_sim_time <= 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Sim clock active. Starting {self.warmup_duration}s warm-up...")
        warmup_start = time.time()
        while rclpy.ok() and (time.time() - warmup_start) < self.warmup_duration:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.publish_cmd_vel(time.time() - warmup_start)

        self.get_logger().info(f"Warm-up complete. Starting {self.target_duration}s measurement window...")
        self.start_wall_time = time.time()

        prev_wall_time = self.start_wall_time
        prev_sim_time = self.latest_sim_time
        prev_scan_count = self.scan_count
        prev_depth_count = self.depth_count

        while rclpy.ok():
            loop_start = time.time()
            elapsed_total = loop_start - self.start_wall_time

            if elapsed_total >= self.target_duration:
                break

            # Spin to process ROS messages
            rclpy.spin_once(self, timeout_sec=0.02)
            self.publish_cmd_vel(elapsed_total)

            # Sample every 1.0s window
            current_wall_time = time.time()
            d_wall = current_wall_time - prev_wall_time

            if d_wall >= 1.0:
                current_sim_time = self.latest_sim_time
                d_sim = current_sim_time - prev_sim_time

                # RTF calculation
                rtf = d_sim / d_wall if d_wall > 0 else 0.0

                # Sensor rates (Wall clock and Sim clock)
                d_scan = self.scan_count - prev_scan_count
                d_depth = self.depth_count - prev_depth_count
                scan_hz_wall = d_scan / d_wall
                scan_hz_sim = (d_scan / d_sim) if d_sim > 0.001 else 0.0
                depth_hz_wall = d_depth / d_wall
                depth_hz_sim = (d_depth / d_sim) if d_sim > 0.001 else 0.0

                # Hardware metrics
                gpu_util, vram_mb = self._query_gpu()
                cpu_total, cpu_gz = self._query_cpu()
                ram_mb = psutil.virtual_memory().used / (1024 * 1024)

                sample = {
                    't_elapsed': elapsed_total,
                    'rtf': rtf,
                    'scan_hz': scan_hz_wall,
                    'scan_hz_wall': scan_hz_wall,
                    'scan_hz_sim': scan_hz_sim,
                    'depth_hz': depth_hz_wall,
                    'depth_hz_wall': depth_hz_wall,
                    'depth_hz_sim': depth_hz_sim,
                    'gpu_util': gpu_util,
                    'vram_mb': vram_mb,
                    'cpu_total': cpu_total,
                    'cpu_gz': cpu_gz,
                    'ram_mb': ram_mb,
                }
                self.samples.append(sample)

                self.get_logger().info(
                    f"[{elapsed_total:4.1f}s] RTF: {rtf:5.3f} | Scan: {scan_hz_wall:4.1f}Hz(wall)/{scan_hz_sim:4.1f}Hz(sim) | "
                    f"Depth: {depth_hz_wall:4.1f}Hz(wall)/{depth_hz_sim:4.1f}Hz(sim) | "
                    f"GPU: {gpu_util:2d}% ({vram_mb:4d}MB) | CPU: {cpu_total:4.1f}% (gz: {cpu_gz:4.1f}%)"
                )

                # Reset deltas
                prev_wall_time = current_wall_time
                prev_sim_time = current_sim_time
                prev_scan_count = self.scan_count
                prev_depth_count = self.depth_count

        return self._compute_summary()

    def _query_gpu(self) -> tuple[int, int]:
        return query_gpu_int()

    def _query_cpu(self) -> tuple[float, float]:
        cpu_total = psutil.cpu_percent(interval=None)
        gz_cpu = 0.0
        for proc in psutil.process_iter(['name', 'cpu_percent']):
            try:
                name = proc.info['name'] or ''
                if 'gz' in name or 'sim' in name or 'ruby' in name:
                    gz_cpu += proc.info['cpu_percent'] or 0.0
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return float(cpu_total), float(gz_cpu)

    def _compute_summary(self) -> dict:
        if not self.samples:
            return {}

        rtfs = [s['rtf'] for s in self.samples]
        scans_wall = [s['scan_hz_wall'] for s in self.samples]
        scans_sim = [s['scan_hz_sim'] for s in self.samples]
        depths_wall = [s['depth_hz_wall'] for s in self.samples]
        depths_sim = [s['depth_hz_sim'] for s in self.samples]
        gpus = [s['gpu_util'] for s in self.samples]
        vrams = [s['vram_mb'] for s in self.samples]
        cpu_t = [s['cpu_total'] for s in self.samples]
        cpu_gz = [s['cpu_gz'] for s in self.samples]
        rams = [s['ram_mb'] for s in self.samples]

        return {
            'rtf_median': float(np.median(rtfs)),
            'rtf_p5': float(np.percentile(rtfs, 5)),
            'rtf_mean': float(np.mean(rtfs)),
            'rtf_std': float(np.std(rtfs)),
            'scan_hz_mean': float(np.mean(scans_wall)),
            'scan_hz_wall_mean': float(np.mean(scans_wall)),
            'scan_hz_sim_mean': float(np.mean(scans_sim)),
            'depth_hz_mean': float(np.mean(depths_wall)),
            'depth_hz_wall_mean': float(np.mean(depths_wall)),
            'depth_hz_sim_mean': float(np.mean(depths_sim)),
            'gpu_util_mean': float(np.mean(gpus)),
            'gpu_util_peak': int(np.max(gpus)),
            'vram_mb_mean': float(np.mean(vrams)),
            'vram_mb_peak': int(np.max(vrams)),
            'cpu_total_mean': float(np.mean(cpu_t)),
            'cpu_gz_mean': float(np.mean(cpu_gz)),
            'ram_mb_mean': float(np.mean(rams)),
            'ram_mb_peak': float(np.max(rams)),
            'samples_count': len(self.samples),
        }
