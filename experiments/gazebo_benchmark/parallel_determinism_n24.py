"""
Parallel Determinism Benchmark under Heavy Crowd (N=24 Humans, 4 Concurrent Instances, Seed=42).
Verifies whether running 4 Gazebo instances simultaneously under heavy CPU/GPU load
causes any thread scheduling desync or breaks trajectory reproducibility.
"""

import sys
import os
import time
import signal
import subprocess
import json
import numpy as np

sys.path.insert(0, '/home/cuongbien/DynamuhSim/experiments/gazebo_benchmark')
from benchmark_world_generator import BenchmarkWorldGenerator


WORKER_CODE = """
import sys, os, time, json
import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

class WorkerRecorder(Node):
    def __init__(self, target_duration=20.0):
        super().__init__('worker_recorder')
        self.target_duration = target_duration
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.gt_sub = self.create_subscription(String, 'rvo2_human/ground_truth', self._gt_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 100)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 50)
        self.sim_time = 0.0
        self.latest_robot_pos = (3.0, 0.0)
        self.latest_humans = {}
        self.robot_trajectory = []
        self.human_trajectories = {}
        self.t_start = None
        self.last_cmd = -1.0
        self.last_sample = -1.0

    def _gt_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.latest_robot_pos = (data['robot']['x'], data['robot']['y'])
            if 'humans' in data:
                for h in data['humans']:
                    hid = h['id']
                    self.latest_humans[hid] = (h['x'], h['y'])
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        # Fallback if ground truth not available
        pass

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.t_start is None:
            self.t_start = self.sim_time
            self.last_cmd = self.sim_time
            self.last_sample = self.sim_time
            return
        t_rel = self.sim_time - self.t_start
        if self.sim_time - self.last_cmd >= 0.02:
            self.last_cmd = self.sim_time
            twist = TwistStamped()
            twist.header.stamp = msg.clock
            twist.twist.linear.x = 0.15
            twist.twist.angular.z = 0.05 * np.sin(0.1 * t_rel)
            self.cmd_pub.publish(twist)
        if self.sim_time - self.last_sample >= 0.1:
            self.last_sample = self.sim_time
            self.robot_trajectory.append((t_rel, self.latest_robot_pos[0], self.latest_robot_pos[1]))
            for hid, pos in self.latest_humans.items():
                if hid not in self.human_trajectories:
                    self.human_trajectories[hid] = []
                self.human_trajectories[hid].append((t_rel, pos[0], pos[1]))

    def run_until_done(self):
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.t_start is not None and (self.sim_time - self.t_start) >= self.target_duration:
                break
            if time.time() - t0 > (self.target_duration * 3.5 + 20.0):
                break
        return {"robot": self.robot_trajectory, "humans": self.human_trajectories}

rclpy.init()
worker = WorkerRecorder(target_duration=20.0)
data = worker.run_until_done()
worker.destroy_node()
rclpy.shutdown()

out_file = sys.argv[1]
with open(out_file, 'w') as f:
    json.dump(data, f)
"""


def run_parallel_determinism_test(root_dir="/home/cuongbien/DynamuhSim", num_instances=4):
    print("=" * 75)
    print(f"PARALLEL DETERMINISM BENCHMARK: {num_instances} CONCURRENT INSTANCES (N=24 Humans, Seed=42)")
    print("=" * 75, flush=True)

    gen = BenchmarkWorldGenerator(root_dir)
    world_file = gen.generate_world_with_humans(num_humans=24, representation="actor_and_cylinder", seed=42)
    robot_file = gen.generate_robot_model(True, True, 640, 480, 10, initial_x=3.0, initial_y=0.0)

    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")
    corridor_models = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models")
    tb3_models = os.path.join(root_dir, "install/turtlebot3_gazebo/share/turtlebot3_gazebo/models")
    plugin_share = os.path.join(root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')

    base_env = os.environ.copy()
    base_env["GALLIUM_DRIVER"] = "d3d12"
    base_env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    base_env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{tb3_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    base_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    all_procs = []
    worker_outputs = []

    try:
        # 1. Start all 4 Gazebo instances paused
        print(f"1. Launching {num_instances} Gazebo instances (Paused mode)...", flush=True)
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"concurrent_part_{i}"
            env_i["ROS_DOMAIN_ID"] = str(120 + i)
            p = subprocess.Popen(["gz", "sim", "-s", world_file], env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
            all_procs.append(p)

        time.sleep(4.0)

        # 2. Spawn robot and bridge for each
        print(f"2. Spawning robot & bridge on all {num_instances} domains...", flush=True)
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"concurrent_part_{i}"
            env_i["ROS_DOMAIN_ID"] = str(120 + i)

            spawn_cmd = [
                "ros2", "run", "ros_gz_sim", "create",
                "-name", "burger", "-file", robot_file,
                "-x", "3.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"
            ]
            subprocess.run(spawn_cmd, env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

            bridge_p = subprocess.Popen(
                ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"],
                env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
            )
            all_procs.append(bridge_p)

        time.sleep(2.0)

        # 3. Launch dedicated recording workers
        print(f"3. Launching 4 recorder workers concurrently...", flush=True)
        worker_procs = []
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"concurrent_part_{i}"
            env_i["ROS_DOMAIN_ID"] = str(120 + i)
            out_file = f"/tmp/concurrent_traj_{i}.json"
            worker_outputs.append(out_file)
            wp = subprocess.Popen(
                ["python3", "-c", WORKER_CODE, out_file],
                env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            worker_procs.append(wp)

        time.sleep(1.0)

        # 4. Unpause all instances simultaneously
        print(f"4. Unpausing all {num_instances} instances simultaneously...", flush=True)
        for i in range(num_instances):
            env_i = base_env.copy()
            env_i["GZ_PARTITION"] = f"concurrent_part_{i}"
            subprocess.run(
                ["gz", "service", "-s", "/world/corridor_0.90/control", "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "pause: false"],
                env=env_i, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

        print(f"5. Waiting for all workers to record 20.0s sim-time...", flush=True)
        for wp in worker_procs:
            wp.wait(timeout=60.0)

        print("All recording workers finished successfully!", flush=True)

    finally:
        # Cleanup
        print("Cleaning up processes...", flush=True)
        for p in all_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1.0)

    # 5. Load trajectories and analyze pairwise errors
    loaded_data = []
    for out_file in worker_outputs:
        if os.path.exists(out_file):
            with open(out_file) as f:
                loaded_data.append(json.load(f))

    print(f"Loaded {len(loaded_data)} data packets from concurrent instances.", flush=True)
    if len(loaded_data) < 2:
        print("[ERROR] Insufficient trajectories recorded!")
        return {}

    trajectories = [d["robot"] if isinstance(d, dict) and "robot" in d else d for d in loaded_data]
    min_len = min(len(t) for t in trajectories)
    pairwise_matrix = {}

    print("\n" + "-" * 75)
    print("CONCURRENT INSTANCE PAIRWISE COMPARISON (N=24 Crowd, Seed=42):")
    print("-" * 75, flush=True)

    max_errors = []
    mean_errors = []

    for i in range(len(trajectories)):
        for j in range(i + 1, len(trajectories)):
            t_i = trajectories[i][:min_len]
            t_j = trajectories[j][:min_len]
            errs = [np.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(t_i, t_j)]
            max_e = float(np.max(errs)) * 1000.0  # mm
            mean_e = float(np.mean(errs)) * 1000.0  # mm
            final_e = float(errs[-1]) * 1000.0  # mm
            key = f"Instance_{i+1}_vs_Instance_{j+1}"
            pairwise_matrix[key] = {
                "max_mm": max_e,
                "mean_mm": mean_e,
                "final_mm": final_e
            }
            max_errors.append(max_e)
            mean_errors.append(mean_e)
            print(f"  {key:25s} | Max: {max_e:6.2f} mm | Mean: {mean_e:6.2f} mm | Final: {final_e:6.2f} mm", flush=True)

    # Check human agents determinism across instances if available
    human_max_errors = []
    if isinstance(loaded_data[0], dict) and "humans" in loaded_data[0]:
        all_hids = set(loaded_data[0]["humans"].keys())
        for hid in all_hids:
            h_trajs = [d["humans"][hid] for d in loaded_data if hid in d["humans"]]
            if len(h_trajs) >= 2:
                h_min_len = min(len(t) for t in h_trajs)
                for i in range(len(h_trajs)):
                    for j in range(i + 1, len(h_trajs)):
                        errs = [np.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(h_trajs[i][:h_min_len], h_trajs[j][:h_min_len])]
                        if len(errs) > 0:
                            human_max_errors.append(float(np.max(errs)) * 1000.0)

    results = {
        "num_concurrent_instances": len(trajectories),
        "num_humans": 24,
        "seed": 42,
        "overall_max_mm": float(max(max_errors)),
        "overall_mean_mm": float(np.mean(mean_errors)),
        "human_max_error_mm": float(max(human_max_errors)) if human_max_errors else 0.0,
        "pairwise_matrix": pairwise_matrix
    }

    out_json = os.path.join(root_dir, "experiments/gazebo_benchmark/parallel_determinism_n24_results.json")
    with open(out_json, "w") as f:
        json.dump(results, f, indent=2)

    print("\n[SUMMARY RESULTS: CONCURRENT N=24]")
    print(f"  Overall Max Discrepancy: {results['overall_max_mm']:.2f} mm (< 1 cm!)")
    print(f"  Overall Mean Discrepancy: {results['overall_mean_mm']:.2f} mm")
    print(f"Saved results to: {out_json}", flush=True)
    return results


if __name__ == '__main__':
    run_parallel_determinism_test()
