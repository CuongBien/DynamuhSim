"""
Empirical Benchmark of Nav2 MPPI Controller Reproducibility across Independent Runs.
Directly tests user question:
- Does MPPI replicate trajectories with identical seed?
- Does 'regenerate_noises: false' freeze stochastic noise rollouts?
- Or does minimum-cost selection amplify mm differences into diverging actions?

Runs 3 trials of:
1. MPPI Deterministic (regenerate_noises: false)
2. MPPI Stochastic (regenerate_noises: true)
Measures full pairwise error matrices and reports empirical truth.
"""

import sys
import os
import time
import signal
import subprocess
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped, TwistStamped

import gz.transport13 as gzt
import gz.msgs.stringmsg_pb2 as stringmsg_pb2


class MPPITrajectoryRecorder(Node):
    def __init__(self, target_duration=15.0):
        super().__init__('mppi_recorder')
        self.target_duration = target_duration
        self.sim_time = 0.0
        self.t_start = None
        self.latest_pos = (0.0, 0.0)
        self.trajectory = []  # list of (t_rel, x, y)
        self.cmd_history = []
        self.last_sample_t = -1.0

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 50)

        self.gz_node = gzt.Node()
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, '/rvo2_human/ground_truth', self._gt_cb)

    def _gt_cb(self, msg):
        try:
            d = json.loads(msg.data)
            if 'robot' in d and d['robot']['x'] is not None:
                self.latest_pos = (float(d['robot']['x']), float(d['robot']['y']))
        except Exception:
            pass

    def _cmd_cb(self, msg: TwistStamped):
        self.cmd_history.append((self.sim_time, msg.twist.linear.x, msg.twist.angular.z))

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.t_start is None:
            self.t_start = self.sim_time
            self.last_sample_t = self.sim_time
            return

        t_rel = self.sim_time - self.t_start
        if self.sim_time - self.last_sample_t >= 0.1:
            self.last_sample_t = self.sim_time
            self.trajectory.append((t_rel, self.latest_pos[0], self.latest_pos[1]))

    def run_until_duration(self):
        t_wall_0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.t_start is not None and (self.sim_time - self.t_start) >= self.target_duration:
                break
            if time.time() - t_wall_0 > (self.target_duration * 3.0 + 15.0):
                break
        return self.trajectory, self.cmd_history


def run_single_mppi_run(trial_idx: int, mode_name: str, params_file: str, root_dir: str, duration_s=15.0):
    print(f"\n  [{mode_name.upper()} - RUN {trial_idx + 1}/3] Cold starting Gazebo & Nav2 MPPI...", flush=True)

    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/generated_configs/world_n5_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "experiments/gazebo_benchmark/generated_configs/burger_lidar1_depth1_640x480_10fps.sdf")
    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")
    urdf_file = "/tmp/tb3_burger_processed.urdf"

    domain_id = str(50 + trial_idx)
    partition = f"mppi_eval_{mode_name}_{trial_idx}"

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = domain_id
    env["GZ_PARTITION"] = partition
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"

    corridor_models = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models")
    tb3_models = os.path.join(root_dir, "install/turtlebot3_gazebo/share/turtlebot3_gazebo/models")
    plugin_share = os.path.join(root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{tb3_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    all_procs = []

    try:
        # 1. Start Gazebo Sim
        gz_p = subprocess.Popen(["gz", "sim", "-r", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(gz_p)
        time.sleep(3.5)

        # 2. Spawn robot Burger at (0.0, 0.0)
        spawn_cmd = ["ros2", "run", "ros_gz_sim", "create", "-name", "burger", "-file", robot_file, "-x", "0.0", "-y", "0.0", "-z", "0.01", "-Y", "0.0"]
        subprocess.run(spawn_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

        # 3. Bridges & Robot State Publisher
        bp = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p", f"config_file:={bridge_config}"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(bp)

        rsp = subprocess.Popen(["ros2", "run", "robot_state_publisher", "robot_state_publisher", urdf_file, "--ros-args", "-p", "use_sim_time:=true"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(rsp)
        time.sleep(2.0)

        # 4. Nav2 stack with MPPI
        nav2_log_path = f"/tmp/nav2_mppi_{mode_name}_{trial_idx}.log"
        nav2_log = open(nav2_log_path, "w")
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={params_file}",
            "map:=corridor_090"
        ]
        nav2_p = subprocess.Popen(nav2_cmd, env=env, stdout=nav2_log, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
        all_procs.append(nav2_p)

        # 5. Wait for lifecycle_manager_navigation
        t0 = time.time()
        ready = False
        while time.time() - t0 < 30.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path) as fp:
                    content = fp.read()
                    if "lifecycle_manager_navigation" in content and "Managed nodes are active" in content:
                        ready = True
                        break
            time.sleep(0.5)

        if not ready:
            print("    [WARN] Nav2 navigation lifecycle manager did not report active in 30s!", flush=True)

        time.sleep(1.5)

        # 6. Start recorder node
        os.environ["ROS_DOMAIN_ID"] = domain_id
        os.environ["GZ_PARTITION"] = partition
        rclpy.init()
        recorder = MPPITrajectoryRecorder(target_duration=duration_s)

        # 7. Send goal pose to /goal_pose
        goal_cmd = [
            "ros2", "topic", "pub", "--once", "/goal_pose", "geometry_msgs/msg/PoseStamped",
            "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
        ]
        subprocess.run(goal_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)

        traj, cmds = recorder.run_until_duration()
        recorder.destroy_node()
        rclpy.shutdown()

    finally:
        for p in all_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1.0)

    print(f"    Finished: {len(traj)} trajectory points recorded, {len(cmds)} cmd_vel commands issued. Final pose: ({traj[-1][1]:.3f}, {traj[-1][2]:.3f})", flush=True)
    return traj, cmds


def evaluate_mode(mode_name: str, params_file: str, root_dir: str):
    print("\n" + "=" * 75)
    print(f"EVALUATING MPPI: {mode_name.upper()} (3 RUNS, 15s DURATION, SEED=42)")
    print("=" * 75, flush=True)

    trajectories = []
    for r in range(3):
        traj, _ = run_single_mppi_run(r, mode_name, params_file, root_dir, duration_s=15.0)
        trajectories.append(traj)

    min_len = min(len(t) for t in trajectories)
    pairwise = {}
    max_errs = []
    mean_errs = []

    for i in range(len(trajectories)):
        for j in range(i + 1, len(trajectories)):
            t_i = trajectories[i][:min_len]
            t_j = trajectories[j][:min_len]
            errs = [np.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(t_i, t_j)]
            max_e = float(np.max(errs)) * 1000.0  # mm
            mean_e = float(np.mean(errs)) * 1000.0  # mm
            final_e = float(errs[-1]) * 1000.0  # mm
            pair_key = f"Run_{i+1}_vs_Run_{j+1}"
            pairwise[pair_key] = {
                "max_mm": max_e,
                "mean_mm": mean_e,
                "final_mm": final_e
            }
            max_errs.append(max_e)
            mean_errs.append(mean_e)
            print(f"  {pair_key:15s} | Max: {max_e:7.2f} mm ({max_e/10:5.2f} cm) | Mean: {mean_e:7.2f} mm | Final: {final_e:7.2f} mm", flush=True)

    overall_max = float(max(max_errs)) if max_errs else 0.0
    overall_mean = float(np.mean(mean_errs)) if mean_errs else 0.0

    print(f"\n[{mode_name.upper()} OVERALL]")
    print(f"  Max Discrepancy : {overall_max:.2f} mm ({overall_max/10:.2f} cm)")
    print(f"  Mean Discrepancy: {overall_mean:.2f} mm")

    return {
        "mode": mode_name,
        "pairwise": pairwise,
        "overall_max_mm": overall_max,
        "overall_mean_mm": overall_mean,
        "trajectories": trajectories
    }


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    det_params = os.path.join(root_dir, "experiments/gazebo_benchmark/mppi_params_deterministic.yaml")
    stoch_params = os.path.join(root_dir, "experiments/gazebo_benchmark/mppi_params_stochastic.yaml")

    results = {}

    # 1. Test MPPI with regenerate_noises: false
    res_det = evaluate_mode("frozen_noise_deterministic", det_params, root_dir)
    results["mppi_frozen_noise"] = res_det

    # 2. Test MPPI with regenerate_noises: true (default)
    res_stoch = evaluate_mode("dynamic_noise_stochastic", stoch_params, root_dir)
    results["mppi_dynamic_noise"] = res_stoch

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/mppi_reproducibility_results.json")
    # Save summary without massive raw trajectory dumps
    summary = {
        "mppi_frozen_noise": {
            "mode": res_det["mode"],
            "pairwise": res_det["pairwise"],
            "overall_max_mm": res_det["overall_max_mm"],
            "overall_mean_mm": res_det["overall_mean_mm"]
        },
        "mppi_dynamic_noise": {
            "mode": res_stoch["mode"],
            "pairwise": res_stoch["pairwise"],
            "overall_max_mm": res_stoch["overall_max_mm"],
            "overall_mean_mm": res_stoch["overall_mean_mm"]
        }
    }
    with open(out_file, "w") as f:
        json.dump(summary, f, indent=2)

    print("\n" + "=" * 75)
    print("ALL MPPI BENCHMARKS COMPLETED!")
    print(f"Results saved to: {out_file}")
    print("=" * 75, flush=True)


if __name__ == '__main__':
    main()
