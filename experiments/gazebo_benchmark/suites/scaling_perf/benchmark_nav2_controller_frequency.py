#!/usr/bin/env python3
"""
Comprehensive Nav2 MPPI Controller Frequency Benchmark & Diagnostic Suite.

Subprocess-isolated architecture for 100% reliable ROS 2 / DDS execution.
"""

import os
import sys
import time
import subprocess
import json
import re
import argparse
import yaml
import psutil
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes

ROOT_DIR = "/home/cuongbien/DynamuhSim"
OUTPUT_JSON = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/scaling_perf/nav2_controller_frequency_benchmark_results.json")
CONFIGS_DIR = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/configs/nav2_mppi")
os.makedirs(CONFIGS_DIR, exist_ok=True)


def create_modified_params_yaml(base_yaml_path: str, out_yaml_path: str, modifications: dict) -> str:
    with open(base_yaml_path) as f:
        cfg = yaml.safe_load(f)

    if "controller_frequency" in modifications:
        cfg["controller_server"]["ros__parameters"]["controller_frequency"] = float(modifications["controller_frequency"])

    follow_path = cfg["controller_server"]["ros__parameters"]["FollowPath"]
    if "batch_size" in modifications:
        follow_path["batch_size"] = int(modifications["batch_size"])
    if "time_steps" in modifications:
        follow_path["time_steps"] = int(modifications["time_steps"])
    if "visualize" in modifications:
        follow_path["visualize"] = bool(modifications["visualize"])
    if "critics" in modifications:
        follow_path["critics"] = list(modifications["critics"])

    if "consider_footprint" in modifications:
        if "CostCritic" in follow_path:
            follow_path["CostCritic"]["consider_footprint"] = bool(modifications["consider_footprint"])

    if "robot_radius" in modifications:
        radius = float(modifications["robot_radius"])
        local_p = cfg["local_costmap"]["local_costmap"]["ros__parameters"]
        if "footprint" in local_p:
            del local_p["footprint"]
        local_p["robot_radius"] = radius

        global_p = cfg["global_costmap"]["global_costmap"]["ros__parameters"]
        if "footprint" in global_p:
            del global_p["footprint"]
        global_p["robot_radius"] = radius

    if "local_costmap_width" in modifications:
        w = int(modifications["local_costmap_width"])
        cfg["local_costmap"]["local_costmap"]["ros__parameters"]["width"] = w
        cfg["local_costmap"]["local_costmap"]["ros__parameters"]["height"] = w

    if "local_costmap_resolution" in modifications:
        res = float(modifications["local_costmap_resolution"])
        cfg["local_costmap"]["local_costmap"]["ros__parameters"]["resolution"] = res

    with open(out_yaml_path, "w") as f:
        yaml.dump(cfg, f)

    return out_yaml_path


@scoped_processes
def run_instance_trial_in_worker(
    trial_name: str,
    domain_id: int,
    rtf: float,
    params_yaml: str,
    duration_s: float = 12.0,
    corridor_type: str = "090",
    *, _processes=None
) -> dict:
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import TwistStamped, PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    class CmdVelRecorderNode(Node):
        def __init__(self, node_name: str = 'cmdvel_rec'):
            super().__init__(node_name)
            self.cmd_msgs = []
            self.sim_time = 0.0
            self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
            self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_cb, 100)
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        def _clock_cb(self, msg: Clock):
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        def _cmd_cb(self, msg: TwistStamped):
            w_t = time.time()
            s_t = self.sim_time
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            w = msg.twist.angular.z
            self.cmd_msgs.append((w_t, s_t, vx, vy, w))

        def send_nav_goal(self, gx: float = -7.0, gy: float = 0.0) -> bool:
            if not self.nav_client.wait_for_server(timeout_sec=15.0):
                return False
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(gx)
            goal.pose.pose.position.y = float(gy)
            goal.pose.pose.orientation.w = 1.0

            for attempt in range(5):
                future = self.nav_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.done():
                    handle = future.result()
                    if handle.accepted:
                        return True
                time.sleep(0.5)
            return False

    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = f"gz_trial_{trial_name}_{domain_id}"
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"

    world_src = os.path.join(ROOT_DIR, f"src/custom_corridor/worlds/corridor_{corridor_type}.sdf")
    with open(world_src) as f:
        w_content = f.read()
    w_content = re.sub(r"<real_time_factor>.*?</real_time_factor>", f"<real_time_factor>{rtf}</real_time_factor>", w_content)
    test_world = f"/tmp/world_{trial_name}_{domain_id}.sdf"
    with open(test_world, "w") as f:
        f.write(w_content)

    nav2_log_path = f"/tmp/nav2_{trial_name}_{domain_id}.log"
    log_fp = open(nav2_log_path, "w")

    try:
        # Launch Sim
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={'0.90' if corridor_type == '090' else '1.20'}",
            f"custom_world:={test_world}",
            "obstacle:=none", "gui:=false", "rviz:=false",
            "robot_model:=mecanum", "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        p_sim = _processes.popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(3.8)

        # Launch Nav2
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi", f"params_file:={params_yaml}",
            f"map:=corridor_{corridor_type}", "robot_model:=mecanum"
        ]
        p_nav = _processes.popen(nav2_cmd, env=env, stdout=log_fp, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)

        t0 = time.time()
        active = False
        while time.time() - t0 < 35.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path) as fp:
                    if fp.read().count("Managed nodes are active") >= 2:
                        active = True
                        break
            time.sleep(0.4)

        if not active:
            return {"error": "Nav2 failed to activate within 35s"}

        ctrl_pid = None
        for p in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmd = " ".join(p.info['cmdline'] or [])
                if "controller_server" in cmd:
                    p_env = p.environ()
                    if p_env.get("ROS_DOMAIN_ID") == str(domain_id):
                        ctrl_pid = p.info['pid']
                        break
            except: pass

        rclpy.init()
        rec_node = CmdVelRecorderNode(f"rec_{domain_id}")
        rec_node.send_nav_goal(-7.0, 0.0)

        thread_cpu_samples = []
        last_ps_time = 0.0
        t_start = time.time()
        while time.time() - t_start < duration_s:
            rclpy.spin_once(rec_node, timeout_sec=0.04)
            now = time.time()
            if ctrl_pid and (now - t_start) > 2.0 and (now - last_ps_time) >= 0.5:
                last_ps_time = now
                try:
                    out = subprocess.check_output(["ps", "-T", "-p", str(ctrl_pid), "-o", "tid,%cpu,comm"], text=True)
                    lines = out.strip().splitlines()[1:]
                    for l in lines:
                        parts = l.split()
                        if len(parts) >= 3:
                            thread_cpu_samples.append({
                                "tid": parts[0],
                                "cpu_pct": float(parts[1]),
                                "comm": parts[2]
                            })
                except: pass

        cmd_msgs = list(rec_node.cmd_msgs)
        rec_node.destroy_node()
        rclpy.shutdown()

    finally:
        _processes.stop()
        time.sleep(1.0)
        log_fp.close()

    loop_warnings = []
    loop_rates = []
    if os.path.exists(nav2_log_path):
        with open(nav2_log_path) as fp:
            for line in fp:
                if "Control loop missed its desired rate" in line:
                    loop_warnings.append(line.strip())
                    m = re.search(r"Current loop rate is ([\d\.]+) Hz", line)
                    if m:
                        loop_rates.append(float(m.group(1)))

    wall_hz = 0.0
    sim_hz = 0.0
    measured_rtf = 0.0
    dt_sim_list = []
    dt_wall_list = []

    if len(cmd_msgs) >= 2:
        wall_dur = cmd_msgs[-1][0] - cmd_msgs[0][0]
        sim_dur = cmd_msgs[-1][1] - cmd_msgs[0][1]
        wall_hz = round((len(cmd_msgs) - 1) / wall_dur, 2) if wall_dur > 0 else 0.0
        sim_hz = round((len(cmd_msgs) - 1) / sim_dur, 2) if sim_dur > 0 else 0.0
        measured_rtf = round(sim_dur / wall_dur, 2) if wall_dur > 0 else 0.0

        for i in range(1, len(cmd_msgs)):
            dt_sim_list.append(cmd_msgs[i][1] - cmd_msgs[i-1][1])
            dt_wall_list.append(cmd_msgs[i][0] - cmd_msgs[i-1][0])

    max_thread_cpu = 0.0
    if thread_cpu_samples:
        max_thread_cpu = max(s["cpu_pct"] for s in thread_cpu_samples)

    return {
        "trial_name": trial_name,
        "domain_id": domain_id,
        "rtf_setting": rtf,
        "measured_rtf": measured_rtf,
        "cmd_vel_count": len(cmd_msgs),
        "wall_hz": wall_hz,
        "sim_hz": sim_hz,
        "mean_sim_period_s": round(float(np.mean(dt_sim_list)), 4) if dt_sim_list else None,
        "mean_wall_period_s": round(float(np.mean(dt_wall_list)), 4) if dt_wall_list else None,
        "max_controller_thread_cpu_pct": max_thread_cpu,
        "log_warnings_count": len(loop_warnings),
        "mean_logged_loop_rate_hz": round(float(np.mean(loop_rates)), 2) if loop_rates else None,
        "sample_warnings": loop_warnings[:2]
    }


@scoped_processes
def run_single_trial_subprocess(trial_name: str, domain_id: int, rtf: float, params_yaml: str, duration_s: float = 12.0, *, _processes=None) -> dict:
    out_json = f"/tmp/res_{trial_name}_{domain_id}.json"
    if os.path.exists(out_json):
        os.remove(out_json)
    cmd = [
        sys.executable, __file__,
        "--run_worker",
        "--trial_name", str(trial_name),
        "--domain_id", str(domain_id),
        "--rtf", str(rtf),
        "--params_yaml", str(params_yaml),
        "--duration_s", str(duration_s),
        "--out_json", str(out_json)
    ]
    _processes.run(cmd, check=True)
    with open(out_json) as f:
        return json.load(f)


@scoped_processes
def run_parallel_4instances_subprocesses(trial_name: str, rtf: float, params_yaml: str, duration_s: float = 12.0, *, _processes=None) -> dict:
    print(f"\n>>> Running 4 Parallel Subprocesses: {trial_name} (RTF={rtf})...", flush=True)
    domain_ids = [50, 51, 52, 53]
    workers = []
    out_files = {}

    for d in domain_ids:
        out_json = f"/tmp/res_{trial_name}_{d}.json"
        if os.path.exists(out_json):
            os.remove(out_json)
        out_files[d] = out_json

        cmd = [
            sys.executable, __file__,
            "--run_worker",
            "--trial_name", f"{trial_name}_inst{d}",
            "--domain_id", str(d),
            "--rtf", str(rtf),
            "--params_yaml", str(params_yaml),
            "--duration_s", str(duration_s),
            "--out_json", str(out_json)
        ]
        p = _processes.popen(cmd)
        workers.append((d, p))
        time.sleep(1.2)  # Staggered launch

    for d, p in workers:
        p.wait()

    results = {}
    for d in domain_ids:
        if os.path.exists(out_files[d]):
            with open(out_files[d]) as f:
                results[f"instance_{d}"] = json.load(f)
        else:
            results[f"instance_{d}"] = {"error": "Worker produced no output"}

    sim_hz_list = [r.get("sim_hz", 0.0) for r in results.values() if "sim_hz" in r]
    wall_hz_list = [r.get("wall_hz", 0.0) for r in results.values() if "wall_hz" in r]
    rtf_list = [r.get("measured_rtf", 0.0) for r in results.values() if "measured_rtf" in r]

    summary = {
        "trial_name": trial_name,
        "parallel_instances": 4,
        "rtf_setting": rtf,
        "instances": results,
        "mean_sim_hz": round(float(np.mean(sim_hz_list)), 2) if sim_hz_list else 0.0,
        "mean_wall_hz": round(float(np.mean(wall_hz_list)), 2) if wall_hz_list else 0.0,
        "mean_measured_rtf": round(float(np.mean(rtf_list)), 2) if rtf_list else 0.0,
        "target_ge_10hz_met": bool(np.mean(sim_hz_list) >= 10.0) if sim_hz_list else False
    }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run_worker", action="store_true")
    parser.add_argument("--trial_name", type=str, default="test")
    parser.add_argument("--domain_id", type=int, default=40)
    parser.add_argument("--rtf", type=float, default=1.0)
    parser.add_argument("--params_yaml", type=str, default="")
    parser.add_argument("--duration_s", type=float, default=12.0)
    parser.add_argument("--out_json", type=str, default="/tmp/res.json")
    args = parser.parse_args()

    if args.run_worker:
        res = run_instance_trial_in_worker(
            args.trial_name, args.domain_id, args.rtf, args.params_yaml, args.duration_s
        )
        with open(args.out_json, "w") as fp:
            json.dump(res, fp)
        return

    print("=" * 80, flush=True)
    print("NAV2 MPPI CONTROLLER FREQUENCY BENCHMARK & DIAGNOSTIC SUITE", flush=True)
    print("=" * 80, flush=True)

    time.sleep(1.0)

    base_params = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_pilot.yaml")

    # 1. Baseline Measurements (1 instance)
    print("\n[Stage 1] Baseline Measurements (1 instance)...", flush=True)
    res_b1 = run_single_trial_subprocess("baseline_1inst_rtf1", domain_id=40, rtf=1.0, params_yaml=base_params, duration_s=12.0)
    print(f"  * Baseline RTF 1.0: Wall Hz = {res_b1['wall_hz']}, Sim Hz = {res_b1['sim_hz']}, RTF = {res_b1['measured_rtf']}, Thread CPU = {res_b1['max_controller_thread_cpu_pct']}%", flush=True)

    res_b2 = run_single_trial_subprocess("baseline_1inst_rtf0", domain_id=41, rtf=0.0, params_yaml=base_params, duration_s=12.0)
    print(f"  * Baseline RTF 0.0: Wall Hz = {res_b2['wall_hz']}, Sim Hz = {res_b2['sim_hz']}, RTF = {res_b2['measured_rtf']}, Thread CPU = {res_b2['max_controller_thread_cpu_pct']}%", flush=True)

    # 2. Systematic Parameter Ablations (1 instance)
    print("\n[Stage 2] Systematic Parameter Ablations (1 instance)...", flush=True)
    ablation_results = {}

    # Ablation 1: Circular radius 0.16m
    p1 = create_modified_params_yaml(base_params, os.path.join(CONFIGS_DIR, "cfg1_circular_radius.yaml"), {
        "robot_radius": 0.16,
        "consider_footprint": False
    })
    res_a1 = run_single_trial_subprocess("abl1_circ_radius", domain_id=42, rtf=1.0, params_yaml=p1, duration_s=10.0)
    ablation_results["cfg1_circular_radius"] = res_a1
    print(f"  * Cfg 1 (Circular Radius 0.16m): Wall Hz = {res_a1['wall_hz']}, Sim Hz = {res_a1['sim_hz']}, Thread CPU = {res_a1['max_controller_thread_cpu_pct']}%", flush=True)

    # Ablation 2: Local costmap size 3x3m, resolution 0.05m
    p2 = create_modified_params_yaml(p1, os.path.join(CONFIGS_DIR, "cfg2_costmap_3m.yaml"), {
        "local_costmap_width": 3,
        "local_costmap_resolution": 0.05
    })
    res_a2 = run_single_trial_subprocess("abl2_costmap_3m", domain_id=43, rtf=1.0, params_yaml=p2, duration_s=10.0)
    ablation_results["cfg2_costmap_3m"] = res_a2
    print(f"  * Cfg 2 (Costmap 3x3m): Wall Hz = {res_a2['wall_hz']}, Sim Hz = {res_a2['sim_hz']}, Thread CPU = {res_a2['max_controller_thread_cpu_pct']}%", flush=True)

    # Ablation 3: 5 Critics (remove redundant angle critics)
    critics_5 = ["ConstraintCritic", "CostCritic", "GoalCritic", "PathAlignCritic", "PathFollowCritic"]
    p3 = create_modified_params_yaml(p2, os.path.join(CONFIGS_DIR, "cfg3_critics_5.yaml"), {
        "critics": critics_5
    })
    res_a3 = run_single_trial_subprocess("abl3_critics_5", domain_id=44, rtf=1.0, params_yaml=p3, duration_s=10.0)
    ablation_results["cfg3_critics_5"] = res_a3
    print(f"  * Cfg 3 (5 Critics): Wall Hz = {res_a3['wall_hz']}, Sim Hz = {res_a3['sim_hz']}, Thread CPU = {res_a3['max_controller_thread_cpu_pct']}%", flush=True)

    # Ablation 4: Batch size 150, time steps 30 (horizon 1.5s)
    p4 = create_modified_params_yaml(p3, os.path.join(CONFIGS_DIR, "cfg4_batch150.yaml"), {
        "batch_size": 150,
        "time_steps": 30
    })
    res_a4 = run_single_trial_subprocess("abl4_batch150", domain_id=45, rtf=1.0, params_yaml=p4, duration_s=10.0)
    ablation_results["cfg4_batch150"] = res_a4
    print(f"  * Cfg 4 (Batch 150, Steps 30): Wall Hz = {res_a4['wall_hz']}, Sim Hz = {res_a4['sim_hz']}, Thread CPU = {res_a4['max_controller_thread_cpu_pct']}%", flush=True)

    # Ablation 5: Stress test at 50 Hz controller frequency
    p5 = create_modified_params_yaml(p4, os.path.join(CONFIGS_DIR, "cfg5_freq50hz.yaml"), {
        "controller_frequency": 50.0
    })
    res_a5 = run_single_trial_subprocess("abl5_freq50hz", domain_id=46, rtf=1.0, params_yaml=p5, duration_s=10.0)
    ablation_results["cfg5_freq50hz"] = res_a5
    print(f"  * Cfg 5 (50 Hz Target): Wall Hz = {res_a5['wall_hz']}, Sim Hz = {res_a5['sim_hz']}, Thread CPU = {res_a5['max_controller_thread_cpu_pct']}%", flush=True)

    # 3. Parallel 4-instance benchmark
    print("\n[Stage 3] Testing 4 Parallel Instances...", flush=True)
    # Test 4-instance with RTF = 1.0 (baseline pilot params)
    res_p4_rtf1_base = run_parallel_4instances_subprocesses("par4_rtf1_base", rtf=1.0, params_yaml=base_params, duration_s=12.0)
    print(f"  * 4 Inst (RTF=1.0, Base): Mean Sim Hz = {res_p4_rtf1_base['mean_sim_hz']} Hz (Target >=10Hz: {res_p4_rtf1_base['target_ge_10hz_met']})", flush=True)

    # Test 4-instance with RTF = 1.0 (optimized params p4)
    res_p4_rtf1_opt = run_parallel_4instances_subprocesses("par4_rtf1_opt", rtf=1.0, params_yaml=p4, duration_s=12.0)
    print(f"  * 4 Inst (RTF=1.0, Opt):  Mean Sim Hz = {res_p4_rtf1_opt['mean_sim_hz']} Hz (Target >=10Hz: {res_p4_rtf1_opt['target_ge_10hz_met']})", flush=True)

    # Test 4-instance with RTF = 0.0 (uncapped, base params)
    res_p4_rtf0_base = run_parallel_4instances_subprocesses("par4_rtf0_base", rtf=0.0, params_yaml=base_params, duration_s=12.0)
    print(f"  * 4 Inst (RTF=0.0, Base): Mean Sim Hz = {res_p4_rtf0_base['mean_sim_hz']} Hz (Target >=10Hz: {res_p4_rtf0_base['target_ge_10hz_met']})", flush=True)

    # Test 4-instance with RTF = 0.0 (uncapped, 50 Hz controller)
    res_p4_rtf0_50hz = run_parallel_4instances_subprocesses("par4_rtf0_50hz", rtf=0.0, params_yaml=p5, duration_s=12.0)
    print(f"  * 4 Inst (RTF=0.0, 50Hz): Mean Sim Hz = {res_p4_rtf0_50hz['mean_sim_hz']} Hz (Target >=10Hz: {res_p4_rtf0_50hz['target_ge_10hz_met']})", flush=True)

    # Export full JSON
    all_data = {
        "system_info": {
            "cpu": "AMD Ryzen 7 7735H (8 cores, 16 threads)",
            "gpu": "NVIDIA GeForce RTX 4060 Laptop GPU",
            "nav2_build": "apt binary release (ros-jazzy-nav2-mppi-controller 1.3.13-1noble)"
        },
        "stage1_baseline_1instance": {
            "rtf_1.0": res_b1,
            "rtf_0.0": res_b2
        },
        "stage2_ablations_1instance": ablation_results,
        "stage3_parallel_4instances": {
            "rtf_1.0_baseline_params": res_p4_rtf1_base,
            "rtf_1.0_optimized_params": res_p4_rtf1_opt,
            "rtf_0.0_baseline_params": res_p4_rtf0_base,
            "rtf_0.0_50hz_params": res_p4_rtf0_50hz
        }
    }

    with open(OUTPUT_JSON, "w") as fp:
        json.dump(all_data, fp, indent=2)

    print(f"\n[OK] Benchmark completed and exported to: {OUTPUT_JSON}", flush=True)


if __name__ == '__main__':
    main()
