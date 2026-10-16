#!/usr/bin/env python3
"""
A3.3: Real Head-on Interaction Benchmark with Nav2 MPPI in Corridors 0.90m and 1.20m.

Specification:
- Corridors: corridor_090 (0.90m) and corridor_120 (1.20m)
- Scenario: 1 robot from start (-13.0, 0.0) to goal (-7.0, 0.0).
  1 human starts 6.0m ahead from opposite side (-7.0, 0.0), walking head-on towards (-15.0, 0.0).
  Human speed sampled in [1.0, 1.3] m/s.
- Personas:
  - H1: Cooperative (responsibility 0.5, steps aside / yields)
  - H3: Robot-blind / unyielding (responsibility 0.0, ignores robot, walks straight)
- Runs: 10 seeds per cell (2 corridors x 2 personas = 4 cells, total 40 runs).
- Real Nav2 MPPI controller with dynamic noise (regenerate_noises: true, pilot config).
- Metrics reported per run:
  - dmin (robot-human)
  - Collision occurrence (dmin < 0.44m or wall contact)
  - Time to goal (seconds)
  - Max lateral deviation (|y_robot|)
  - Minimum robot speed
  - Robot stopped or yielded
- Summary per cell:
  - Median dmin
  - Min-Max dmin
  - Collision rate
  - Goal reach rate
- Output:
  - Raw JSON: experiments/gazebo_benchmark/results/correctness_a3/a3_3_real_interaction_results.json
"""

import os
import sys
import time
import signal
import subprocess
import json
import re
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


class A33InteractionMonitor(Node):
    def __init__(self, target_gx=-7.0, target_gy=0.0, max_duration_s=25.0, corridor_half_width=0.45):
        super().__init__('a3_3_interaction_monitor')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.target_gx = target_gx
        self.target_gy = target_gy
        self.max_duration_s = max_duration_s
        self.corridor_half_width = corridor_half_width

        self.sim_time = 0.0
        self.t_start = None
        self.robot_pos = (-13.0, 0.0)
        self.robot_vel = (0.0, 0.0)
        self.human_pos = None
        self.human_vel = (0.0, 0.0)
        self.human_speed = 0.0

        self.dmin = 999.0
        self.max_lat_dev = 0.0
        self.min_robot_speed = 999.0
        self.min_speed_during_encounter = 999.0
        self.robot_stopped = False
        self.robot_yielded = False
        self.collision_human = False
        self.collision_wall = False
        self.goal_reached = False

        self.telemetry = []

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 50)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 50)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _odom_cb(self, msg: Odometry):
        if self.robot_pos == (-13.0, 0.0):
            self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _gt_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'robot' in data and data['robot']['x'] is not None:
                self.robot_pos = (data['robot']['x'], data['robot']['y'])
                self.robot_vel = (data['robot'].get('vx', 0.0), data['robot'].get('vy', 0.0))

            if 'humans' in data and len(data['humans']) > 0:
                h = data['humans'][0]
                hx = h.get('x', None)
                hy = h.get('y', None)
                if hx is not None and hy is not None:
                    self.human_pos = (hx, hy)
                    self.human_vel = (h.get('vx', 0.0), h.get('vy', 0.0))
                    self.human_speed = h.get('speed', 0.0)
        except Exception:
            pass

    def send_goal(self):
        print("    [NAV2] Waiting for /navigate_to_pose action server...", flush=True)
        if not self.nav_client.wait_for_server(timeout_sec=15.0):
            print("    [ERROR] /navigate_to_pose server not available!", flush=True)
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target_gx)
        goal_msg.pose.pose.position.y = float(self.target_gy)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        for attempt in range(5):
            print(f"    [NAV2] Sending goal ({self.target_gx}, {self.target_gy}) (attempt {attempt+1})...", flush=True)
            send_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=6.0)

            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle.accepted:
                    print("    [NAV2] Goal ACCEPTED!", flush=True)
                    return True
            time.sleep(1.0)
        return False

    def run_navigation(self):
        t_w0 = time.time()
        while time.time() - t_w0 < 2.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.send_goal():
            return {
                "goal_reached": False,
                "time_to_goal": None,
                "dmin": 999.0,
                "collision_human": False,
                "collision_wall": False,
                "collision_occurred": False,
                "max_lateral_dev": 0.0,
                "min_robot_speed": 0.0,
                "min_speed_encounter": None,
                "robot_stopped": False,
                "robot_yielded": False,
                "robot_stopped_or_yielded": False,
                "final_dist_to_goal": 999.0,
                "error": "goal_rejected_or_timeout",
                "telemetry": []
            }

        self.t_start = self.sim_time
        t_wall_start = time.time()
        last_sample_t = -1.0
        last_moving_time = self.sim_time

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            t_rel = self.sim_time - self.t_start

            dist_to_goal = np.hypot(self.robot_pos[0] - self.target_gx, self.robot_pos[1] - self.target_gy)
            robot_spd = np.hypot(self.robot_vel[0], self.robot_vel[1])

            lat_dev = abs(self.robot_pos[1])
            if lat_dev > self.max_lat_dev:
                self.max_lat_dev = lat_dev

            cur_h_dist = 999.0
            if self.human_pos is not None:
                cur_h_dist = np.hypot(self.robot_pos[0] - self.human_pos[0], self.robot_pos[1] - self.human_pos[1])
                if cur_h_dist < self.dmin:
                    self.dmin = cur_h_dist

                if cur_h_dist <= 2.5:
                    if robot_spd < self.min_speed_during_encounter:
                        self.min_speed_during_encounter = robot_spd
                    if robot_spd < 0.05:
                        self.robot_stopped = True
                    if lat_dev > 0.08:
                        self.robot_yielded = True

                # Collision threshold: circumscribed robot (0.16) + human proxy (0.28) = 0.44m
                if cur_h_dist < 0.44:
                    self.collision_human = True

            # Wall collision threshold: half_width - half_chassis_width (0.10)
            if lat_dev > (self.corridor_half_width - 0.10):
                self.collision_wall = True

            if t_rel > 0.5 and dist_to_goal > 0.35:
                if robot_spd < self.min_robot_speed:
                    self.min_robot_speed = robot_spd

            # Deadlock detection
            if robot_spd > 0.03:
                last_moving_time = self.sim_time
            elif (self.sim_time - last_moving_time) > 8.0 and dist_to_goal > 0.5:
                print(f"    [DEADLOCK] Robot stationary for > 8s at dist {dist_to_goal:.2f}m!", flush=True)
                break

            if self.sim_time - last_sample_t >= 0.1:
                last_sample_t = self.sim_time
                self.telemetry.append({
                    "t": round(t_rel, 2),
                    "rx": round(self.robot_pos[0], 3),
                    "ry": round(self.robot_pos[1], 3),
                    "rspd": round(robot_spd, 3),
                    "hx": round(self.human_pos[0], 3) if self.human_pos else None,
                    "hy": round(self.human_pos[1], 3) if self.human_pos else None,
                    "hspd": round(self.human_speed, 3),
                    "d": round(cur_h_dist, 3)
                })

            if dist_to_goal <= 0.25:
                print(f"    [GOAL REACHED] at t_sim = {t_rel:.2f}s! Dist to goal: {dist_to_goal:.3f}m", flush=True)
                self.goal_reached = True
                break

            if t_rel >= self.max_duration_s:
                print(f"    [TIMEOUT] Max duration {self.max_duration_s}s reached. Dist to goal: {dist_to_goal:.3f}m", flush=True)
                break

            if time.time() - t_wall_start > (self.max_duration_s * 3.0 + 20.0):
                print("    [TIMEOUT] Hard wall timeout reached!", flush=True)
                break

        time_to_goal = (self.sim_time - self.t_start) if self.goal_reached else None

        return {
            "goal_reached": self.goal_reached,
            "time_to_goal": round(time_to_goal, 2) if time_to_goal is not None else None,
            "dmin": round(self.dmin, 3),
            "collision_human": self.collision_human,
            "collision_wall": self.collision_wall,
            "collision_occurred": self.collision_human or self.collision_wall,
            "max_lateral_dev": round(self.max_lat_dev, 3),
            "min_robot_speed": round(self.min_robot_speed, 3) if self.min_robot_speed < 900 else 0.0,
            "min_speed_encounter": round(self.min_speed_during_encounter, 3) if self.min_speed_during_encounter < 900 else None,
            "robot_stopped": self.robot_stopped,
            "robot_yielded": self.robot_yielded,
            "robot_stopped_or_yielded": self.robot_stopped or self.robot_yielded,
            "final_dist_to_goal": round(dist_to_goal, 3),
            "telemetry_count": len(self.telemetry)
        }


def run_single_benchmark_trial(root_dir: str, corridor_type: str, persona: str, seed: int, v_human: float) -> dict:
    gen_dir = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated")
    os.makedirs(gen_dir, exist_ok=True)

    # 1. Generate Scenario JSON
    scen_file = os.path.join(gen_dir, f"scenario_a3_3_{corridor_type}_{persona}_{seed}.json")
    scen_data = {
        "episode_id": seed,
        "corridor_mode": True,
        "humans": [
            {
                "id": 1,
                "name": "human_01",
                "mode": "autonomous",
                "speed": v_human,
                "max_speed": v_human,
                "persona": persona,
                "loop": False,
                "wait_for_robot": True,
                "start": {"x": -7.0, "y": 0.0, "yaw": 3.14159265},
                "waypoints": [[-7.0, 0.0], [-15.0, 0.0]]
            }
        ]
    }
    with open(scen_file, "w") as f:
        json.dump(scen_data, f, indent=2)

    # 2. Patch SDF world
    base_sdf_path = os.path.join(root_dir, f"src/custom_corridor/worlds/corridor_{corridor_type}.sdf")
    with open(base_sdf_path, "r") as f:
        sdf = f.read()

    sdf = re.sub(r'<real_time_factor>.*?</real_time_factor>', '<real_time_factor>0.0</real_time_factor>', sdf)
    sdf = re.sub(
        r'<randomize_politeness>.*?</randomize_politeness>',
        f'<randomize_politeness>false</randomize_politeness>\n      <seed>{seed}</seed>\n      <default_scenario_file>{scen_file}</default_scenario_file>',
        sdf
    )
    if f"<seed>{seed}</seed>" not in sdf:
        sdf = re.sub(r'<seed>\d+</seed>', f'<seed>{seed}</seed>', sdf)

    proxy_z = "0.80" if corridor_type == "090" else "0.875"
    sdf = re.sub(r'<actor name=\"human_01_visual\">\s*<pose>.*?</pose>', '<actor name=\"human_01_visual\">\n      <pose>-7.0 0.0 0.0 0 0 3.14159</pose>', sdf)
    sdf = re.sub(r'<model name=\"human_01_proxy\">\s*<pose>.*?</pose>', f'<model name=\"human_01_proxy\">\n      <pose>-7.0 0.0 {proxy_z} 0 0 3.14159</pose>', sdf)

    trial_sdf = os.path.join(gen_dir, f"world_a3_3_{corridor_type}_{persona}_{seed}.sdf")
    with open(trial_sdf, "w") as f:
        f.write(sdf)

    # Environment
    corridor_models = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    domain_id = str(70 + (seed % 100))
    env["ROS_DOMAIN_ID"] = domain_id
    partition = f"a3_3_{corridor_type}_{persona}_{seed}"
    env["GZ_PARTITION"] = partition

    all_procs = []
    nav2_log_path = f"/tmp/nav2_a3_3_{corridor_type}_{persona}_{seed}.log"
    if os.path.exists(nav2_log_path):
        os.remove(nav2_log_path)

    width_arg = "0.90" if corridor_type == "090" else "1.20"
    map_arg = f"corridor_{corridor_type}"
    params_file = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/config/nav2_mppi_pilot.yaml")
    corridor_half_w = 0.45 if corridor_type == "090" else 0.60

    result = {}
    try:
        # Launch Sim
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={width_arg}",
            f"custom_world:={trial_sdf}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0"
        ]
        gz_p = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        all_procs.append(gz_p)
        time.sleep(4.5)

        # Launch Nav2
        nav2_log_file = open(nav2_log_path, "w")
        nav2_cmd = [
            "ros2", "launch", "custom_corridor", "nav2_corridor.launch.py",
            "controller:=mppi",
            f"params_file:={params_file}",
            f"map:={map_arg}",
            "robot_model:=mecanum"
        ]
        nav2_p = subprocess.Popen(nav2_cmd, env=env, stdout=nav2_log_file, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
        all_procs.append(nav2_p)

        t0 = time.time()
        nav2_ready = False
        while time.time() - t0 < 35.0:
            if os.path.exists(nav2_log_path):
                with open(nav2_log_path) as fp:
                    content = fp.read()
                    if "lifecycle_manager_navigation" in content and "Managed nodes are active" in content:
                        nav2_ready = True
                        print(f"  Nav2 became active in {time.time()-t0:.1f}s!", flush=True)
                        break
            time.sleep(0.4)

        if not nav2_ready:
            print("  [ERROR] Nav2 failed to activate within 35s!", flush=True)
            return {"error": "nav2_activation_timeout"}

        time.sleep(1.5)

        # Connect ROS 2 Monitor
        os.environ["ROS_DOMAIN_ID"] = domain_id
        os.environ["GZ_PARTITION"] = partition
        rclpy.init()
        monitor = A33InteractionMonitor(target_gx=-7.0, target_gy=0.0, max_duration_s=25.0, corridor_half_width=corridor_half_w)
        result = monitor.run_navigation()
        monitor.destroy_node()
        rclpy.shutdown()

    finally:
        for p in all_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.3)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1.5)

    result["seed"] = seed
    result["corridor"] = f"corridor_{corridor_type}"
    result["corridor_width_m"] = 0.90 if corridor_type == "090" else 1.20
    result["persona"] = persona
    result["v_human"] = v_human
    return result


def main():
    parser = argparse.ArgumentParser(description="A3.3 Real Interaction Benchmark")
    parser.add_argument("--cell", type=str, default="all", choices=["all", "090_H1", "090_H3", "120_H1", "120_H3"], help="Run specific cell")
    args = parser.parse_args()

    root_dir = "/home/cuongbien/DynamuhSim"
    out_json = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/a3_3_real_interaction_results.json")

    cells = [
        {"corridor": "090", "persona": "H1", "seed_base": 101, "name": "090_H1"},
        {"corridor": "090", "persona": "H3", "seed_base": 201, "name": "090_H3"},
        {"corridor": "120", "persona": "H1", "seed_base": 301, "name": "120_H1"},
        {"corridor": "120", "persona": "H3", "seed_base": 401, "name": "120_H3"},
    ]

    if args.cell != "all":
        cells = [c for c in cells if c["name"] == args.cell]

    # Load existing results if available
    all_results = {"trials": [], "summary": {}}
    if os.path.exists(out_json):
        try:
            with open(out_json, "r") as f:
                all_results = json.load(f)
        except Exception:
            pass

    existing_keys = {(t.get("corridor"), t.get("persona"), t.get("seed")) for t in all_results.get("trials", [])}

    print(f"\n================================================================================")
    print(f"=== BENCHMARK A3.3: REAL HEAD-ON INTERACTION IN CORRIDORS (40 RUNS) ===")
    print(f"================================================================================")
    print(f"Cells to execute: {[c['name'] for c in cells]}")
    print(f"Already completed trials: {len(existing_keys)}")

    total_runs = len(cells) * 10
    current_run = 0

    for cell in cells:
        c_type = cell["corridor"]
        persona = cell["persona"]
        s_base = cell["seed_base"]
        cell_name = cell["name"]

        print(f"\n--------------------------------------------------------------------------------")
        print(f">>> STARTING CELL: Corridor {c_type} (width {0.9 if c_type=='090' else 1.2}m) | Persona {persona}")
        print(f"--------------------------------------------------------------------------------")

        for i in range(10):
            seed = s_base + i
            current_run += 1
            key = (f"corridor_{c_type}", persona, seed)

            if key in existing_keys:
                print(f"[{current_run}/{total_runs}] Skipping already completed trial: {key}")
                continue

            # Deterministic speed sampling per seed in [1.0, 1.3] m/s
            rng = np.random.RandomState(seed)
            v_human = round(float(rng.uniform(1.0, 1.3)), 3)

            print(f"\n[{current_run}/{total_runs}] Running {cell_name} | Seed {seed} | v_human={v_human:.3f} m/s...")
            trial_res = run_single_benchmark_trial(root_dir, c_type, persona, seed, v_human)

            all_results["trials"].append(trial_res)
            existing_keys.add(key)

            print(f"  --> Result: GoalReached={trial_res.get('goal_reached')}, dmin={trial_res.get('dmin')}m, "
                  f"Collision={trial_res.get('collision_occurred')}, Time={trial_res.get('time_to_goal')}s, "
                  f"MaxLatDev={trial_res.get('max_lateral_dev')}m, MinSpd={trial_res.get('min_robot_speed')}m/s, "
                  f"Stopped={trial_res.get('robot_stopped')}, Yielded={trial_res.get('robot_yielded')}")

            # Recompute summary
            summary = {}
            for c_info in [
                ("corridor_090", "H1"), ("corridor_090", "H3"),
                ("corridor_120", "H1"), ("corridor_120", "H3")
            ]:
                c_corr, c_pers = c_info
                c_key = f"{c_corr}_{c_pers}"
                matching = [t for t in all_results["trials"] if t.get("corridor") == c_corr and t.get("persona") == c_pers]
                if matching:
                    dmins = [t["dmin"] for t in matching if t.get("dmin") is not None]
                    collisions = [1 if t.get("collision_occurred") else 0 for t in matching]
                    goals = [1 if t.get("goal_reached") else 0 for t in matching]
                    lat_devs = [t["max_lateral_dev"] for t in matching if t.get("max_lateral_dev") is not None]
                    min_spds = [t["min_robot_speed"] for t in matching if t.get("min_robot_speed") is not None]
                    times = [t["time_to_goal"] for t in matching if t.get("time_to_goal") is not None]

                    summary[c_key] = {
                        "n_runs": len(matching),
                        "dmin_median_m": round(float(np.median(dmins)), 3) if dmins else None,
                        "dmin_min_m": round(float(np.min(dmins)), 3) if dmins else None,
                        "dmin_max_m": round(float(np.max(dmins)), 3) if dmins else None,
                        "collision_rate_pct": round(float(np.mean(collisions) * 100), 1),
                        "goal_reach_rate_pct": round(float(np.mean(goals) * 100), 1),
                        "time_to_goal_median_s": round(float(np.median(times)), 2) if times else None,
                        "max_lateral_dev_median_m": round(float(np.median(lat_devs)), 3) if lat_devs else None,
                        "min_robot_speed_median_mps": round(float(np.median(min_spds)), 3) if min_spds else None,
                    }
            all_results["summary"] = summary

            # Save incrementally
            with open(out_json, "w") as f:
                json.dump(all_results, f, indent=2)

    print(f"\n================================================================================")
    print(f"=== ALL BENCHMARK A3.3 TRIALS COMPLETED! SUMMARY TABLE: ===")
    print(f"================================================================================")
    for k, v in all_results.get("summary", {}).items():
        print(f"[{k}] (N={v['n_runs']}):")
        print(f"   dmin: median={v['dmin_median_m']}m [min={v['dmin_min_m']}m, max={v['dmin_max_m']}m]")
        print(f"   Collision Rate: {v['collision_rate_pct']}% | Goal Reach Rate: {v['goal_reach_rate_pct']}%")
        print(f"   Time to Goal: {v['time_to_goal_median_s']}s | Max Lat Dev: {v['max_lateral_dev_median_m']}m | Min Spd: {v['min_robot_speed_median_mps']}m/s")
    print(f"\nSaved results to: {out_json}")


if __name__ == "__main__":
    main()
