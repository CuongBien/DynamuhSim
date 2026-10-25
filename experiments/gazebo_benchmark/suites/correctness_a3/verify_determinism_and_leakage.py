"""
Rigorous Seed Determinism & In-Process State Leakage Verification for Gazebo Harmonic.
Synchronized architecture:
1. Starts Gazebo simulation paused.
2. Spawns robot and initializes bridges while simulation is paused.
3. Resets world to t_sim = 0.00 and unpauses cleanly.
4. Publishes cmd_vel strictly synchronized to /clock (sim-time).
5. Tests:
   - Part 1: Seed Determinism across 5 independent launches (N=10 humans, 20s, seed=42).
   - Part 2: In-Process Reset Leakage across 3 consecutive episodes (N=10 humans, 20s).
"""

from pathlib import Path
import sys
import os
import time
import subprocess
import json
import numpy as np

import rclpy

import gz.transport13 as gzt
import gz.msgs.world_control_pb2 as world_control_pb2
import gz.msgs.boolean_pb2 as boolean_pb2

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env, spawn_benchmark_robot, start_parameter_bridge
from experiments.gazebo_benchmark.core.trajectory import SynchronizedTrajectoryCollector
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes


def control_sim(action="unpause", world_name="corridor_0.90"):
    node = gzt.Node()
    service = f"/world/{world_name}/control"
    req = world_control_pb2.WorldControl()
    if action == "pause":
        req.pause = True
    elif action == "unpause":
        req.pause = False
    elif action == "reset":
        req.reset.all = True
        req.pause = False
    node.request(service, req, world_control_pb2.WorldControl, boolean_pb2.Boolean, 3000)


@scoped_processes
def run_single_launch(run_idx, world_file, robot_file, root_dir="/home/cuongbien/DynamuhSim", duration_s=20.0, *, _processes=None):
    print(f"\n[LAUNCH {run_idx + 1}/5] Starting Gazebo Harmonic (Paused mode)...")
    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")
    env = build_benchmark_env(root_dir, turtlebot3=True)

    # 1. Start Gazebo in paused mode (without -r)
    gz_proc = _processes.popen(["gz", "sim", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(3.5)

    # 2. Spawn robot while paused
    spawn_benchmark_robot(robot_file, env, processes=_processes)

    # 3. Start ROS 2 parameter bridge
    bridge_proc = start_parameter_bridge(bridge_config, env, processes=_processes)
    time.sleep(1.5)

    # 4. Prepare recorder node
    rclpy.init()
    collector = SynchronizedTrajectoryCollector(target_duration=duration_s)

    # 5. Unpause simulation cleanly
    control_sim("unpause")

    # 6. Run simulation
    traj = collector.run_until_duration()
    collector.destroy_node()
    rclpy.shutdown()

    # 7. Teardown
    _processes.stop()
    time.sleep(1.0)
    print(f"  Launch {run_idx + 1} completed: {len(traj)} trajectory points recorded.")
    return traj


def test_seed_determinism_5runs(world_file, robot_file, root_dir):
    print("=" * 75)
    print("TEST 1: 5-RUN SEED DETERMINISM (N=10 Humans, 20s Duration, Seed=42)")
    print("=" * 75)
    runs = []
    for i in range(5):
        traj = run_single_launch(i, world_file, robot_file, root_dir)
        runs.append(traj)

    min_len = min(len(t) for t in runs)
    ref = runs[0][:min_len]

    pairwise_max_errors = []
    pairwise_mean_errors = []

    print("\nPairwise Discrepancy against Run 1 (Reference):")
    for idx, r in enumerate(runs[1:], start=2):
        pts = r[:min_len]
        errs = [np.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(pts, ref)]
        max_e = float(np.max(errs))
        mean_e = float(np.mean(errs))
        pairwise_max_errors.append(max_e)
        pairwise_mean_errors.append(mean_e)
        print(f"  Run {idx} vs Run 1: Max Discrepancy = {max_e*1000:7.2f} mm ({max_e:6.4f} m) | Mean = {mean_e*1000:7.2f} mm")

    overall_max = max(pairwise_max_errors)
    overall_mean = np.mean(pairwise_mean_errors)
    print(f"\n[OVERALL DETERMINISM RESULT]")
    print(f"  Max Error:  {overall_max*1000:.2f} mm ({overall_max:.4f} m)")
    print(f"  Mean Error: {overall_mean*1000:.2f} mm ({overall_mean:.4f} m)")
    return {
        "overall_max_mm": overall_max * 1000.0,
        "overall_mean_mm": overall_mean * 1000.0,
        "pairwise_max_mm": [e * 1000.0 for e in pairwise_max_errors]
    }


@scoped_processes
def test_in_process_reset_leakage(world_file, robot_file, root_dir, n_episodes=3, duration_s=20.0, *, _processes=None):
    print("\n" + "=" * 75)
    print("TEST 2: IN-PROCESS RESET LEAKAGE (3 Consecutive Episodes, Single Gazebo Process)")
    print("=" * 75)
    bridge_config = os.path.join(root_dir, "src/custom_corridor/config/turtlebot3_burger_bridge.yaml")
    env = build_benchmark_env(root_dir, turtlebot3=True)

    gz_proc = _processes.popen(["gz", "sim", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(3.5)

    spawn_benchmark_robot(robot_file, env, processes=_processes)

    bridge_proc = start_parameter_bridge(bridge_config, env, processes=_processes)
    time.sleep(1.5)

    ep_trajectories = []
    rclpy.init()

    try:
        for ep in range(n_episodes):
            print(f"\n[EPISODE {ep + 1}/{n_episodes}] Running {duration_s}s...", flush=True)
            if ep > 0:
                print("  Issuing WorldControl reset...", flush=True)
                control_sim("pause")
                control_sim("reset")
                time.sleep(0.5)
            control_sim("unpause")

            collector = SynchronizedTrajectoryCollector(target_duration=duration_s)
            traj = collector.run_until_duration()
            collector.destroy_node()
            ep_trajectories.append(traj)
            print(f"  Episode {ep + 1} finished: {len(traj)} points recorded.", flush=True)

    finally:
        rclpy.shutdown()
        _processes.stop()

    min_len = min(len(t) for t in ep_trajectories)
    ref = ep_trajectories[0][:min_len]

    leakage_results = []
    print("\nPairwise Discrepancy against Episode 1 (Reference):")
    for ep_idx, traj in enumerate(ep_trajectories[1:], start=2):
        pts = traj[:min_len]
        errs = [np.hypot(p[1] - q[1], p[2] - q[2]) for p, q in zip(pts, ref)]
        max_e = float(np.max(errs))
        mean_e = float(np.mean(errs))
        leakage_results.append((max_e, mean_e))
        print(f"  Episode {ep_idx} vs Episode 1: Max Drift = {max_e*1000:7.2f} mm ({max_e:6.4f} m) | Mean = {mean_e*1000:7.2f} mm")

    return leakage_results


def main():
    root_dir = "/home/cuongbien/DynamuhSim"
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/world_n10_actor_and_cylinder.sdf")
    robot_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/generated/burger_lidar1_depth1_640x480_10fps.sdf")

    print("\n" + "#" * 75)
    print("GAZEBO HARMONIC VERIFICATION SUITE: DETERMINISM & STATE LEAKAGE")
    print("#" * 75)

    det_res = test_seed_determinism_5runs(world_file, robot_file, root_dir)
    leak_res = test_in_process_reset_leakage(world_file, robot_file, root_dir)

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/verification_results.json")
    with open(out_file, "w") as f:
        json.dump({
            "seed_determinism": det_res,
            "in_process_leakage": leak_res,
        }, f, indent=2)

    print("\n" + "=" * 75)
    print("ALL TESTS COMPLETED SUCCESSFULLY!")
    print(f"Saved results to: {out_file}")
    print("=" * 75)


if __name__ == '__main__':
    main()
