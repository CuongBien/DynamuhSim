"""
Episode Reset Benchmark for Gazebo Harmonic (Section 6 / User Request).
Measures empirical wall-clock time required to reset an episode:
1. Method A: Process Restart (killing gz sim and restarting).
2. Method B: Service Reset (/world/.../control reset service).
3. Method C: In-process Teleportation (teleporting robot & humans to initial spawn poses).
"""

from pathlib import Path
import sys
import os
import time
import subprocess
import json
import numpy as np

import gz.transport13 as gzt
import gz.msgs10.boolean_pb2 as boolean_pb2
import gz.msgs10.world_control_pb2 as world_control_pb2
import gz.msgs10.pose_pb2 as pose_pb2

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from experiments.gazebo_benchmark.core.sim_env import build_benchmark_env
from experiments.gazebo_benchmark.core.mppi_trial import scoped_processes


@scoped_processes
def benchmark_process_restart(world_file, root_dir=str(Path(__file__).resolve().parents[4]), n_trials=3, *, _processes=None):
    print("\n--- Measuring Method A: Process Restart (Cold Restart) ---")
    env = build_benchmark_env(root_dir, turtlebot3=True)

    restart_times = []
    for i in range(n_trials):
        t0 = time.time()
        p = _processes.popen(["gz", "sim", "-r", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        # Wait until /stats topic is alive
        alive = False
        while time.time() - t0 < 10.0:
            try:
                subprocess.check_output(["gz", "topic", "-t", "/clock", "-n", "1"], env=env, text=True, timeout=1.0)
                alive = True
                break
            except Exception:
                time.sleep(0.1)
        t_restart = time.time() - t0
        restart_times.append(t_restart)
        print(f"  Trial {i+1}: {t_restart:.2f}s (Clock Ready)")

        _processes.stop()
        time.sleep(1.0)

    return restart_times


@scoped_processes
def benchmark_service_and_teleport(world_file, root_dir=str(Path(__file__).resolve().parents[4]), n_trials=10, *, _processes=None):
    print("\n--- Measuring Method B & C: In-Process Reset / Teleport ---")
    env = build_benchmark_env(root_dir, turtlebot3=True)

    p = _processes.popen(["gz", "sim", "-r", "-s", world_file], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(4.0)

    node = gzt.Node()
    control_service = "/world/corridor_0.90/control"
    set_pose_service = "/world/corridor_0.90/set_pose"

    # Method B: Service Reset
    req = world_control_pb2.WorldControl()
    req.reset.all = True

    service_times = []
    for i in range(n_trials):
        t0 = time.time()
        success, _ = node.request(control_service, req, world_control_pb2.WorldControl, boolean_pb2.Boolean, 2000)
        dt = (time.time() - t0) * 1000.0  # ms
        if success:
            service_times.append(dt)
        time.sleep(0.05)

    # Method C: Pose Teleport
    pose_req = pose_pb2.Pose()
    pose_req.name = "corridor_090"
    pose_req.position.x = 0.0
    pose_req.position.y = 0.0
    pose_req.position.z = 0.0
    teleport_times = []
    for i in range(n_trials):
        t0 = time.time()
        success, _ = node.request(set_pose_service, pose_req, pose_pb2.Pose, boolean_pb2.Boolean, 1000)
        dt = (time.time() - t0) * 1000.0  # ms
        if success:
            teleport_times.append(dt)
        time.sleep(0.02)

    _processes.stop()

    return service_times, teleport_times


def main():
    root_dir = str(Path(__file__).resolve().parents[4])
    world_file = os.path.join(root_dir, "experiments/gazebo_benchmark/configs/fixtures/world_n5_actor_and_cylinder.sdf")

    restarts = benchmark_process_restart(world_file, root_dir, n_trials=3)
    service_resets, teleports = benchmark_service_and_teleport(world_file, root_dir, n_trials=10)

    results = {
        "process_restart_s": {
            "mean": float(np.mean(restarts)),
            "std": float(np.std(restarts)),
            "samples": restarts
        },
        "service_reset_ms": {
            "mean": float(np.mean(service_resets)) if service_resets else 0.0,
            "std": float(np.std(service_resets)) if service_resets else 0.0,
            "count": len(service_resets)
        },
        "teleport_reset_ms": {
            "mean": float(np.mean(teleports)) if teleports else 0.0,
            "std": float(np.std(teleports)) if teleports else 0.0,
            "count": len(teleports)
        }
    }

    out_file = os.path.join(root_dir, "experiments/gazebo_benchmark/results/correctness_a3/reset_benchmark_results.json")
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)

    print("\n" + "=" * 65)
    print("EPISODE RESET BENCHMARK RESULTS")
    print("=" * 65)
    print(f"Method A (Process Restart):   {results['process_restart_s']['mean']:.2f} s +/- {results['process_restart_s']['std']:.2f} s")
    print(f"Method B (WorldControl Reset):{results['service_reset_ms']['mean']:.2f} ms +/- {results['service_reset_ms']['std']:.2f} ms")
    print(f"Method C (Pose Teleport):     {results['teleport_reset_ms']['mean']:.2f} ms +/- {results['teleport_reset_ms']['std']:.2f} ms")
    print("=" * 65)


if __name__ == '__main__':
    main()
