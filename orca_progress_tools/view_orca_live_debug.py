#!/usr/bin/env python3
"""TEMPORARY local desktop ORCA visual check on the MuJoCo viewer.

This debug-only script accesses MuJoCoEngine's PRIVATE adapter attributes.
Do not turn this into production API or use this as research benchmark evidence.
It does NOT change repository files. Run from project root on a desktop session.
"""
from __future__ import annotations

import argparse
from math import isfinite
from time import perf_counter, sleep

from mujoco import viewer
from narrow_nav.core.actions import RobotAction
from narrow_nav.core.types import Twist2D
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.humans.registry import create_human_backend

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="corridor")
    parser.add_argument("--scenario", default="head_on")
    parser.add_argument("--width", choices=("w070", "w090", "w120"))
    parser.add_argument("--robot", default="mecanum_robot")
    parser.add_argument("--policy", choices=("advance", "stop"), default="advance")
    parser.add_argument("--vx", type=float, default=0.35)
    parser.add_argument("--ticks", type=int, default=400)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--fps-scale", type=float, default=1.0, help="1.0 = real-time simulation rate")
    args = parser.parse_args()
    if args.ticks < 1 or not isfinite(args.vx) or args.fps_scale <= 0:
        parser.error("ticks > 0, finite vx, fps-scale > 0 required")
    options = [
        f"worlds={args.world}", f"scenarios={args.scenario}",
        "humans=orca", f"robots={args.robot}", f"seed={args.seed}",
        "simulation.observation_sensors=[]",
    ]
    if args.width:
        options.append("corridor_width=" + args.width)
    config = compose_config("configs", options)
    engine = MuJoCoEngine(
        human_backend=create_human_backend(config.humans.backend)
    )
    try:
        engine.load(config)
        engine.reset(seed=args.seed)
        # The current repository exposes no public viewer state handle.
        # Private access is intentional and restricted to this diagnostic script.
        adapter = getattr(engine, "_adapter", None)
        model = getattr(adapter, "_model", None)
        data = getattr(adapter, "_data", None)
        if model is None or data is None:
            raise RuntimeError("Adapter native state unavailable. Use an official runtime-driven viewer instead.")
        initial = engine.get_ground_truth()
        print(f"MuJoCo dynamic ORCA: world={args.world} scenario={args.scenario} seed={args.seed} humans={len(initial.humans)}")
        control_dt_s = config.simulation.control_dt_ns / 1e9
        with viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as window:
            with window.lock():
                window.opt.geomgroup[3] = 0  # hide collision cylinders
                window.opt.geomgroup[5] = 1  # show clothed visuals
                if initial.humans:
                    h = initial.humans[0]
                    window.cam.lookat[:] = (h.pose.x, h.pose.y, 0.7)
                else:
                    r = initial.robot
                    window.cam.lookat[:] = (r.pose.x, r.pose.y, 0.7)
                window.cam.distance = 9.0
                window.cam.elevation = -48
            window.sync()
            contacts = 0
            for tick in range(args.ticks):
                if not window.is_running():
                    break
                start = perf_counter()
                robot = engine.get_ground_truth().robot
                vx = args.vx if args.policy == "advance" else 0.0
                action = RobotAction(robot.entity_id, Twist2D(vx, 0.0, 0.0), robot.timestamp)
                with window.lock():
                    result = engine.step(action)
                contacts += len(result.contacts)
                window.sync()
                if tick % 20 == 0 or result.terminated or result.truncated:
                    humans = result.ground_truth.humans
                    positions = ", ".join(f"{h.entity_id}=({h.pose.x:.2f},{h.pose.y:.2f})" for h in humans[:5])
                    print(f"t={result.ground_truth.stamp.timestamp.time_s:5.2f}s humans: {positions}; contacts_so_far={contacts}", flush=True)
                if result.terminated or result.truncated:
                    print(f"Episode ended: reason={result.end_reason}, contact samples={contacts}", flush=True)
                    break
                remaining = control_dt_s / args.fps_scale - (perf_counter() - start)
                if remaining > 0:
                    sleep(remaining)
            print("Simulation stopped. Inspect the scene; close the MuJoCo window to exit.")
            while window.is_running():
                sleep(0.1)
    finally:
        engine.close()


if __name__ == "__main__":
    main()
