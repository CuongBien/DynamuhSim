#!/usr/bin/env python3
"""Measure HuNav target motion and optional pairwise separation."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", nargs="+", default=["demo_primary"])
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--min-movement", type=float, default=0.20)
    parser.add_argument("--min-separation", type=float, default=0.50)
    args = parser.parse_args()

    rclpy.init()
    node = Node("demo_1_hunav_runtime_probe")
    tracks = {name: [] for name in args.agents}
    separations = []

    def callback(msg):
        current = {}
        for transform in msg.transforms:
            name = transform.child_frame_id.strip("/")
            if name in tracks:
                point = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                )
                tracks[name].append(point)
                current[name] = point
        if len(args.agents) == 2 and all(name in current for name in args.agents):
            separations.append(math.dist(current[args.agents[0]], current[args.agents[1]]))

    node.create_subscription(TFMessage, "/school_hunav/target_poses", callback, 10)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    movements = {
        name: math.dist(points[0], points[-1]) if len(points) > 1 else 0.0
        for name, points in tracks.items()
    }
    print("samples:", {name: len(points) for name, points in tracks.items()})
    print("movements_m:", movements)
    if min(movements.values(), default=0.0) < args.min_movement:
        raise SystemExit("FAIL: one or more HuNav agents did not move enough")
    if separations:
        minimum = min(separations)
        print(f"minimum_separation_m: {minimum:.3f}")
        if minimum < args.min_separation:
            raise SystemExit("FAIL: agents violated minimum separation")
    print("PASS: HuNav runtime probe")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

