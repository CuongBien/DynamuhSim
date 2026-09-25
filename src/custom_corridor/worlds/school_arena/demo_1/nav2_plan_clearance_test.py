#!/usr/bin/env python3
"""Ask the live Nav2 planner for every baseline leg and verify wall clearance."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from collision_safe_pose_applier import OccupancyGuard

HERE = Path(__file__).resolve().parent
LEGS = [
    ("A_to_Room05", (-13.5, -8.15), (7.05, -10.60)),
    ("Room05_to_CorridorC", (7.05, -10.60), (10.0, -4.0)),
    ("CorridorC_to_JunctionCD", (10.0, -4.0), (7.0, -4.0)),
    ("JunctionCD_to_JunctionDE", (7.0, -4.0), (7.0, 0.0)),
    ("JunctionDE_to_JunctionEF", (7.0, 0.0), (15.0, 0.0)),
    ("JunctionEF_to_Room12", (15.0, 0.0), (17.60, 7.30)),
]


class PlanClearanceTest(Node):
    def __init__(self):
        super().__init__(
            "school_plan_clearance_test",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        # 0.16 m + map-cell conservatism covers the rectangular robot corners.
        self.guard = OccupancyGuard(str(HERE / "map.yaml"), 0.16)
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def run_leg(self, name, start, goal):
        request = ComputePathToPose.Goal()
        request.use_start = True
        request.planner_id = "GridBased"
        for pose, xy in ((request.start, start), (request.goal, goal)):
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = xy
            pose.pose.orientation.w = 1.0
        sent = self.client.send_goal_async(request)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{name}: planner rejected request")
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.result().status != 4:
            raise RuntimeError(f"{name}: planner action failed")
        result = future.result().result
        if result.error_code != ComputePathToPose.Result.NONE:
            raise RuntimeError(f"{name}: error={result.error_code} {result.error_msg}")
        poses = result.path.poses
        if len(poses) < 2:
            raise RuntimeError(f"{name}: empty path")
        for index, item in enumerate(poses):
            x, y = item.pose.position.x, item.pose.position.y
            if not self.guard.pose_is_free(x, y):
                raise RuntimeError(
                    f"{name}: footprint collision at pose {index} ({x:.2f}, {y:.2f})"
                )
        length = sum(
            math.hypot(
                b.pose.position.x - a.pose.position.x,
                b.pose.position.y - a.pose.position.y,
            )
            for a, b in zip(poses, poses[1:])
        )
        print(f"PASS: {name}: {length:.2f} m, {len(poses)} poses, footprint-clear")


def main():
    rclpy.init()
    node = PlanClearanceTest()
    try:
        if not node.client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("/compute_path_to_pose unavailable")
        for leg in LEGS:
            node.run_leg(*leg)
        print("PASS: all live Nav2 plans are footprint-clear")
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
