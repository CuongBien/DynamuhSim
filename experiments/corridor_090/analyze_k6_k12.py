#!/usr/bin/env python3

import math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG = "trial_01"


def load_bag():
    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {
        x.name: x.type
        for x in reader.get_all_topics_and_types()
    }

    wanted = {
        "/odom": [],
        "/scan": [],
        "/cmd_vel": [],
        "/plan": [],
        "/local_plan": [],
    }

    while reader.has_next():
        topic, raw, timestamp = reader.read_next()

        if topic not in wanted:
            continue

        msg_cls = get_message(topic_types[topic])
        msg = deserialize_message(raw, msg_cls)

        wanted[topic].append((timestamp / 1e9, msg))

    return wanted


def path_length(poses):
    total = 0.0

    for i in range(1, len(poses)):
        a = poses[i - 1].pose.position
        b = poses[i].pose.position

        total += math.hypot(
            b.x - a.x,
            b.y - a.y
        )

    return total


def analyze_odom(odom):
    if len(odom) < 2:
        return 0, 0, 0, 0

    duration = odom[-1][0] - odom[0][0]

    distance = 0.0
    velocities = []

    for i in range(1, len(odom)):
        t0, m0 = odom[i - 1]
        t1, m1 = odom[i]

        p0 = m0.pose.pose.position
        p1 = m1.pose.pose.position

        distance += math.hypot(
            p1.x - p0.x,
            p1.y - p0.y
        )

        velocities.append(
            math.hypot(
                m1.twist.twist.linear.x,
                m1.twist.twist.linear.y
            )
        )

    avg_velocity = sum(velocities) / len(velocities)
    max_velocity = max(velocities)

    return duration, distance, avg_velocity, max_velocity


def analyze_cmd(cmd):
    if len(cmd) < 2:
        return 0, 0, 0

    velocities = []
    stop_time = 0.0

    for i in range(len(cmd)):
        t, msg = cmd[i]

        v = math.hypot(
            msg.linear.x,
            msg.linear.y
        )

        velocities.append(v)

        if i > 0:
            dt = t - cmd[i - 1][0]

            if v < 0.01:
                stop_time += dt

    return (
        sum(velocities) / len(velocities),
        max(velocities),
        stop_time,
    )


def analyze_scan(scan):
    if not scan:
        return float("inf"), 0, 0

    valid_min = float("inf")
    valid_values = []

    for _, msg in scan:
        for r in msg.ranges:
            if (
                math.isfinite(r)
                and r >= msg.range_min
                and r <= msg.range_max
            ):
                valid_values.append(r)
                valid_min = min(valid_min, r)

    if not valid_values:
        return float("inf"), 0, 0

    return (
        valid_min,
        sum(valid_values) / len(valid_values),
        len(valid_values),
    )


def analyze_plans(plans):
    if not plans:
        return 0, 0

    lengths = []

    for _, msg in plans:
        if len(msg.poses) >= 2:
            lengths.append(path_length(msg.poses))

    if not lengths:
        return 0, 0

    return lengths[-1], len(lengths)


def main():
    data = load_bag()

    odom = data["/odom"]
    scan = data["/scan"]
    cmd = data["/cmd_vel"]
    plan = data["/plan"]
    local_plan = data["/local_plan"]

    duration, distance, avg_odom_v, max_odom_v = analyze_odom(odom)

    avg_cmd_v, max_cmd_v, stop_time = analyze_cmd(cmd)

    min_scan, avg_scan, valid_scan = analyze_scan(scan)

    global_path, global_updates = analyze_plans(plan)

    local_path, local_updates = analyze_plans(local_plan)

    print()
    print("=" * 60)
    print("             K6-K12 ANALYSIS")
    print("             Corridor 0.90 m")
    print("             Trial 01")
    print("=" * 60)

    print()
    print("[K6] Navigation / trial duration")
    print(f"     Duration              : {duration:.3f} s")

    print()
    print("[K7] Travel distance")
    print(f"     Odom distance         : {distance:.3f} m")

    print()
    print("[K8] Velocity")
    print(f"     Average odom velocity : {avg_odom_v:.3f} m/s")
    print(f"     Maximum odom velocity : {max_odom_v:.3f} m/s")
    print(f"     Average cmd velocity  : {avg_cmd_v:.3f} m/s")
    print(f"     Maximum cmd velocity  : {max_cmd_v:.3f} m/s")

    print()
    print("[K9] Laser clearance proxy")
    print(f"     Minimum range         : {min_scan:.3f} m")
    print(f"     Mean valid range      : {avg_scan:.3f} m")
    print(f"     Valid measurements    : {valid_scan}")

    print()
    print("[K10] Stop / zero-velocity time")
    print(f"      Stop time            : {stop_time:.3f} s")

    print()
    print("[K11] Global planning")
    print(f"      Final path length    : {global_path:.3f} m")
    print(f"      Plan updates         : {global_updates}")

    print()
    print("[K12] Local planning")
    print(f"      Final local path     : {local_path:.3f} m")
    print(f"      Local plan updates   : {local_updates}")

    print()
    print("[Data]")
    print(f"      /odom                : {len(odom)}")
    print(f"      /scan                : {len(scan)}")
    print(f"      /cmd_vel             : {len(cmd)}")
    print(f"      /plan                : {len(plan)}")
    print(f"      /local_plan          : {len(local_plan)}")

    print()
    print("=" * 60)


if __name__ == "__main__":
    main()
