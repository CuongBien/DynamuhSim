import os
import math
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG = "trial_01"


def read_bag():
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=BAG,
        storage_id="mcap"
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)

    topics = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    data = {
        "/odom": [],
        "/scan": [],
        "/cmd_vel": [],
        "/plan": [],
    }

    while reader.has_next():
        topic, raw, timestamp = reader.read_next()

        if topic not in data:
            continue

        msg_type = get_message(topics[topic])
        msg = deserialize_message(raw, msg_type)

        data[topic].append((timestamp, msg))

    return data


def distance_from_odom(odom):
    if len(odom) < 2:
        return 0.0

    total = 0.0

    for i in range(1, len(odom)):
        p1 = odom[i - 1][1].pose.pose.position
        p2 = odom[i][1].pose.pose.position

        dx = p2.x - p1.x
        dy = p2.y - p1.y

        total += math.hypot(dx, dy)

    return total


def analyze_cmd_vel(cmd_vel):
    if not cmd_vel:
        return 0.0, 0.0, 0.0

    linear = []
    stopped = 0.0

    for i, (timestamp, msg) in enumerate(cmd_vel):
        v = abs(msg.linear.x)
        linear.append(v)

        if i > 0:
            dt = (timestamp - cmd_vel[i - 1][0]) / 1e9

            if v < 0.01:
                stopped += max(0.0, dt)

    avg_v = sum(linear) / len(linear)
    max_v = max(linear)

    return avg_v, max_v, stopped


def min_clearance(scan):
    if not scan:
        return float("inf")

    minimum = float("inf")

    for _, msg in scan:
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                minimum = min(minimum, r)

    return minimum


def path_lengths(plans):
    lengths = []

    for _, msg in plans:
        if len(msg.poses) < 2:
            continue

        length = 0.0

        for i in range(1, len(msg.poses)):
            p1 = msg.poses[i - 1].pose.position
            p2 = msg.poses[i].pose.position

            dx = p2.x - p1.x
            dy = p2.y - p1.y

            length += math.hypot(dx, dy)

        lengths.append(length)

    return lengths


def main():
    data = read_bag()

    odom = data["/odom"]
    scan = data["/scan"]
    cmd_vel = data["/cmd_vel"]
    plans = data["/plan"]

    print()
    print("=" * 45)
    print("        CORRIDOR 0.90 m - TRIAL 01")
    print("=" * 45)

    if odom:
        duration = (odom[-1][0] - odom[0][0]) / 1e9
        print(f"Odom duration       : {duration:.2f} s")
    else:
        duration = 0.0
        print("Odom duration       : N/A")

    distance = distance_from_odom(odom)
    print(f"Travel distance     : {distance:.3f} m")

    avg_v, max_v, stopped = analyze_cmd_vel(cmd_vel)

    print(f"Average cmd velocity: {avg_v:.3f} m/s")
    print(f"Maximum cmd velocity: {max_v:.3f} m/s")
    print(f"Stopped time        : {stopped:.2f} s")

    clearance = min_clearance(scan)

    if math.isfinite(clearance):
        print(f"Minimum scan range  : {clearance:.3f} m")
    else:
        print("Minimum scan range  : N/A")

    lengths = path_lengths(plans)

    if lengths:
        print(f"Final global path   : {lengths[-1]:.3f} m")
        print(f"Global plan updates : {len(lengths)}")
    else:
        print("Global path         : N/A")

    print()
    print("Recorded messages:")
    print(f"  /odom             : {len(odom)}")
    print(f"  /scan             : {len(scan)}")
    print(f"  /cmd_vel          : {len(cmd_vel)}")
    print(f"  /plan             : {len(plans)}")

    print("=" * 45)


if __name__ == "__main__":
    main()
