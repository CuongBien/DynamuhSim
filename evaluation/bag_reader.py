"""ROS 2 Jazzy rosbag2 MCAP reading and timestamp synchronization."""
from __future__ import annotations

import bisect
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

DEFAULT_TOPICS: Set[str] = {"/clock", "/odom", "/cmd_vel", "/scan", "/plan", "/local_plan"}


def stamp_seconds(stamp: Any) -> Optional[float]:
    """Convert a builtin_interfaces/Time message or struct into float seconds."""
    if stamp is None or not hasattr(stamp, "sec"):
        return None
    sec = float(stamp.sec)
    nanosec = float(getattr(stamp, "nanosec", 0))
    val = sec + nanosec * 1e-9
    return val if math.isfinite(val) else None


def header_seconds(msg: Any) -> Optional[float]:
    """Extract float simulation seconds from a message Header."""
    header = getattr(msg, "header", None)
    return stamp_seconds(getattr(header, "stamp", None)) if header else None


def compute_path_length(path_msg: Any) -> Optional[float]:
    """Compute total Euclidean path length from a nav_msgs/msg/Path."""
    poses = getattr(path_msg, "poses", None)
    if not poses:
        return 0.0 if poses is not None else None
    points = []
    for pose_stamped in poses:
        pos = pose_stamped.pose.position
        points.append((float(pos.x), float(pos.y), float(pos.z)))
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def clock_bracket(clock_storage_times: Sequence[float], storage_t: float) -> Optional[Tuple[int, float]]:
    """Return the right /clock index and enclosing storage-time gap."""
    if not clock_storage_times or storage_t < clock_storage_times[0] or storage_t > clock_storage_times[-1]:
        return None
    idx = bisect.bisect_left(clock_storage_times, storage_t)
    if idx == 0:
        return idx, 0.0
    if idx == len(clock_storage_times):
        return idx - 1, 0.0
    return idx, clock_storage_times[idx] - clock_storage_times[idx - 1]


def map_storage_to_clock(
    clock_storage_times: Sequence[float],
    clock_sim_times: Sequence[float],
    storage_t: float,
) -> Optional[float]:
    """Map storage timestamp to simulation time via piecewise linear interpolation."""
    if not clock_storage_times or storage_t < clock_storage_times[0] or storage_t > clock_storage_times[-1]:
        return None
    idx = bisect.bisect_left(clock_storage_times, storage_t)
    if idx == 0:
        return clock_sim_times[0]
    if idx == len(clock_storage_times):
        return clock_sim_times[-1]

    t0 = clock_storage_times[idx - 1]
    t1 = clock_storage_times[idx]
    s0 = clock_sim_times[idx - 1]
    s1 = clock_sim_times[idx]
    if t1 == t0:
        return s0
    return s0 + (storage_t - t0) * (s1 - s0) / (t1 - t0)


def read_bag_data(bag_path: Path, topics: Optional[Set[str]] = None) -> Dict[str, Any]:
    """Open an MCAP bag and parse all target messages."""
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python libraries are not sourced. Please source your ROS 2 environment\n"
            "(e.g. source /opt/ros/$ROS_DISTRO/setup.bash and source your workspace install/setup.bash).\n"
            f"Error details: {exc}"
        ) from exc

    if not bag_path.is_dir():
        raise FileNotFoundError(f"Bag directory not found: {bag_path}")

    target_topics = topics or DEFAULT_TOPICS
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
    except Exception as exc:
        raise RuntimeError(f"Could not open MCAP bag at {bag_path}: {exc}") from exc

    topic_types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    present_topics = sorted(target_topics.intersection(topic_types))
    if not present_topics:
        raise RuntimeError(
            f"None of target topics {target_topics} found in bag. Available: {sorted(topic_types.keys())}"
        )

    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=present_topics))
    except Exception:
        pass

    classes = {name: get_message(topic_types[name]) for name in present_topics}

    data: Dict[str, Any] = {
        "topic_types": topic_types,
        "counts": Counter(),
        "clock": [],      # (storage_t, sim_t)
        "odom": [],       # (sim_t, x, y, twist_speed)
        "cmd_raw": [],    # (storage_t, linear_x, angular_z)
        "scan": [],       # (sim_t, min_range)
        "plan": [],       # (sim_t, length, pose_count)
        "local_plan": [], # (sim_t, length, pose_count)
    }

    storage_timestamps: List[int] = []

    while reader.has_next():
        topic, raw_bytes, storage_ns = reader.read_next()
        if topic not in classes:
            continue

        storage_timestamps.append(storage_ns)
        data["counts"][topic] += 1
        storage_t = storage_ns * 1e-9

        try:
            msg = deserialize_message(raw_bytes, classes[topic])
        except Exception as exc:
            print(f"Warning: Failed to deserialize {topic}: {exc}", file=sys.stderr)
            continue

        if topic == "/clock":
            val = stamp_seconds(getattr(msg, "clock", None))
            if val is not None:
                data["clock"].append((storage_t, val))

        elif topic == "/odom":
            t = header_seconds(msg)
            pos = msg.pose.pose.position
            twist = msg.twist.twist.linear
            if t is not None:
                speed = math.hypot(float(twist.x), float(twist.y))
                data["odom"].append((t, float(pos.x), float(pos.y), speed))

        elif topic == "/cmd_vel":
            # ROS 2 / Nav2 may publish either:
            #   geometry_msgs/msg/Twist
            # or
            #   geometry_msgs/msg/TwistStamped
            #
            # TwistStamped stores velocity inside msg.twist.
            cmd = msg.twist if hasattr(msg, "twist") else msg

            linear = cmd.linear
            angular = cmd.angular

            data["cmd_raw"].append(
                (storage_t, float(linear.x), float(angular.z))
            )

        elif topic == "/scan":
            t = header_seconds(msg)
            if t is not None:
                valid_ranges = [
                    float(r) for r in msg.ranges
                    if math.isfinite(float(r)) and float(r) >= float(msg.range_min)
                ]
                min_r = min(valid_ranges) if valid_ranges else None
                data["scan"].append((t, min_r))

        elif topic in ("/plan", "/local_plan"):
            t = header_seconds(msg)
            if t is not None:
                key = topic[1:]
                poses = getattr(msg, "poses", [])
                length = compute_path_length(msg)
                data[key].append((t, length, len(poses)))

    data["storage_range"] = (
        (min(storage_timestamps) * 1e-9, max(storage_timestamps) * 1e-9)
        if storage_timestamps else (None, None)
    )

    for key in ("clock", "odom", "cmd_raw", "scan", "plan", "local_plan"):
        data[key].sort(key=lambda r: r[0])

    return data
