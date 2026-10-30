#!/usr/bin/env python3
"""Installed ROS transport smoke in an isolated domain; no Gazebo or robot commands."""
import argparse
import importlib.util
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from ament_index_python.packages import get_package_prefix


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--depth-file', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    prefix = Path(get_package_prefix('custom_corridor'))
    spec = importlib.util.spec_from_file_location('installed_depth_person_node', prefix/'lib/custom_corridor/depth_person_tracker_node.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    target, bus = module.DepthPersonTrackerNode(), Node('depth_person_smoke_inputs')
    executor = SingleThreadedExecutor()
    executor.add_node(target); executor.add_node(bus)
    rows, marker_rows = [], []
    bus.create_subscription(String, '/perception/experimental_depth/tracks_json', lambda m: rows.append(json.loads(m.data)), 10)
    bus.create_subscription(MarkerArray, '/perception/experimental_depth/markers', marker_rows.append, 10)
    publishers = dict(clock=bus.create_publisher(Clock, '/clock', 10),
                      odom=bus.create_publisher(Odometry, '/odom', qos_profile_sensor_data),
                      camera=bus.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data),
                      depth=bus.create_publisher(Image, '/camera/depth/image_rect_raw', qos_profile_sensor_data))
    broadcaster = StaticTransformBroadcaster(bus)

    def spin(seconds=.12):
        end = time.monotonic()+seconds
        while time.monotonic()<end: executor.spin_once(timeout_sec=.01)

    def stamp(t):
        from builtin_interfaces.msg import Time
        return Time(sec=int(t), nanosec=int(round((t-int(t))*1e9)))

    def clock(t):
        m = Clock(); m.clock = stamp(t); publishers['clock'].publish(m); spin()

    def statics():
        a, b = TransformStamped(), TransformStamped()
        a.header.frame_id, a.child_frame_id = 'base_link', 'camera_optical_frame'
        a.transform.translation.x, a.transform.translation.z = .12, .08
        a.transform.rotation.x, a.transform.rotation.y, a.transform.rotation.z, a.transform.rotation.w = -.5, .5, -.5, .5
        b.header.frame_id, b.child_frame_id = 'base_footprint', 'base_link'
        b.transform.rotation.w = 1.
        broadcaster.sendTransform([a, b]); spin()

    def inputs(t):
        od = Odometry(); od.header.stamp = stamp(t)
        od.header.frame_id, od.child_frame_id = 'odom', 'base_footprint'
        od.pose.pose.orientation.w = 1.
        publishers['odom'].publish(od)
        ci = CameraInfo(); ci.header.stamp = stamp(t); ci.header.frame_id = 'camera_optical_frame'
        ci.width, ci.height = 424, 240
        ci.k = [223.40148705705286, 0., 212., 0., 223.40148705705283, 120., 0., 0., 1.]
        publishers['camera'].publish(ci); spin()

    depth = np.load(args.depth_file, mmap_mode='r', allow_pickle=False)[232]

    def frame(t, encoding='32FC1', frame_id='camera_optical_frame'):
        data = depth.astype('<f4') if encoding=='32FC1' else (np.clip(np.nan_to_num(depth, nan=0, posinf=0, neginf=0), 0, 60)*1000).astype('<u2')
        msg = Image(); msg.header.stamp = stamp(t); msg.header.frame_id = frame_id
        msg.height, msg.width = data.shape; msg.encoding = encoding
        msg.step = msg.width*data.dtype.itemsize; msg.data = data.tobytes()
        before = len(rows); publishers['depth'].publish(msg); spin(.3)
        if len(rows)!=before+1: raise AssertionError('Expected one output for one depth input')
        return rows[-1]

    checks = {}
    try:
        spin(.4); clock(1.); statics(); inputs(1.)
        clock(1.1); a = frame(1.1)
        checks['32FC1_transport'] = a['observation_valid'] and a['odom_stamp_ns']<=a['stamp_ns'] and a['odom_available_ns']<=a['stamp_ns']
        clock(1.2); inputs(1.2); b = frame(1.21, '16UC1')
        checks['16UC1_transport'] = b['observation_valid']
        clock(1.5); inputs(1.6); c = frame(1.5)
        checks['future_and_stale_odom_rejected'] = not c['observation_valid'] and c['invalid_reason']=='missing_causal_odom_or_camera_info'
        clock(1.7); inputs(1.7); d = frame(1.71, frame_id='wrong_camera')
        checks['camera_frame_mismatch_rejected'] = not d['observation_valid'] and d['invalid_reason']=='camera_info_frame_or_shape_mismatch'
        before = len(rows); publishers['depth'].publish(Image(header=module.PoseArray().header)); spin()
        # This stamp 0 is a new episode, not silently matched to old IDs.
        checks['clock_reset_clears_tracking'] = len(rows)==before+1 and rows[-1]['episode']==1 and not rows[-1]['tracks']
        clock(.2); statics(); inputs(.2); e = frame(.3)
        checks['fresh_inputs_after_reset'] = e['observation_valid'] and e['episode']==1
        header = module.PoseArray().header; header.stamp = stamp(.4)
        fake = [dict(track_id=i, xy_odom=[float(n), 0.], person_score=.5) for n, i in enumerate([1, -1, -1])]
        target.publish_tracks(header, fake, valid=False, reason='marker_collision_unit_fixture'); spin()
        markers = [m for m in marker_rows[-1].markers if m.action!=m.DELETEALL]
        checks['ambiguous_marker_ids_distinct'] = len(markers)==6 and len({(m.ns,m.id) for m in markers})==6
        checks['source_checkout_not_needed'] = str(module.ROOT).startswith(str(prefix))
        result = dict(status='PASS' if all(checks.values()) else 'FAIL', checks=checks,
                      installed_prefix=str(prefix), perception_resources=str(module.ROOT),
                      valid_frame_track_counts=[len(v['tracks']) for v in [a,b,e]],
                      scope='Installed ROS node and message transport with recorded depth + controlled TF/clock/odom; no Gazebo/navigation or semantic accuracy acceptance')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream: json.dump(result, stream, indent=2)
        print(json.dumps(result, indent=2))
        if result['status']!='PASS': raise SystemExit(1)
    finally:
        executor.remove_node(target); executor.remove_node(bus)
        target.destroy_node(); bus.destroy_node(); executor.shutdown(); rclpy.shutdown()


if __name__ == '__main__': main()
