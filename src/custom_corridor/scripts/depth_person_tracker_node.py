#!/usr/bin/env python3
"""Opt-in depth person candidates. No Nav2 commands, GT or training labels."""
import json
import math
from pathlib import Path
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


def perception_root():
    repository = Path(__file__).resolve().parents[3]
    source = repository / 'experiments/social_navigation/perception'
    if (source / 'depth_identity_guard/integrated_pipeline.py').is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('custom_corridor')) / 'perception'


ROOT = perception_root()
sys.path.insert(0, str(ROOT / 'depth_identity_guard'))
from integrated_pipeline import EndToEndPerceptionPipeline
from ros_support import CausalBuffer, decode_depth, transform_matrix


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class DepthPersonTrackerNode(Node):
    def __init__(self):
        super().__init__('depth_person_tracker_node')
        for key, value in dict(depth_topic='/camera/depth/image_rect_raw',
                               camera_info_topic='/camera/camera_info', odom_topic='/odom',
                               base_frame='base_link', odom_frame='odom', odom_child_frame='base_footprint',
                               min_person_score=.45, stride=4, odom_max_age_ms=150).items():
            self.declare_parameter(key, value)
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.odom_child_frame = self.get_parameter('odom_child_frame').value
        self.stride = self.get_parameter('stride').value
        self.max_age_ns = int(self.get_parameter('odom_max_age_ms').value * 1_000_000)
        if self.stride < 1 or self.max_age_ns < 1:
            raise ValueError('Invalid stride or odom age')
        self.pipeline = EndToEndPerceptionPipeline(min_person_score=self.get_parameter('min_person_score').value)
        self.odom = CausalBuffer()
        self.camera = CausalBuffer(maxlen=100)
        self.last_depth_ns = None
        self.episode = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.on_camera, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.on_depth, qos_profile_sensor_data)
        prefix = '/perception/experimental_depth/'
        self.marker_pub = self.create_publisher(MarkerArray, prefix + 'markers', 10)
        self.pose_pub = self.create_publisher(PoseArray, prefix + 'poses', 10)
        self.track_pub = self.create_publisher(String, prefix + 'tracks_json', 10)
        self.get_logger().info('Experimental depth detector: waiting for CameraInfo, causal odom and timestamped TF')

    def on_odom(self, msg):
        if msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.odom_child_frame:
            self.get_logger().warning('Ignoring odometry with unexpected frame', throttle_duration_sec=5.)
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = np.array([p.x, p.y, yaw], float)
        if self.odom_child_frame != self.base_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.odom_child_frame, self.base_frame, Time.from_msg(msg.header.stamp)).transform
                offset = transform_matrix([tf.translation.x, tf.translation.y, tf.translation.z],
                                          [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
                if not np.allclose(offset[2, :2], 0., atol=1e-5):
                    return  # This tracker accepts planar odometry only.
                c, s = math.cos(yaw), math.sin(yaw)
                pose[:2] += np.array([[c, -s], [s, c]]) @ offset[:2, 3]
                pose[2] += math.atan2(offset[1, 0], offset[0, 0])
            except (TransformException, ValueError):
                return
        if np.isfinite(pose).all():
            self.odom.add(stamp_ns(msg.header.stamp), self.get_clock().now().nanoseconds, pose)

    def on_camera(self, msg):
        self.camera.add(stamp_ns(msg.header.stamp), self.get_clock().now().nanoseconds, msg)

    def on_depth(self, msg):
        t = stamp_ns(msg.header.stamp)
        if self.last_depth_ns is not None and t < self.last_depth_ns:
            self.pipeline.reset()
            self.odom.clear()
            self.camera.clear()
            self.episode += 1
            self.last_depth_ns = t
            self.publish_tracks(msg.header, [], valid=False, reason='clock_reset_wait_for_inputs')
            return
        if t == self.last_depth_ns:
            return
        self.last_depth_ns = t
        od = self.odom.select(t, self.max_age_ns)
        cam = self.camera.select(t, max_age_ns=None)
        if od is None or cam is None:
            self.publish_tracks(msg.header, [], valid=False, reason='missing_causal_odom_or_camera_info')
            return
        info = cam[2]
        if (info.width, info.height, info.header.frame_id) != (msg.width, msg.height, msg.header.frame_id):
            self.publish_tracks(msg.header, [], valid=False, reason='camera_info_frame_or_shape_mismatch')
            return
        if any(abs(v) > 1e-9 for v in info.d):
            self.publish_tracks(msg.header, [], valid=False, reason='requires_rectified_depth_camera')
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
            optical_to_base = transform_matrix([tf.translation.x, tf.translation.y, tf.translation.z],
                                                [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
            depth = decode_depth(msg)
            tracks = self.pipeline.process_depth_frame(t, depth, info.k, optical_to_base, od[2], stride=self.stride)
        except (TransformException, ValueError) as exc:
            self.publish_tracks(msg.header, [], valid=False, reason=type(exc).__name__ + ': ' + str(exc))
            self.get_logger().warning('Skipping invalid depth input: ' + str(exc), throttle_duration_sec=5.)
            return
        diagnostics = self.pipeline.last_diagnostics
        inference_ok = diagnostics['inference_failures'] == 0
        self.publish_tracks(msg.header, tracks, valid=inference_ok, reason=None if inference_ok else 'classifier_inference_failed',
                            diagnostics=diagnostics,
                            odom_stamp_ns=od[0], odom_available_ns=od[1])

    def publish_tracks(self, header, tracks, *, valid, reason, **metadata):
        stamp = stamp_ns(header.stamp)
        payload = dict(schema='experimental_depth_tracks/1', stamp_ns=stamp, episode=self.episode,
                       frame_id=self.odom_frame, observation_valid=valid, invalid_reason=reason,
                       semantic_validated=False, training_authorized=False, tracks=tracks, **metadata)
        message = String()
        message.data = json.dumps(payload, allow_nan=False)
        self.track_pub.publish(message)
        poses, markers = PoseArray(), MarkerArray()
        poses.header.stamp, poses.header.frame_id = header.stamp, self.odom_frame
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, track in enumerate(tracks):
            x, y = track['xy_odom']
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(x), float(y), .85
            pose.orientation.w = 1.
            poses.poses.append(pose)
            for kind in ('body', 'id'):
                m = Marker()
                m.header = poses.header
                m.ns, m.id = kind, index  # Distinct proposals even if multiple IDs are -1.
                m.action = Marker.ADD
                m.pose = pose
                m.color.r, m.color.g, m.color.b, m.color.a = .95, .65, .1, .8
                if kind == 'body':
                    m.type = Marker.CYLINDER
                    m.scale.x, m.scale.y, m.scale.z = .45, .45, 1.7
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    # Separate Pose object: do not change the cylinder position.
                    m.pose = Pose()
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 1.85
                    m.pose.orientation.w = 1.
                    m.scale.z = .22
                    m.text = f"ID {track['track_id']} / p={track['person_score']:.2f}"
                markers.markers.append(m)
        self.pose_pub.publish(poses)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = DepthPersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
