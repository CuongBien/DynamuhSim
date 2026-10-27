#!/usr/bin/env python3
"""
ROS 2 Node for Real-Time Depth Person Detection & Tracking (Direction 2).
Subscribes:
  - /camera/depth/image_rect_raw (sensor_msgs/Image)
  - /odom (nav_msgs/Odometry)
Publishes:
  - /perception/tracked_persons_markers (visualization_msgs/MarkerArray)
  - /perception/person_tracks (geometry_msgs/PoseArray)
Uses:
  - Generalized 3D Euclidean clustering + depth_person_classifier
  - IdentityGuard (causal ID preservation)
"""
import math
from pathlib import Path
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker

# Add benchmark modules to path
REPO_ROOT = Path(__file__).resolve().parents[3]
BENCHMARK = REPO_ROOT / "experiments/social_navigation/benchmark"
sys.path.insert(0, str(BENCHMARK / "depth_identity_guard"))
sys.path.insert(0, str(BENCHMARK / "depth_person_classifier/artifacts"))

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from integrated_pipeline import EndToEndPerceptionPipeline


class DepthPersonTrackerNode(Node):
    def __init__(self):
        super().__init__('depth_person_tracker_node')

        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('min_person_score', 0.45)
        self.declare_parameter('stride', 4)

        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.min_person_score = float(self.get_parameter('min_person_score').value)
        self.stride = int(self.get_parameter('stride').value)

        # Standard Intel RealSense D435i Intrinsic Matrix (640x480)
        # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_k = np.array([
            386.0, 0.0, 320.0,
            0.0, 386.0, 240.0,
            0.0, 0.0, 1.0
        ], dtype=float)

        # Camera optical frame to base_link transform matrix (4x4)
        # Jackal / D435i default mounting: facing forward X, height ~0.28m
        self.optical_to_base = np.array([
            [0.0, 0.0, 1.0, 0.08],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.28],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.pipeline = EndToEndPerceptionPipeline(min_person_score=self.min_person_score)

        self.robot_pose_odom = np.zeros(3)  # [x, y, yaw]
        self.odom_received = False

        # Subscribers
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.on_odom,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            self.depth_topic,
            self.on_depth,
            qos_profile_sensor_data
        )

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/tracked_persons_markers', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/perception/person_tracks', 10)

        self.get_logger().info('DepthPersonTrackerNode initialized (Direction 2: 3D KDTree + IdentityGuard).')

    def on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.robot_pose_odom = np.array([pos.x, pos.y, yaw], dtype=float)
        self.odom_received = True

    def on_depth(self, msg: Image):
        if not self.odom_received:
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # Decode image buffer to depth array in meters
        if self.bridge is not None:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        else:
            # Fallback raw buffer conversion (16UC1 or 32FC1)
            if msg.encoding == '16UC1':
                depth_img = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
                depth_img = depth_img.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            else:
                return

        # Replace invalid depth values
        depth_m = np.nan_to_num(depth_img, nan=0.0, posinf=0.0, neginf=0.0)

        # Process frame through integrated pipeline
        tracks = self.pipeline.process_depth_frame(
            stamp_ns=stamp_ns,
            depth_array=depth_m,
            camera_k=self.camera_k,
            optical_to_base_tf=self.optical_to_base,
            robot_pose_odom=self.robot_pose_odom,
            stride=self.stride,
            filter_non_person=True
        )

        self.publish_tracks(msg.header, tracks)

    def publish_tracks(self, header, tracks):
        marker_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header.stamp = header.stamp
        pose_array.header.frame_id = 'odom'

        # Clear old markers
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        for track in tracks:
            # Only visualize confirmed tracks or valid detections
            xy_odom = track['xy_odom']
            track_id = track['track_id']
            is_confirmed = track['confirmed']

            # Pose message
            p = Pose()
            p.position.x = float(xy_odom[0])
            p.position.y = float(xy_odom[1])
            p.position.z = 0.85
            pose_array.poses.append(p)

            # Cylinder body marker
            m = Marker()
            m.header.stamp = header.stamp
            m.header.frame_id = 'odom'
            m.ns = 'persons'
            m.id = int(abs(track_id))
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(xy_odom[0])
            m.pose.position.y = float(xy_odom[1])
            m.pose.position.z = 0.85
            m.scale.x = 0.45
            m.scale.y = 0.45
            m.scale.z = 1.70

            if track_id > 0 and is_confirmed:
                m.color.r = 0.1
                m.color.g = 0.8
                m.color.b = 0.2
                m.color.a = 0.85
            else:
                m.color.r = 0.9
                m.color.g = 0.6
                m.color.b = 0.1
                m.color.a = 0.65

            marker_array.markers.append(m)

            # ID text marker
            t = Marker()
            t.header.stamp = header.stamp
            t.header.frame_id = 'odom'
            t.ns = 'person_ids'
            t.id = int(abs(track_id)) + 10000
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(xy_odom[0])
            t.pose.position.y = float(xy_odom[1])
            t.pose.position.z = 1.85
            t.scale.z = 0.25
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 1.0
            t.text = f"ID: {track_id}"
            marker_array.markers.append(t)

        self.marker_pub.publish(marker_array)
        self.pose_pub.publish(pose_array)


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

