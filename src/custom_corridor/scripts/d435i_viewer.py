#!/usr/bin/env python3
"""D435i-style RGB-D viewer, obstacle range publisher, and keyboard teleop."""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32


class D435iViewer(Node):
    """Visualize aligned RGB-D data and drive the robot from one window."""

    CAMERA_OFFSET_X_M = 0.08
    MIN_RANGE_M = 0.195
    MAX_RANGE_M = 10.0
    DEPTH_FOV_RAD = math.radians(87.0)

    def __init__(self):
        super().__init__('d435i_viewer')

        self.declare_parameter('show_gui', True)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('detection_max_range', 3.0)

        self.show_gui = bool(self.get_parameter('show_gui').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.detection_max_range = float(
            self.get_parameter('detection_max_range').value)

        self.bridge = CvBridge()
        self.color_frame = None
        self.depth_m = None
        self.depth_header = None
        self.last_detection = None
        self.last_detection_time = 0.0
        self.command = (0.0, 0.0)
        self.command_until = 0.0
        self.stop_sent = True

        self.create_subscription(
            Image, '/camera/color/image_raw', self.on_color,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.on_depth,
            qos_profile_sensor_data)

        self.range_pub = self.create_publisher(
            Range, '/camera/obstacle/range', 10)
        self.distance_pub = self.create_publisher(
            Float32, '/camera/obstacle/distance', 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.create_timer(1.0 / 20.0, self.update)

        if self.show_gui:
            cv2.namedWindow('D435i RGB-D + Teleop', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('D435i RGB-D + Teleop', 1060, 600)

        self.get_logger().info(
            'D435i viewer ready: W/S forward/back, A/D turn, '
            'X or SPACE stop, Q quit')

    def on_color(self, msg):
        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Cannot convert color image: {exc}')

    def on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'Cannot convert depth image: {exc}')
            return

        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32, copy=False)

        self.depth_m = depth
        self.depth_header = msg.header

    def detect_forward_obstacle(self, depth):
        """Find the nearest robust component in the robot's forward gate.

        The central gate rejects corridor side walls and the lower crop rejects
        the floor. In the supplied scenario, the dynamic human/object crosses
        this gate. Invalid stereo pixels (NaN, Inf, zero) are ignored.
        """
        height, width = depth.shape[:2]
        # D435i is mounted only 0.20 m above the floor. A narrow band around
        # the optical horizon keeps the floor and close corridor side walls
        # out of the range estimate while retaining a person in front.
        x1, x2 = int(width * 0.44), int(width * 0.56)
        y1, y2 = int(height * 0.10), int(height * 0.55)
        roi = depth[y1:y2, x1:x2]

        valid = (
            np.isfinite(roi)
            & (roi >= self.MIN_RANGE_M)
            & (roi <= self.detection_max_range)
        )
        values = roi[valid]
        if values.size < 120:
            return None

        seed = float(np.percentile(values, 8.0))
        mask = valid & (roi <= min(seed + 0.22, self.detection_max_range))
        mask_u8 = mask.astype(np.uint8) * 255
        mask_u8 = cv2.morphologyEx(
            mask_u8, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask_u8 = cv2.morphologyEx(
            mask_u8, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask_u8, connectivity=8)
        candidates = []
        min_area = max(80, int(roi.size * 0.003))
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < min_area:
                continue
            component_depth = roi[(labels == label) & valid]
            if component_depth.size == 0:
                continue
            distance = float(np.percentile(component_depth, 25.0))
            cx = float(centroids[label][0])
            center_penalty = abs(cx - roi.shape[1] / 2.0) / roi.shape[1]
            candidates.append((distance + 0.35 * center_penalty, distance, label))

        if not candidates:
            return None

        _, distance, label = min(candidates)
        left = x1 + int(stats[label, cv2.CC_STAT_LEFT])
        top = y1 + int(stats[label, cv2.CC_STAT_TOP])
        box_width = int(stats[label, cv2.CC_STAT_WIDTH])
        box_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        return distance, (left, top, box_width, box_height), (x1, y1, x2, y2)

    def publish_distance(self, distance):
        range_msg = Range()
        if self.depth_header is not None:
            range_msg.header = self.depth_header
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = self.DEPTH_FOV_RAD
        range_msg.min_range = self.MIN_RANGE_M
        range_msg.max_range = self.MAX_RANGE_M
        range_msg.range = float(distance)
        self.range_pub.publish(range_msg)

        robot_distance = Float32()
        robot_distance.data = float(
            distance + self.CAMERA_OFFSET_X_M
            if math.isfinite(distance) else math.inf)
        self.distance_pub.publish(robot_distance)

    def send_command(self, linear, angular, duration=0.35):
        self.command = (linear, angular)
        self.command_until = time.monotonic() + duration
        self.stop_sent = False
        self.publish_cmd(linear, angular)

    def publish_cmd(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def update_motion(self):
        if time.monotonic() < self.command_until:
            self.publish_cmd(*self.command)
        elif not self.stop_sent:
            self.publish_cmd(0.0, 0.0)
            self.stop_sent = True

    def handle_key(self, key):
        if key in (ord('w'), ord('W')):
            self.send_command(self.linear_speed, 0.0)
        elif key in (ord('s'), ord('S')):
            self.send_command(-self.linear_speed, 0.0)
        elif key in (ord('a'), ord('A')):
            self.send_command(0.0, self.angular_speed)
        elif key in (ord('d'), ord('D')):
            self.send_command(0.0, -self.angular_speed)
        elif key in (ord('x'), ord('X'), ord(' ')):
            self.send_command(0.0, 0.0, duration=0.0)
        elif key in (ord('q'), ord('Q'), 27):
            self.publish_cmd(0.0, 0.0)
            rclpy.shutdown()

    def draw_ui(self, detection):
        if self.color_frame is None:
            return

        frame = self.color_frame.copy()
        height, width = frame.shape[:2]
        status_color = (0, 255, 0)
        status = (
            'No obstacle in central gate '
            f'({self.MIN_RANGE_M:.3f}-{self.detection_max_range:.1f} m)')

        if detection is not None:
            distance, (x, y, w, h), gate = detection
            robot_distance = distance + self.CAMERA_OFFSET_X_M
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 220, 255), 2)
            status = (
                f'Depth: {distance:.2f} m | Robot: {robot_distance:.2f} m')
            status_color = (0, 220, 255)
            gx1, gy1, gx2, gy2 = gate
            cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), (255, 120, 0), 1)

        cv2.rectangle(frame, (0, 0), (width, 76), (25, 25, 25), -1)
        cv2.putText(
            frame, 'Intel RealSense D435i simulation | 848x480 @ 30 FPS',
            (14, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (255, 255, 255), 2)
        cv2.putText(frame, status, (14, 59), cv2.FONT_HERSHEY_SIMPLEX,
                    0.72, status_color, 2)
        cv2.rectangle(frame, (0, height - 43), (width, height), (25, 25, 25), -1)
        cv2.putText(
            frame, 'W/S: forward/back  A/D: turn  X/SPACE: stop  Q: quit',
            (14, height - 14), cv2.FONT_HERSHEY_SIMPLEX,
            0.62, (255, 255, 255), 2)

        if self.depth_m is not None:
            depth = self.depth_m
            if depth.shape[:2] != (height, width):
                depth = cv2.resize(
                    depth, (width, height), interpolation=cv2.INTER_NEAREST)
            normalized = np.clip(depth, self.MIN_RANGE_M, 5.0)
            normalized = (
                (normalized - self.MIN_RANGE_M)
                / (5.0 - self.MIN_RANGE_M) * 255)
            normalized = np.nan_to_num(
                normalized, nan=255.0, posinf=255.0).astype(np.uint8)
            depth_color = cv2.applyColorMap(
                255 - normalized, cv2.COLORMAP_TURBO)
            inset_width, inset_height = 300, 170
            inset = cv2.resize(depth_color, (inset_width, inset_height))
            ix, iy = width - inset_width - 10, 86
            frame[iy:iy + inset_height, ix:ix + inset_width] = inset
            cv2.rectangle(
                frame, (ix, iy), (ix + inset_width, iy + inset_height),
                (255, 255, 255), 1)
            cv2.putText(
                frame, 'Aligned depth (near=red)', (ix + 6, iy + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

        cv2.imshow('D435i RGB-D + Teleop', frame)
        self.handle_key(cv2.waitKey(1) & 0xFF)

    def update(self):
        self.update_motion()
        detection = None
        if self.depth_m is not None:
            detection = self.detect_forward_obstacle(self.depth_m)
            if detection is not None:
                self.last_detection = detection
                self.last_detection_time = time.monotonic()
                self.publish_distance(detection[0])
            elif time.monotonic() - self.last_detection_time < 0.25:
                detection = self.last_detection
                self.publish_distance(detection[0])
            else:
                # sensor_msgs/Range uses +Inf for a valid reading with no
                # return inside max_range. Keep the output topics alive even
                # while the dynamic obstacle is still far away.
                self.publish_distance(math.inf)

        if self.show_gui:
            self.draw_ui(detection)

    def destroy_node(self):
        if rclpy.ok():
            self.publish_cmd(0.0, 0.0)
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = D435iViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
