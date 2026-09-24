#!/usr/bin/env python3
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from sensor_msgs.msg import Image


WINDOW = 'School Robot Camera - WASD to drive, X/K to stop, Q to quit'


class CameraTeleop(Node):
    def __init__(self):
        super().__init__('school_camera_teleop')
        self.declare_parameter('image_topic', '/robot/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 0.9)

        image_topic = self.get_parameter('image_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.bridge = CvBridge()
        self.frame = None
        self.frame_lock = threading.Lock()
        self.last_image_time = 0.0
        self.last_unpause_attempt = 0.0
        self.linear = 0.0
        self.angular = 0.0
        self.publisher = self.create_publisher(Twist, cmd_topic, 10)
        self.control_client = self.create_client(
            ControlWorld, '/world/school_arena/control'
        )
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.create_timer(0.1, self._publish_cmd)

        self.get_logger().info(f'Camera: {image_topic}')
        self.get_logger().info(f'Robot command: {cmd_topic}')

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Cannot decode camera image: {exc}')
            return
        with self.frame_lock:
            self.frame = frame
            self.last_image_time = time.monotonic()

    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher.publish(msg)
        self._ensure_world_running()

    def _ensure_world_running(self):
        now = time.monotonic()
        camera_stale = (
            self.last_image_time == 0.0
            or now - self.last_image_time > 1.0
        )
        if not camera_stale or now - self.last_unpause_attempt < 1.0:
            return
        if not self.control_client.service_is_ready():
            return
        req = ControlWorld.Request()
        req.world_control.pause = False
        self.control_client.call_async(req)
        self.last_unpause_attempt = now

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0
        for _ in range(3):
            self._publish_cmd()

    def handle_key(self, key):
        key &= 0xFF
        if key in (ord('w'), ord('W')):
            self.linear = self.linear_speed
            self.angular = 0.0
        elif key in (ord('s'), ord('S')):
            self.linear = -self.linear_speed
            self.angular = 0.0
        elif key in (ord('a'), ord('A')):
            self.linear = 0.0
            self.angular = self.angular_speed
        elif key in (ord('d'), ord('D')):
            self.linear = 0.0
            self.angular = -self.angular_speed
        elif key in (ord('x'), ord('X'), ord('k'), ord('K')):
            self.linear = 0.0
            self.angular = 0.0
        elif key in (ord('q'), ord('Q'), 27):
            return False
        return True

    def display_frame(self):
        with self.frame_lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            frame = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(
                frame, 'Waiting for /robot/camera/image_raw ...',
                (55, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 200, 255), 2, cv2.LINE_AA,
            )
        stale = (
            self.last_image_time == 0.0
            or time.monotonic() - self.last_image_time > 1.0
        )
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 78), (20, 20, 20), -1)
        cv2.putText(
            frame, 'W: forward  S: back  A/D: turn  X/K: stop  Q: quit',
            (12, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
            (255, 255, 255), 1, cv2.LINE_AA,
        )
        cv2.putText(
            frame, f'linear={self.linear:+.2f} m/s   angular={self.angular:+.2f} rad/s',
            (12, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
            (80, 255, 120), 1, cv2.LINE_AA,
        )
        if stale:
            cv2.putText(
                frame, 'CAMERA STALE - requesting Gazebo unpause...',
                (12, 69), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (0, 80, 255), 2, cv2.LINE_AA,
            )
        cv2.imshow(WINDOW, frame)


def main():
    rclpy.init()
    node = CameraTeleop()
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 960, 540)
    try:
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.01)
            node.display_frame()
            key = cv2.waitKeyEx(1)
            if key >= 0:
                running = node.handle_key(key)
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
