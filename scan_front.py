import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FrontObstacle(Node):
    def __init__(self):
        super().__init__('front_obstacle')
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        distances = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Chỉ lấy vùng phía trước: -10° đến +10°
            if abs(angle) > math.radians(10):
                continue

            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                distances.append(r)

        if distances:
            print(f"\rFront obstacle: {min(distances):.2f} m", end="", flush=True)

rclpy.init()
rclpy.spin(FrontObstacle())
rclpy.shutdown()
