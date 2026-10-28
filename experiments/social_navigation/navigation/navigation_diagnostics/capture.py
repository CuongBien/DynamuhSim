"""Extra passive subscribers. Global grids are losslessly packed to limit I/O."""
import base64
from collections import Counter
import time
import zlib
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.msg import ParticleCloud
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.msg import Log
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data

TOPICS = [
    ('global_costmap', '/global_costmap/costmap', OccupancyGrid),
    ('global_plan', '/plan', Path),
    ('particle_cloud', '/particle_cloud', ParticleCloud),
    ('navigation_status', '/navigate_to_pose/_action/status', GoalStatusArray),
    ('follow_path_status', '/follow_path/_action/status', GoalStatusArray),
    ('behavior_spin_status', '/spin/_action/status', GoalStatusArray),
    ('behavior_backup_status', '/backup/_action/status', GoalStatusArray),
    ('navigation_rosout', '/rosout', Log),
]

class NavigationCapture:
    def __init__(self, capture, node, clock):
        self.capture, self.node, self.clock = capture, node, clock
        self.counts = Counter()
        self.callbacks_wall_seconds = 0.
        self.callback_wall_max_seconds = 0.
        self.subscriptions = []
        for key, topic, kind in TOPICS:
            # rclpy inspects callback arity; use a factory, not a third default arg.
            def make_callback(key):
                def callback(msg, info):
                    begin = time.monotonic()
                    if key == 'global_costmap':
                        data = dict(header=message_to_ordereddict(msg.header),
                            info=message_to_ordereddict(msg.info),
                            data_encoding='zlib_base64_int8',
                            data_b64=base64.b64encode(zlib.compress(msg.data.tobytes(), 1)).decode('ascii'))
                    else:
                        data = message_to_ordereddict(msg)
                    data['_message_info'] = {k: (bytes(v).hex() if isinstance(v, (bytes, bytearray)) else v) for k, v in info.items()}
                    data['_publisher_gid'] = bytes(info['publisher_gid']).hex() if 'publisher_gid' in info else None
                    data['_source_timestamp_ns'] = info.get('source_timestamp')
                    self.capture.write(key, data, self.clock())
                    self.counts[key] += 1
                    elapsed = time.monotonic() - begin
                    self.callbacks_wall_seconds += elapsed
                    self.callback_wall_max_seconds = max(self.callback_wall_max_seconds, elapsed)
                return callback
            self.subscriptions.append(node.create_subscription(kind, topic,
                make_callback(key), qos_profile_sensor_data))

    def graph(self, phase):
        publishers = {}
        for topic in [t for _, t, _ in TOPICS] + ['/amcl_pose', '/tf', '/tf_static', '/scan', '/odom']:
            publishers[topic] = [dict(node_name=e.node_name, node_namespace=e.node_namespace,
                topic_type=e.topic_type, endpoint_gid=bytes(e.endpoint_gid).hex(),
                reliability=str(e.qos_profile.reliability), durability=str(e.qos_profile.durability),
                depth=e.qos_profile.depth) for e in self.node.get_publishers_info_by_topic(topic)]
        self.capture.write('navigation_graph', dict(phase=phase, publishers=publishers), self.clock())

    def summary(self):
        return dict(counts=dict(self.counts), callbacks_wall_seconds=self.callbacks_wall_seconds,
            callback_wall_max_seconds=self.callback_wall_max_seconds,
            global_grid_encoding='lossless zlib_base64_int8',
            particle_cloud_encoding='full native nav2_msgs/ParticleCloud message',
            policy='Subscribers only; no parameters, services, publishers or action goals added')
