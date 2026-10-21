"""Read-only AMCL observation QoS; does not reset or update localization."""
import time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped


def observer_qos(mode):
    if mode == 'sensor_data':
        return qos_profile_sensor_data
    if mode != 'transient_local':
        raise ValueError(mode)
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


def publisher_evidence(node):
    return [dict(node_name=e.node_name, node_namespace=e.node_namespace,
                 reliability=e.qos_profile.reliability.name,
                 durability=e.qos_profile.durability.name,
                 depth=e.qos_profile.depth)
            for e in node.get_publishers_info_by_topic('/amcl_pose')]


def late_join_check(node, wait, seconds=3.):
    """Observe the real publisher after its first pose, while robot is stationary."""
    received={'sensor_data': [], 'transient_local': []}
    subscriptions=[]
    for mode in received:
        subscriptions.append(node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            lambda msg, mode=mode: received[mode].append(dict(
                frame_id=msg.header.frame_id,
                stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                x=msg.pose.pose.position.x, y=msg.pose.pose.position.y)), observer_qos(mode)))
    start=time.monotonic()
    try:
        wait(lambda:False, seconds)
        return dict(wall_seconds=time.monotonic()-start, samples=received,
                    publishers=publisher_evidence(node),
                    scope='Read-only late subscriptions before goal/start gate; no force-update or reset')
    finally:
        for sub in subscriptions: node.destroy_subscription(sub)
