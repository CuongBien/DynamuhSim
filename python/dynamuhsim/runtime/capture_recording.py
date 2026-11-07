"""Full ROS messages and simulator labels; no robot command publisher."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import numpy as np

HERE = Path(__file__).resolve().parents[3] / "configs/capture"
TOPICS = [
    '/clock', '/scan', '/imu', '/odom', '/tf', '/tf_static', '/cmd_vel',
    '/camera/depth/image_rect_raw', '/camera/camera_info',
    '/common/ground_truth', '/dataset/scene_poses', '/dataset/events',
    '/plan', '/amcl_pose', '/map', '/navigate_to_pose/_action/status',
]


def validate_recorded_messages(bag):
    """Read the closed MCAP once: required messages must decode and not go back in time."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {t: get_message(types[t]) for t in TOPICS[:12] if t in types}
    previous, counts, issues = {}, {}, []
    issue_count = 0
    def check(key, value):
        nonlocal issue_count
        if value < 0 or value < previous.get(key, 0):
            issue_count += 1
            if len(issues) < 20:issues.append(dict(stream=str(key), previous=previous.get(key), stamp=value))
        previous[key] = value
    def stamp(key, value):
        if not 0 <= value.nanosec < 1_000_000_000:
            raise RuntimeError('Invalid ROS timestamp on ' + str(key))
        check(key, value.sec * 1_000_000_000 + value.nanosec)
    while reader.has_next():
        topic, payload, received = reader.read_next()
        if topic not in classes:continue
        counts[topic] = counts.get(topic, 0) + 1
        check(('bag', topic), received)
        msg = deserialize_message(payload, classes[topic])
        if hasattr(msg, 'header'):stamp(topic, msg.header.stamp)
        if topic == '/clock':stamp(topic, msg.clock)
        elif topic == '/tf':
            for transform in msg.transforms:
                stamp((topic, transform.header.frame_id, transform.child_frame_id), transform.header.stamp)
        elif topic in ['/dataset/scene_poses', '/dataset/events', '/common/ground_truth']:
            row = json.loads(msg.data)
            check(topic, row['stamp_ns'] if 'stamp_ns' in row else round(row['time'] * 1e9))
    missing = [t for t in TOPICS[:12] if not counts.get(t)]
    return dict(passed=not missing and issue_count == 0, required_topic_counts=counts,
                missing_topics=missing, timestamp_issue_count=issue_count, timestamp_issues=issues)

class Recording:
    def __init__(self, node, output, *, config_dir=None, extra_topics=(), visualize_scene=False):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.message_type = String
        self.node = node
        self.output = Path(output)
        self.seq = 0
        self.timestamp_errors = []
        self.event_pub = node.create_publisher(String, '/dataset/events', 20)
        self.scene_pub = node.create_publisher(String, '/dataset/scene_poses', 50)
        self.marker_pub = None
        if visualize_scene:
            from visualization_msgs.msg import Marker, MarkerArray
            self.marker_type, self.marker_array_type = Marker, MarkerArray
            self.marker_pub = node.create_publisher(MarkerArray, '/dataset/gazebo_markers', 10)
        self.scene_count = 0
        self.last_scene_stamp = None
        self.observed = {}
        from sensor_msgs.msg import Image, CameraInfo
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import qos_profile_sensor_data
        self.last_clock_ns = None
        node.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        for topic, kind in [('/camera/depth/image_rect_raw', Image),
                            ('/camera/camera_info', CameraInfo)]:
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(kind, topic, lambda msg, t=topic: self.witness(t, msg), qos)
        config_dir = HERE if config_dir is None else Path(config_dir)
        self.log = (self.output / 'recorder.log').open('w')
        self.topics = [*TOPICS, *extra_topics]
        cmd = ['ros2', 'bag', 'record', '--storage', 'mcap', '--use-sim-time',
               '--disable-keyboard-controls', '--node-name', 'dataset_pilot_recorder',
               '--storage-config-file', str(config_dir / 'storage.yaml'),
               '--qos-profile-overrides-path', str(config_dir / 'qos.yaml'),
               '--max-cache-size', '33554432', '--include-hidden-topics',
               '--output', str(self.output / 'raw_bag'), '--topics', *self.topics]
        self.process = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self.command = cmd

    def witness(self, topic, msg):
        value = self.observed.setdefault(topic, {'count': 0})
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp < value.get('stamp_ns', 0):
            self.timestamp_errors.append(dict(topic=topic, previous=value['stamp_ns'], stamp_ns=stamp))
        value.update(count=value['count'] + 1, frame_id=msg.header.frame_id,
                     stamp_ns=msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        if hasattr(msg, 'encoding'):
            value.update(encoding=msg.encoding, width=msg.width, height=msg.height,
                         step=msg.step, bytes=len(msg.data))
        else:
            value.update(k=list(msg.k), width=msg.width, height=msg.height,
                         distortion_model=msg.distortion_model)

    def clock(self, msg):
        stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.last_clock_ns is not None and stamp < self.last_clock_ns:
            self.timestamp_errors.append(dict(topic='/clock', previous=self.last_clock_ns, stamp_ns=stamp))
        self.last_clock_ns = stamp

    def readiness(self):
        return dict(missing_publishers=[t for t in TOPICS[:12] if not self.node.get_publishers_info_by_topic(t)],
                    missing_recorder_subscriptions=[t for t in TOPICS[:12] if not any(
                        e.node_name == 'dataset_pilot_recorder' for e in self.node.get_subscriptions_info_by_topic(t))],
                    observed=self.observed, scene_count=self.scene_count)

    def ready(self, *, required_topics=None):
        if self.process.poll() is not None:
            raise RuntimeError('Recorder exited; see recorder.log')
        if not all(self.observed.get(t, {}).get('count', 0) >= 5
                   for t in ['/camera/depth/image_rect_raw', '/camera/camera_info']):
            return False
        if self.timestamp_errors:
            raise RuntimeError('Sensor timestamps regressed; preserve this episode')
        required = TOPICS[:12] if required_topics is None else required_topics
        if not all(self.node.get_publishers_info_by_topic(t) for t in required):
            return False
        return self.scene_count >= 5 and all(any(e.node_name == 'dataset_pilot_recorder'
                       for e in self.node.get_subscriptions_info_by_topic(t))
                   for t in required)

    def event(self, name, stamp_ns, **details):
        self.seq += 1
        row = dict(schema='social_episode/1', seq=self.seq, stamp_ns=int(stamp_ns),
                   event=name, **details)
        self.event_pub.publish(self.message_type(data=json.dumps(row, allow_nan=False)))
        with (self.output / 'events.jsonl').open('a') as f:
            f.write(json.dumps(row, allow_nan=False) + '\n')

    def scene(self, stamp_ns, entities, phase):
        if 'mecanum_robot' not in entities:
            return
        if self.last_scene_stamp is not None and stamp_ns < self.last_scene_stamp:
            self.timestamp_errors.append(dict(topic='/dataset/scene_poses', previous=self.last_scene_stamp, stamp_ns=stamp_ns))
        self.last_scene_stamp = stamp_ns
        self.scene_count += 1
        humans = []
        for name, pose in entities.items():
            if name.startswith('common_') and name.endswith('_proxy'):
                humans.append(dict(id=int(name[len('common_'):-len('_proxy')]), **pose))
        row = dict(schema='social_scene/1', stamp_ns=int(stamp_ns), frame_id='world',
                   robot=entities['mecanum_robot'], humans=sorted(humans, key=lambda h: h['id']),
                   phase=phase, source='gazebo_pose_info', persona=None,
                   persona_valid=False, persona_reason='backend_personas_not_validated')
        self.scene_pub.publish(self.message_type(data=json.dumps(row, allow_nan=False)))
        if self.marker_pub is not None:
            markers = []
            for index, (label, pose) in enumerate([('Gazebo robot', row['robot'])] +
                                                  [('Human ' + str(h['id']), h) for h in row['humans']]):
                marker = self.marker_type()
                marker.header.frame_id = 'world'
                marker.header.stamp.sec, marker.header.stamp.nanosec = divmod(int(stamp_ns), 1_000_000_000)
                marker.ns = 'gazebo_ground_truth'
                marker.id = index
                marker.type = marker.CUBE if index == 0 else marker.CYLINDER
                marker.action = marker.ADD
                marker.pose.position.x, marker.pose.position.y = float(pose['x']), float(pose['y'])
                marker.pose.position.z = .08 if index == 0 else .85
                marker.pose.orientation.z = math.sin(pose['yaw'] / 2)
                marker.pose.orientation.w = math.cos(pose['yaw'] / 2)
                marker.scale.x, marker.scale.y, marker.scale.z = (.24, .20, .08) if index == 0 else (.72, .72, 1.7)
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0., 1., .2, .6) if index == 0 else (1., .5, .1, .6)
                markers.append(marker)
            self.marker_pub.publish(self.marker_array_type(markers=markers))

    def close(self):
        escalated = False
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                escalated = True
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
        self.log.close()
        import yaml
        metadata_path = self.output / 'raw_bag/metadata.yaml'
        counts = {}
        if metadata_path.exists():
            metadata = yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information']
            counts = {row['topic_metadata']['name']: row['message_count'] for row in metadata['topics_with_message_count']}
        return dict(returncode=self.process.returncode, command=self.command,
                    shutdown_escalated=escalated, timestamp_errors=self.timestamp_errors,
                    scene_count=self.scene_count,
                    topic_message_counts=counts,
                    missing_required_messages=[t for t in TOPICS[:12] if counts.get(t, 0) == 0],
                    observed=self.observed,
                    metadata_present=metadata_path.exists(),
                    topics=getattr(self, "topics", TOPICS))


# ON/OFF sidecars retain the original float32 payload and duplicate-stamp policy.
class Pairs:
    def __init__(self,node,output,phase):
        from sensor_msgs.msg import Image
        from rclpy.qos import qos_profile_sensor_data
        self.output=Path(output)/'paired_depth';self.phase=phase;self.rows={name:{} for name in ['on','off']}
        for name,topic in [('on','/proxy_check/on/depth'),('off','/camera/depth/image_rect_raw')]:
            (self.output/name).mkdir(parents=True)
            node.create_subscription(Image,topic,lambda msg,k=name:self.receive(k,msg),qos_profile_sensor_data)
    def receive(self,name,msg):
        assert msg.encoding=='32FC1' and (msg.height,msg.width)==(240,424)
        assert msg.header.frame_id=='camera_optical_frame'
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if stamp in self.rows[name]:return
        array=np.frombuffer(bytes(msg.data),dtype='>f4' if msg.is_bigendian else '<f4').reshape(msg.height,msg.step//4)[:,:msg.width].astype(np.float32)
        np.save(self.output/name/(str(stamp)+'.npy'),array,allow_pickle=False)
        self.rows[name][stamp]=dict(stamp_ns=stamp,phase=self.phase(),frame_id=msg.header.frame_id)
    def ready(self):return all(len(v)>=5 for v in self.rows.values())
    def close(self):
        (self.output/'index.json').write_text(json.dumps(self.rows,indent=2)+'\n')
        return dict(on_count=len(self.rows['on']),off_count=len(self.rows['off']),paired_stamps=sorted(set(self.rows['on'])&set(self.rows['off'])))
