"""Full ROS messages and simulator labels; no robot command publisher."""
import json
import os
from pathlib import Path
import signal
import subprocess
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

HERE = Path(__file__).resolve().parent
TOPICS = [
    '/clock', '/scan', '/imu', '/odom', '/tf', '/tf_static', '/cmd_vel',
    '/camera/depth/image_rect_raw', '/camera/camera_info',
    '/common/ground_truth', '/dataset/scene_poses', '/dataset/events',
    '/plan', '/amcl_pose', '/map', '/navigate_to_pose/_action/status',
]

class Recording:
    def __init__(self, node, output):
        self.node = node
        self.output = Path(output)
        self.seq = 0
        self.event_pub = node.create_publisher(String, '/dataset/events', 20)
        self.scene_pub = node.create_publisher(String, '/dataset/scene_poses', 50)
        self.observed = {}
        from sensor_msgs.msg import Image, CameraInfo
        for topic, kind in [('/camera/depth/image_rect_raw', Image),
                            ('/camera/camera_info', CameraInfo)]:
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(kind, topic, lambda msg, t=topic: self.witness(t, msg), qos)
        self.log = (self.output / 'recorder.log').open('w')
        cmd = ['ros2', 'bag', 'record', '--storage', 'mcap', '--use-sim-time',
               '--disable-keyboard-controls', '--node-name', 'dataset_pilot_recorder',
               '--storage-config-file', str(HERE / 'storage.yaml'),
               '--qos-profile-overrides-path', str(HERE / 'qos.yaml'),
               '--max-cache-size', '33554432', '--include-hidden-topics',
               '--output', str(self.output / 'raw_bag'), '--topics', *TOPICS]
        self.process = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self.command = cmd

    def witness(self, topic, msg):
        value = self.observed.setdefault(topic, {'count': 0})
        value.update(count=value['count'] + 1, frame_id=msg.header.frame_id,
                     stamp_ns=msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        if hasattr(msg, 'encoding'):
            value.update(encoding=msg.encoding, width=msg.width, height=msg.height,
                         step=msg.step, bytes=len(msg.data))
        else:
            value.update(k=list(msg.k), width=msg.width, height=msg.height,
                         distortion_model=msg.distortion_model)

    def ready(self):
        if self.process.poll() is not None:
            raise RuntimeError('Recorder exited; see recorder.log')
        if not all(self.observed.get(t, {}).get('count', 0) >= 5
                   for t in ['/camera/depth/image_rect_raw', '/camera/camera_info']):
            return False
        return all(any(e.node_name == 'dataset_pilot_recorder'
                       for e in self.node.get_subscriptions_info_by_topic(t))
                   for t in TOPICS[:12])

    def event(self, name, stamp_ns, **details):
        self.seq += 1
        row = dict(schema='social_episode/1', seq=self.seq, stamp_ns=int(stamp_ns),
                   event=name, **details)
        self.event_pub.publish(String(data=json.dumps(row, allow_nan=False)))
        with (self.output / 'events.jsonl').open('a') as f:
            f.write(json.dumps(row, allow_nan=False) + '\n')

    def scene(self, stamp_ns, entities, phase):
        if 'mecanum_robot' not in entities:
            return
        humans = []
        for name, pose in entities.items():
            if name.startswith('common_') and name.endswith('_proxy'):
                humans.append(dict(id=int(name[len('common_'):-len('_proxy')]), **pose))
        row = dict(schema='social_scene/1', stamp_ns=int(stamp_ns), frame_id='world',
                   robot=entities['mecanum_robot'], humans=sorted(humans, key=lambda h: h['id']),
                   phase=phase, source='gazebo_pose_info', persona=None,
                   persona_valid=False, persona_reason='backend_personas_not_validated')
        self.scene_pub.publish(String(data=json.dumps(row, allow_nan=False)))

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
        self.log.close()
        return dict(returncode=self.process.returncode, command=self.command,
                    observed=self.observed,
                    metadata_present=(self.output / 'raw_bag/metadata.yaml').exists(),
                    topics=TOPICS)
