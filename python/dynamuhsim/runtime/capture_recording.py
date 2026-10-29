"""Full ROS messages and simulator labels; no robot command publisher."""
import json
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

class Recording:
    def __init__(self, node, output, *, config_dir=None, extra_topics=()):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.message_type = String
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
        self.event_pub.publish(self.message_type(data=json.dumps(row, allow_nan=False)))
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
        self.scene_pub.publish(self.message_type(data=json.dumps(row, allow_nan=False)))

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
