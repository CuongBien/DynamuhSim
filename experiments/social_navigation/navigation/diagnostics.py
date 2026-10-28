"""Optional passive capture; no publishers, parameter changes or controller calls."""
from collections import Counter
import json
from pathlib import Path
import threading
import time
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from geometry_msgs.msg import TwistStamped, PolygonStamped
from tf2_msgs.msg import TFMessage

class Capture:
    def __init__(self,path):
        self.path=Path(path)
        self.file=self.path.open('w')
        self.lock=threading.Lock()
        self.counts=Counter()
        self.last_gz=None
        self.subscriptions=[]

    def write(self,kind,data,clock=None):
        receipt=None if clock is None else clock.clock.sec+clock.clock.nanosec/1e9
        row=dict(kind=kind,receipt_sim=receipt,receipt_monotonic=time.monotonic(),data=data)
        encoded=json.dumps(row,separators=(',',':'))+'\n'
        with self.lock:
            self.file.write(encoded)
            self.counts[kind]+=1

    def ros(self,kind,msg,clock=None):
        self.write(kind,message_to_ordereddict(msg),clock)

    def gazebo(self,t,poses):
        if self.last_gz is not None and t-self.last_gz<.019: return
        self.last_gz=t
        self.write('gazebo',dict(time=t,poses={k:v.copy() for k,v in poses.items()}))

    def subscribe(self,node,clock):
        static=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
        for key,topic,kind,qos in [
            ('tf','/tf',TFMessage,qos_profile_sensor_data),
            ('tf_static','/tf_static',TFMessage,static),
            ('costmap','/local_costmap/costmap',OccupancyGrid,qos_profile_sensor_data),
            ('footprint','/local_costmap/published_footprint',PolygonStamped,qos_profile_sensor_data),
            ('cmd_vel','/cmd_vel',TwistStamped,qos_profile_sensor_data),
            ('local_plan','/local_plan',NavPath,qos_profile_sensor_data)]:
            self.subscriptions.append(node.create_subscription(kind,topic,
                lambda msg,key=key:self.ros(key,msg,clock()),qos))

    def close(self):
        with self.lock: self.file.close()
