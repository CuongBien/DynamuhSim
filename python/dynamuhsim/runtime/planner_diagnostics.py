"""Opt-in observation of native costmaps. Never publishes commands or changes planner parameters."""
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path
import zlib

TOPICS=('/global_costmap/costmap_raw','/local_costmap/costmap_raw')
PLANNER_PARAMETERS={'/planner_server':{'GridBased.plugin':'nav2_navfn_planner::NavfnPlanner','GridBased.use_astar':False,'GridBased.allow_unknown':True,'GridBased.tolerance':.5}}

def costmap_row(msg):
    meta=msg.metadata
    raw=bytes(msg.data)
    if len(raw)!=meta.size_x*meta.size_y:raise ValueError('Native costmap dimensions differ from payload')
    pose=meta.origin;p=pose.position;q=pose.orientation
    ns=lambda t:t.sec*1_000_000_000+t.nanosec
    return dict(stamp_ns=ns(msg.header.stamp),frame=msg.header.frame_id,width=meta.size_x,height=meta.size_y,resolution=meta.resolution,
                origin=dict(x=p.x,y=p.y,z=p.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w),update_ns=ns(meta.update_time),layer=meta.layer,
                sha256=hashlib.sha256(raw).hexdigest(),data=base64.b64encode(zlib.compress(raw)).decode())

def identity(topic,row):
    return (topic,row['stamp_ns'],row['update_ns'],row['sha256'],row['width'],row['height'],row['frame'],row['resolution'],json.dumps(row['origin'],sort_keys=True),row['layer'])

def verify_rows(path,bag_counts):
    rows=[json.loads(s) for s in Path(path).read_text().splitlines()]
    expected=Counter()
    for row in rows:
        raw=zlib.decompress(base64.b64decode(row['data']))
        if len(raw)!=row['width']*row['height'] or hashlib.sha256(raw).hexdigest()!=row['sha256']:return False
        expected[identity(row['topic'],row)]+=1
    return all(any(row['topic']==topic for row in rows) for topic in TOPICS) and not (expected-bag_counts)

class NativeCostmaps:
    def __init__(self,node,output,telemetry):
        from nav2_msgs.msg import Costmap
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        self.output=Path(output)/'native_costmaps.jsonl'
        self.stream=self.output.open('x');self.telemetry=telemetry;self.counts=Counter()
        qos=QoSProfile(depth=2,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions=[node.create_subscription(Costmap,t,lambda msg,t=t:self.receive(t,msg),qos) for t in TOPICS]
    def receive(self,topic,msg):
        row=costmap_row(msg);row.update(schema='native_costmap_snapshot/1',topic=topic,phase=self.telemetry.phase,observed_sim=self.telemetry.simtime())
        self.stream.write(json.dumps(row,allow_nan=False)+'\n');self.stream.flush();self.counts[topic]+=1
    def ready(self):return all(self.counts[t]>=2 for t in TOPICS)
    def close(self):
        self.stream.close()
        return dict(schema='planner_diagnostics/1',topics=list(TOPICS),counts=dict(self.counts),sidecar=self.output.name,sidecar_sha256=hashlib.sha256(self.output.read_bytes()).hexdigest(),native_encoding='uint8 0..255; exact raw bytes, zlib+base64',control_changed=False)
