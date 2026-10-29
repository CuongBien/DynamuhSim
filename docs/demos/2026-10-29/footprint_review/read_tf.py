from pathlib import Path
import json
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
BASE=Path(__file__).resolve().parent.parent
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(BASE/'capture/episode/raw_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
found={}
while reader.has_next():
 topic,payload,stamp=reader.read_next()
 if topic!='/tf_static':continue
 msg=deserialize_message(payload,TFMessage)
 for t in msg.transforms:
  if t.child_frame_id in ['base_link','camera_link','camera_optical_frame']:
   q=t.transform.translation;found[t.child_frame_id]={'parent':t.header.frame_id,'translation':[q.x,q.y,q.z]}
 if len(found)==3:break
(Path(__file__).resolve().parent/'actual_tf_static.json').write_text(json.dumps(found,indent=2)+'\n');print(json.dumps(found,indent=2))
