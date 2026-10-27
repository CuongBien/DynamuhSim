"""Observed ON/OFF images saved at original float32 precision, no command publisher."""
import json,sys
from pathlib import Path
import numpy as np
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
class Pairs:
    def __init__(self,node,output,phase):
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
