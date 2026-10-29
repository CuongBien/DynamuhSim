"""Raw exporter boundaries without ROS: calibration, bitwise depth and invalid frames."""
import copy,importlib.util,json,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'));spec=importlib.util.spec_from_file_location('handoff_export',ROOT/'experiments/social_navigation/datasets/handoff_pilot/export.py');export=importlib.util.module_from_spec(spec);spec.loader.exec_module(export)

def fixture():
 def xyz(x=0.,y=0.,z=0.):return NS(x=x,y=y,z=z)
 q=NS(x=0.,y=0.,z=0.,w=1.);pose=NS(position=xyz(),orientation=q)
 header=lambda frame:NS(frame_id=frame,stamp=NS(sec=1,nanosec=0))
 tf=lambda child,parent,t,quat:NS(child_frame_id=child,header=header(parent),transform=NS(translation=xyz(*t),rotation=NS(x=quat[0],y=quat[1],z=quat[2],w=quat[3])))
 depth=np.ones((240,424),np.float32);depth.view(np.uint32)[0,0]=0x7fc00042;depth[0,1]=np.inf
 messages=[('/clock',NS(clock=NS(sec=0,nanosec=1))),('/clock',NS(clock=NS(sec=1,nanosec=0))),('/tf_static',NS(transforms=[tf('base_link','base_footprint',[0,0,0],[0,0,0,1]),tf('camera_link','base_link',[.12,0,.08],[0,0,0,1]),tf('camera_optical_frame','camera_link',[0,0,0],[-.5,.5,-.5,.5])])),('/camera/camera_info',NS(header=header('camera_optical_frame'),height=240,width=424,k=[223.,0,212,0,223.,120,0,0,1],p=[0.]*12,d=[0.]*5,distortion_model='plumb_bob')),('/camera/depth/image_rect_raw',NS(header=header('camera_optical_frame'),height=240,width=424,step=1696,encoding='32FC1',is_bigendian=False,data=depth.tobytes())),('/scan',NS(header=header('base_scan'),ranges=[1.,float('inf'),float('nan')],range_min=.1,range_max=5.,angle_min=-3.14,angle_max=3.14,angle_increment=.1,scan_time=.2,time_increment=0.)),('/imu',NS(header=header('camera_imu_frame'),orientation=q,orientation_covariance=[-1.]+[0.]*8,angular_velocity=xyz(),linear_acceleration=xyz(z=9.8))),('/odom',NS(header=header('odom'),pose=NS(pose=pose),twist=NS(twist=NS(linear=xyz(),angular=xyz())))),('/cmd_vel',NS(header=header('base_footprint'),twist=NS(linear=xyz(x=.2),angular=xyz()))),('/amcl_pose',NS(header=header('map'),pose=NS(pose=pose))),('/dataset/scene_poses',NS(data=json.dumps(dict(stamp_ns=1000000000,humans=[dict(id=i,x=0.,y=0.) for i in range(1,7)],robot=dict(x=0.,y=0.,yaw=0.),persona=None,persona_valid=False)))),('/common/ground_truth',NS(data=json.dumps(dict(time=1.,groups=[]))))]
 return messages,depth

def read(messages):
 class Reader:
  def __init__(self):self.values=iter([(t,m,999000000) for t,m in messages]);self.next=None
  def open(self,*args):pass
  def get_all_topics_and_types(self):return [NS(name=t,type='fake') for t in {t for t,m in messages}]
  def has_next(self):
   try:self.next=next(self.values);return True
   except StopIteration:return False
  def read_next(self):return self.next
 modules={'rosbag2_py':NS(SequentialReader=Reader,StorageOptions=lambda **k:None,ConverterOptions=lambda *a:None),'rclpy':NS(__path__=[]),'rclpy.serialization':NS(deserialize_message=lambda data,kind:data),'rosidl_runtime_py':NS(__path__=[]),'rosidl_runtime_py.utilities':NS(get_message=lambda kind:object)}
 with patch.dict(sys.modules,modules):return export.read_raw(Path('/unused'))

class HandoffRawTests(unittest.TestCase):
 def test_nan_payload_depth_masks_availability_and_tf_preserved(self):
  messages,original=fixture();mods,depth,cal,*_=read(messages);self.assertEqual(original.tobytes(),depth[0].tobytes());np.testing.assert_array_equal(mods['scan']['valid'],[[True,False,False]]);np.testing.assert_array_equal(mods['scan']['values'],[[1,0,0]]);self.assertFalse(mods['imu']['orientation_valid'][0]);self.assertEqual(mods['depth']['available_ns'][0],1000000000);np.testing.assert_allclose(np.array(cal['optical_to_base_link'])[:3,3],[.12,0,.08]);np.testing.assert_allclose(np.array(cal['optical_to_base_link'])[:3,:3],[[0,0,1],[-1,0,0],[0,-1,0]])
 def test_wrong_command_frame_rejected(self):
  messages,_=fixture();next(m for t,m in messages if t=='/cmd_vel').header.frame_id='world'
  with self.assertRaisesRegex(ValueError,'Action planar frame'):read(messages)
 def test_conflicting_static_calibration_rejected(self):
  messages,_=fixture();tf=next(m for t,m in messages if t=='/tf_static');other=copy.deepcopy(tf.transforms[1]);other.transform.translation.x=.13;tf.transforms.append(other)
  with self.assertRaisesRegex(ValueError,'Conflicting static TF'):read(messages)
 def test_wrong_depth_frame_rejected(self):
  messages,_=fixture();next(m for t,m in messages if t=='/camera/depth/image_rect_raw').header.frame_id='camera_link'
  with self.assertRaisesRegex(ValueError,'Depth encoding/shape/frame'):read(messages)
