"""Middleware replay covers synchronization, float32 encoding and owned teardown."""
import contextlib,importlib.util,io,json,sys,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT)]
from dynamuhsim.runtime import sensor_capture as capture
PROFILES=['depth_visibility_pilot','lidar_person_pilot','depth_surface_guard','depth_perception_corpus']

def replay(profile, *, source=None, moving=False, fault=None, condition='full'):
 with tempfile.TemporaryDirectory(dir=ROOT) as directory,contextlib.ExitStack() as stack:
  root=Path(directory)/"benchmark";here=root/profile;here.mkdir(parents=True);(root/'common_gazebo/generated').mkdir(parents=True)
  modes=['full','control','drop1'];protocol=dict(modes=modes,warmup_ns=2_000_000_000,max_wall_s=30,frames=4,static_depth_tolerance_m=.001,min_scans=20,scan_topic='/person/scan',motion_proxy_actor_tolerance_m=.01,motion_actual_min_m=.1)
  for name in ['protocol.json','capture_protocol.json']:(here/name).write_text(json.dumps(protocol))
  for name in ['fixture.py','visibility_filter.cc','snapshot.json','generated/build/libvisibility_filter.so','generated/build/libcorpus_motion.so','camera_height_ab/fixture.py','camera_height_ab/freeze_actors.cc','camera_height_ab/generated/build/libfreeze_actors.so','lidar_person_pilot/fixture.py']:
   p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('source')
  for name in ['gazebo_adapter/run.py','warehouse/warehouse.sdf']:
   p=root.parent/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('source')
  for name in ['libvisibility_filter.so','libcorpus_motion.so']:
   p=here/'generated/build'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('source')
  frame=np.ones((240,424),np.float32);frame[0,0]=np.nan;frame[0,1]=np.inf
  events=[];subscriptions={};tick=[0];processes=[]
  def stamp(ns):return types.SimpleNamespace(sec=ns//10**9,nanosec=ns%10**9)
  def image(ns,index):
   a=frame.copy()
   if moving:a[1:4,:]=index+1
   # Big-endian and row padding are genuine ROS image layouts, not packed assumptions.
   padded=np.pad(a,((0,0),(0,2)),constant_values=19).astype('>f4')
   return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(ns),frame_id='visibility_optical_frame'),encoding='bad' if fault=='encoding' else '32FC1',width=424,height=240,step=426*4,is_bigendian=True,data=padded.tobytes())
  class Node:
   def __init__(self,name):events.append(('node',name))
   def create_subscription(self,kind,topic,callback,qos):subscriptions[topic]=callback
   def destroy_node(self):events.append(('destroy',))
  def spin(node,timeout_sec):
   tick[0]+=1;n=tick[0];ns=(n+1)*10**9
   if '/clock' in subscriptions:subscriptions['/clock'](types.SimpleNamespace(clock=stamp(ns)))
   if '/person/scan' in subscriptions:
    subscriptions['/person/scan'](types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(ns),frame_id='base_scan'),ranges=[1.]*360,angle_min=0.,angle_max=6.28,angle_increment=6.28/359,range_min=.12,range_max=3.5))
   for index,mode in enumerate(modes):
    t=ns+index if fault=='unsynchronized' else ns
    subscriptions['/visibility/'+mode+'/depth'](image(t,n))
    subscriptions['/visibility/'+mode+'/camera_info'](types.SimpleNamespace(k=list(range(9)),width=424,height=240,header=types.SimpleNamespace(frame_id='visibility_optical_frame')))
  ros=types.ModuleType('rclpy');ros.init=lambda:events.append(('init',));ros.ok=lambda:True;ros.shutdown=lambda:events.append(('shutdown',));ros.spin_once=spin
  node_module=types.ModuleType('rclpy.node');node_module.Node=Node
  qos=types.ModuleType('rclpy.qos');qos.qos_profile_sensor_data=object()
  sensor=types.ModuleType('sensor_msgs.msg');sensor.Image=type('Image',(),{});sensor.CameraInfo=type('CameraInfo',(),{});sensor.LaserScan=type('LaserScan',(),{})
  graph=types.ModuleType('rosgraph_msgs.msg');graph.Clock=type('Clock',(),{})
  fixture=types.ModuleType('fixture');fixture.HERE=here;fixture.BENCH=root;fixture.REPO=ROOT;fixture.BASE=root;scene=dict(moving=moving)
  def create(runtime,*args):
   (runtime/'warehouse.sdf').write_text('<sdf/>')
   rows=[dict(stamp_ns=2_000_000_000+i*200_000_000,humans=[dict(actor_xy=[i*.02 if moving else 0.,0.],proxy_xy=[i*.02 if moving else 0.,0.]) for _ in range(6)]) for i in range(21)]
   (runtime/'motion_LABEL_ONLY.jsonl').write_text('\n'.join(json.dumps(x) for x in rows))
   return runtime/'warehouse.sdf'
  fixture.create=create;fixture.get_scene=lambda _:scene
  class Guard:
   def __init__(self,runtime):self.runtime=runtime
   def cleanup(self):events.append(('cleanup',));return dict(status='PASS',remaining=[],ogre_renderer_lines=['NVIDIA'])
  class Process:
   def __init__(self,cmd,stdout,**kwargs):
    self.cmd=cmd;self.returncode=1 if fault=='process' else None;processes.append(self);events.append(('start',cmd))
    if cmd[0]=='gz':
     lines=['[Visibility] teardown pre=4 post=4']
     if condition!='null':lines += ['[Visibility] human='+str(i)+' proxy=0' for i in range(1,7)]
     stdout.write('\n'.join(lines));stdout.flush()
   def poll(self):return self.returncode
  def stop(proc):events.append(('stop',proc.cmd[0]));proc.returncode=proc.returncode if proc.returncode is not None else 0
  def stop_paused(guard,original):events.append(('pause',));return dict(status='STOPPED',quiesce=dict(simulation_time_stable=True))
  modules={'rclpy':ros,'rclpy.node':node_module,'rclpy.qos':qos,'sensor_msgs.msg':sensor,'rosgraph_msgs.msg':graph,'fixture':fixture,'quiesce':types.SimpleNamespace(stop_paused=stop_paused),'run':types.SimpleNamespace(stop=stop)}
  stack.enter_context(patch.dict(sys.modules,modules));stack.enter_context(patch.object(sys,'path',list(sys.path)))
  stack.enter_context(patch.object(capture,'RuntimeGuard',Guard));stack.enter_context(patch.object(capture,'gpu_preflight',lambda _:dict(renderer='NVIDIA')));stack.enter_context(patch.object(capture,'launch_environment',lambda _ : ({},dict(mode='owned'))));stack.enter_context(patch.object(capture,'stop_paused',stop_paused));stack.enter_context(patch.object(capture,'stop',stop));stack.enter_context(patch.object(capture.subprocess,'Popen',Process));stack.enter_context(patch.object(capture.fcntl,'flock',lambda *a:None));stack.enter_context(patch.dict(capture.os.environ,{},clear=False))
  stack.enter_context(patch.object(capture.time,'monotonic',side_effect=lambda:tick[0]))
  if source is not None:
   module=types.ModuleType('legacy_capture');module.__file__=str(source)
   # Dependencies are stubbed identically; the oracle is the exact pre-refactor run body.
   with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):exec(compile(Path(source).read_text(),str(source),'exec'),module.__dict__)
   module.RuntimeGuard=Guard;module.gpu_preflight=capture.gpu_preflight;module.launch_environment=capture.launch_environment;module.stop_paused=stop_paused;module.stop=stop
  out=here/'run';error=None
  with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
   try:
    if source is not None:
     args=[out] if profile==PROFILES[0] else [out,'train_scene'] if profile==PROFILES[3] else [out,condition];module.run(*args)
    else:
     scan=profile!=PROFILES[0];corpus=profile==PROFILES[3]
     capture.run_capture(out,here=here,repo=ROOT,benchmark=root,create_world=create,sources=[here/'protocol.json'],protocol_name='capture_protocol.json' if profile in PROFILES[2:] else 'protocol.json',condition=condition if scan else None,scene_id='train_scene' if corpus else None,scene=scene if corpus else None)
   except AssertionError as exc:error=str(exc)
  result=json.loads((out/'result.json').read_text());result.pop('source_sha256');result.pop('source_unchanged')
  arrays={p.name:np.load(p,allow_pickle=False) for p in out.glob('*.npy')};metadata={p.name:json.loads(p.read_text()) for p in out.glob('*.json') if p.name not in ['result.json','source_before.json']}
  scan=None
  if (out/'scan_SENSOR_ONLY.npz').exists():
   with np.load(out/'scan_SENSOR_ONLY.npz',allow_pickle=False) as archive:scan={k:archive[k] for k in archive.files}
  events=[x for x in events if x[0] not in ['start']]+[('commands',[p.cmd if p.cmd[0]=='ros2' else p.cmd[:3] for p in processes])]
  return result,arrays,metadata,scan,events,error

class SensorCaptureTests(unittest.TestCase):
 def test_all_profiles_preserve_float32_bits_and_owned_teardown(self):
  for name in PROFILES:
   with self.subTest(profile=name):
    result,arrays,meta,scan,events,error=replay(name)
    self.assertTrue(result['capture_accepted']);self.assertIsNone(error)
    self.assertTrue(np.isnan(arrays['full.npy'][0,0,0]));self.assertTrue(np.isposinf(arrays['full.npy'][0,0,1]))
    np.testing.assert_array_equal(arrays['full.npy'].view('u4'),arrays['control.npy'].view('u4'))
    self.assertLess(events.index(('pause',)),events.index(('stop','gz')));self.assertEqual(events[-2],('cleanup',))
    self.assertEqual(scan is not None,name!=PROFILES[0])
 def test_failed_captures_preserve_partial_data_and_stop_owned_processes(self):
  for name in PROFILES:
   for fault in ['unsynchronized','encoding','process']:
    with self.subTest(profile=name,fault=fault):
     result,arrays,meta,scan,events,error=replay(name,fault=fault)
     self.assertFalse(result['capture_accepted']);self.assertIsNotNone(error);self.assertIn(('cleanup',),events)
     if fault=='unsynchronized':self.assertIn('full_UNSYNCHRONIZED.npy',arrays)
 def test_moving_scene_does_not_require_static_interframe_equality(self):
  result,*_=replay(PROFILES[3],moving=True)
  self.assertTrue(result['capture_accepted']);self.assertTrue(result['motion_observer']['accepted']);self.assertFalse(result['strict_interframe_bitwise_gate_pass'])
 def test_null_lidar_requires_no_human_mapping(self):
  result,*_=replay(PROFILES[1],condition='null')
  self.assertTrue(result['capture_accepted']);self.assertFalse(result['six_human_geometries']);self.assertTrue(result['expected_actor_mapping'])

if __name__=='__main__':unittest.main()
