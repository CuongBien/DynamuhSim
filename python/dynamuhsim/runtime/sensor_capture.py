"""Shared counterfactual sensor capture. No model inference, labels or robot commands."""
import fcntl,hashlib,json,os,re,subprocess,time,traceback
from pathlib import Path
import numpy as np
from .gpu import RuntimeGuard,gpu_preflight
from .shutdown import launch_environment,stop_server,stop_paused
from experiments.social_navigation.gazebo_adapter.run import stop

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def run_capture(out, *, here, repo, benchmark, create_world, sources,
                protocol_name="protocol.json", condition=None, scene_id=None, scene=None):
 """One bounded capture; fixture/labels and each profile's acceptance stay separate."""
 import rclpy
 from rclpy.node import Node
 from rclpy.qos import qos_profile_sensor_data
 from sensor_msgs.msg import Image,CameraInfo,LaserScan
 from rosgraph_msgs.msg import Clock
 HERE,BENCH,REPO=Path(here),Path(benchmark),Path(repo)
 with_scan=condition is not None
 out.mkdir(parents=True,exist_ok=False);runtime=out/'runtime';runtime.mkdir();protocol=json.loads((HERE/protocol_name).read_text())
 sources=[*sources,Path(__file__),REPO/'python/dynamuhsim/runtime/gpu.py',REPO/'python/dynamuhsim/runtime/shutdown.py',REPO/'python/dynamuhsim/runtime/ordered_stop.py']
 before={str(p.relative_to(REPO)):sha(p) for p in sources}
 (out/'source_before.json').write_text(json.dumps(before,indent=2)+'\n')
 lock=(BENCH/'common_gazebo/generated/domain91.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 os.environ.update(ROS_DOMAIN_ID='91',GZ_PARTITION=f'{"lidar_person" if with_scan else "visibility"}_{os.getpid()}',ROS_LOG_DIR=str(runtime/'ros_logs'),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4',QT_QPA_PLATFORM='xcb')
 guard=RuntimeGuard(runtime);processes=[];logs=[];node=None;rows={mode:{} for mode in protocol['modes']};infos={};result={'status':'ERROR','static_fixture':not scene['moving'] if scene is not None else True,'training_ready':False,'nav2_actions':0,'native_human_steps':0}
 if scene is not None:result.update(scripted_motion=scene['moving'],scene_id=scene_id)
 try:
  world=create_world(runtime);result['preflight']=gpu_preflight(runtime);env,result['shutdown_mode']=launch_environment(runtime)
  rclpy.init();node=Node('lidar_person_capture' if with_scan else 'depth_visibility_labels');scans=[];scan_geometry={};clock=[-1];image_available={m:{} for m in rows}
  def clock_cb(m):clock[0]=m.clock.sec*10**9+m.clock.nanosec
  if with_scan:node.create_subscription(Clock,'/clock',clock_cb,qos_profile_sensor_data)
  def scan_cb(m):
   stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
   if stamp<protocol['warmup_ns'] or (with_scan and clock[0]<0):return
   assert m.header.frame_id=='base_scan' and len(m.ranges)==360
   if len(scans)<200:scans.append((stamp,max(stamp,clock[0]),np.array(m.ranges,dtype=np.float32)))
   scan_geometry.update(angle_min=m.angle_min,angle_max=m.angle_max,angle_increment=m.angle_increment,range_min=m.range_min,range_max=m.range_max,frame_id=m.header.frame_id,rays=len(m.ranges))
  if with_scan:node.create_subscription(LaserScan,protocol['scan_topic'],scan_cb,qos_profile_sensor_data)
  def image(mode,m):
   stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
   if stamp<protocol['warmup_ns'] or (with_scan and clock[0]<0):return
   assert m.encoding=='32FC1' and m.width==424 and m.height==240 and m.header.frame_id=='visibility_optical_frame'
   a=np.frombuffer(bytes(m.data),dtype='>f4' if m.is_bigendian else '<f4').reshape(m.height,m.step//4)[:,:m.width]
   if len(rows[mode])<20:
    rows[mode][stamp]=a.astype(np.float32,copy=True);image_available[mode][stamp]=max(stamp,clock[0])
  bridge=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',protocol['scan_topic']+'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'] if with_scan else []
  for mode in protocol['modes']:
   topic='/visibility/'+mode
   node.create_subscription(Image,topic+'/depth',lambda m,k=mode:image(k,m),qos_profile_sensor_data)
   node.create_subscription(CameraInfo,topic+'/camera_info',lambda m,k=mode:infos.update({k:m}),qos_profile_sensor_data)
   bridge.extend([topic+'/depth@sensor_msgs/msg/Image[gz.msgs.Image',topic+'/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'])
  for name,cmd in [('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',*bridge]),('gazebo',['gz','sim','-r','-s',str(world)])]:
   log=(out/(name+'.log')).open('w');logs.append(log);processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True))
  deadline=time.monotonic()+protocol['max_wall_s']
  while time.monotonic()<deadline:
   if any(p.poll() is not None for p in processes):raise RuntimeError('Owned process exited')
   rclpy.spin_once(node,timeout_sec=.05)
   common=sorted(set.intersection(*(set(v) for v in rows.values())))
   if len(common)>=protocol['frames'] and len(infos)==len(rows) and (not with_scan or len(scans)>=protocol['min_scans']):break
   if all(len(v)>=8 for v in rows.values()) and not common:raise RuntimeError('All modes have frames but no exact common timestamps')
  else:raise RuntimeError('Missing synchronized depth/info')
  common=common[:protocol['frames']];np.save(out/'stamp_ns.npy',np.array(common,dtype=np.int64),allow_pickle=False)
  stability={};numeric_stability={};numeric_details={}
  for mode in rows:
   a=np.stack([rows[mode][t] for t in common]);np.save(out/(mode+'.npy'),a,allow_pickle=False)
   stability[mode]=all(v.tobytes()==a[0].tobytes() for v in a)
   both=np.isfinite(a)&np.isfinite(a[0]);delta=np.zeros(a.shape);np.subtract(a,a[0],out=delta,where=both);maximum=float(np.abs(delta).max());changes=int(np.sum(np.isfinite(a)!=np.isfinite(a[0])))
   numeric_stability[mode]=changes==0 and maximum<=protocol['static_depth_tolerance_m'];numeric_details[mode]=dict(max_abs_delta_m=maximum,finite_mask_changes=changes)
  calibration={k:{'k':list(m.k),'width':m.width,'height':m.height,'frame_id':m.header.frame_id} for k,m in infos.items()}
  assert all(v==calibration['full'] for v in calibration.values())
  if with_scan:
   np.savez_compressed(out/'scan_SENSOR_ONLY.npz',stamp_ns=np.array([v[0] for v in scans],dtype=np.int64),available_ns=np.array([v[1] for v in scans],dtype=np.int64),ranges=np.stack([v[2] for v in scans]))
   (out/'scan_geometry.json').write_text(json.dumps(scan_geometry,indent=2)+'\n')
   (out/'depth_available.json').write_text(json.dumps({m:[image_available[m][t] for t in common] for m in rows},indent=2)+'\n')
  (out/'calibration.json').write_text(json.dumps(calibration,indent=2)+'\n')
  result.update(status='MEASURED',stamps_ns=common,stability=stability,numeric_stability=numeric_stability,numeric_details=numeric_details,strict_interframe_bitwise_gate_pass=all(stability.values()),full_control_bitwise=all(rows['full'][t].tobytes()==rows['control'][t].tobytes() for t in common))
 except (Exception,KeyboardInterrupt) as e:result['error']=str(e) or type(e).__name__;traceback.print_exc()
 finally:
  result['counts']={k:len(v) for k,v in rows.items()}
  result['info_received']=sorted(infos);result['stamps_per_mode']={k:sorted(v) for k,v in rows.items()}
  for mode,values in rows.items():
   if values and not (out/(mode+'.npy')).exists():np.save(out/(mode+'_UNSYNCHRONIZED.npy'),np.stack(list(values.values())),allow_pickle=False)
  try:result['shutdown']=stop_paused(guard,stop_server)
  except Exception as e:result['shutdown_error']=str(e)
  for p in reversed(processes):stop(p)
  if node:node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
  for log in logs:log.close()
  result['cleanup']=guard.cleanup();result['gazebo_exit']=processes[-1].returncode if processes else None
  text=(out/'gazebo.log').read_text(errors='replace') if (out/'gazebo.log').exists() else ''
  result['hook_lines']=[v for v in text.splitlines() if '[Visibility]' in v]
  counts=re.findall(r'teardown pre=(\d+) post=(\d+)',text);result['callbacks_balanced']=bool(counts and all(int(a)==int(b) and int(a)>0 for a,b in counts))
  result['six_human_geometries']=all(any('human='+str(i)+' proxy=0' in line for line in result['hook_lines']) for i in range(1,7))
  if with_scan:
   result['condition']=condition;result['expected_actor_mapping']=result['six_human_geometries'] if condition=='full' else not any(' human=' in line and ' proxy=0' in line for line in result['hook_lines']);result['scan_count']=len(scans) if 'scans' in locals() else 0
  result['source_sha256']=before;result['source_unchanged']=all(sha(REPO/n)==h for n,h in before.items())
  result['actual_nvidia_renderer']=any('NVIDIA' in x for x in result['cleanup'].get('ogre_renderer_lines',[]))
  if scene is not None:
   motion_rows=[json.loads(v) for v in (runtime/'motion_LABEL_ONLY.jsonl').read_text().splitlines()] if (runtime/'motion_LABEL_ONLY.jsonl').exists() else []
   observed=[v for v in motion_rows if 2_000_000_000<=v['stamp_ns']<=6_000_000_000];motion_error=max((float(np.linalg.norm(np.array(h['actor_xy'])-h['proxy_xy'])) for v in observed for h in v['humans']),default=1e9)
   displacement=max((float(np.linalg.norm(np.array(observed[-1]['humans'][i]['actor_xy'])-observed[0]['humans'][i]['actor_xy'])) for i in range(6)),default=0.) if observed else 0.
   changed_pixels=int(np.sum(np.any(np.load(out/'full.npy').view('u4')!=np.load(out/'full.npy')[0].view('u4'),axis=0))) if (out/'full.npy').exists() else 0
   result['motion_observer']=dict(rows=len(observed),max_actor_proxy_xy_error_m=motion_error,max_actor_displacement_m=displacement,changed_depth_pixels=changed_pixels,accepted=bool(len(observed)>=20 and motion_error<=protocol['motion_proxy_actor_tolerance_m'] and ((displacement>=protocol['motion_actual_min_m'] and changed_pixels>=100) if scene['moving'] else displacement<=.001)))
  result['capture_accepted']=bool(result['status']=='MEASURED' and result['gazebo_exit']==0 and result['cleanup']['status']=='PASS' and result['callbacks_balanced'] and (result['expected_actor_mapping'] and result['scan_count']>=protocol['min_scans'] if with_scan else result['six_human_geometries']) and result['actual_nvidia_renderer'] and result['source_unchanged'] and (scene is None or result['motion_observer']['accepted']) and result.get('full_control_bitwise') and ((scene is not None and scene['moving']) or all(result.get('numeric_stability',{}).values())) and result.get('shutdown',{}).get('quiesce',{}).get('simulation_time_stable'))
  (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k] for k in ['status','capture_accepted','counts','gazebo_exit','callbacks_balanced','six_human_geometries','actual_nvidia_renderer']}),flush=True)
 assert result['capture_accepted'],'Capture rejected; preserve failure, no automatic retry'
