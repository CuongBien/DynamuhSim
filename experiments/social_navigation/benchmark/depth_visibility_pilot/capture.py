#!/usr/bin/env python3
"""One bounded static GPU capture. All intervention arrays are LABEL ONLY."""
import argparse,fcntl,hashlib,json,os,re,subprocess,sys,time,traceback
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from fixture import create,HERE,BENCH,REPO
sys.path[:0]=[str(BENCH),str(BENCH/'gpu_runtime'),str(BENCH/'depth_proxy_lifecycle'),str(BENCH.parent/'gazebo_adapter')]
from runtime_guard import RuntimeGuard,gpu_preflight
from shutdown_fix.ordered_stop import launch_environment,stop_server
from quiesce import stop_paused
from run import stop

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(out):
 out.mkdir(parents=True,exist_ok=False);runtime=out/'runtime';runtime.mkdir();protocol=json.loads((HERE/'protocol.json').read_text())
 sources=[p for p in HERE.iterdir() if p.is_file()]+[BENCH/'camera_height_ab/fixture.py',BENCH/'camera_height_ab/freeze_actors.cc',BENCH/'camera_height_ab/generated/build/libfreeze_actors.so',HERE/'generated/build/libvisibility_filter.so',REPO/'src/custom_corridor/models/mecanum_robot/model.sdf',BENCH.parent/'gazebo_adapter/run.py',BENCH.parent/'warehouse/warehouse.sdf']
 before={str(p.relative_to(REPO)):sha(p) for p in sources}
 (out/'source_before.json').write_text(json.dumps(before,indent=2)+'\n')
 lock=(BENCH/'common_gazebo/generated/domain91.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 os.environ.update(ROS_DOMAIN_ID='91',GZ_PARTITION=f'visibility_{os.getpid()}',ROS_LOG_DIR=str(runtime/'ros_logs'),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4',QT_QPA_PLATFORM='xcb')
 guard=RuntimeGuard(runtime);processes=[];logs=[];node=None;rows={mode:{} for mode in protocol['modes']};infos={};result={'status':'ERROR','static_fixture':True,'training_ready':False,'nav2_actions':0,'native_human_steps':0}
 try:
  world=create(runtime);result['preflight']=gpu_preflight(runtime);env,result['shutdown_mode']=launch_environment(runtime)
  rclpy.init();node=Node('depth_visibility_labels')
  def image(mode,m):
   stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
   if stamp<protocol['warmup_ns']:return
   assert m.encoding=='32FC1' and m.width==424 and m.height==240 and m.header.frame_id=='visibility_optical_frame'
   a=np.frombuffer(bytes(m.data),dtype='>f4' if m.is_bigendian else '<f4').reshape(m.height,m.step//4)[:,:m.width]
   if len(rows[mode])<20:rows[mode][stamp]=a.astype(np.float32,copy=True)
  bridge=[]
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
   if len(common)>=protocol['frames'] and len(infos)==len(rows):break
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
  result['source_sha256']=before;result['source_unchanged']=all(sha(REPO/n)==h for n,h in before.items())
  result['actual_nvidia_renderer']=any('NVIDIA' in x for x in result['cleanup'].get('ogre_renderer_lines',[]))
  result['capture_accepted']=bool(result['status']=='MEASURED' and result['gazebo_exit']==0 and result['cleanup']['status']=='PASS' and result['callbacks_balanced'] and result['six_human_geometries'] and result['actual_nvidia_renderer'] and result['source_unchanged'] and result.get('full_control_bitwise') and all(result.get('numeric_stability',{}).values()) and result.get('shutdown',{}).get('quiesce',{}).get('simulation_time_stable'))
  (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k] for k in ['status','capture_accepted','counts','gazebo_exit','callbacks_balanced','six_human_geometries','actual_nvidia_renderer']}),flush=True)
 assert result['capture_accepted'],'Capture rejected; preserve failure, no automatic retry'
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.output.resolve())
