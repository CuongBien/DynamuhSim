"""New two-episode MCAP exporter; historical writers/dataset01 are never executed."""
import argparse,collections,hashlib,importlib.util,json,math,shutil,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.datasets.normalization import causal_indices,interpolate_labels,rotation,validate_clock,group_membership
from dynamuhsim.analysis.evidence import sha256_file as sha
NS=1_000_000_000


def dump(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def ns(s):return s.sec*NS+s.nanosec
def pose(p):
 q=p.orientation;return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

def load_module(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def read_raw(run):
 import rosbag2_py
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'episode/raw_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()};topics={'/scan':'scan','/imu':'imu','/odom':'odom','/cmd_vel':'action','/camera/depth/image_rect_raw':'depth','/amcl_pose':'amcl'};streams=collections.defaultdict(list);counts=collections.Counter();depth=[];static=collections.defaultdict(set);scenes={};groups={};events=[];clocks=[];infos=[];scan_geometry=None;frames=collections.defaultdict(set)
 while reader.has_next():
  topic,data,received=reader.read_next();counts[topic]+=1
  if topic not in topics and topic not in ['/tf_static','/clock','/dataset/scene_poses','/dataset/events','/common/ground_truth','/camera/camera_info']:continue
  m=deserialize_message(data,types[topic])
  if hasattr(m,'header'):frames[topic].add(m.header.frame_id)
  if topic=='/clock':clocks.append(ns(m.clock))
  elif topic=='/tf_static':
   for t in m.transforms:
    v,q=t.transform.translation,t.transform.rotation;static[t.child_frame_id].add((t.header.frame_id,v.x,v.y,v.z,q.x,q.y,q.z,q.w))
  elif topic=='/dataset/scene_poses':
   value=json.loads(m.data);scenes[value['stamp_ns']]=value
  elif topic=='/common/ground_truth':
   value=json.loads(m.data);groups[round(value['time']*NS)]=value
  elif topic=='/dataset/events':events.append(json.loads(m.data))
  elif topic=='/camera/camera_info':infos.append(dict(frame_id=m.header.frame_id,width=m.width,height=m.height,k=list(m.k),p=list(m.p),d=list(m.d),distortion_model=m.distortion_model))
  else:
   key=topics[topic];extra={}
   if key=='depth':
    if m.encoding!='32FC1' or m.header.frame_id!='camera_optical_frame' or (m.height,m.width)!=(240,424) or m.step%4 or len(m.data)!=m.height*m.step:raise ValueError('Depth encoding/shape/frame')
    value=np.frombuffer(bytes(m.data),dtype='>f4' if m.is_bigendian else '<f4').reshape(m.height,m.step//4)[:,:m.width].astype(np.float32);depth.append(value.copy());value=[len(depth)-1]
   elif key=='scan':
    raw=np.array(m.ranges,np.float32);valid=np.isfinite(raw)&(raw>=m.range_min)&(raw<=m.range_max);value=np.where(valid,raw,0);extra['valid']=valid;geometry=dict(frame_id=m.header.frame_id,angle_min=m.angle_min,angle_max=m.angle_max,angle_increment=m.angle_increment,range_min=m.range_min,range_max=m.range_max,rays=len(raw),scan_time=m.scan_time,time_increment=m.time_increment)
    if scan_geometry is not None and geometry!=scan_geometry:raise ValueError('Changed scan calibration')
    scan_geometry=geometry
   elif key=='imu':value=[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z];extra['orientation_valid']=m.orientation_covariance[0]>=0
   elif key=='odom':
    t=m.twist.twist;value=pose(m.pose.pose)+[t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z]
   elif key=='amcl':value=pose(m.pose.pose)
   else:value=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]
   streams[key].append((ns(m.header.stamp),int(received),value,extra))
 validate_clock(clocks)
 if set(streams)!=set(topics.values()) or not infos or not scenes or not groups:raise ValueError('Missing raw streams')
 if any(v!=infos[0] for v in infos):raise ValueError('Changed camera calibration')
 if any(len(v)!=1 for v in static.values()):raise ValueError('Conflicting static TF')
 modalities={}
 for key,records in streams.items():
  records=sorted(records,key=lambda r:(max(r[0],r[1]),r[0]));stamps=np.array([v[0] for v in records],np.int64);received=np.array([v[1] for v in records],np.int64);m=dict(stamp_ns=stamps,received_ns=received,available_ns=np.maximum(stamps,received),values=np.array([v[2] for v in records],np.int32 if key=='depth' else np.float32))
  for field in records[0][3]:m[field]=np.array([v[3][field] for v in records])
  modalities[key]=m
 # Remap depth storage into sorted modality order; depth indices and image rows agree.
 original=np.stack(depth);order=modalities['depth']['values'][:,0];depth=original[order];modalities['depth']['values']=np.arange(len(depth),dtype=np.int32)[:,None]
 if np.any(np.diff(modalities['depth']['stamp_ns'])<=0):raise ValueError('Duplicate/nonincreasing depth stamps require an explicit new policy')
 info=infos[0];tf={k:list(next(iter(v))) for k,v in static.items()}
 def transform(child,visited=()):
  if child=='base_link':return np.eye(4)
  if child in visited or child not in tf:raise ValueError('Disconnected/cyclic optical TF')
  parent,*value=tf[child];q=np.array(value[3:]);
  if abs(np.linalg.norm(q)-1)>1e-6:raise ValueError('Invalid TF quaternion')
  t=np.eye(4);t[:3,:3]=rotation(q);t[:3,3]=value[:3];return transform(parent,visited+(child,))@t
 optical=transform('camera_optical_frame')
 if info['frame_id']!='camera_optical_frame' or (info['height'],info['width'])!=(240,424) or not np.allclose(optical[:3,:3],[[0,0,1],[-1,0,0],[0,-1,0]],atol=1e-9) or any(abs(v)>1e-12 for v in info['d']):raise ValueError('Optical calibration/rectification unsupported')
 if frames['/cmd_vel']!={'base_footprint'} or tf['base_link'][0]!='base_footprint' or not np.allclose(tf['base_link'][1:3],[0,0]) or not np.allclose(rotation(tf['base_link'][4:]),np.eye(3)):raise ValueError('Action planar frame mismatch')
 calibration=dict(schema='social_calibration/1',depth_topic='/camera/depth/image_rect_raw',**info,optical_to_base_link=optical.tolist(),depth_unit='m',valid_depth_range_m=[.28,5.],tf_static=tf,lidar_geometry=scan_geometry,imu_channels=['qx','qy','qz','qw','wx','wy','wz','ax','ay','az'],odom_channels=['x','y','yaw','vx','vy','vz','wx','wy','wz'],action_channels=['vx','vy','wz'],action_frame='base_link planar axes (validated aligned with base_footprint)',goal_source='Frozen commanded map goal + causal AMCL; no GT ego',caveat='Planar SE2 labels; no general world-3D calibration claim')
 return modalities,depth,calibration,[scenes[t] for t in sorted(scenes)],groups,events,dict(counts)


def normalize(run,out,protocol):
 result=json.loads((run/'episode/result.json').read_text())
 if result.get('status')!='MEASURED' or result.get('navigation_status')!='SUCCEEDED' or result.get('actual_goal_reached') is not True:raise ValueError('Pilot requires accepted moving-robot goal')
 out.mkdir(parents=True,exist_ok=False);mods,depth,cal,scene,groups,events,counts=read_raw(run);by={v['event']:v for v in events}
 if [v['event'] for v in events]!=['ready','goal_accepted','action_end','observation_end']:raise ValueError('Missing/duplicate event')
 start,end=by['goal_accepted']['stamp_ns'],by['action_end']['stamp_ns'];goal=np.array(protocol['navigation_goal_map'][:2]);assert by['goal_accepted']['goal_world']==protocol['navigation_goal_map'];assert by['observation_end']['stamp_ns']-end>=3*NS
 np.save(out/'depth.npy',depth,allow_pickle=False);np.savez_compressed(out/'observations.npz',**{k+'__'+f:v for k,m in mods.items() for f,v in m.items()});dump(out/'calibration.json',cal);dump(out/'events.json',events)
 with (out/'human_states.jsonl').open('x') as f:
  for v in scene:f.write(json.dumps(v,allow_nan=False)+'\n')
 ts=np.array([v['stamp_ns'] for v in scene],np.int64);humans=np.array([[[h['x'],h['y']] for h in v['humans']] for v in scene]);robots=np.array([[v['robot'][k] for k in ['x','y','yaw']] for v in scene]);robots[:,2]=np.unwrap(robots[:,2]);motion_accepted=float(np.linalg.norm(robots[:,:2]-robots[0,:2],axis=1).max())>1.
 if not motion_accepted:raise ValueError('Robot did not translate at least1m')
 ids=list(range(1,7));assert all([h['id'] for h in v['humans']]==ids and v['persona'] is None and not v['persona_valid'] for v in scene)
 gt=np.array(sorted(groups),np.int64);gids=[];gmodes=[]
 for t in gt:
  a,b=group_membership(groups[t]['groups'],ids);gids.append(a);gmodes.append(b)
 gids=np.array(gids,np.int32);gmodes=np.array(gmodes,np.int8);x=collections.defaultdict(list);y=collections.defaultdict(list);rejected=[]
 for anchor in np.arange(((start+2*NS+99999999)//100000000)*100000000,end-3*NS+1,100000000,dtype=np.int64):
  queries=anchor+np.array(protocol['history_offsets_ns'],np.int64);indices={};valid={}
  for key in ['scan','imu','odom','action','depth']:
   m=mods[key];indices[key],valid[key]=causal_indices(m['stamp_ns'],m['available_ns'],queries,protocol['max_sensor_age_ns'][key])
  if not all(valid[k].all() for k in ['imu','odom','action']) or not all(valid[k].mean()>=.9 and valid[k][-1] for k in ['scan','depth']):rejected.append(dict(anchor_ns=int(anchor),reason='history_coverage'));continue
  future=anchor+np.arange(1,31,dtype=np.int64)*100000000;hp,hv=interpolate_labels(ts,humans,future,100000000);rp,rv=interpolate_labels(ts,robots,np.array([anchor]),100000000);hc,cv=interpolate_labels(ts,humans,np.array([anchor]),100000000);aq=anchor+np.array(protocol['future_action_offsets_ns'],np.int64);act=mods['action'];ai,av=causal_indices(act['stamp_ns'],act['available_ns'],aq,150000000)
  if not(hv.all() and rv.all() and cv.all() and av.all()):rejected.append(dict(anchor_ns=int(anchor),reason='future_support'));continue
  x['anchor_ns'].append(anchor);x['history_query_ns'].append(queries)
  for key,index in indices.items():
   m=mods[key]
   for name in ['stamp_ns','available_ns']:x[key+'_'+name].append(m[name][index])
   x[key+'_valid'].append(valid[key]);values=m['values'][index].copy()
   if key=='depth':x['depth_index'].append(values[:,0].astype(np.int32))
   elif key=='scan':x['scan_ranges_m'].append(values);x['scan_ray_valid'].append(m['valid'][index]&valid[key][:,None])
   elif key=='odom':
    origin=values[-1,:3].copy();c,s=np.cos(origin[2]),np.sin(origin[2]);values[:,:2]=(values[:,:2]-origin[:2])@np.array([[c,-s],[s,c]]);values[:,2]=np.arctan2(np.sin(values[:,2]-origin[2]),np.cos(values[:,2]-origin[2]));x['odom_local_pose_and_twist'].append(values)
   else:x[key].append(values)
   if key=='imu':x['imu_orientation_valid'].append(m['orientation_valid'][index]&valid[key])
  amcl=mods['amcl'];gi,gv=causal_indices(amcl['stamp_ns'],amcl['available_ns'],np.array([anchor]),NS);ap=amcl['values'][gi[0]];c,s=np.cos(ap[2]),np.sin(ap[2]);x['goal_xy_from_amcl'].append((goal-ap[:2])@np.array([[c,-s],[s,c]]) if gv[0] else np.zeros(2));x['goal_valid'].append(gv[0]);x['goal_stamp_ns'].append(amcl['stamp_ns'][gi[0]]);x['goal_available_ns'].append(amcl['available_ns'][gi[0]])
  robot=rp[0];c,s=np.cos(robot[2]),np.sin(robot[2]);rot=np.array([[c,-s],[s,c]]);y['future_stamp_ns'].append(future);y['human_xy_future_m'].append((hp-robot[:2])@rot);y['human_xy_at_anchor_m'].append((hc[0]-robot[:2])@rot);y['human_valid'].append(np.ones((30,6),bool));y['executed_action_query_ns'].append(aq);y['executed_action_future'].append(act['values'][ai]);y['executed_action_stamp_ns'].append(act['stamp_ns'][ai]);y['executed_action_available_ns'].append(act['available_ns'][ai]);ix=np.searchsorted(gt,future,side='right')-1;assert (ix>=0).all();y['group_id_future'].append(gids[ix]);y['group_mode_future'].append(gmodes[ix]);y['group_valid'].append(future-gt[ix]<=150000000);y['persona_index'].append(np.full(6,-1,np.int8));y['persona_valid'].append(np.zeros(6,bool))
 if not x['anchor_ns']:raise ValueError('No supported windows')
 x={k:np.array(v) for k,v in x.items()};y={k:np.array(v) for k,v in y.items()};y['human_ids']=np.array(ids,np.int32);np.savez_compressed(out/'samples.npz',**x);np.savez_compressed(out/'labels.npz',**y);dump(out/'qa.json',dict(windows=len(x['anchor_ns']),depth_frames=len(depth),topic_counts=counts,rejected=rejected,raw_goal_start_ns=start,raw_action_end_ns=end,robot_motion_accepted=motion_accepted));return len(x['anchor_ns'])


def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();protocol=json.loads((HERE/'protocol.json').read_text());freeze=json.loads((HERE/'export_freeze.json').read_text())
 for name,digest in freeze['source_sha256'].items():
  if sha(ROOT/name)!=digest:raise RuntimeError('Export source changed: '+name)
 for case in protocol['episodes']:
  for name,digest in case['source_sha256'].items():
   if sha(ROOT/name)!=digest:raise RuntimeError('Raw input changed: '+name)
  if json.loads((ROOT/case['source_audit']).read_text())['status']!='PASS':raise ValueError('Unaccepted source capture')
 if not a.output.resolve().is_relative_to((HERE/'generated').resolve()):raise ValueError('Export only into a NEW handoff_pilot/generated directory')
 a.output.mkdir(parents=True,exist_ok=False);dataset=a.output/'dataset';(dataset/'episodes').mkdir(parents=True);(dataset/'tools').mkdir();shutil.copy2(ROOT/'experiments/social_navigation/datasets/wm_episode_batch/loader.py',dataset/'tools/loader.py');shutil.copy2(HERE/'protocol.json',dataset/'protocol.json');shutil.copy2(HERE/'export_freeze.json',dataset/'export_freeze.json');meta=[];targets=load_module('handoff_existing_targets',ROOT/'experiments/social_navigation/datasets/wm_episode_batch/targets.py')
 for case in protocol['episodes']:
  name=case['episode_id'];run=ROOT/case['source_run'];norm=a.output/'normalized'/name;print('NORMALIZE',name,flush=True);n=normalize(run,norm,protocol);episode=dataset/'episodes'/name;(episode/'inputs').mkdir(parents=True);(episode/'labels').mkdir();inp=episode/'inputs';labels=episode/'labels'
  for src,dst in [('samples.npz','samples_SENSOR_ONLY.npz'),('depth.npy','depth_SENSOR_ONLY.npy'),('observations.npz','observations_SENSOR_ONLY.npz'),('calibration.json','calibration_SENSOR_ONLY.json')]:shutil.copy2(norm/src,inp/dst)
  for src,dst in [('labels.npz','future_trajectories_actions_LABEL_ONLY.npz'),('human_states.jsonl','human_states_LABEL_ONLY.jsonl')]:shutil.copy2(norm/src,labels/dst)
  item=dict(episode_id=name,split='pilot_only',relative_path='episodes/'+name,source_run=case['source_run']);summary=targets.run(norm,episode,item,dict(moving_episode_accepted=json.loads((norm/'qa.json').read_text())['robot_motion_accepted']));masks=dict(np.load(labels/'task_masks_LABEL_ONLY.npz'));masks['training_authorized']=np.zeros(n,bool);np.savez_compressed(labels/'task_masks_LABEL_ONLY.npz',**masks);item.update(summary);meta.append(item);dump(episode/'inventory.json',{str(f.relative_to(episode)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in episode.rglob('*') if f.is_file()});print('EXPORTED',name,n,flush=True)
 dump(dataset/'dataset.json',dict(schema='handoff_wm_pilot/1',complete=True,episodes=meta,split_policy='pilot_only; no train/validation/test assigned',training_ready=False,fit_allowed=False,model_evaluations=0,source_protocol_sha256=sha(HERE/'protocol.json')));dump(a.output/'export_summary.json',dict(episodes=meta,training_ready=False));print('EXPORTED TWO-EPISODE PILOT',flush=True)

if __name__=='__main__':main()
