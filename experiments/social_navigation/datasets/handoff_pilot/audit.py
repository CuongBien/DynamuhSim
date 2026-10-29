"""Independent MCAP-to-pilot integrity and standalone NumPy loader audit."""
import argparse,collections,hashlib,importlib.util,json,math,os,shutil,subprocess,sys,tempfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.analysis.evidence import sha256_file as sha

def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def ns(s):return s.sec*1000000000+s.nanosec

def pose(p):
 q=p.orientation;return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

def rows(p):return [json.loads(s) for s in p.read_text().splitlines() if s.strip()]

def raw_records(run,obs,depth):
 import rosbag2_py
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'episode/raw_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));types={v.name:get_message(v.type) for v in reader.get_all_topics_and_types()};mapping={'/scan':'scan','/imu':'imu','/odom':'odom','/cmd_vel':'action','/camera/depth/image_rect_raw':'depth','/amcl_pose':'amcl'};records=collections.defaultdict(list);scenes={};groups={};events=[];compared=0;counts=collections.Counter();lookup=collections.defaultdict(list)
 for i,(s,b) in enumerate(zip(obs['depth__stamp_ns'],obs['depth__received_ns'])):lookup[int(s),int(b)].append(i)
 while reader.has_next():
  topic,data,received=reader.read_next();counts[topic]+=1
  if topic not in mapping and topic not in ['/dataset/scene_poses','/dataset/events','/common/ground_truth']:continue
  m=deserialize_message(data,types[topic])
  if topic=='/dataset/scene_poses':v=json.loads(m.data);scenes[v['stamp_ns']]=v;continue
  if topic=='/common/ground_truth':v=json.loads(m.data);groups[round(v['time']*1e9)]=v;continue
  if topic=='/dataset/events':events.append(json.loads(m.data));continue
  stamp=ns(m.header.stamp);key=mapping[topic];extra=None
  if key=='depth':
   index=lookup[stamp,received].pop(0);actual=np.frombuffer(bytes(m.data),dtype='>f4' if m.is_bigendian else '<f4').reshape(m.height,m.step//4)[:,:m.width].astype(np.float32);assert actual.tobytes()==depth[index].tobytes(),'Raw depth mismatch';compared+=1;value=[index]
  elif key=='scan':
   v=np.asarray(m.ranges,np.float32);extra=np.isfinite(v)&(v>=m.range_min)&(v<=m.range_max);value=np.where(extra,v,0)
  elif key=='imu':value=[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z];extra=m.orientation_covariance[0]>=0
  elif key=='odom':
   v=m.twist.twist;value=pose(m.pose.pose)+[v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]
  elif key=='amcl':value=pose(m.pose.pose)
  else:value=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]
  records[key].append((stamp,received,value,extra))
 assert compared==len(depth) and all(not v for v in lookup.values())
 raw={}
 for key,values in records.items():
  values.sort(key=lambda v:(max(v[0],v[1]),v[0]));s=np.array([v[0] for v in values],np.int64);b=np.array([v[1] for v in values],np.int64);v=np.array([v[2] for v in values],np.int32 if key=='depth' else np.float32);raw[key]=dict(stamp=s,available=np.maximum(s,b),values=v,extra=np.array([v[3] for v in values]) if key in ['scan','imu'] else None)
  for field,expected in [('stamp_ns',s),('received_ns',b),('available_ns',np.maximum(s,b)),('values',v)]:np.testing.assert_array_equal(obs[key+'__'+field],expected)
 return raw,scenes,groups,events,counts,compared


def audit_episode(dataset,case,protocol,geometry):
 episode=dataset/'episodes'/case['episode_id'];loader=load('handoff_audit_loader',dataset/'tools/loader.py');e=loader.Dataset(dataset).episode(case['episode_id']);x=e.x;y=dict(np.load(episode/'labels/future_trajectories_actions_LABEL_ONLY.npz'));z=dict(np.load(episode/'labels/future_observed_bev_LABEL_ONLY.npz'));m=dict(np.load(episode/'labels/task_masks_LABEL_ONLY.npz'));o=dict(np.load(episode/'inputs/observations_SENSOR_ONLY.npz'));n=len(e);run=ROOT/case['source_run'];raw,scenes,groups,events,counts,depth_count=raw_records(run,o,e.depth);times={v['event']:v['stamp_ns'] for v in events};assert n>0;assert x['history_query_ns'].shape==(n,20);assert e.depth.shape[1:]==(240,424);assert y['executed_action_future'].shape==(n,30,3);assert z['occupancy'].shape==(n,3,64,64);assert (x['anchor_ns']>=times['goal_accepted']+2000000000).all() and (x['anchor_ns']+3000000000<=times['action_end']).all();np.testing.assert_array_equal(x['history_query_ns'],x['anchor_ns'][:,None]+np.array(protocol['history_offsets_ns']))
 assert not any(word in key for key in x for word in ['human','persona','backend','future','world'])
 for key in ['scan','imu','odom','action','depth']:
  v=raw[key];q=x['history_query_ns'];ix=np.searchsorted(v['available'],q,side='right')-1;assert (ix>=0).all();good=(q-v['stamp'][ix]<=protocol['max_sensor_age_ns'][key])&(v['stamp'][ix]<=q)&(v['available'][ix]<=q);np.testing.assert_array_equal(x[key+'_valid'],good);np.testing.assert_array_equal(x[key+'_stamp_ns'],v['stamp'][ix]);np.testing.assert_array_equal(x[key+'_available_ns'],v['available'][ix]);assert (x[key+'_stamp_ns']<=q).all() and (x[key+'_available_ns']<=q).all()
  if key=='scan':np.testing.assert_array_equal(x['scan_ranges_m'],v['values'][ix]);np.testing.assert_array_equal(x['scan_ray_valid'],v['extra'][ix]&good[:,:,None])
  elif key=='imu':np.testing.assert_array_equal(x['imu'],v['values'][ix]);np.testing.assert_array_equal(x['imu_orientation_valid'],v['extra'][ix]&good)
  elif key=='depth':np.testing.assert_array_equal(x['depth_index'],v['values'][ix,0])
  elif key=='action':np.testing.assert_array_equal(x['action'],v['values'][ix])
  else:
   a=v['values'][ix].astype(float);origin=a[:,-1,:3].copy();delta=a[:,:,:2]-origin[:,None,:2];c,s=np.cos(origin[:,2]),np.sin(origin[:,2]);expected=a.copy();expected[:,:,0]=delta[:,:,0]*c[:,None]+delta[:,:,1]*s[:,None];expected[:,:,1]=-delta[:,:,0]*s[:,None]+delta[:,:,1]*c[:,None];expected[:,:,2]=np.arctan2(np.sin(a[:,:,2]-origin[:,None,2]),np.cos(a[:,:,2]-origin[:,None,2]));np.testing.assert_allclose(x['odom_local_pose_and_twist'],expected,atol=2e-6,rtol=0)
  if key in ['imu','odom','action']:assert good.all()
  else:assert (good.mean(1)>=.9).all() and good[:,-1].all()
 amcl=raw['amcl'];ix=np.searchsorted(amcl['available'],x['anchor_ns'],side='right')-1;assert (ix>=0).all();valid=x['anchor_ns']-amcl['stamp'][ix]<=1000000000;np.testing.assert_array_equal(x['goal_valid'],valid);ap=amcl['values'][ix];delta=np.array(protocol['navigation_goal_map'][:2])-ap[:,:2];expected=np.stack([delta[:,0]*np.cos(ap[:,2])+delta[:,1]*np.sin(ap[:,2]),-delta[:,0]*np.sin(ap[:,2])+delta[:,1]*np.cos(ap[:,2])],axis=1);expected[~valid]=0;np.testing.assert_allclose(x['goal_xy_from_amcl'],expected,atol=2e-6,rtol=0);np.testing.assert_array_equal(x['goal_stamp_ns'],amcl['stamp'][ix]);np.testing.assert_array_equal(x['goal_available_ns'],amcl['available'][ix])
 np.testing.assert_array_equal(y['executed_action_query_ns'],x['anchor_ns'][:,None]+np.arange(30)*100000000);a=raw['action'];ix=np.searchsorted(a['available'],y['executed_action_query_ns'],side='right')-1;assert (ix>=0).all();np.testing.assert_array_equal(y['executed_action_future'],a['values'][ix]);np.testing.assert_array_equal(y['executed_action_stamp_ns'],a['stamp'][ix]);np.testing.assert_array_equal(y['executed_action_available_ns'],a['available'][ix]);np.testing.assert_array_equal(y['executed_action_future'][:,0],x['action'][:,-1])
 ts=np.array(sorted(scenes),np.int64);robots=np.array([[scenes[t]['robot'][k] for k in ['x','y','yaw']] for t in ts]);robots[:,2]=np.unwrap(robots[:,2]);people=np.array([[[h['x'],h['y']] for h in scenes[t]['humans']] for t in ts]);anchor_pose=np.stack([np.interp(x['anchor_ns'],ts,robots[:,j]) for j in range(3)],axis=-1);np.testing.assert_array_equal(y['future_stamp_ns'],x['anchor_ns'][:,None]+np.arange(1,31)*100000000);q=y['future_stamp_ns'];assert q.min()>=ts[0] and q.max()<=ts[-1];human=np.stack([np.stack([np.interp(q,ts,people[:,i,j]) for j in range(2)],axis=-1) for i in range(6)],axis=-2);delta=human-anchor_pose[:,None,None,:2];c,s=np.cos(anchor_pose[:,2]),np.sin(anchor_pose[:,2]);expected=np.stack([delta[:,:,:,0]*c[:,None,None]+delta[:,:,:,1]*s[:,None,None],-delta[:,:,:,0]*s[:,None,None]+delta[:,:,:,1]*c[:,None,None]],axis=-1);np.testing.assert_allclose(y['human_xy_future_m'],expected,atol=1e-10,rtol=0);assert y['human_valid'].all();assert not y['persona_valid'].any() and (y['persona_index']==-1).all();assert not m['m1_track_aligned_valid'].any() and not m['persona_valid'].any() and not m['rl_reward_valid'].any() and np.isnan(m['rl_reward']).all() and not m['training_authorized'].any()
 gt=np.array(sorted(groups),np.int64);ix=np.searchsorted(gt,q,side='right')-1;assert (ix>=0).all();gi=[];gm=[]
 for t in gt:
  members={h:(g['id'],{'standing':2,'moving':1,'walking':1}[g['type']]) for g in groups[t]['groups'] for h in g['members']};gi.append([members.get(i,(-1,0))[0] for i in range(1,7)]);gm.append([members.get(i,(-1,0))[1] for i in range(1,7)])
 np.testing.assert_array_equal(y['group_id_future'],np.array(gi)[ix]);np.testing.assert_array_equal(y['group_mode_future'],np.array(gm)[ix]);np.testing.assert_array_equal(y['group_valid'],q-gt[ix]<=150000000)
 stamps=raw['depth']['stamp'];np.testing.assert_array_equal(z['query_ns'],x['anchor_ns'][:,None]+np.array([500000000,1000000000,3000000000]));selected=stamps[z['depth_index']];np.testing.assert_array_equal(z['depth_stamp_ns'],selected);np.testing.assert_array_equal(z['depth_time_error_ns'],selected-z['query_ns']);assert abs(z['depth_time_error_ns']).max()<=50000000 and (selected>x['anchor_ns'][:,None]).all();nearest=np.abs(stamps[:,None,None]-z['query_ns'][None,:,:]).argmin(0);np.testing.assert_array_equal(z['depth_index'],nearest);future_pose=np.stack([np.interp(selected,ts,robots[:,j]) for j in range(3)],axis=-1);np.testing.assert_allclose(z['anchor_pose_world_gt'],anchor_pose,atol=1e-10,rtol=0);np.testing.assert_allclose(z['future_pose_world_gt'],future_pose,atol=1e-10,rtol=0);np.testing.assert_array_equal(z['cell_loss_valid'],(z['occupancy']!=-1)&z['frame_valid'][:,:,None,None]);np.testing.assert_array_equal(m['wm_observed_bev_train_valid'],e.bev['frame_valid']&z['frame_valid'].all(1));assert m['executed_action_condition_valid'].all();assert set(np.unique(z['occupancy']))<={-1,0,1}
 cal=e.calibration;k=cal['k'];t=np.array(cal['optical_to_base_link']);cache={}
 def points(index):
  if int(index) not in cache:
   image=e.depth[index];v,u=np.mgrid[0:240:4,0:424:4];z=image[::4,::4];good=np.isfinite(z)&(z>=.28)&(z<=5.);p=np.column_stack(((u[good]-k[2])*z[good]/k[0],(v[good]-k[5])*z[good]/k[4],z[good]));cache[int(index)]=p@t[:3,:3].T+t[:3,3]
  return cache[int(index)]
 def warp(p,source,anchor):
  wx=source[0]+p[:,0]*np.cos(source[2])-p[:,1]*np.sin(source[2]);wy=source[1]+p[:,0]*np.sin(source[2])+p[:,1]*np.cos(source[2]);out=p.copy();out[:,0]=(wx-anchor[0])*np.cos(anchor[2])+(wy-anchor[1])*np.sin(anchor[2]);out[:,1]=-(wx-anchor[0])*np.sin(anchor[2])+(wy-anchor[1])*np.cos(anchor[2]);return out
 def odom(query):
  v=raw['odom'];good=(v['stamp']<=query)&(v['available']<=query)&(query-v['stamp']<=150000000);ids=np.flatnonzero(good)
  if not len(ids):return None
  index=ids[np.argmax(v['stamp'][ids])];return v['values'][index,:3].astype(float),v['stamp'][index],v['available'][index]
 replayed=0
 for i in range(n):
  for j in range(3):
   if z['frame_valid'][i,j]:
    source=future_pose[i,j];anchor=anchor_pose[i];expected=geometry.raster(warp(points(z['depth_index'][i,j]),source,anchor),warp(t[None,:3,3],source,anchor)[0]);np.testing.assert_array_equal(expected,z['occupancy'][i,j]);replayed+=1
  di=x['depth_index'][i,-1];source=odom(int(stamps[di]));anchor=odom(int(x['anchor_ns'][i]));supported=source is not None and anchor is not None and x['depth_valid'][i,-1];assert bool(e.bev['frame_valid'][i])==supported
  if supported:
   expected=geometry.raster(warp(points(di),source[0],anchor[0]),warp(t[None,:3,3],source[0],anchor[0])[0]);np.testing.assert_array_equal(expected,e.bev['occupancy'][i]);assert e.bev['source_odom_stamp_ns'][i]==source[1] and e.bev['source_odom_available_ns'][i]==source[2] and e.bev['anchor_odom_stamp_ns'][i]==anchor[1] and e.bev['anchor_odom_available_ns'][i]==anchor[2];replayed+=1
  if i%100==0:print('AUDIT BEV',case['episode_id'],i,'/',n,flush=True)
 for file,info in json.loads((episode/'inventory.json').read_text()).items():assert (episode/file).stat().st_size==info['bytes'] and sha(episode/file)==info['sha256']
 return dict(episode_id=case['episode_id'],status='PASS',windows=n,wm_target_valid=int(m['wm_observed_bev_train_valid'].sum()),raw_depth_bitwise_equal=depth_count,grids_replayed=replayed,observation_shape=dict(depth_history=[20,240,424],action_history=[20,3],imu_history=[20,10],odom_history=[20,9]),future_bev_shape=[3,64,64],source_topic_counts=dict(counts),training_authorized=False)


def fingerprint(observation):
 h=hashlib.sha256()
 for key in sorted(observation['history']):h.update(key.encode());h.update(observation['history'][key].tobytes())
 h.update(observation['depth_history'].tobytes());h.update(observation['bev_anchor_estimated'].tobytes());return h.hexdigest()

def main():
 p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);protocol=json.loads((HERE/'protocol.json').read_text());dataset=a.dataset.resolve();meta=json.loads((dataset/'dataset.json').read_text());assert len(meta['episodes'])==2 and meta['complete'] and meta['training_ready'] is False and meta['fit_allowed'] is False and all(e['split']=='pilot_only' for e in meta['episodes']);freeze=json.loads((dataset/'export_freeze.json').read_text())
 for name,digest in freeze['source_sha256'].items():assert sha(ROOT/name)==digest
 for case in protocol['episodes']:
  for name,digest in case['source_sha256'].items():assert sha(ROOT/name)==digest
 sensor=load('sensor',ROOT/'experiments/social_navigation/datasets/perception_target_pilot/sensor.py');sys.modules['sensor']=sensor;geometry=load('handoff_audit_geometry',ROOT/'experiments/social_navigation/datasets/perception_target_pilot/geometry.py');reports=[];independence=[]
 try:
  for case in protocol['episodes']:reports.append(audit_episode(dataset,case,protocol,geometry))
  # Standalone reader runs from a temporary directory with Python isolated mode.
  script="import sys,json;from pathlib import Path;sys.path.insert(0,sys.argv[1]);from loader import Dataset;d=Dataset(sys.argv[2]);out=[]\nfor m in d.metadata['episodes']:\n e=d.episode(m['episode_id']);i=len(e)//2;x=e.observation(i);a=e.action_condition(i);y=e.supervision(i);out.append(dict(id=m['episode_id'],windows=len(e),depth_shape=list(x['depth_history'].shape),future_actions=list(a['executed_action_future'].shape),future_bev=list(y['future_bev']['occupancy'].shape)))\nprint(json.dumps(out))"
  with tempfile.TemporaryDirectory(dir=a.output) as temp:
   result=subprocess.run([sys.executable,'-I','-c',script,str(dataset/'tools'),str(dataset)],cwd=temp,capture_output=True,text=True,check=True);standalone=json.loads(result.stdout)
  loader=load('handoff_independence_loader',dataset/'tools/loader.py')
  for case in protocol['episodes']:
   with tempfile.TemporaryDirectory(dir=a.output) as temp:
    copy=Path(temp);(copy/'dataset.json').write_text(json.dumps(dict(episodes=[next(e for e in meta['episodes'] if e['episode_id']==case['episode_id'])])));target=copy/'episodes'/case['episode_id'];(target/'inputs').mkdir(parents=True)
    for file in (dataset/'episodes'/case['episode_id']/'inputs').iterdir():os.link(file,target/'inputs'/file.name)
    original=loader.Dataset(dataset).episode(case['episode_id']);i=len(original)//2;portable=loader.Dataset(copy).episode(case['episode_id']);assert fingerprint(original.observation(i))==fingerprint(portable.observation(i));independence.append(case['episode_id'])
  report=dict(schema='handoff_pilot_audit/1',status='PASS',episodes=reports,total_windows=sum(v['windows'] for v in reports),wm_target_valid=sum(v['wm_target_valid'] for v in reports),depth_frames_bitwise=sum(v['raw_depth_bitwise_equal'] for v in reports),all_bev_grids_replayed=sum(v['grids_replayed'] for v in reports),standalone_numpy_loader=standalone,label_independent_observation=independence,training_ready=False,WM_models_trained=0,RL_models_trained=0)
 except Exception as error:
  report=dict(schema='handoff_pilot_audit/1',status='FAIL',error=type(error).__name__+': '+str(error),completed_episode_reports=reports,training_ready=False)
  (a.output/'audit.json').write_text(json.dumps(report,indent=2)+'\n');raise
 (a.output/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print('PILOT AUDIT',report['status'],report['total_windows'],flush=True)

if __name__=='__main__':main()
