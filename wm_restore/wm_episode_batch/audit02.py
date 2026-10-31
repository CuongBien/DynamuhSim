#!/usr/bin/env python3
"""Raw/causal/geometry/portable integrity of fixed corpus; no learned model evaluation."""
import hashlib,json,sys,tempfile,shutil,os
from pathlib import Path
import numpy as np
from loader import Dataset
from targets import raster
from export import HERE,REPO,DATASET,sha,verify,dump

def points(depth,k,t):
 v,u=np.mgrid[0:depth.shape[0]:4,0:depth.shape[1]:4];z=depth[::4,::4];ok=np.isfinite(z)&(z>=.28)&(z<=5.);q=np.c_[(u[ok]-k[2])*z[ok]/k[0],(v[ok]-k[5])*z[ok]/k[4],z[ok]];return q@t[:3,:3].T+t[:3,3]
def transform(p,source,anchor):
 c,s=np.cos(source[2]),np.sin(source[2]);wx=source[0]+p[:,0]*c-p[:,1]*s;wy=source[1]+p[:,0]*s+p[:,1]*c;c,s=np.cos(anchor[2]),np.sin(anchor[2]);q=p.copy();q[:,0]=(wx-anchor[0])*c+(wy-anchor[1])*s;q[:,1]=-(wx-anchor[0])*s+(wy-anchor[1])*c;return q
def digest_observation(sample):
 h=hashlib.sha256()
 for key in sorted(sample['history']):h.update(key.encode());h.update(np.asarray(sample['history'][key]).tobytes())
 h.update(sample['depth_history'].tobytes());h.update(sample['bev_anchor_estimated'].tobytes());h.update(str(sample['bev_frame_valid']).encode());return h.hexdigest()
def run():
 verify();capture=json.loads((HERE/'capture_frozen.json').read_text());assert all(sha(REPO/n)==h for n,h in capture['source_sha256'].items());cfg=json.loads((HERE/'protocol.json').read_text());ds=Dataset(DATASET);assert ds.metadata['complete'] and len(ds.metadata['episodes'])==8;assert ds.metadata['source_capture_freeze_sha256']==sha(HERE/'capture_frozen.json') and ds.metadata['source_export_freeze_sha256']==sha(HERE/'export_frozen.json');reports=[];seen={};duplicates=[];replays=0;portable=0;frame_hashes={};test_denied=0;family_split={};source_hashes={};np.seterr(invalid='ignore')
 review=json.loads((HERE/'reports/quarantine_review02.json').read_text());assert ds.metadata['review_amendment_sha256']==sha(HERE/'reports/quarantine_review02.json');quarantined={q['episode_id'] for q in review['quarantined']};assert quarantined=={'wm_standing_47','wm_walking_49'};assert len(ds.metadata['episodes'])+len(quarantined)==10
 for q in review['quarantined']:
  rejected=HERE/'generated/runs'/q['episode_id'];assert sha(rejected/'result.json')==q['raw_result_sha256'];assert not json.loads((rejected/'capture_audit.json').read_text())['infrastructure_pass'];assert not (DATASET/'episodes'/q['episode_id']).exists()
 accepted_plan=[v for v in cfg['episodes'] if v['episode_id'] not in quarantined]
 for meta,plan in zip(ds.metadata['episodes'],accepted_plan,strict=True):
  sid=meta['episode_id'];assert sid==plan['episode_id'] and meta['split']==plan['split'];family_split.setdefault(meta['family'],set()).add(meta['split']);root=DATASET/meta['relative_path'];inventory=json.loads((root/'inventory.json').read_text());assert all((root/n).stat().st_size==v['bytes'] and sha(root/n)==v['sha256'] for n,v in inventory.items());assert not any(f.is_symlink() for f in root.rglob('*'))
  e=ds.episode(sid,allow_test=True);x=e.x;raw=HERE/'generated/runs'/sid;ca=json.loads((raw/'capture_audit.json').read_text());r=json.loads((raw/'result.json').read_text());ra=json.loads((raw/'raw_export_audit.json').read_text());assert ca['infrastructure_pass'] and ca['raw_result_sha256']==sha(raw/'result.json');assert ra['status']=='PASS' and ra['raw_future_actions_exact'] and ra['no_gt_or_future_fields_in_observations'];source_hashes[sid]=sha(raw/'result.json');assert meta['raw_depth_frames']==ra['depth_frames_bitwise_equal']==len(e.depth);runtime=REPO/r['runtime'];assert sha(runtime/'mecanum_camera.sdf')==json.loads((HERE/'reports/preflight.json').read_text())['camera_sha256'];assert r['sim_budget_s']==90. and r['live_parameters']['/controller_server']['FollowPath.vx_max']==.3
  for modality in ['scan','imu','odom','action','depth']:
   assert np.all(x[modality+'_stamp_ns']<=x['history_query_ns']) and np.all(x[modality+'_available_ns']<=x['history_query_ns'])
  assert np.all(x['history_query_ns']<=x['anchor_ns'][:,None]);assert not any(any(w in key for w in ['human','future','persona','world','backend']) for key in x);assert np.all(e.bev['source_odom_stamp_ns'][e.bev['frame_valid']]<=e.bev['depth_stamp_ns'][e.bev['frame_valid']]);assert np.all(e.bev['source_odom_available_ns'][e.bev['frame_valid']]<=e.bev['depth_stamp_ns'][e.bev['frame_valid']]);assert np.all(e.bev['anchor_odom_stamp_ns'][e.bev['frame_valid']]<=x['anchor_ns'][e.bev['frame_valid']]);assert np.all(e.bev['anchor_odom_available_ns'][e.bev['frame_valid']]<=x['anchor_ns'][e.bev['frame_valid']])
  y=dict(np.load(root/'labels/future_trajectories_actions_LABEL_ONLY.npz'));z=dict(np.load(root/'labels/future_observed_bev_LABEL_ONLY.npz'));m=dict(np.load(root/'labels/task_masks_LABEL_ONLY.npz'));obs=dict(np.load(root/'inputs/observations_SENSOR_ONLY.npz'));stamps=obs['depth__stamp_ns'];np.testing.assert_array_equal(z['query_ns'],x['anchor_ns'][:,None]+np.array([500000000,1000000000,3000000000]));np.testing.assert_array_equal(z['depth_stamp_ns'],stamps[z['depth_index']]);assert (z['depth_stamp_ns']>x['anchor_ns'][:,None]).all();assert abs(z['depth_time_error_ns']).max()<=50000000;np.testing.assert_array_equal(z['cell_loss_valid'],(z['occupancy']!=-1)&z['frame_valid'][:,:,None,None]);assert set(np.unique(z['occupancy']))<={-1,0,1};assert not m['persona_valid'].any() and not m['m1_track_aligned_valid'].any() and not m['rl_reward_valid'].any() and np.isnan(m['rl_reward']).all();np.testing.assert_array_equal(m['wm_observed_bev_train_valid'],e.bev['frame_valid']&z['frame_valid'].all(1)&ca['moving_episode_accepted']);assert int(m['wm_observed_bev_train_valid'].sum())==meta['wm_observed_bev_eligible_windows'];assert not y['persona_valid'].any() and (y['persona_index']==-1).all();np.testing.assert_array_equal(y['executed_action_future'][:,0],x['action'][:,-1])
  scene=[json.loads(v) for v in (root/'labels/human_states_LABEL_ONLY.jsonl').read_text().splitlines()];ts=np.array([v['stamp_ns'] for v in scene]);rp=np.array([[v['robot'][k] for k in ['x','y','yaw']] for v in scene]);rp[:,2]=np.unwrap(rp[:,2]);ap=np.stack([np.interp(x['anchor_ns'],ts,rp[:,j]) for j in range(3)],axis=-1);fp=np.stack([np.interp(z['depth_stamp_ns'],ts,rp[:,j]) for j in range(3)],axis=-1);np.testing.assert_allclose(ap,z['anchor_pose_world_gt'],atol=1e-10,rtol=0);np.testing.assert_allclose(fp,z['future_pose_world_gt'],atol=1e-10,rtol=0);T=np.array(e.calibration['optical_to_base_link']);k=e.calibration['k']
  for i in sorted({0,len(e)//2,len(e)-1}):
   for j in range(3):
    if z['frame_valid'][i,j]:
     p=points(e.depth[z['depth_index'][i,j]],k,T);expected=raster(transform(p,fp[i,j],ap[i]),transform(T[None,:3,3],fp[i,j],ap[i])[0]);np.testing.assert_array_equal(expected,z['occupancy'][i,j]);replays+=1
   if e.bev['frame_valid'][i]:
    def odom(query):
     valid=(obs['odom__stamp_ns']<=query)&(obs['odom__available_ns']<=query)&(query-obs['odom__stamp_ns']<=150000000);ids=np.flatnonzero(valid);return obs['odom__values'][ids[np.argmax(obs['odom__stamp_ns'][ids])],:3]
    dp=odom(int(e.bev['depth_stamp_ns'][i]));pa=odom(int(x['anchor_ns'][i]));p=points(e.depth[e.bev['depth_index'][i]],k,T);expected=raster(transform(p,dp,pa),transform(T[None,:3,3],dp,pa)[0]);np.testing.assert_array_equal(expected,e.bev['occupancy'][i]);replays+=1
   inp=e.observation(i);assert set(inp)=={'history','depth_history','bev_anchor_estimated','bev_frame_valid','calibration'};assert e.supervision(i)['future_bev']['offset_ns'].shape==(3,)
  # Fingerprint only sensor/action histories; no timestamp, ID or labels that could hide duplicates.
  local_hash={}
  for di in np.unique(x['depth_index']):
   d=np.asarray(e.depth[di]);finite=np.isfinite(d);q=np.full(d.shape,-1,np.int32);q[finite]=np.rint(np.minimum(d[finite],100)*1000).astype(np.int32);local_hash[int(di)]=hashlib.sha256(q.tobytes()).digest()
  for i in range(len(e)):
   h=hashlib.sha256(b''.join(local_hash[int(di)] for di in x['depth_index'][i]));h.update(np.rint(x['action'][i]*1000).astype(np.int32).tobytes());h.update(np.rint(x['odom_local_pose_and_twist'][i]*1000).astype(np.int32).tobytes());finger=h.hexdigest()
   if finger in seen and seen[finger][0]!=meta['split']:duplicates.append(dict(current=sid,index=i,previous=seen[finger]))
   seen[finger]=(meta['split'],sid,i)
  if meta['split']=='test':
   try:ds.episode(sid)
   except PermissionError:test_denied+=1
   else:raise AssertionError('Test API not sealed')
  else:
   # Read-only input hardlinks, independent copied LABEL files; destructive edits only in temporary copy.
   with tempfile.TemporaryDirectory(dir=HERE/'generated') as directory:
    copy=Path(directory);(copy/'dataset.json').write_text(json.dumps(dict(episodes=[meta])));dest=copy/'episodes'/sid;(dest/'inputs').mkdir(parents=True);(dest/'labels').mkdir()
    for file in (root/'inputs').iterdir():os.link(file,dest/'inputs'/file.name)
    for file in (root/'labels').iterdir():shutil.copy2(file,dest/'labels'/file.name)
    temp=Dataset(copy);i=len(e)//2;before=digest_observation(temp.episode(sid).observation(i))
    for f in (dest/'labels').iterdir():f.unlink()
    after=digest_observation(temp.episode(sid).observation(i));assert before==after;portable+=1
  reports.append(dict(episode_id=sid,split=meta['split'],windows=len(e),wm_valid=int(m['wm_observed_bev_train_valid'].sum()),depth_bitwise=ra['depth_frames_bitwise_equal'],navigation_status=meta['navigation_status'],max_robot_translation_m=ca['max_robot_translation_from_start_m'],goal_valid_windows=int(x['goal_valid'].sum())))
 assert all(len(v)==1 for v in family_split.values());assert not duplicates,'Cross-split duplicate sensor/action windows';verify();result=dict(status='PASS',episodes=reports,total_episodes=8,planned_attempts=10,quarantined_episodes=2,total_windows=sum(v['windows'] for v in reports),wm_observed_bev_valid_windows=sum(v['wm_valid'] for v in reports),total_raw_depth_bitwise=sum(v['depth_bitwise'] for v in reports),geometry_grids_replayed=replays,cross_split_duplicate_windows=len(duplicates),portable_label_independence_episodes=portable,test_episodes_default_denied=test_denied,WM_models_evaluated=0,persona_valid=False,RL_reward_valid=False,HuNavSim_used=False,raw_result_sha256=source_hashes)
 dump(HERE/'reports/audit.json',result);print(json.dumps({k:v for k,v in result.items() if k not in ['episodes','raw_result_sha256']},indent=2))
if __name__=='__main__':run()
