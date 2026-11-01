"""Separate sensor-only input BEV and GT-side future observed BEV, no semantic inference."""
import json,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
sys.path.insert(0,str(BENCH/'perception_target_pilot'))
from sensor import project,odom_at
from geometry import interpolate,nearest_frame,warp,raster

def run(norm,episode,meta,capture_audit):
 x=np.load(norm/'samples.npz');o=np.load(norm/'observations.npz');depth=np.load(norm/'depth.npy',mmap_mode='r');cal=json.loads((norm/'calibration.json').read_text());transform=np.array(cal['optical_to_base_link']);cfg=json.loads((HERE/'export_protocol.json').read_text());scene=[json.loads(v) for v in (norm/'human_states.jsonl').read_text().splitlines()];times=np.array([v['stamp_ns'] for v in scene],np.int64);poses=np.array([[v['robot'][k] for k in ['x','y','yaw']] for v in scene]);poses[:,2]=np.unwrap(poses[:,2]);stamps=o['depth__stamp_ns'];assert np.all(np.diff(stamps)>0);anchor=x['anchor_ns'];offset=np.array(cfg['future_offsets_ns'],np.int64);queries=anchor[:,None]+offset;indices,valid=nearest_frame(stamps,queries,cfg['nearest_future_depth_tolerance_ns']);selected=stamps[indices];ap,av=interpolate(times,poses,anchor);fp,fv=interpolate(times,poses,selected);valid&=fv&av[:,None]&(selected>anchor[:,None]);n=len(anchor);current=np.full((n,64,64),-1,np.int8);future=np.full((n,3,64,64),-1,np.int8);input_valid=np.zeros(n,bool);source_odom_stamp=np.full(n,-1,np.int64);source_odom_available=np.full(n,-1,np.int64);anchor_odom_stamp=source_odom_stamp.copy();anchor_odom_available=source_odom_available.copy();current_index=x['depth_index'][:,-1];needed=np.unique(np.r_[indices[valid],current_index]);points={int(i):project(depth[i],cal['k'],transform) for i in needed};od=o['odom__values']
 for i in range(n):
  di=int(current_index[i]);source=odom_at(int(stamps[di]),o['odom__stamp_ns'],o['odom__available_ns'],od);target=odom_at(int(anchor[i]),o['odom__stamp_ns'],o['odom__available_ns'],od)
  if source is not None and target is not None and x['depth_valid'][i,-1]:
   p=warp(points[di],source['pose'],target['pose']);origin=warp(transform[None,:3,3],source['pose'],target['pose'])[0];current[i]=raster(p,origin);input_valid[i]=True;source_odom_stamp[i]=source['stamp_ns'];source_odom_available[i]=source['available_ns'];anchor_odom_stamp[i]=target['stamp_ns'];anchor_odom_available[i]=target['available_ns']
  for j in range(3):
   if valid[i,j]:
    p=warp(points[int(indices[i,j])],fp[i,j],ap[i]);origin=warp(transform[None,:3,3],fp[i,j],ap[i])[0];future[i,j]=raster(p,origin)
    if not np.any(future[i,j]!=-1):valid[i,j]=False
  if i%100==0:print(meta['episode_id'],'BEV',i,'/',n,flush=True)
 eligible=input_valid&valid.all(1)&bool(capture_audit['moving_episode_accepted']);labels=episode/'labels';inp=episode/'inputs';np.savez_compressed(inp/'bev_anchor_estimated_SENSOR_ONLY.npz',anchor_ns=anchor,depth_index=current_index,depth_stamp_ns=stamps[current_index],source_odom_stamp_ns=source_odom_stamp,source_odom_available_ns=source_odom_available,anchor_odom_stamp_ns=anchor_odom_stamp,anchor_odom_available_ns=anchor_odom_available,occupancy=current,frame_valid=input_valid)
 np.savez_compressed(labels/'future_observed_bev_LABEL_ONLY.npz',anchor_ns=anchor,offset_ns=offset,query_ns=queries,depth_index=indices,depth_stamp_ns=selected,depth_time_error_ns=selected-queries,anchor_pose_world_gt=ap,future_pose_world_gt=fp,occupancy=future,frame_valid=valid,cell_loss_valid=(future!=-1)&valid[:,:,None,None])
 np.savez_compressed(labels/'task_masks_LABEL_ONLY.npz',wm_observed_bev_train_valid=eligible,executed_action_condition_valid=np.ones(n,bool),m1_track_aligned_valid=np.zeros((n,30,6),bool),persona_valid=np.zeros((n,6),bool),rl_reward_valid=np.zeros(n,bool),rl_reward=np.full(n,np.nan,np.float32))
 return dict(windows=n,wm_observed_bev_eligible_windows=int(eligible.sum()),input_bev_valid=int(input_valid.sum()),future_frame_valid=int(valid.sum()),future_frames=int(valid.size),unknown_target_cells=int((future<0).sum()),observed_target_fraction=(future!=-1).mean(axis=(0,2,3)).tolist(),max_future_depth_time_error_ns=int(abs(selected-queries).max()))
