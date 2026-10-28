"""Sensor-only geometry; no file access, world state, scene ID, labels or time features."""
import numpy as np
DEPTH_NAMES=['log_points','span_x','span_y','span_z']+[f'z_q{q}' for q in [0,10,25,50,75,90,100]]+[f'xy_radius_q{q}' for q in [25,50,75,95]]+['std_major','std_middle','std_minor']+[f'z_band_fraction_{i}' for i in range(6)]+[f'z_band_radius90_{i}' for i in range(6)]
LIDAR_NAMES=['log_local_points','span_x','span_y','std_major','std_minor','depth_patch_center_gap','radius95','mean_ray_step']
Z_EDGES=np.array([.04,.3,.6,.9,1.2,1.5,1.8])
def extract(sample,geometry):
 p=np.asarray(sample['points_xyz'],dtype=float);assert p.ndim==2 and p.shape[1]==3 and len(p)>0 and np.isfinite(p).all();center=np.median(p[:,:2],axis=0);q=p.copy();q[:,:2]-=center;radius=np.linalg.norm(q[:,:2],axis=1);std=np.sqrt(np.maximum(np.linalg.eigvalsh(np.cov(p.T,bias=True)),0))[::-1] if len(p)>1 else np.zeros(3);band=np.clip(np.searchsorted(Z_EDGES,p[:,2],side='right')-1,0,5);fractions=np.bincount(band,minlength=6)/len(p);radii=[np.quantile(radius[band==i],.9) if np.any(band==i) else 0. for i in range(6)]
 depth=np.r_[np.log1p(len(p)),np.ptp(p,axis=0),np.quantile(p[:,2],[0,.1,.25,.5,.75,.9,1]),np.quantile(radius,[.25,.5,.75,.95]),std,fractions,radii].astype(np.float32);lidar=np.zeros(8,np.float32);mask=False
 if sample['scan_valid']:
  ranges=np.asarray(sample['scan_ranges'],dtype=float);assert len(ranges)==360 and geometry['rays']==360 and geometry['frame_id']=='base_scan';angles=geometry['angle_min']+np.arange(len(ranges)-1)*geometry['angle_increment'];valid=np.isfinite(ranges[:-1])&(ranges[:-1]>=geometry['range_min'])&(ranges[:-1]<=geometry['range_max']);xy=np.zeros((len(angles),2));xy[valid]=ranges[:-1][valid,None]*np.c_[np.cos(angles[valid]),np.sin(angles[valid])];local=valid&(np.linalg.norm(xy-center,axis=1)<=.6);xy=xy[local]
  if len(xy)>=3:
   mid=np.median(xy,axis=0);eigen=np.sqrt(np.maximum(np.linalg.eigvalsh(np.cov(xy.T,bias=True)),0))[::-1];lidar=np.r_[np.log1p(len(xy)),np.ptp(xy,axis=0),eigen,np.linalg.norm(mid-center),np.quantile(np.linalg.norm(xy-mid,axis=1),.95),np.linalg.norm(np.diff(xy,axis=0),axis=1).mean()].astype(np.float32);mask=True
 assert depth.shape==(30,) and lidar.shape==(8,) and np.isfinite(depth).all() and np.isfinite(lidar).all();return depth,lidar,mask
