"""Sensor-only ray indexing and conservative corroboration, no simulator/labels."""
import numpy as np

def prepare_scan(ranges,geometry):
 raw=np.asarray(ranges,dtype=np.float32);n=len(raw);angle=geometry['angle_min']+np.arange(n)*geometry['angle_increment']
 assert n==geometry['rays'] and np.all(np.diff(angle)>0) and not np.any(np.isfinite(raw)&(raw<0))
 # The original 360-ray scan includes both 0 and 2pi. Remove only that repeated bearing.
 duplicate=abs((angle[-1]-angle[0])-2*np.pi)<1e-5
 indices=np.arange(n-1 if duplicate else n);wrapped=(angle[indices]+np.pi)%(2*np.pi)-np.pi
 order=np.argsort(wrapped);indices=indices[order];wrapped=wrapped[order]
 step=np.diff(wrapped);assert np.max(abs(step-np.median(step)))<1e-5
 # Existing sensor grid is centered around forward after cyclic ordering; no resampling.
 assert abs(wrapped[0]+wrapped[-1])<1e-5
 scan=raw[indices].copy();invalid=~np.isfinite(scan)|(scan==0);scan[invalid]=29.99
 return scan,wrapped.astype(np.float32),indices,dict(raw_rays=n,model_rays=len(scan),duplicate_endpoint_removed=duplicate,no_return_replaced=int(invalid.sum()),fov_deg=float(np.rad2deg(wrapped[-1]-wrapped[0])),interpolated=False)

def corroborate(proposal_xy,learned_xy,confidences,gate=.6,margin=.15):
 points=np.asarray(proposal_xy,dtype=float).reshape(-1,2);dets=np.asarray(learned_xy,dtype=float).reshape(-1,2);conf=np.asarray(confidences).reshape(-1)
 assert len(dets)==len(conf);rows=[dict(proposal_index=i,semantic_status='unknown',person_confidence=None,learned_detection_index=None,association_accepted=False) for i in range(len(points))]
 if not len(points) or not len(dets):return rows
 cost=np.linalg.norm(points[:,None,:]-dets[None,:,:],axis=-1)
 for i in range(len(points)):
  j=int(np.argmin(cost[i]));alternatives=min(np.delete(cost[i],j).min(initial=np.inf),np.delete(cost[:,j],i).min(initial=np.inf))
  if int(np.argmin(cost[:,j]))==i and cost[i,j]<=gate and alternatives-cost[i,j]>=margin:
   rows[i].update(semantic_status='person_supported_experimental',person_confidence=float(conf[j]),learned_detection_index=j,corroboration_distance_m=float(cost[i,j]))
 return rows
