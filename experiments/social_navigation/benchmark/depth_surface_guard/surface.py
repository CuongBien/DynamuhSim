"""Sensor-only evidence for vertical planar surfaces; no human semantic claim."""
import json,math
from pathlib import Path
import numpy as np
import torch
CFG=json.loads(Path(__file__).with_name('protocol.json').read_text())

def fit_line(points,device,forbidden_normal=None):
 c=CFG;n=len(points)
 if n<c['min_plane_points']:return None
 rng=np.random.default_rng(c['seed']);pairs=rng.integers(0,n,size=(c['line_trials'],2));delta=points[pairs[:,1],:2]-points[pairs[:,0],:2];length=np.linalg.norm(delta,axis=1);valid=length>1e-4;pairs=pairs[valid];delta=delta[valid];length=length[valid]
 if not len(pairs):return None
 normals=np.column_stack((-delta[:,1],delta[:,0]))/length[:,None];offset=np.sum(normals*points[pairs[:,0],:2],axis=1)
 if forbidden_normal is not None:
  ok=np.abs(normals@forbidden_normal)<=math.cos(math.radians(c['min_normal_angle_deg']));normals=normals[ok];offset=offset[ok]
 if not len(normals):return None
 q=torch.as_tensor(points,dtype=torch.float64,device=device);normal=torch.as_tensor(normals,dtype=torch.float64,device=device);d=torch.as_tensor(offset,dtype=torch.float64,device=device)
 mask=torch.abs(q[:,:2]@normal.T-d)<=c['plane_tolerance_m'];counts=mask.sum(dim=0);z=q[:,2:3].expand_as(mask);height=torch.where(mask,z,-torch.inf).amax(dim=0)-torch.where(mask,z,torch.inf).amin(dim=0);eligible=(counts>=c['min_plane_points'])&(height>=c['min_plane_vertical_span_m']);score=torch.where(eligible,counts,-1);best=int(score.argmax().item())
 if int(score[best].item())<0:return None
 return dict(normal=normals[best],offset=float(offset[best]),mask=mask[:,best].cpu().numpy(),points=int(counts[best].item()),vertical_span_m=float(height[best].item()))

def evidence(points,device='cuda'):
 q=np.asarray(points,dtype=float).reshape(-1,3);n=len(q);c=CFG
 out=dict(point_count=n,planar_surface_evidence=False,explained_fraction=0.,planes=[],semantic_class='unknown_geometry_candidate',semantic_valid=False)
 if n<c['min_cluster_points']:out['reason']='insufficient_points';return out
 first=fit_line(q,device)
 if first is None:out['reason']='no_supported_vertical_plane';return out
 planes=[first];covered=first['mask'].copy();fraction=float(covered.mean())
 if fraction<c['min_explained_fraction']:
  remainder=np.flatnonzero(~covered);second=fit_line(q[remainder],device,first['normal'])
  if second is not None:
   covered[remainder[second['mask']]]=True;planes.append(second);fraction=float(covered.mean())
 out['planes']=[dict(normal_xy=v['normal'].tolist(),offset_m=v['offset'],points=v['points'],vertical_span_m=v['vertical_span_m']) for v in planes];out['explained_fraction']=fraction;out['planar_surface_evidence']=fraction>=c['min_explained_fraction'];out['reason']='strong_vertical_planar_surface' if out['planar_surface_evidence'] else 'unexplained_surface_retained';return out
