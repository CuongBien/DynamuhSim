"""Frozen pilot label/proposal rules, reusable without running a cohort exporter."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.ndimage import binary_dilation,label
from dynamuhsim.analysis.evidence import load_json,sha256_file,module_from_path
STAGE=Path('experiments/social_navigation/benchmark/depth_perception_corpus')

def corpus_math(repository):
 repository=Path(repository);pins=load_json(repository/STAGE/'frozen.json')['source_sha256'];modules=[]
 for name in ['experiments/social_navigation/benchmark/depth_identity_guard/parent/snapshots/sensor.py','experiments/social_navigation/benchmark/depth_visibility_pilot/labels.py']:
  path=repository/name
  if sha256_file(path)!=pins[name]:raise ValueError('Active corpus math differs from frozen source: '+name)
  modules.append(module_from_path(path,'verified_corpus_'+path.stem))
 project,candidates=modules[0].project,modules[0].candidates
 ownership,revealed=modules[1].ownership,modules[1].revealed
 def label_pixels(full,drops,alone,null,tolerance=.001):
  owner,qa=ownership(full,drops,alone,tolerance);effect=revealed(full,null,tolerance);missing=effect&(owner==0);inconsistent=(owner>0)&~effect;owner[missing|inconsistent]=-1;qa.update(null_effect_without_owner=int(missing.sum()),owner_without_null_effect=int(inconsistent.sum()),final_unknown_pixels=int((owner<0).sum()));return owner,qa

 def clusters(depth,cal):
  points=project(depth,cal['k'],np.array(cal['optical_to_base_link']));v,u=np.mgrid[0:depth.shape[0]:4,0:depth.shape[1]:4];sub=depth[::4,::4];valid=np.isfinite(sub)&(sub>=.28)&(sub<=5.);uv=np.c_[u[valid],v[valid]]
  keep=(points[:,0]>=0)&(points[:,0]<5.3)&(abs(points[:,1])<3.2)&(points[:,2]>=.04)&(points[:,2]<=1.8);p=points[keep];pix=uv[keep];result=[]
  if not len(p):return result
  cells=np.floor((p[:,:2]+[0,3.2])/.08).astype(int);grid=np.zeros((68,82),bool);grid[cells[:,0],cells[:,1]]=True;cc,_=label(binary_dilation(grid,iterations=1),structure=np.ones((3,3)));ident=cc[cells[:,0],cells[:,1]];groups=[(p[ident==i],pix[ident==i]) for i in np.unique(ident)]
  for proposal in candidates(points):
   q,pixels=min(groups,key=lambda pair:np.linalg.norm(np.median(pair[0][:,:2],axis=0)-proposal['xy']));assert len(q)==proposal['points'] and np.allclose(np.median(q[:,:2],axis=0),proposal['xy'],atol=1e-12,rtol=0);result.append((q,pixels))
  return result

 def classify(point_owner,cfg):
  owners=np.asarray(point_owner);humans=sorted(int(i) for i in np.unique(owners) if i>0);unknown=int(np.sum(owners<0));counts={str(int(i)):int(np.sum(owners==i)) for i in np.unique(owners)};ident=-1;target=-1;purity=max([counts.get(str(i),0) for i in humans],default=0)/len(owners)
  if not unknown and not humans:target=0
  elif not unknown and len(humans)==1 and counts[str(humans[0])]>=cfg['min_person_points'] and purity>=cfg['person_purity']:target=1;ident=humans[0]
  return dict(class_index=target,label_valid=target>=0,person_id=ident,person_purity=purity,unknown_point_count=unknown),counts
 return SimpleNamespace(label_pixels=label_pixels,clusters=clusters,classify=classify)
