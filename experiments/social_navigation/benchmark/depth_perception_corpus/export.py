#!/usr/bin/env python3
"""Build a labelled perception pilot. Counterfactuals stay in the LABEL branch."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_dilation,label
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
sys.path[:0]=[str(BENCH/'depth_identity_guard/parent'),str(BENCH/'depth_visibility_pilot')]
from snapshots.sensor import project,candidates
from labels import ownership,revealed

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
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

def build_scene(capture,scene,root,cfg):
 cap=json.loads((capture/'result.json').read_text());assert cap['capture_accepted'];scene_id=scene['scene_id'];inp=root/'inputs'/scene_id;lab=root/'labels'/scene_id;inp.mkdir(parents=True,exist_ok=False);lab.mkdir(parents=True,exist_ok=False);cal=json.loads((root/'inputs/calibration_SENSOR_ONLY.json').read_text());camera=json.loads((capture/'calibration.json').read_text())['full'];assert np.array_equal(camera['k'],cal['k'])
 modes=['full','null']+[prefix+str(i) for prefix in ['drop','only'] for i in range(1,7)];a={mode:np.load(capture/(mode+'.npy'),mmap_mode='r') for mode in modes};selected=scene['selected_depth_indices'];stamps=np.load(capture/'stamp_ns.npy');available=json.loads((capture/'depth_available.json').read_text())['full'];depth=np.stack([a['full'][i] for i in selected]);np.savez_compressed(inp/'depth_SENSOR_ONLY.npz',depth=depth,stamp_ns=stamps[selected],available_ns=np.array(available,dtype=np.int64)[selected]);(inp/'lidar_SENSOR_ONLY.npz').write_bytes((capture/'scan_SENSOR_ONLY.npz').read_bytes());(inp/'lidar_geometry_SENSOR_ONLY.json').write_bytes((capture/'scan_geometry.json').read_bytes())
 points=[];pixels=[];offset=[0];bbox=[];frame_index=[];proposal_in_frame=[];targets=[];validity=[];person_ids=[];purity=[];unknown_counts=[];label_images=[];details=[];visibilities=[];n=0
 for frame,raw_index in enumerate(selected):
  full=a['full'][raw_index];owner,qa=label_pixels(full,[a['drop'+str(i)][raw_index] for i in range(1,7)],[a['only'+str(i)][raw_index] for i in range(1,7)],a['null'][raw_index]);assert qa['final_unknown_pixels']/owner.size<=cfg['max_unknown_pixel_fraction'];label_images.append(owner);groups=clusters(full,cal)
  visible=[]
  for hid in range(1,7):
   isolated=a['only'+str(hid)][raw_index];amodal=np.isfinite(isolated)&(isolated>=.28)&(isolated<=5.);count=int(np.sum(owner==hid));denom=int(amodal.sum());visible.append(dict(person_id=hid,visible_pixels=count,isolated_in_view_pixels=denom,visible_fraction=count/denom if denom else None))
  visibilities.append(dict(frame_index=frame,source_depth_index=raw_index,stamp_ns=int(stamps[raw_index]),persons=visible,label_qa=qa))
  for pidx,(q,uv) in enumerate(groups):
   point_owner=owner[uv[:,1],uv[:,0]];label_info,hist=classify(point_owner,cfg);points.append(q.astype(np.float32));pixels.append(uv.astype(np.uint16));offset.append(offset[-1]+len(q));box=[int(uv[:,0].min()),int(uv[:,1].min()),int(uv[:,0].max()+1),int(uv[:,1].max()+1)];bbox.append(box);frame_index.append(frame);proposal_in_frame.append(pidx);targets.append(label_info['class_index']);validity.append(label_info['label_valid']);person_ids.append(label_info['person_id']);purity.append(label_info['person_purity']);unknown_counts.append(label_info['unknown_point_count']);details.append(dict(sample_index=n,frame_index=frame,source_depth_index=raw_index,proposal_index=pidx,label_counts=hist,point_count=len(q)));n+=1
 np.savez_compressed(inp/'proposals_SENSOR_ONLY.npz',points_xyz=np.concatenate(points) if points else np.empty((0,3),np.float32),point_pixel_uv=np.concatenate(pixels) if pixels else np.empty((0,2),np.uint16),point_offsets=np.array(offset,np.int64),bbox_uv=np.array(bbox,np.int16).reshape(-1,4),frame_index=np.array(frame_index,np.int16),proposal_in_frame=np.array(proposal_in_frame,np.int16))
 np.savez_compressed(lab/'proposal_labels_LABEL_ONLY.npz',class_index=np.array(targets,np.int8),label_valid=np.array(validity,bool),person_id=np.array(person_ids,np.int16),person_purity=np.array(purity,np.float32),unknown_point_count=np.array(unknown_counts,np.int32),persona_index=np.full(n,-1,np.int8),persona_valid=np.zeros(n,bool))
 np.savez_compressed(lab/'instance_pixels_LABEL_ONLY.npz',person_instance=np.stack(label_images));(lab/'proposal_details_LABEL_ONLY.json').write_text(json.dumps(details,indent=2)+'\n');(lab/'visibility_LABEL_ONLY.json').write_text(json.dumps(visibilities,indent=2)+'\n')
 raw_hash={mode+'.npy':sha(capture/(mode+'.npy')) for mode in modes};raw_hash.update({name:sha(capture/name) for name in ['scan_SENSOR_ONLY.npz','result.json','calibration.json','depth_available.json','stamp_ns.npy']});(lab/'source_provenance_LABEL_ONLY.json').write_text(json.dumps(dict(capture=str(capture.relative_to(REPO)),sha256=raw_hash),indent=2)+'\n')
 return dict(scene_id=scene_id,split=scene['split'],family=scene['family'],moving=scene['moving'],frames=len(selected),proposals=n,person_samples=targets.count(1),environment_samples=targets.count(0),ambiguous_samples=targets.count(-1),all_proposals_retained=True,raw_frame_indices=selected)

def run(out):
 frozen=json.loads((HERE/'frozen.json').read_text());assert all(sha(REPO/p)==h for p,h in frozen['source_sha256'].items());out.mkdir(parents=True,exist_ok=False);(out/'inputs').mkdir();(out/'labels').mkdir();(out/'inputs/calibration_SENSOR_ONLY.json').write_bytes((BENCH/'lidar_person_pilot/calibration_SENSOR_ONLY.json').read_bytes());protocol=json.loads((HERE/'protocol.json').read_text());scenes=json.loads((HERE/'scenes_LABEL_ONLY.json').read_text());summaries=[]
 for scene in scenes:
  summary=build_scene(HERE/'generated/runs'/scene['scene_id'],scene,out,protocol['label_rules']);summaries.append(summary);print(json.dumps(summary),flush=True)
 manifest=dict(schema='portable_depth_perception_corpus/1',scenes=summaries,splits=protocol['splits'],input_modalities=['depth','native_lidar'],source_frozen_sha256=sha(HERE/'frozen.json'),class_indices={'unknown':-1,'environment':0,'person':1},data_use='Small simulator-specific perception development pilot; test not evaluated by any model',classifier_validated=False,world_model_training_ready=False,persona_valid=False,hu_nav_sim_used=False)
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(out/'protocol.json').write_bytes((HERE/'protocol.json').read_bytes());(out/'loader.py').write_bytes((HERE/'loader.py').read_bytes());assert all(sha(REPO/p)==h for p,h in frozen['source_sha256'].items());print('Export complete',sum(v['proposals'] for v in summaries),'proposals')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.output.resolve())
