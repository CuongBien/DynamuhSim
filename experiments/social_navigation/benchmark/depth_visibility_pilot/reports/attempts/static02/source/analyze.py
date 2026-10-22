#!/usr/bin/env python3
"""Label-side validation; detector consumes only original full/null depth arrays."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_dilation,label
from labels import ownership,revealed
from fixture import HERE,BENCH,REPO
PARENT=BENCH/'depth_identity_guard'
sys.path[:0]=[str(PARENT),str(PARENT/'parent')]
from parent.snapshots.sensor import candidates,project
from refinement import enrich
from guard_tracker import IdentityGuard

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def cluster_support(depth,k,tr,pixel_labels,proposals,protocol):
 points=project(depth,k,tr);sub=depth[::4,::4];valid=np.isfinite(sub)&(sub>=.28)&(sub<=5.)
 point_labels=pixel_labels[::4,::4][valid]
 keep=(points[:,0]>=0)&(points[:,0]<5.3)&(abs(points[:,1])<3.2)&(points[:,2]>=.04)&(points[:,2]<=1.8)
 p=points[keep];ids=point_labels[keep];cells=np.floor((p[:,:2]+[0,3.2])/.08).astype(int)
 grid=np.zeros((68,82),bool);grid[cells[:,0],cells[:,1]]=True
 cc,_=label(binary_dilation(grid,iterations=1),structure=np.ones((3,3)));comp=cc[cells[:,0],cells[:,1]]
 clusters=[(p[comp==i],ids[comp==i]) for i in np.unique(comp)]
 result=[]
 for j,proposal in enumerate(proposals):
  q,labels=min(clusters,key=lambda item:np.linalg.norm(np.median(item[0][:,:2],axis=0)-proposal['xy']))
  assert len(q)==proposal['points'] and np.allclose(np.median(q[:,:2],axis=0),proposal['xy'],rtol=0,atol=1e-12)
  counts={int(i):int(np.sum(labels==i)) for i in np.unique(labels)}
  humans=[i for i in range(1,7) if counts.get(i,0)>=protocol['proposal_person_support_min_points']]
  purity=max([counts.get(i,0) for i in range(1,7)],default=0)/len(labels)
  if len(humans)>1:classification='merged_humans'
  elif len(humans)==1 and purity>=protocol['proposal_purity']:classification='pure_person'
  elif counts.get(0,0)/len(labels)>=protocol['proposal_purity']:classification='non_person'
  else:classification='mixed_or_uncertain'
  result.append(dict(proposal_index=j,xy_base=proposal['xy'].tolist(),point_count=len(q),label_counts=counts,classification=classification,person_id=humans[0] if classification=='pure_person' else None,person_purity=purity))
 return result

def run(capture,out):
 out.mkdir(parents=True,exist_ok=False);protocol=json.loads((HERE/'protocol.json').read_text());result=json.loads((capture/'result.json').read_text());assert result['capture_accepted']
 modes=protocol['modes'];arrays={m:np.load(capture/(m+'.npy'),allow_pickle=False) for m in modes};full=arrays['full'][0];eps=protocol['depth_equality_tolerance_m']
 owners,qa=ownership(full,[arrays['drop'+str(i)][0] for i in range(1,7)],[arrays['only'+str(i)][0] for i in range(1,7)],eps)
 null=arrays['null'][0];all_effect=revealed(full,null,eps);qa['all_human_effect_pixels']=int(all_effect.sum());qa['effect_without_owned_label']=int((all_effect&(owners<=0)).sum());qa['owned_without_all_effect']=int(((owners>0)&~all_effect).sum())
 label_stable=True
 for frame in range(len(arrays['full'])):
  other,_=ownership(arrays['full'][frame],[arrays['drop'+str(i)][frame] for i in range(1,7)],[arrays['only'+str(i)][frame] for i in range(1,7)],eps)
  label_stable &= bool(np.array_equal(other,owners))
 qa['labels_stable_all_frames']=label_stable;np.save(out/'person_instance_LABEL_ONLY.npy',owners,allow_pickle=False)
 calibration=json.loads((capture/'calibration.json').read_text())['full'];k=calibration['k']
 # Reuse calibrated original camera optical-to-base, verified against copied static URDF/robot pose.
 calibration_source=BENCH/'depth_identity_guard/generated/runs/identity_seed44_01/normalized/calibration.json';native_cal=json.loads(calibration_source.read_text());assert np.array_equal(k,native_cal['k'])
 tr=np.array(native_cal['optical_to_base_link']);
 persons=[];eligible=[]
 for i in range(1,7):
  alone=arrays['only'+str(i)][0];amodal=np.isfinite(alone)&(alone>=.28)&(alone<=5.);visible=(owners==i)&np.isfinite(full)&(full>=.28)&(full<=5.)
  # Exact sampled, cropped detector support, distinct from full-resolution pixel visibility.
  sub=full[::4,::4];valid=np.isfinite(sub)&(sub>=.28)&(sub<=5.);points=project(full,k,tr);sampled=owners[::4,::4][valid];crop=(points[:,0]>=0)&(points[:,0]<5.3)&(abs(points[:,1])<3.2)&(points[:,2]>=.04)&(points[:,2]<=1.8)
  count=int(visible.sum());n=int(amodal.sum());support=int(np.sum((sampled==i)&crop));ratio=count/n if n else None
  status='outside_detector_range_or_view' if n==0 else 'fully_occluded' if count==0 else 'partially_occluded' if ratio<protocol['occlusion_partial_ratio'] else 'visible'
  ok=count>=protocol['visible_min_pixels'] and support>=protocol['proposal_person_support_min_points']
  if ok:eligible.append(i)
  persons.append(dict(person_id=i,visible_pixels=count,isolated_pixels_in_detector_range=n,visible_fraction=ratio,sampled_cropped_points=support,visibility=status,evaluation_eligible=ok))
 points=project(full,k,tr);base=candidates(points);filtered,_=enrich(points)
 evaluations={}
 for name,depth,proposals,labels in [('unfiltered',full,base,owners),('filtered',full,filtered,owners),('null_unfiltered',null,candidates(project(null,k,tr)),np.zeros(null.shape,dtype=np.int16)),('null_filtered',null,enrich(project(null,k,tr))[0],np.zeros(null.shape,dtype=np.int16))]:
  rows=cluster_support(depth,k,tr,labels,proposals,protocol);clean=[v for v in rows if v['classification']=='pure_person'];detected=sorted(set(v['person_id'] for v in clean)&set(eligible));duplicates=len(clean)-len(set(v['person_id'] for v in clean));missed=sorted(set(eligible)-set(detected)) if not name.startswith('null') else []
  evaluations[name]=dict(proposals=rows,counts={q:sum(v['classification']==q for v in rows) for q in ['pure_person','non_person','merged_humans','mixed_or_uncertain']},eligible_people=eligible if not name.startswith('null') else [],detected_people=detected,missed_people=missed,duplicate_person_proposals=duplicates,instance_precision=len(clean)/len(rows) if rows else None,instance_recall=len(detected)/len(eligible) if eligible and not name.startswith('null') else None)
 tracker=IdentityGuard();last=[]
 for stamp in np.load(capture/'stamp_ns.npy'):
  last=tracker.update(int(stamp),filtered,np.zeros(3))
 confirmed={v['proposal_index'] for v in last if v['confirmed']};evaluations['filtered']['confirmed_nonperson_proposals']=[v['proposal_index'] for v in evaluations['filtered']['proposals'] if v['proposal_index'] in confirmed and v['classification']!='pure_person']
 summary=dict(schema='static_visibility_evaluation/1',capture=str(capture.relative_to(REPO)),capture_accepted=True,label_qa=qa,persons=persons,evaluation=evaluations,static_frames=len(arrays['full']),independent_scenes=1,label_harness_pass=not any(qa[k] for k in ['contradictions','overlap_pixels','uncertain_pixels','effect_without_owned_label','owned_without_all_effect']) and label_stable,semantic_person_validated=False,association_accepted=False,training_ready=False,hu_nav_sim_used=False,limitations=['Designed static frame0 native mesh; not dynamic recording annotation','No occlusion labels retrospectively applied to seed42/43/44','Pixel ownership is renderer intervention truth only, not real-world semantic ground truth','No thresholds/model/default ROS changed'],source_sha256={str(p.relative_to(REPO)):sha(p) for p in list(HERE.glob('*.py'))+[HERE/'protocol.json',HERE/'snapshot.json',calibration_source]},capture_files_sha256={p.name:sha(p) for p in capture.glob('*.npy')})
 (out/'evaluation.json').write_text(json.dumps(summary,indent=2)+'\n')
 np.savez_compressed(out/'representative_LABEL_ONLY.npz',**{k:v[0] for k,v in arrays.items()},person_instance=owners)
 preview(out,arrays,owners,persons,evaluations,k,tr)
 print(json.dumps({k:summary[k] for k in ['label_qa','persons','label_harness_pass']},indent=2));print(json.dumps({n:{k:v[k] for k in ['counts','detected_people','missed_people','instance_precision','instance_recall']} for n,v in evaluations.items()},indent=2))

def preview(out,arrays,owners,persons,evaluation,k,tr):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.colors import ListedColormap,BoundaryNorm
 fig,axes=plt.subplots(2,3,figsize=(16,8));colors=['#aeb6bf','#e74c3c','#2ecc71','#3498db','#f1c40f','#9b59b6','#e67e22']
 axes[0,0].imshow(arrays['full'][0],vmin=.28,vmax=5,cmap='viridis');axes[0,0].set_title('Measured full depth (m), original low camera')
 axes[0,1].imshow(owners,cmap=ListedColormap(colors),vmin=0,vmax=6);axes[0,1].set_title('LABEL ONLY: visible pixel owner, 0 = environment')
 axes[0,2].imshow(arrays['null'][0],vmin=.28,vmax=5,cmap='viridis');axes[0,2].set_title('Measured no-human control (clutter remains)')
 for ax in axes[0]:ax.set_axis_off()
 ids=[p['person_id'] for p in persons];axes[1,0].bar(ids,[p['isolated_pixels_in_detector_range'] for p in persons],color='#dddddd',label='isolated silhouette');axes[1,0].bar(ids,[p['visible_pixels'] for p in persons],color=colors[1:],label='visible');axes[1,0].legend();axes[1,0].set_title('Visibility within detector 0.28-5 m');axes[1,0].set_xlabel('Designed person ID');axes[1,0].set_ylabel('pixels')
 for ax,name,depth in [(axes[1,1],'filtered',arrays['full'][0]),(axes[1,2],'null_filtered',arrays['null'][0])]:
  pts=project(depth,k,tr);ax.scatter(pts[:,0],pts[:,1],s=.8,c='lightgray')
  for q in evaluation[name]['proposals']:
   x,y=q['xy_base'];good=q['classification']=='pure_person';ax.scatter(x,y,s=80,c='green' if good else 'red',marker='o' if good else 'x');ax.text(x,y,' '+str(q['proposal_index'])+':'+q['classification'],fontsize=7)
  ax.set(xlim=(0,5.3),ylim=(-3.2,3.2),xlabel='base x (m)',ylabel='base y (m)',title=name+' proposals (GT only used for scoring)');ax.set_aspect('equal');ax.grid(alpha=.2)
 fig.suptitle('Real Gazebo GPU depth: static visibility diagnostic, NOT AI / NOT train-ready');fig.tight_layout();fig.savefig(out/'visibility_review.png',dpi=150);plt.close(fig)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.capture.resolve(),a.output.resolve())
