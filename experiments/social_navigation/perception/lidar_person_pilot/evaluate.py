#!/usr/bin/env python3
"""LABEL ONLY: score frozen sensor predictions against rendered pixel ownership."""
import argparse,hashlib,importlib.util,json,sys
from pathlib import Path
import numpy as np
from fixture import HERE,BENCH,REPO,BASE
sys.path[:0]=[str(BASE),str(BENCH/'depth_identity_guard/parent')]
from labels import ownership
from snapshots.sensor import project
from refinement import enrich
spec=importlib.util.spec_from_file_location('previous_label_metrics',BASE/'analyze.py');previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def readrows(p):return [json.loads(v) for v in p.read_text().splitlines()]
def evaluate(capture,inference,out):
 out.mkdir(parents=True,exist_ok=False);cfg=json.loads((HERE/'protocol.json').read_text());cal=json.loads((HERE/'calibration_SENSOR_ONLY.json').read_text());tr=np.array(cal['optical_to_base_link']);capture_result=json.loads((capture/'result.json').read_text());assert capture_result['capture_accepted'];condition=capture_result['condition'];sensor=readrows(inference/'corroboration_SENSOR_ONLY.jsonl');arrays={m:np.load(capture/(m+'.npy'),allow_pickle=False) for m in cfg['modes']};evaluated=[];maps=[]
 for frame,row in enumerate(sensor):
  full=arrays['full'][frame];owner,qa=ownership(full,[arrays['drop'+str(i)][frame] for i in range(1,7)],[arrays['only'+str(i)][frame] for i in range(1,7)],cfg['depth_equality_tolerance_m']);assert not any(qa.values());maps.append(owner)
  proposals,_=enrich(project(full,cal['k'],tr));scored=previous.cluster_support(full,cal['k'],tr,owner,proposals,cfg);assert len(scored)==len(row['proposals'])
  sub=full[::4,::4];valid=np.isfinite(sub)&(sub>=.28)&(sub<=5.);points=project(full,cal['k'],tr);labels=owner[::4,::4][valid];crop=(points[:,0]>=0)&(points[:,0]<5.3)&(abs(points[:,1])<3.2)&(points[:,2]>=.04)&(points[:,2]<=1.8)
  eligible=[i for i in range(1,7) if int((owner==i).sum())>=cfg['visible_min_pixels'] and int(((labels==i)&crop).sum())>=cfg['proposal_person_support_min_points']]
  supported=[];false=[];found=[]
  for q,decision in zip(scored,row['proposals']):
   assert np.array_equal(q['xy_base'],decision['xy_base']) and q['point_count']==decision['point_count'];q['sensor_decision']=decision
   if decision['semantic_status']=='person_supported_experimental':
    supported.append(q['proposal_index'])
    if q['classification']=='pure_person':found.append(q['person_id'])
    else:false.append(q['proposal_index'])
  found=sorted(set(found)&set(eligible));evaluated.append(dict(depth_index=frame,stamp_ns=row['stamp_ns'],available_ns=row['available_ns'],selected_scan_index=row['selected_scan_index'],eligible_people=eligible,supported_people=found,unsupported_visible_people=sorted(set(eligible)-set(found)),supported_proposals=supported,false_supported_proposals=false,unknown_proposals=[q['proposal_index'] for q in scored if q['proposal_index'] not in supported],all_proposals_retained=True,proposals_LABEL_ONLY=scored,label_qa=qa))
 np.save(out/'owner_LABEL_ONLY.npy',np.stack(maps),allow_pickle=False)
 counts=dict(depth_frames=len(evaluated),supported_rows=sum(len(v['supported_proposals']) for v in evaluated),false_supported_rows=sum(len(v['false_supported_proposals']) for v in evaluated),unknown_rows=sum(len(v['unknown_proposals']) for v in evaluated),eligible_person_rows=sum(len(v['eligible_people']) for v in evaluated),supported_person_rows=sum(len(v['supported_people']) for v in evaluated))
 verdict=all(not v['false_supported_proposals'] and (condition=='null' or len(v['supported_people'])>=cfg['validation']['min_supported_visible_people']) for v in evaluated)
 summary=dict(schema='pretrained_lidar_label_diagnostic/1',condition=condition,counts=counts,restricted_static_gate_pass=verdict,rows=evaluated,semantic_person_validated=False,association_accepted=False,training_ready=False,independent_scenes=1,condition_role='Known development fixture; null is a negative control of the same layout',inference_result_sha256=sha(inference/'result.json'),sensor_predictions_sha256={n:sha(inference/n) for n in ['detections_SENSOR_ONLY.jsonl','corroboration_SENSOR_ONLY.jsonl']},source_sha256={str(p.relative_to(REPO)):sha(p) for p in [Path(__file__),BASE/'analyze.py',BASE/'labels.py',HERE/'protocol.json',HERE/'calibration_SENSOR_ONLY.json']})
 (out/'evaluation_LABEL_ONLY.json').write_text(json.dumps(summary,indent=2)+'\n');preview(out,arrays['full'][0],maps[0],evaluated[0],capture,inference,cal,condition)
 print(json.dumps({k:summary[k] for k in ['condition','counts','restricted_static_gate_pass']},indent=2))

def preview(out,depth,owner,row,capture,inference,cal,condition):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axes=plt.subplots(1,3,figsize=(16,5));axes[0].imshow(depth,vmin=.28,vmax=5,cmap='viridis');axes[0].set_title('Real measured depth, '+condition);axes[0].set_axis_off();axes[1].imshow(owner,cmap='tab10',vmin=0,vmax=9);axes[1].set_title('LABEL ONLY: person pixel ownership');axes[1].set_axis_off()
 ax=axes[2];scans=np.load(capture/'scan_SENSOR_ONLY.npz',allow_pickle=False);g=json.loads((capture/'scan_geometry.json').read_text());j=row['selected_scan_index'];preds=readrows(inference/'detections_SENSOR_ONLY.jsonl')
 if j>=0:
  angles=g['angle_min']+np.arange(g['rays'])*g['angle_increment'];scan=scans['ranges'][j];valid=np.isfinite(scan);ax.scatter(scan[valid]*np.cos(angles[valid]),scan[valid]*np.sin(angles[valid]),s=3,c='lightgray',label='native scan')
  p=preds[j];xy=np.array(p['detections_xy_base']).reshape(-1,2)[p['retained_indices']]
  if len(xy):ax.scatter(xy[:,0],xy[:,1],s=65,facecolors='none',edgecolors='blue',label='DR-SPAAM >=0.5')
 for q in row['proposals_LABEL_ONLY']:
  x,y=q['xy_base'];supported=q['sensor_decision']['semantic_status']=='person_supported_experimental';ax.scatter(x,y,c='green' if supported else 'orange',s=50,marker='o' if supported else 'x');ax.text(x+.03,y+.04,str(q['proposal_index'])+':'+('person-supported' if supported else 'unknown'),fontsize=7)
 ax.set(xlim=(-3.8,5.3),ylim=(-3.8,3.8),xlabel='base x (m)',ylabel='base y (m)',title='Sensor predictions; unknown geometry retained');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7,loc='lower right')
 fig.suptitle('Real Gazebo + pretrained GPU detector; static development diagnostic; not train-ready');fig.tight_layout();fig.savefig(out/'semantic_review.png',dpi=150);plt.close(fig)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--inference',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();evaluate(a.capture.resolve(),a.inference.resolve(),a.output.resolve())
