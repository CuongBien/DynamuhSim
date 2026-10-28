"""SENSOR ONLY: depth surface veto, preserve all previous measurements."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
import torch
from scipy.ndimage import binary_dilation,label
from surface import evidence
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2];sys.path.insert(0,str(BENCH/'depth_identity_guard/parent'))
from snapshots.sensor import project
from refinement import enrich

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify():
 f=json.loads((HERE/'frozen.json').read_text());assert all(sha(REPO/p)==h for p,h in f['source_sha256'].items())
def clouds(points,proposals):
 p=points[(points[:,0]>=0)&(points[:,0]<5.3)&(abs(points[:,1])<3.2)&(points[:,2]>=.04)&(points[:,2]<=1.8)]
 if not len(p):return []
 cells=np.floor((p[:,:2]+[0,3.2])/.08).astype(int);grid=np.zeros((68,82),bool);grid[cells[:,0],cells[:,1]]=True;cc,_=label(binary_dilation(grid,iterations=1),structure=np.ones((3,3)));comp=cc[cells[:,0],cells[:,1]];groups=[p[comp==i] for i in np.unique(comp)];out=[]
 for item in proposals:
  q=min(groups,key=lambda v:np.linalg.norm(np.median(v[:,:2],axis=0)-item['xy']));assert len(q)==item['points'];assert np.allclose(np.median(q[:,:2],axis=0),item['xy'],atol=1e-12,rtol=0);out.append(q)
 return out

def run(capture,prior,out):
 verify();out.mkdir(parents=True,exist_ok=False);assert torch.cuda.is_available();torch.set_num_threads(1);cal=json.loads((BENCH/'lidar_person_pilot/calibration_SENSOR_ONLY.json').read_text());tr=np.array(cal['optical_to_base_link']);depth=np.load(capture/'full.npy',mmap_mode='r');rows=[json.loads(v) for v in (prior/'corroboration_SENSOR_ONLY.jsonl').read_text().splitlines()];assert len(rows)==len(depth);result=[];flagged=0;withdrawn=0
 for i,row in enumerate(rows):
  points=project(depth[i],cal['k'],tr);proposals,_=enrich(points);groups=clouds(points,proposals);assert len(proposals)==len(row['proposals'])
  for j,(p,q,decision) in enumerate(zip(proposals,groups,row['proposals'])):
   assert np.array_equal(p['xy'],decision['xy_base']) and p['points']==decision['point_count'];facts=evidence(q);decision['surface_evidence']=facts;decision['prior_semantic_status']=decision['semantic_status'];decision['support_vetoed_by_planar_surface']=False
   if facts['planar_surface_evidence']:
    flagged+=1
    if decision['semantic_status']=='person_supported_experimental':decision['semantic_status']='unknown';decision['support_vetoed_by_planar_surface']=True;withdrawn+=1
  result.append(row)
 with (out/'corroboration_SENSOR_ONLY.jsonl').open('w') as f:
  for row in result:f.write(json.dumps(row,allow_nan=False)+'\n')
 (out/'detections_SENSOR_ONLY.jsonl').write_bytes((prior/'detections_SENSOR_ONLY.jsonl').read_bytes())
 summary=dict(status='COMPLETE',cuda=True,gpu=torch.cuda.get_device_name(0),frames=len(rows),planar_evidence_rows=flagged,person_support_withdrawn_rows=withdrawn,all_proposals_retained=True,semantic_person_validated=False,association_accepted=False,training_ready=False,input_sha256={'depth':sha(capture/'full.npy'),'prior_sensor_predictions':sha(prior/'corroboration_SENSOR_ONLY.jsonl')},frozen_sha256=sha(HERE/'frozen.json'))
 verify();(out/'result.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--prior',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.capture.resolve(),a.prior.resolve(),a.output.resolve())
