"""Recompute surface evidence on CPU and verify no measurements/person support lost."""
import argparse,hashlib,json,sys,math
from pathlib import Path
import numpy as np
import torch
from surface import evidence,CFG
from surface_runner import clouds,verify,HERE,BENCH,REPO,project,enrich

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return [json.loads(v) for v in p.read_text().splitlines()]
def run(out):
 out.mkdir(parents=True,exist_ok=False);verify();torch.set_num_threads(1);scene=json.loads((HERE/'scene_frozen.json').read_text());assert all(sha(REPO/p)==h for p,h in scene['source_sha256'].items())
 cases=[('regression_full',BENCH/'lidar_person_pilot/generated/runs/full01',BENCH/'lidar_person_pilot/generated/inference/full02',HERE/'generated/regression_full01',HERE/'generated/regression_full_eval'),('regression_null',BENCH/'lidar_person_pilot/generated/runs/null01',BENCH/'lidar_person_pilot/generated/inference/null01',HERE/'generated/regression_null01',HERE/'generated/regression_null_eval'),('fresh',HERE/'generated/runs/newscene01',HERE/'generated/newscene_prior',HERE/'generated/newscene_guard',HERE/'generated/newscene_guard_eval')]
 cal=json.loads((BENCH/'lidar_person_pilot/calibration_SENSOR_ONLY.json').read_text());tr=np.array(cal['optical_to_base_link']);report={};checked=plane_checks=0
 for name,capture,prior,guard,labels in cases:
  before=read(prior/'corroboration_SENSOR_ONLY.jsonl');after=read(guard/'corroboration_SENSOR_ONLY.jsonl');ev=json.loads((labels/'evaluation_LABEL_ONLY.json').read_text());depth=np.load(capture/'full.npy');assert len(before)==len(after)==len(depth);assert (prior/'detections_SENSOR_ONLY.jsonl').read_bytes()==(guard/'detections_SENSOR_ONLY.jsonl').read_bytes();objects=people=withdrawn_bad=withdrawn_good=0
  for k,(a,b) in enumerate(zip(before,after)):
   assert {x:v for x,v in a.items() if x!='proposals'}=={x:v for x,v in b.items() if x!='proposals'};proposals,_=enrich(project(depth[k],cal['k'],tr));groups=clouds(project(depth[k],cal['k'],tr),proposals);assert len(a['proposals'])==len(b['proposals'])==len(groups)
   for i,(old,new,q) in enumerate(zip(a['proposals'],b['proposals'],groups)):
    restore={x:v for x,v in new.items() if x not in ['surface_evidence','prior_semantic_status','support_vetoed_by_planar_surface']};restore['semantic_status']=new['prior_semantic_status'];assert restore==old
    cpu=evidence(q,'cpu');gpu=new['surface_evidence'];assert cpu['planar_surface_evidence']==gpu['planar_surface_evidence'];assert abs(cpu['explained_fraction']-gpu['explained_fraction'])<1e-12;assert len(cpu['planes'])==len(gpu['planes']);checked+=1
    # Independent residual/height recomputation from saved plane parameters.
    remaining=np.ones(len(q),bool);normals=[]
    for plane in gpu['planes']:
     normal=np.array(plane['normal_xy']);assert abs(np.linalg.norm(normal)-1)<1e-12;mask=remaining&(abs(q[:,:2]@normal-plane['offset_m'])<=CFG['plane_tolerance_m']);assert mask.sum()==plane['points'];assert np.ptp(q[mask,2])>=CFG['min_plane_vertical_span_m'];remaining[mask]=False;normals.append(normal);plane_checks+=int(mask.sum())
    if gpu['planes']:assert abs((1-float(remaining.mean()))-gpu['explained_fraction'])<1e-12
    if len(normals)==2:assert abs(normals[0]@normals[1])<=math.cos(math.radians(CFG['min_normal_angle_deg']))+1e-12
    truth=ev['rows'][k]['proposals_LABEL_ONLY'][i];person=truth['classification']=='pure_person'
    if gpu['planar_surface_evidence']:
     if person:people+=1
     else:objects+=1
    if new['support_vetoed_by_planar_surface']:
     if person:withdrawn_good+=1
     else:withdrawn_bad+=1
     assert old['semantic_status']=='person_supported_experimental' and new['semantic_status']=='unknown'
    elif old['semantic_status']=='person_supported_experimental':assert new['semantic_status']==old['semantic_status']
  assert people==0 and withdrawn_good==0;report[name]=dict(rows=sum(len(v['proposals']) for v in after),object_surface_rows=objects,person_surface_rows=people,false_support_withdrawn=withdrawn_bad,true_support_withdrawn=withdrawn_good,remaining_false_support=ev['counts']['false_supported_rows'],remaining_true_support=ev['counts']['supported_person_rows'],all_raw_measurements_retained=True,legacy_semantic_gate=ev['restricted_static_gate_pass'])
 cap=json.loads((HERE/'generated/runs/newscene01/result.json').read_text());assert cap['capture_accepted'] and cap['gazebo_exit']==0 and cap['callbacks_balanced'] and cap['actual_nvidia_renderer'] and not cap['cleanup']['remaining'];assert all(sha(REPO/p)==h for p,h in cap['source_sha256'].items())
 initial=json.loads((HERE/'generated/checkpoint/files.json').read_text());assert all(sha(REPO/p)==h for p,h in initial.items());verify()
 result=dict(status='PASS',kind='Surface-only integrity/retention audit; overall person semantics still FAIL',cases=report,proposal_cpu_gpu_comparisons=checked,saved_plane_point_residual_checks=plane_checks,source_unchanged_after_freeze=True,new_scene_source_unchanged_after_capture=True,preexisting_files_unchanged=len(initial),fresh_gpu_capture=True,training_ready=False,semantic_person_validated=False,association_accepted=False)
 (out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.output.resolve())
