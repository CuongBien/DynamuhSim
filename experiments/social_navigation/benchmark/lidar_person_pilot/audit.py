#!/usr/bin/env python3
"""Independent capture, ray geometry, prefix inference and label-metric audit."""
import argparse,hashlib,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import torch
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2];sys.path[:0]=[str(HERE/'runtime'),str(BENCH/'depth_visibility_pilot')]
from dr_spaam.detector import Detector
from labels import ownership
from sensor_adapter import prepare_scan

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):return [json.loads(v) for v in p.read_text().splitlines()]
def run(out):
 out.mkdir(parents=True,exist_ok=False);frozen=json.loads((HERE/'frozen.json').read_text());assert all(sha(REPO/p)==h for p,h in frozen['engine_sha256'].items());cfg=json.loads((HERE/'protocol.json').read_text());torch.set_num_threads(1);assert torch.cuda.is_available();reports={};point_checks=0;prefix_checks=0;archived_checks=0;causal_checks=0
 for condition,capture_name,infer_name in [('full','full01','full02'),('null','null01','null01')]:
  capture=HERE/'generated/runs'/capture_name;inference=HERE/'generated/inference'/infer_name;evaldir=HERE/'generated/evaluation'/capture_name
  cap=json.loads((capture/'result.json').read_text());ev=json.loads((evaldir/'evaluation_LABEL_ONLY.json').read_text());inf=json.loads((inference/'result.json').read_text());assert cap['capture_accepted'] and cap['gazebo_exit']==0 and cap['actual_nvidia_renderer'] and cap['callbacks_balanced'] and not cap['cleanup']['remaining'];assert inf['frozen_sha256']==sha(HERE/'frozen.json')
  for p,h in cap['source_sha256'].items():
   if sha(REPO/p)==h:continue
   archived=HERE/'reports/engine01/source'/p
   if p.endswith('/frozen.json'):archived=HERE/'reports/engine01/frozen.json'
   assert archived.exists() and sha(archived)==h,(p,'captured source not retained');archived_checks+=1
  assert all(sha(capture/p)==h for p,h in inf['input_sha256'].items());assert all(sha(REPO/p)==h for p,h in ev['source_sha256'].items())
  geometry=json.loads((capture/'scan_geometry.json').read_text());scan_data=np.load(capture/'scan_SENSOR_ONLY.npz',allow_pickle=False);predictions=rows(inference/'detections_SENSOR_ONLY.jsonl');fusion=rows(inference/'corroboration_SENSOR_ONLY.jsonl');assert len(predictions)==len(scan_data['stamp_ns'])==20
  prefix=Detector(str(HERE/'generated'/cfg['checkpoint']),model='DR-SPAAM',gpu=True,stride=1,panoramic_scan=True);torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
  for i,raw in enumerate(scan_data['ranges']):
   ordered,phi,index,meta=prepare_scan(raw,geometry);original_phi=geometry['angle_min']+index*geometry['angle_increment'];np.testing.assert_allclose(np.cos(phi),np.cos(original_phi),atol=2e-7,rtol=0);np.testing.assert_allclose(np.sin(phi),np.sin(original_phi),atol=2e-7,rtol=0);assert np.array_equal(index,np.array(predictions[i]['original_ray_indices']))
   expected=raw[index].copy();expected[(~np.isfinite(expected))|(expected==0)]=29.99;assert ordered.tobytes()==expected.tobytes();point_checks+=len(index)
   # Replay only a real prefix; never reads any future scan into the model.
   if i<3:
    if i==0:prefix.set_laser_fov(meta['fov_deg'])
    xy,score,mask=prefix(ordered);np.testing.assert_allclose(xy,predictions[i]['detections_xy_base'],rtol=0,atol=1e-6);np.testing.assert_allclose(score.reshape(-1),predictions[i]['scores'],rtol=0,atol=1e-6);np.testing.assert_array_equal(mask,predictions[i]['instance_mask']);prefix_checks+=1
  full=np.load(capture/'full.npy');control=np.load(capture/'control.npy');assert full.tobytes()==control.tobytes();saved_owner=np.load(evaldir/'owner_LABEL_ONLY.npy');depth_stamp=np.load(capture/'stamp_ns.npy');depth_available=json.loads((capture/'depth_available.json').read_text())['full'];drops=[np.load(capture/('drop'+str(i)+'.npy')) for i in range(1,7)];alone=[np.load(capture/('only'+str(i)+'.npy')) for i in range(1,7)]
  false_rows=0;true_rows=0;unknown=0;available_frames=0
  for k,f in enumerate(fusion):
   labels,qa=ownership(full[k],[v[k] for v in drops],[v[k] for v in alone],cfg['depth_equality_tolerance_m']);assert not any(qa.values());np.testing.assert_array_equal(labels,saved_owner[k]);assert f['stamp_ns']==int(depth_stamp[k]) and f['available_ns']==int(depth_available[k])
   legal=np.flatnonzero((scan_data['stamp_ns']<=f['stamp_ns'])&(scan_data['available_ns']<=f['available_ns'])&(f['stamp_ns']-scan_data['stamp_ns']<=cfg['semantic_fusion']['max_scan_age_ns']));selected=int(legal[np.argmax(scan_data['stamp_ns'][legal])]) if len(legal) else -1;assert selected==f['selected_scan_index'];causal_checks+=1;available_frames+=selected>=0
   scored=ev['rows'][k];assert len(f['proposals'])==len(scored['proposals_LABEL_ONLY'])
   supported=[];false=[];persons=[]
   if selected>=0:
    d=predictions[selected];keep=np.flatnonzero(np.array(d['scores'])>=cfg['confidence_threshold']);assert keep.tolist()==d['retained_indices'];dxy=np.array(d['detections_xy_base']).reshape(-1,2)[keep]
   else:dxy=np.empty((0,2))
   pxy=np.array([q['xy_base'] for q in f['proposals']]).reshape(-1,2);distance=np.linalg.norm(pxy[:,None]-dxy[None,:],axis=-1)
   for i,q in enumerate(f['proposals']):
    record=scored['proposals_LABEL_ONLY'][i];hist={int(v):n for v,n in record['label_counts'].items()};assert sum(hist.values())==q['point_count'];assert np.array_equal(q['xy_base'],record['xy_base']);assert q['semantic_valid'] is False and q['training_ready'] is False
    if q['semantic_status']=='person_supported_experimental':
     j=q['learned_detection_index'];assert distance[i,j]<=.6 and int(distance[i].argmin())==j and int(distance[:,j].argmin())==i;alternative=min(np.delete(distance[i],j).min(initial=np.inf),np.delete(distance[:,j],i).min(initial=np.inf));assert alternative-distance[i,j]>=.15;supported.append(i)
     if record['classification']=='pure_person':persons.append(record['person_id'])
     else:false.append(i)
    else:unknown+=1
   assert supported==scored['supported_proposals'] and false==scored['false_supported_proposals'];assert sorted(set(persons)&set(scored['eligible_people']))==scored['supported_people'];false_rows+=len(false);true_rows+=len(set(persons))
  assert false_rows==ev['counts']['false_supported_rows'];assert true_rows==ev['counts']['supported_person_rows'];assert unknown==ev['counts']['unknown_rows'];reports[condition]=dict(capture_pass=True,actual_gpu=cap['cleanup']['ogre_renderer_lines'],scan_count=len(predictions),frames=len(fusion),frames_with_legal_scan=available_frames,false_supported_rows=false_rows,true_supported_person_rows=true_rows,restricted_gate=ev['restricted_static_gate_pass'])
 original=json.loads((HERE/'generated/checkpoint/files.json').read_text());assert all(sha(REPO/p)==h for p,h in original.items())
 result=dict(status='PASS',scope='Integrity audit; semantic quality gate FAIL remains',conditions=reports,native_ray_geometry_checks=point_checks,real_prefix_scans_replayed=prefix_checks,causal_selection_checks=causal_checks,capture_source_checks_from_archived_version=archived_checks,preexisting_files_unchanged=len(original),model_cuda=True,semantic_person_validated=False,association_accepted=False,training_ready=False,availability_limit='Receive-clock max(header,latest observed sim clock); discrete clock does not measure wall-time callback latency or ordering within equal clock ticks',original_engine_manifest_retained=True,serialization_only_revision_declared=True)
 (out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.output.resolve())
