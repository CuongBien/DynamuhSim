#!/usr/bin/env python3
"""Offline SENSOR ONLY inference: no actors/scene poses/instance masks/GT loaded."""
import argparse,hashlib,json,sys,time
from pathlib import Path
import numpy as np
import torch
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
sys.path[:0]=[str(HERE/'runtime'),str(BENCH/'depth_identity_guard/parent')]
from dr_spaam.detector import Detector
from snapshots.sensor import project
from refinement import enrich
from sensor_adapter import prepare_scan,corroborate

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def json_value(value):
 if isinstance(value,np.generic):return value.item()
 raise TypeError('Unsupported JSON type: '+type(value).__name__)
def verify():
 frozen=json.loads((HERE/'frozen.json').read_text());assert all(sha(REPO/p)==h for p,h in frozen['engine_sha256'].items());return frozen

def run(capture,out):
 frozen=verify();out.mkdir(parents=True,exist_ok=False);assert torch.cuda.is_available();torch.set_num_threads(1)
 cfg=json.loads((HERE/'protocol.json').read_text());cal=json.loads((HERE/'calibration_SENSOR_ONLY.json').read_text());tr=np.array(cal['optical_to_base_link']);
 inputs=['scan_SENSOR_ONLY.npz','scan_geometry.json','full.npy','stamp_ns.npy','depth_available.json'];input_hash={n:sha(capture/n) for n in inputs}
 scan_data=np.load(capture/'scan_SENSOR_ONLY.npz',allow_pickle=False);geometry=json.loads((capture/'scan_geometry.json').read_text());depth=np.load(capture/'full.npy',mmap_mode='r');depth_stamp=np.load(capture/'stamp_ns.npy',allow_pickle=False);depth_available=np.array(json.loads((capture/'depth_available.json').read_text())['full'],dtype=np.int64)
 stamps=scan_data['stamp_ns'];available=scan_data['available_ns'];assert np.all(np.diff(stamps)>0) and np.all(available>=stamps) and np.all(depth_available>=depth_stamp)
 checkpoint=HERE/'generated'/cfg['checkpoint'];detector=Detector(str(checkpoint),model=cfg['model'],gpu=True,stride=cfg['stride'],panoramic_scan=cfg['panoramic_scan']);torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
 predictions=[];durations=[];metadata=[]
 for i,raw in enumerate(scan_data['ranges']):
  scan,phi,index,meta=prepare_scan(raw,geometry)
  if i==0:detector.set_laser_fov(meta['fov_deg'])
  torch.cuda.synchronize();begin=time.perf_counter();xy,confidence,mask=detector(scan);torch.cuda.synchronize();elapsed=time.perf_counter()-begin;durations.append(elapsed)
  assert np.max(abs(detector._scan_phi-phi))<1e-6;assert np.isfinite(xy).all() and np.isfinite(confidence).all()
  score=np.asarray(confidence).reshape(-1);keep=np.flatnonzero(score>=cfg['confidence_threshold'])
  predictions.append(dict(scan_index=i,stamp_ns=int(stamps[i]),available_ns=int(available[i]),detections_xy_base=xy.tolist(),scores=score.tolist(),retained_indices=keep.tolist(),instance_mask=mask.tolist(),original_ray_indices=index.tolist(),ray_grid=meta));metadata.append(meta)
 with (out/'detections_SENSOR_ONLY.jsonl').open('w') as f:
  for row in predictions:f.write(json.dumps(row,allow_nan=False,default=json_value)+'\n')
 fusion=[];params=cfg['semantic_fusion']
 for i,stamp in enumerate(depth_stamp):
  valid=(stamps<=stamp)&(available<=depth_available[i])&(stamp-stamps<=params['max_scan_age_ns']);valid_indices=np.flatnonzero(valid);j=int(valid_indices[np.argmax(stamps[valid_indices])]) if len(valid_indices) else -1
  proposals,_=enrich(project(depth[i],cal['k'],tr));xy=np.array([p['xy'] for p in proposals]).reshape(-1,2)
  if j>=0:
   predicted=predictions[j];indices=predicted['retained_indices'];dets=np.array(predicted['detections_xy_base']).reshape(-1,2)[indices];scores=np.array(predicted['scores'])[indices]
  else:dets=np.empty((0,2));scores=np.empty(0)
  decisions=corroborate(xy,dets,scores,params['position_gate_m'],params['ambiguity_margin_m'])
  for k,p in enumerate(proposals):decisions[k].update(xy_base=p['xy'].tolist(),point_count=p['points'],semantic_class='unknown_geometry_candidate',semantic_valid=False,training_ready=False)
  fusion.append(dict(depth_index=i,stamp_ns=int(stamp),available_ns=int(depth_available[i]),selected_scan_index=j,selected_scan_stamp_ns=int(stamps[j]) if j>=0 else -1,selected_scan_available_ns=int(available[j]) if j>=0 else -1,proposals=decisions))
 with (out/'corroboration_SENSOR_ONLY.jsonl').open('w') as f:
  for row in fusion:f.write(json.dumps(row,allow_nan=False,default=json_value)+'\n')
 verify();assert all(sha(capture/n)==h for n,h in input_hash.items())
 result=dict(schema='pretrained_lidar_sensor_pilot/1',input_sha256=input_hash,frozen_sha256=sha(HERE/'frozen.json'),checkpoint_sha256=sha(checkpoint),cuda=True,torch=torch.__version__,cuda_version=torch.version.cuda,gpu=torch.cuda.get_device_name(0),parameter_device=str(next(detector._model.parameters()).device),model_eval=not detector._model.training,weights_trained_here=False,scan_count=len(stamps),depth_frames=len(depth_stamp),threshold=cfg['confidence_threshold'],scan_rate_hz=float(1e9/np.median(np.diff(stamps))),all_detection_outputs_retained=True,all_depth_proposals_retained=True,confirmed_person_semantics=False,training_ready=False,recurrence_reset='New Detector instance for each run; synthetic smoke not carried into sensor inference',elapsed_s=durations,first_scan_elapsed_s=durations[0],median_warm_elapsed_s=float(np.median(durations[2:])),cuda_max_allocated_bytes=torch.cuda.max_memory_allocated(),semantic_support_rows=sum(v['semantic_status']=='person_supported_experimental' for row in fusion for v in row['proposals']))
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='elapsed_s'},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.capture.resolve(),a.output.resolve())
