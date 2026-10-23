#!/usr/bin/env python3
"""One-shot test of the already frozen validation winner. No fitting or threshold search."""
import argparse,json,os
from pathlib import Path
import numpy as np
import torch
from data import HERE,load_split
from model import Model,normalized,numpy_logits
from metrics import sigmoid,measure,gate
from integrity import verify,sha

def run(folder):
 verify();frozen=json.loads((folder/'selected_frozen.json').read_text());cfg=json.loads((HERE/'protocol.json').read_text());assert frozen['source_freeze_sha256']==sha(HERE/'frozen.json') and frozen['training_sha256']==sha(folder/'training.json') and frozen['model_sha256']==sha(folder/'selected_model.npz') and frozen['torch_weights_sha256']==sha(folder/'selected_model.pt');assert torch.cuda.is_available();torch.set_num_threads(1);torch.backends.cuda.matmul.allow_tf32=False
 # Exclusive seal is written before any test feature/label is opened. Never auto retry.
 seal=folder/'test_opened.json'
 with seal.open('x') as stream:json.dump(dict(selected_frozen_sha256=sha(folder/'selected_frozen.json'),purpose='Single declared evaluation of fixed model and threshold; no retuning'),stream,indent=2)
 data=load_split('test',allow_test=True);assert {r['scene_id'] for r in data['rows']}==set(cfg['splits']['test']);arrays=dict(np.load(folder/'selected_model.npz',allow_pickle=False));logits=numpy_logits(data,arrays,frozen['hidden'],frozen['modality']);probability=sigmoid(logits);pred=probability>=frozen['threshold'];model=Model(frozen['hidden'],frozen['modality']).cuda();model.load_state_dict(torch.load(folder/'selected_model.pt',weights_only=True,map_location='cuda:0'));model.eval()
 with torch.inference_mode():gpu=model(*[torch.from_numpy(v).cuda() for v in normalized(data,arrays)]).cpu().numpy()
 error=float(np.max(np.abs(gpu-logits)));prob_error=float(np.max(np.abs(sigmoid(gpu)-probability)));assert error<=1e-4 and prob_error<=1e-5;assert np.array_equal(sigmoid(gpu)>=frozen['threshold'],pred)
 measured=measure(data,probability,frozen['threshold']);rows=[dict(**row,score=float(probability[i]),prediction=int(pred[i]),class_index=int(data['y'][i]),label_valid=bool(data['label_valid'][i])) for i,row in enumerate(data['rows'])];result=dict(status='MEASURED',candidate=frozen['candidate'],threshold=frozen['threshold'],test_metrics=measured,pilot_quality_gate=gate(measured,cfg['quality_gate']),global_claim='No cross-avatar, real-world, navigation or World Model acceptance',test_samples=len(rows),scan_valid_samples=sum(v['scan_valid'] for v in rows),local_lidar_valid_samples=int(data['lidar_mask'].sum()),max_cuda_numpy_logit_error=error,max_cuda_numpy_probability_error=prob_error,selected_frozen_sha256=sha(folder/'selected_frozen.json'),model_unchanged=sha(folder/'selected_model.npz')==frozen['model_sha256'],threshold_tuned_on_test=False,model_evaluations_on_test=1,default_ros_changed=False,world_model_training_ready=False,persona_valid=False)
 np.savez_compressed(folder/'test_predictions_LABEL_REVIEW_ONLY.npz',score=probability,prediction=pred,y=data['y'],label_valid=data['label_valid'],depth_features=data['depth'],lidar_features=data['lidar'],lidar_mask=data['lidar_mask']);(folder/'test_rows_LABEL_REVIEW_ONLY.json').write_text(json.dumps(rows,indent=2)+'\n');(folder/'test_result.json').write_text(json.dumps(result,indent=2)+'\n');verify();print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run(a.run.resolve())
