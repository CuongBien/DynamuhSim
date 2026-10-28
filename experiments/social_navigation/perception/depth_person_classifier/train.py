#!/usr/bin/env python3
"""Four predeclared CUDA fits; train/validation only; freeze winner before test."""
import argparse,json,os,random,shutil,time
from pathlib import Path
import numpy as np
import torch
from torch.nn import functional as F
from data import HERE,CORPUS,load_split,training_weights
from features import DEPTH_NAMES,LIDAR_NAMES
from model import Model,normalization,normalized,portable_arrays,numpy_logits
from metrics import sigmoid,measure,rank,gate
from integrity import verify,sha

def run(out):
 verify();protocol=json.loads((HERE/'protocol.json').read_text());assert torch.cuda.is_available(),'CUDA required by this experiment';torch.set_num_threads(1);torch.set_num_interop_threads(1);torch.use_deterministic_algorithms(True);torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
 out.mkdir(parents=True,exist_ok=False);start=time.monotonic();train=load_split('train');val=load_split('validation');assert {r['scene_id'] for r in train['rows']}==set(protocol['splits']['train']);assert {r['scene_id'] for r in val['rows']}==set(protocol['splits']['validation']);stats=normalization(train);weights=training_weights(train);device=torch.device('cuda:0');td,tl,tm=[torch.from_numpy(v).to(device) for v in normalized(train,stats)];target=torch.tensor(np.maximum(train['y'],0),dtype=torch.float32,device=device);weight=torch.from_numpy(weights).to(device);all_results=[];traces={};score_rows=[];gpu_properties=torch.cuda.get_device_properties(0)
 for order,config in enumerate(protocol['candidates']):
  random.seed(protocol['seed']);np.random.seed(protocol['seed']);torch.manual_seed(protocol['seed']);torch.cuda.manual_seed_all(protocol['seed']);model=Model(config['hidden'],config['modality']).to(device);initial={k:v.detach().clone() for k,v in model.state_dict().items()};params=sum(v.numel() for v in model.parameters());opt=torch.optim.AdamW(model.parameters(),lr=protocol['optimizer']['lr'],weight_decay=protocol['optimizer']['weight_decay']);trace=[]
  for step in range(protocol['optimizer']['steps']):
   opt.zero_grad(set_to_none=True);logit=model(td,tl,tm);loss=(F.binary_cross_entropy_with_logits(logit,target,reduction='none')*weight).sum();assert torch.isfinite(loss);loss.backward();assert all(v.grad is not None and v.grad.device.type=='cuda' and torch.isfinite(v.grad).all() for v in model.parameters());opt.step()
   if step%50==0 or step==protocol['optimizer']['steps']-1:trace.append(dict(step=step+1,weighted_train_bce=float(loss.detach().cpu())))
  model.eval();arrays=portable_arrays(model,stats);np.savez_compressed(out/(config['name']+'.npz'),**arrays);torch.save(model.state_dict(),out/(config['name']+'.pt'));changed=float(sum((model.state_dict()[k]-v).square().sum() for k,v in initial.items()).sqrt().detach().cpu());assert changed>0;parity=[];probabilities={}
  with torch.inference_mode():
   for split,data in [('train',train),('validation',val)]:
    tensors=[torch.from_numpy(v).to(device) for v in normalized(data,stats)];gpu=model(*tensors).cpu().numpy();cpu=numpy_logits(data,arrays,config['hidden'],config['modality']);error=float(np.max(np.abs(gpu-cpu)));prob_error=float(np.max(np.abs(sigmoid(gpu)-sigmoid(cpu))));assert error<=1e-4 and prob_error<=1e-5;parity.append(dict(split=split,rows=len(cpu),max_logit_error=error,max_probability_error=prob_error));probabilities[split]=sigmoid(cpu);np.save(out/(config['name']+'_'+split+'_probabilities.npy'),probabilities[split],allow_pickle=False)
  for threshold in protocol['thresholds']:
   measured=measure(val,probabilities['validation'],threshold);score_rows.append(dict(candidate=config['name'],threshold=threshold,rank=list(rank(measured,protocol['quality_gate'],params,threshold,order)),metrics=measured))
  all_results.append(dict(**config,parameters=params,parameter_device=str(next(model.parameters()).device),gradient_device='cuda:0',parameter_change_l2=changed,parity=parity,final_weighted_train_bce=float((F.binary_cross_entropy_with_logits(model(td,tl,tm),target,reduction='none')*weight).sum().detach().cpu())));traces[config['name']]=trace;print('FIT',config['name'],'CUDA',params,'parameters; validation scored on fixed threshold grid',flush=True)
 winner=max(score_rows,key=lambda v:tuple(v['rank']));selected=next(v for v in all_results if v['name']==winner['candidate']);shutil.copy2(out/(selected['name']+'.npz'),out/'selected_model.npz');shutil.copy2(out/(selected['name']+'.pt'),out/'selected_model.pt');np.savez_compressed(out/'normalization_train_only.npz',**stats);np.save(out/'train_weights.npy',weights,allow_pickle=False)
 summary=dict(status='TRAINED_AND_FROZEN_BEFORE_TEST',device=str(device),gpu_name=torch.cuda.get_device_name(0),cuda_version=torch.version.cuda,torch_version=torch.__version__,gpu_memory_bytes=gpu_properties.total_memory,seed=protocol['seed'],steps=protocol['optimizer']['steps'],train_samples=len(train['y']),validation_samples=len(val['y']),train_scene_weight_totals={sid:float(weights[[r['scene_id']==sid for r in train['rows']]].sum()) for sid in protocol['splits']['train']},train_local_lidar_valid=int(train['lidar_mask'].sum()),validation_local_lidar_valid=int(val['lidar_mask'].sum()),candidates=all_results,threshold_grid_scores=score_rows,selected_candidate=selected['name'],selected_threshold=winner['threshold'],selected_validation_metrics=winner['metrics'],selected_validation_quality_gate=gate(winner['metrics'],protocol['quality_gate']),validation_all_person_reference=measure(val,np.ones(len(val['y'])),.5),elapsed_wall_s=time.monotonic()-start,test_opened=False)
 (out/'training.json').write_text(json.dumps(summary,indent=2)+'\n');(out/'training_trace.json').write_text(json.dumps(traces,indent=2)+'\n');(out/'development_rows.json').write_text(json.dumps(dict(train=train['rows'],validation=val['rows']),indent=2)+'\n')
 frozen=dict(schema='depth_person_selected_model/1',candidate=selected['name'],hidden=selected['hidden'],modality=selected['modality'],threshold=winner['threshold'],feature_names=dict(depth=DEPTH_NAMES,lidar=LIDAR_NAMES),source_freeze_sha256=sha(HERE/'frozen.json'),training_sha256=sha(out/'training.json'),model_sha256=sha(out/'selected_model.npz'),torch_weights_sha256=sha(out/'selected_model.pt'),test_policy=protocol['test_policy'],validation_gate=summary['selected_validation_quality_gate'])
 (out/'selected_frozen.json').write_text(json.dumps(frozen,indent=2)+'\n');verify();print(json.dumps({k:summary[k] for k in ['selected_candidate','selected_threshold','selected_validation_quality_gate','elapsed_wall_s']},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.output.resolve())
