"""Run the authors' Detector API on deterministic synthetic input, not a metric."""
import hashlib,json,sys,time
from pathlib import Path
import numpy as np
import torch
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE/'runtime'))
from dr_spaam.detector import Detector
if __name__=='__main__':
 out=HERE/'generated/smoke01';out.mkdir(exist_ok=False)
 assert torch.cuda.is_available();torch.set_num_threads(1)
 checkpoint=HERE/'generated/ckpt_jrdb_ann_dr_spaam_e20.pth';assert hashlib.sha256(checkpoint.read_bytes()).hexdigest()=='1b62e50d9bcb954975bda28610e09921e8bde94216b0a291a704a579690b6324'
 detector=Detector(str(checkpoint),model='DR-SPAAM',gpu=True,stride=1,panoramic_scan=True);detector.set_laser_fov(360.)
 torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
 scan=np.full(359,3.5,dtype=np.float32);times=[]
 for i in range(4):
  torch.cuda.synchronize();begin=time.perf_counter();xy,score,mask=detector(scan);torch.cuda.synchronize();times.append(time.perf_counter()-begin)
  assert np.all(np.isfinite(xy)) and np.all(np.isfinite(score)) and mask.shape==scan.shape
 result=dict(status='PASS',kind='Synthetic API/CUDA smoke only, not detection quality',torch=torch.__version__,cuda=torch.version.cuda,gpu=torch.cuda.get_device_name(0),parameter_device=str(next(detector._model.parameters()).device),source_detector='Authors Detector API; restricted loading/np.int compatibility only',runs=4,elapsed_s=times,checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),max_cuda_allocated_bytes=torch.cuda.max_memory_allocated(),training=False)
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
