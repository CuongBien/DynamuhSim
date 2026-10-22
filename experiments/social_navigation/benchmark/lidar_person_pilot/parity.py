"""Check compatible wrapper math against unchanged upstream inference on synthetic scans."""
import importlib.util,json,sys,hashlib
from pathlib import Path
import numpy as np
import torch
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE/'runtime'))
from dr_spaam.detector import Detector
from dr_spaam.utils import utils
if __name__=='__main__':
 out=HERE/'generated/parity01';out.mkdir(exist_ok=False);torch.set_num_threads(1);assert torch.cuda.is_available()
 spec=importlib.util.spec_from_file_location('unchanged_upstream_utils',HERE/'upstream_original/dr_spaam/utils/utils.py');original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
 spec=importlib.util.spec_from_file_location('unchanged_upstream_detector',HERE/'upstream_original/dr_spaam/detector.py');det_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(det_module)
 ckpt=str(HERE/'generated/ckpt_jrdb_ann_dr_spaam_e20.pth');a=Detector(ckpt,model='DR-SPAAM',gpu=True,panoramic_scan=True);b=det_module.Detector(ckpt,model='DR-SPAAM',gpu=True,panoramic_scan=True)
 for d in [a,b]:d.set_laser_fov(358.99721448467966)
 torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
 keys=a._model.state_dict();assert all(torch.equal(v,b._model.state_dict()[k]) for k,v in keys.items())
 phi=np.linspace(-np.pi*358/359,np.pi*358/359,359,dtype=np.float32);rng=np.random.default_rng(120);scans=[np.full(359,3.5,dtype=np.float32),rng.uniform(.5,3.5,359).astype(np.float32),np.where(abs(phi)<.15,2.,3.5).astype(np.float32)]
 cutout_checks=0;output_checks=[]
 for scan in scans:
  kwargs=dict(stride=1,centered=True,fixed=True,window_width=1.,window_depth=.5,num_cutout_pts=56,padding_val=29.99,area_mode=True)
  actual=utils.scans_to_cutout(scan[None],phi,**kwargs)
  present='int' in np.__dict__;previous=np.__dict__.get('int');np.int=int
  try:expected=original.scans_to_cutout(scan[None],phi,**kwargs)
  finally:
   if present:np.int=previous
   else:del np.int
  assert actual.tobytes()==expected.tobytes();cutout_checks+=actual.size
  xy,score,mask=a(scan);ref_xy,ref_score,ref_mask=b(scan);np.testing.assert_allclose(xy,ref_xy,atol=1e-6,rtol=0);np.testing.assert_allclose(score,ref_score,atol=1e-6,rtol=0);np.testing.assert_array_equal(mask,ref_mask)
  output_checks.append(dict(detections=len(xy),max_xy_error=float(np.max(abs(xy-ref_xy),initial=0)),max_score_error=float(np.max(abs(score-ref_score),initial=0))))
 result=dict(status='PASS',kind='Synthetic compatibility parity only',checkpoint_state_tensors=len(keys),cutout_values_bitwise_checked=cutout_checks,output_sequences=output_checks,gpu=torch.cuda.get_device_name(0),torch=torch.__version__,no_file_or_checkpoint_modified=True)
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
