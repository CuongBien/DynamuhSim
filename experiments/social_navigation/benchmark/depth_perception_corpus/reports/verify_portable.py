#!/usr/bin/env python3
"""Relocate the actual bundle, adversarially replace labels and remove future sensors."""
import argparse,hashlib,importlib.util,json,shutil,sys,tempfile
from pathlib import Path
import numpy as np

def digest(sample):
 h=hashlib.sha256()
 for key in sorted(sample):
  value=sample[key];h.update(key.encode())
  if isinstance(value,np.ndarray):h.update(value.dtype.str.encode());h.update(str(value.shape).encode());h.update(value.tobytes())
  else:h.update(repr(value).encode())
 return h.hexdigest()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main(root,out):
 originals={str(p.relative_to(root)):sha(p) for p in root.rglob('*') if p.is_file()};meta=json.loads((root/'manifest.json').read_text());n=0;causal=0;sealed=0
 with tempfile.TemporaryDirectory() as directory:
  relocated=Path(directory)/'relocated';shutil.copytree(root,relocated);spec=importlib.util.spec_from_file_location('relocated_corpus_loader',relocated/'loader.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);assert not any(x=='rclpy' or x.startswith('gz.') for x in sys.modules)
  baseline={}
  for scene in meta['scenes']:
   sid=scene['scene_id']
   if scene['split']=='test':
    try:module.load_input(relocated,sid,0)
    except PermissionError:sealed+=1
    else:raise AssertionError('Test API not sealed')
    continue
   for i in range(scene['proposals']):baseline[sid,i]=digest(module.load_input(relocated,sid,i));n+=1
   target=relocated/'labels'/sid/'proposal_labels_LABEL_ONLY.npz'
   with np.load(target) as a:values={k:a[k].copy() for k in a.files}
   values['class_index'][:]=-1;values['person_id'][:]=999;values['label_valid'][:]=False;np.savez_compressed(target,**values)
   # Also delete all pixel labels, visibility and source provenance. Input must still load.
   for extra in (relocated/'labels'/sid).iterdir():
    if extra!=target:extra.unlink()
   for i in range(scene['proposals']):assert digest(module.load_input(relocated,sid,i))==baseline[sid,i]
   folder=relocated/'inputs'/sid
   with np.load(folder/'proposals_SENSOR_ONLY.npz') as a:props={k:a[k].copy() for k in a.files}
   with np.load(folder/'depth_SENSOR_ONLY.npz') as a:depth={k:a[k].copy() for k in a.files}
   with np.load(folder/'lidar_SENSOR_ONLY.npz') as a:scan={k:a[k].copy() for k in a.files}
   for frame in range(scene['frames']):
    i=int(np.flatnonzero(props['frame_index']==frame)[0]);stamp=int(depth['stamp_ns'][frame]);available=int(depth['available_ns'][frame]);keep=(scan['stamp_ns']<=stamp)&(scan['available_ns']<=available)
    np.savez_compressed(folder/'lidar_SENSOR_ONLY.npz',**{k:v[keep] for k,v in scan.items()});np.savez_compressed(folder/'depth_SENSOR_ONLY.npz',**{k:v[:frame+1] for k,v in depth.items()});assert digest(module.load_input(relocated,sid,i))==baseline[sid,i];causal+=1
   np.savez_compressed(folder/'lidar_SENSOR_ONLY.npz',**scan);np.savez_compressed(folder/'depth_SENSOR_ONLY.npz',**depth)
 assert all(sha(root/n)==h for n,h in originals.items());result=dict(status='PASS',relocated_train_validation_samples=n,adversarial_label_independence_samples=n,future_sensor_removal_frame_cases=causal,test_scenes_default_denied=sealed,original_bundle_unchanged=True,ros_gazebo_imports=False,test_model_evaluations=0)
 out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--corpus',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();main(a.corpus.resolve(),a.output.resolve())
