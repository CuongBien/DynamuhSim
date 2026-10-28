"""Portable corpus access with separate feature and label paths; test denied by default."""
import importlib.util,json
from pathlib import Path
import numpy as np
from features import extract
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3];CORPUS=HERE.parent/'depth_perception_corpus/generated/corpus01'
spec=importlib.util.spec_from_file_location('perception_corpus_loader',HERE.parent/'depth_perception_corpus/loader.py');loader=importlib.util.module_from_spec(spec);spec.loader.exec_module(loader)
def load_split(split,root=CORPUS,allow_test=False):
 if split=='test' and not allow_test:raise PermissionError('Classifier test requires a frozen selected model and explicit stage authorization')
 root=Path(root);manifest=json.loads((root/'manifest.json').read_text());xs=[];ls=[];masks=[];ys=[];valid=[];rows=[]
 for scene in manifest['scenes']:
  if scene['split']!=split:continue
  sid=scene['scene_id'];geometry=json.loads((root/'inputs'/sid/'lidar_geometry_SENSOR_ONLY.json').read_text());proposals=np.load(root/'inputs'/sid/'proposals_SENSOR_ONLY.npz',allow_pickle=False)
  for index in range(scene['proposals']):
   sample=loader.load_input(root,sid,index,allow_test=allow_test);depth,lidar,mask=extract(sample,geometry);xs.append(depth);ls.append(lidar);masks.append(mask)
   label=loader.load_label(root,sid,index,allow_test=allow_test);ys.append(int(label['class_index']));valid.append(bool(label['label_valid']));rows.append(dict(scene_id=sid,sample_index=index,frame_index=int(proposals['frame_index'][index]),scan_valid=bool(sample['scan_valid']),local_lidar_valid=bool(mask)))
 return dict(depth=np.stack(xs),lidar=np.stack(ls),lidar_mask=np.array(masks,bool),y=np.array(ys,np.int64),label_valid=np.array(valid,bool),rows=rows)
def training_weights(data):
 weights=np.zeros(len(data['y']),np.float32);scenes=sorted({v['scene_id'] for v in data['rows']})
 for sid in scenes:
  frames=sorted({v['frame_index'] for v in data['rows'] if v['scene_id']==sid})
  for frame in frames:
   indices=np.array([i for i,v in enumerate(data['rows']) if v['scene_id']==sid and v['frame_index']==frame and data['label_valid'][i]],int);classes=np.unique(data['y'][indices])
   for target in classes:
    subset=indices[data['y'][indices]==target];weights[subset]=1/(len(scenes)*len(frames)*len(classes)*len(subset))
 assert abs(float(weights.sum())-1)<1e-6;return weights
