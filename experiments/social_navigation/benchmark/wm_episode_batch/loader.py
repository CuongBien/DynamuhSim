"""Portable NumPy-only WM corpus reader. Observation, action condition and labels are explicit."""
import json
from pathlib import Path
import numpy as np
class Dataset:
 def __init__(self,root):self.root=Path(root).resolve();self.metadata=json.loads((self.root/'dataset.json').read_text())
 def episode(self,episode_id,allow_test=False):
  meta=next(v for v in self.metadata['episodes'] if v['episode_id']==episode_id)
  if meta['split']=='test' and not allow_test:raise PermissionError('WM test seed family sealed by default')
  return Episode(self.root/'episodes'/episode_id,meta)
class Episode:
 def __init__(self,root,meta):
  self.root=Path(root);self.meta=meta;self.x=dict(np.load(self.root/'inputs/samples_SENSOR_ONLY.npz',allow_pickle=False));self.depth=np.load(self.root/'inputs/depth_SENSOR_ONLY.npy',mmap_mode='r');self.bev=dict(np.load(self.root/'inputs/bev_anchor_estimated_SENSOR_ONLY.npz',allow_pickle=False));self.calibration=json.loads((self.root/'inputs/calibration_SENSOR_ONLY.json').read_text());self.y=None;self.targets=None;self.masks=None
  assert not any(any(k in name for k in ['human','persona','backend','future','world']) for name in self.x)
 def __len__(self):return len(self.x['anchor_ns'])
 def observation(self,index):
  if not 0<=index<len(self):raise IndexError(index)
  history={k:v[index].copy() for k,v in self.x.items()};return dict(history=history,depth_history=np.asarray(self.depth[history['depth_index']]).copy(),bev_anchor_estimated=self.bev['occupancy'][index].copy(),bev_frame_valid=bool(self.bev['frame_valid'][index]),calibration=self.calibration)
 def _labels(self):
  if self.y is None:self.y=dict(np.load(self.root/'labels/future_trajectories_actions_LABEL_ONLY.npz',allow_pickle=False))
 def action_condition(self,index):
  self._labels();return {k:self.y[k][index].copy() for k in ['executed_action_future','executed_action_query_ns','executed_action_stamp_ns','executed_action_available_ns']}
 def supervision(self,index):
  self._labels()
  if self.targets is None:self.targets=dict(np.load(self.root/'labels/future_observed_bev_LABEL_ONLY.npz',allow_pickle=False))
  return dict(trajectories={k:(v.copy() if k=='human_ids' else v[index].copy()) for k,v in self.y.items()},future_bev={k:(v.copy() if k=='offset_ns' else v[index].copy()) for k,v in self.targets.items()})
 def task_masks(self,index):
  if self.masks is None:self.masks=dict(np.load(self.root/'labels/task_masks_LABEL_ONLY.npz',allow_pickle=False))
  return {k:v[index].copy() for k,v in self.masks.items()}
