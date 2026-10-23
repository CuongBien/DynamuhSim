"""Portable NumPy sample API: sensor inputs and LABEL data are separate calls."""
import json
from pathlib import Path
import numpy as np
INPUT_KEYS={'points_xyz','point_pixel_uv','bbox_uv','depth','camera_k','depth_stamp_ns','depth_available_ns','scan_ranges','scan_valid','scan_stamp_ns','scan_available_ns'}

def _scene(root,scene_id,allow_test):
 root=Path(root);manifest=json.loads((root/'manifest.json').read_text());scene=next(v for v in manifest['scenes'] if v['scene_id']==scene_id)
 if scene['split']=='test' and not allow_test:raise PermissionError('Pilot test split sealed by default; explicit allow_test required')
 return root,scene

def load_input(root,scene_id,proposal_index,allow_test=False):
 root,meta=_scene(root,scene_id,allow_test);folder=root/'inputs'/scene_id;p=np.load(folder/'proposals_SENSOR_ONLY.npz',allow_pickle=False);d=np.load(folder/'depth_SENSOR_ONLY.npz',allow_pickle=False);laser=np.load(folder/'lidar_SENSOR_ONLY.npz',allow_pickle=False);i=int(proposal_index)
 if not 0<=i<len(p['frame_index']):raise IndexError(i)
 frame=int(p['frame_index'][i]);begin,end=p['point_offsets'][i:i+2];stamp=int(d['stamp_ns'][frame]);available=int(d['available_ns'][frame]);valid=(laser['stamp_ns']<=stamp)&(laser['available_ns']<=available)&(stamp-laser['stamp_ns']<=250_000_000);indices=np.flatnonzero(valid);j=int(indices[np.argmax(laser['stamp_ns'][indices])]) if len(indices) else -1
 result=dict(points_xyz=p['points_xyz'][begin:end].copy(),point_pixel_uv=p['point_pixel_uv'][begin:end].copy(),bbox_uv=p['bbox_uv'][i].copy(),depth=d['depth'][frame].copy(),camera_k=np.array(json.loads((root/'inputs/calibration_SENSOR_ONLY.json').read_text())['k'],dtype=float),depth_stamp_ns=stamp,depth_available_ns=available,scan_ranges=laser['ranges'][j].copy() if j>=0 else np.full(laser['ranges'].shape[1],np.nan,dtype=np.float32),scan_valid=j>=0,scan_stamp_ns=int(laser['stamp_ns'][j]) if j>=0 else -1,scan_available_ns=int(laser['available_ns'][j]) if j>=0 else -1)
 assert set(result)==INPUT_KEYS;return result

def load_label(root,scene_id,proposal_index,allow_test=False):
 root,_=_scene(root,scene_id,allow_test);a=np.load(root/'labels'/scene_id/'proposal_labels_LABEL_ONLY.npz',allow_pickle=False);i=int(proposal_index)
 return {k:a[k][i].copy() for k in a.files}
