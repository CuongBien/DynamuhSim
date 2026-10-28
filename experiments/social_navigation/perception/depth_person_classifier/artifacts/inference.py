"""Portable NumPy inference. No torch, ROS, labels, IDs or world-state imports."""
import numpy as np
def normalized(data,stats):
 d=(data['depth']-stats['depth_mean'])/stats['depth_std'];l=(data['lidar']-stats['lidar_mean'])/stats['lidar_std'];l[~data['lidar_mask']]=0;return d.astype(np.float32),l.astype(np.float32),data['lidar_mask'].astype(np.float32)
def numpy_logits(data,arrays,hidden,modality):
 d,l,mask=normalized(data,arrays)
 def branch(x,prefix):
  z=x@arrays[prefix+'.0.weight'].T+arrays[prefix+'.0.bias']
  if hidden:z=np.tanh(z)@arrays[prefix+'.2.weight'].T+arrays[prefix+'.2.bias']
  return z[:,0]
 result=branch(d,'depth')
 if modality=='fusion':result=result+mask*branch(l,'lidar')
 return result
