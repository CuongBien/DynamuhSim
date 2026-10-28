"""Tiny CUDA trainable classifier and independent portable NumPy forward pass."""
import numpy as np
import torch
from inference import normalized,numpy_logits
from torch import nn
class Model(nn.Module):
 def __init__(self,hidden,modality):
  super().__init__();self.modality=modality;self.depth=nn.Sequential(nn.Linear(30,hidden),nn.Tanh(),nn.Linear(hidden,1)) if hidden else nn.Sequential(nn.Linear(30,1));self.lidar=(nn.Sequential(nn.Linear(8,8),nn.Tanh(),nn.Linear(8,1)) if hidden else nn.Sequential(nn.Linear(8,1))) if modality=='fusion' else None
 def forward(self,depth,lidar,mask):
  result=self.depth(depth).squeeze(-1)
  if self.lidar is not None:result=result+mask*self.lidar(lidar).squeeze(-1)
  return result

def normalization(data):
 valid=data['label_valid'];mask=data['lidar_mask']&valid;assert mask.any();return dict(depth_mean=data['depth'][valid].mean(0),depth_std=np.maximum(data['depth'][valid].std(0),1e-3),lidar_mean=data['lidar'][mask].mean(0),lidar_std=np.maximum(data['lidar'][mask].std(0),1e-3))
def portable_arrays(model,stats):return {**stats,**{k:v.detach().cpu().numpy() for k,v in model.state_dict().items()}}
