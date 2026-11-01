"""Small PointNet-style head candidate. Random weights until a future gated training run.

Not the original PointNet implementation or a pretrained person detector. Requires Torch
only when explicitly imported. No RGB, GT IDs, scene IDs or simulator IDs are inputs.
"""
import torch
from torch import nn


class PointPatchClassifier(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder=nn.Sequential(nn.Conv1d(3,32,1),nn.ReLU(),nn.Conv1d(32,64,1),nn.ReLU(),nn.Conv1d(64,128,1),nn.ReLU())
        self.head=nn.Sequential(nn.Linear(128,64),nn.ReLU(),nn.Linear(64,1))

    def forward(self,points,valid):
        if points.ndim!=3 or points.shape[2]!=3 or valid.shape!=points.shape[:2] or valid.dtype!=torch.bool:
            raise ValueError('Expected B x N x 3 and B x N boolean mask')
        count=valid.sum(dim=1,keepdim=True)
        if bool((count==0).any()) or not bool(torch.isfinite(points[valid]).all()):
            raise ValueError('Empty/nonfinite measured patch')
        clean=torch.where(valid[:,:,None],points,torch.zeros_like(points))
        center=clean[:,:,:2].sum(dim=1,keepdim=True)/count[:,:,None]
        xyz=torch.cat([clean[:,:,:2]-center,clean[:,:,2:]],dim=2)
        encoded=self.encoder(xyz.transpose(1,2))
        encoded=encoded.masked_fill(~valid[:,None,:],float('-inf'))
        return self.head(encoded.max(dim=2).values)[:,0]
