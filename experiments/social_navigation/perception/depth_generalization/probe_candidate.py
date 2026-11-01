#!/usr/bin/env python3
"""CUDA forward/backward compatibility only. No fit, optimizer or dataset labels."""
import argparse,json,time
from pathlib import Path
import torch
from candidate import PointPatchClassifier


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    torch.set_num_threads(1);torch.manual_seed(17)
    if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable; no CPU fallback for this probe')
    device=torch.device('cuda');model=PointPatchClassifier().to(device).eval()
    points=torch.randn(4,512,3,device=device);mask=torch.ones(4,512,dtype=torch.bool,device=device)
    mask[:,400:]=False
    with torch.no_grad():
        y=model(points,mask);order=torch.randperm(512,device=device)
        shuffled=model(points[:,order],mask[:,order])
        altered=points.clone();altered[~mask]=1e9
        padded=model(altered,mask)
    checks=dict(permutation_invariance=bool(torch.allclose(y,shuffled,atol=1e-5,rtol=1e-5)),
                invalid_padding_ignored=bool(torch.allclose(y,padded,atol=1e-5,rtol=1e-5)))
    # A gradient check only; weights never receive optimizer updates.
    before={k:v.detach().clone() for k,v in model.state_dict().items()}
    model(points,mask).square().mean().backward()
    checks['finite_backward']=all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in model.parameters())
    checks['weights_unchanged']=all(torch.equal(v,before[k]) for k,v in model.state_dict().items())
    try:model(points,torch.zeros_like(mask));checks['empty_patch_rejected']=False
    except ValueError:checks['empty_patch_rejected']=True
    obj=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,
             python=__import__('sys').version.split()[0],torch=torch.__version__,cuda=torch.version.cuda,
             device=torch.cuda.get_device_name(),parameters=sum(p.numel() for p in model.parameters()),
             architecture='PointNet-style observed XYZ patch binary head, not original/pretrained PointNet',
             training_performed=False,optimizer_steps=0,semantic_quality_evaluated=False,
             peak_cuda_allocated_bytes=torch.cuda.max_memory_allocated())
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:json.dump(obj,f,indent=2)
    print(json.dumps(obj,indent=2))
    if obj['status']!='PASS':raise SystemExit(1)


if __name__=='__main__':main()
