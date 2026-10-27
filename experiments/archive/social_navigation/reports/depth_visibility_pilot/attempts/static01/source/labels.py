"""LABEL ONLY: causal pixel ownership from synchronized static interventions."""
import numpy as np

def same_depth(a,b,eps):
 valid=np.isfinite(a)&np.isfinite(b);delta=np.zeros(a.shape,dtype=float);np.subtract(a,b,out=delta,where=valid)
 return valid&(np.abs(delta)<=eps)

def revealed(full,dropped,eps):
 valid=np.isfinite(full);delta=np.zeros(full.shape,dtype=float);both=valid&np.isfinite(dropped);np.subtract(dropped,full,out=delta,where=both)
 return valid&((~np.isfinite(dropped))|(delta>eps))

def ownership(full,drops,isolated,eps=.001):
 masks=[];contradictions=[]
 for dropped,alone in zip(drops,isolated):
  effect=revealed(full,dropped,eps);consistent=same_depth(full,alone,eps)
  masks.append(effect&consistent);contradictions.append(effect&~consistent)
 stack=np.stack(masks);overlap=stack.sum(axis=0)>1;uncertain=np.any(contradictions,axis=0)|overlap
 labels=np.zeros(full.shape,dtype=np.int16)
 for i,mask in enumerate(masks,1):labels[mask&~uncertain]=i
 labels[uncertain]=-1
 return labels,dict(contradictions=int(np.sum(contradictions)),overlap_pixels=int(overlap.sum()),uncertain_pixels=int(uncertain.sum()))
