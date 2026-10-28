"""Depth-only observed BEV; SE(2) warp, no inferred free space for invalid rays."""
import numpy as np
from sensor import rot


def warp(points,source_pose,anchor_pose):
    out=points.copy()
    out[:,:2]=(points[:,:2]@rot(source_pose[2]).T+source_pose[:2]-anchor_pose[:2])@rot(anchor_pose[2])
    return out


def raster(points,origin):
    grid=np.full((64,64),-1,dtype=np.int8)
    # Ground returns can support free rays; ceiling/invalid returns cannot.
    rays=points[(points[:,2]>=-.05)&(points[:,2]<=1.8)]
    if len(rays):
        # Dedup only equal XY endpoints. Occupancy height decisions use original points below.
        xy=np.unique(rays[:,:2],axis=0)
        max_length=float(np.linalg.norm(xy-origin[:2],axis=1).max())
        steps=max(1,int(np.ceil(max_length/.04)))
        for start in range(0,len(xy),256):
            end=xy[start:start+256]
            pts=origin[:2]+(end[:,None,:]-origin[:2])*(np.arange(steps)[None,:,None]/steps)
            cells=np.floor((pts.reshape(-1,2)+[0,3.2])/.1).astype(int)
            valid=(cells[:,0]>=0)&(cells[:,0]<64)&(cells[:,1]>=0)&(cells[:,1]<64)
            cells=cells[valid];grid[cells[:,0],cells[:,1]]=0
    p=points[(points[:,2]>=.03)&(points[:,2]<=1.8)]
    cells=np.floor((p[:,:2]+[0,3.2])/.1).astype(int)
    valid=(cells[:,0]>=0)&(cells[:,0]<64)&(cells[:,1]>=0)&(cells[:,1]<64)
    cells=cells[valid];grid[cells[:,0],cells[:,1]]=1
    return grid


def nearest_frame(stamps,queries,tolerance_ns=50_000_000):
    stamps=np.asarray(stamps);queries=np.asarray(queries)
    hi=np.searchsorted(stamps,queries);lo=np.maximum(hi-1,0);hi=np.minimum(hi,len(stamps)-1)
    index=np.where(abs(stamps[lo]-queries)<=abs(stamps[hi]-queries),lo,hi)
    return index,abs(stamps[index]-queries)<=tolerance_ns


def interpolate(stamps,values,queries,max_gap_ns=100_000_000):
    stamps=np.asarray(stamps);values=np.asarray(values);queries=np.asarray(queries)
    j=np.searchsorted(stamps,queries,side='left');hi=np.minimum(j,len(stamps)-1);lo=np.maximum(j-1,0)
    exact=stamps[hi]==queries;lo=np.where(exact,hi,lo)
    gap=stamps[hi]-stamps[lo]
    valid=(queries>=stamps[0])&(queries<=stamps[-1])&(gap<=max_gap_ns)
    weight=(queries-stamps[lo])/np.maximum(gap,1)
    w=weight.reshape(weight.shape+(1,)*(values.ndim-1))
    return values[lo]*(1-w)+values[hi]*w,valid
