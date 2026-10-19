"""Offline map/ray/sensor-score diagnostics; no ROS or simulator imports."""
import math
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

def yaw(q):return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def stamp(d):s=d['header']['stamp'];return s['sec']+s['nanosec']/1e9

def rays(pose,angles,boxes,humans):
    angles=np.asarray(angles)+pose['yaw'];dx=np.cos(angles);dy=np.sin(angles);x,y=pose['x'],pose['y']
    static=np.full(len(angles),np.inf);dynamic=np.full(len(angles),np.inf)
    for b in boxes:
        def interval(origin,direction,low,high):
            parallel=np.abs(direction)<1e-12;inside=low<=origin<=high
            first=np.divide(low-origin,direction,out=np.zeros_like(direction),where=~parallel)
            second=np.divide(high-origin,direction,out=np.zeros_like(direction),where=~parallel)
            lower=np.minimum(first,second);upper=np.maximum(first,second)
            return np.where(parallel,-np.inf if inside else np.inf,lower),np.where(parallel,np.inf if inside else -np.inf,upper)
        tx=interval(x,dx,b['x']-b['sx']/2,b['x']+b['sx']/2);ty=interval(y,dy,b['y']-b['sy']/2,b['y']+b['sy']/2)
        near=np.maximum(tx[0],ty[0]);far=np.minimum(tx[1],ty[1]);hit=np.where(near>=0,near,far)
        static=np.minimum(static,np.where((far>=np.maximum(near,0))&(hit>=0),hit,np.inf))
    for h in humans:
        ox,oy=x-h['x'],y-h['y'];projection=ox*dx+oy*dy;disc=projection**2-(ox*ox+oy*oy-.36**2)
        hit=-projection-np.sqrt(np.maximum(disc,0));dynamic=np.minimum(dynamic,np.where((disc>=0)&(hit>=0),hit,np.inf))
    return static,dynamic

class MapField:
    def __init__(self,path):
        cfg=yaml.safe_load(path.read_text());self.res=cfg['resolution'];self.origin=cfg['origin'][:2]
        pixels=np.asarray(Image.open(path.parent/cfg['image']));self.occupied=np.flipud((255-pixels)/255.>cfg['occupied_thresh'])
        self.height,self.width=self.occupied.shape
        self.distance=np.minimum(distance_transform_edt(~self.occupied)*self.res,2.)
    def cells(self,x,y,amcl=False):
        # Stock AMCL MAP_GXWX rounds around its centered map origin; grid server floors.
        offset=.5 if amcl else 0.
        return np.floor((np.asarray(x)-self.origin[0])/self.res+offset).astype(int),np.floor((np.asarray(y)-self.origin[1])/self.res+offset).astype(int)
    def occupied_at(self,x,y):
        ix,iy=self.cells(x,y);valid=(ix>=0)&(iy>=0)&(ix<self.width)&(iy<self.height)
        return np.where(valid,self.occupied[np.clip(iy,0,self.height-1),np.clip(ix,0,self.width-1)],True)
    def distances(self,x,y):
        ix,iy=self.cells(x,y,amcl=True);valid=(ix>=0)&(iy>=0)&(ix<self.width)&(iy<self.height)
        return np.where(valid,self.distance[np.clip(iy,0,self.height-1),np.clip(ix,0,self.width-1)],2.)
    def score(self,poses,scan,mask=None):
        # 1.3.13 likelihood_field sensor weighting only; not a PF posterior/replay.
        poses=np.atleast_2d(np.asarray(poses,dtype=float));ranges=np.asarray(scan['ranges'],dtype=float)
        indices=np.arange(0,len(ranges),max(1,(len(ranges)-1)//59));rs=ranges[indices]
        valid=np.isfinite(rs)&(rs>max(scan['range_min'],.12))&(rs<min(scan['range_max'],3.5))
        if mask is not None:valid&=np.asarray(mask,dtype=bool)[indices]
        indices=indices[valid];rs=rs[valid];angles=scan['angle_min']+indices*scan['angle_increment']
        endpoints_x=poses[:,0,None]+rs*np.cos(poses[:,2,None]+angles);endpoints_y=poses[:,1,None]+rs*np.sin(poses[:,2,None]+angles)
        distance=self.distances(endpoints_x,endpoints_y);pz=.5*np.exp(-distance**2/(2*.2**2))+.5/min(scan['range_max'],3.5)
        return 1.+np.sum(pz**3,axis=1),len(rs)

def grid_footprint_cost(grid,pose):
    info=grid['info'];origin=info['origin']['position'];res=info['resolution'];values=[];c,s=math.cos(pose['yaw']),math.sin(pose['yaw']);data=grid['data']
    for x in np.linspace(-.13,.13,15):
        for y in np.linspace(-.11,.11,13):
            wx=pose['x']+c*x-s*y;wy=pose['y']+s*x+c*y;ix=math.floor((wx-origin['x'])/res);iy=math.floor((wy-origin['y'])/res)
            if 0<=ix<info['width'] and 0<=iy<info['height']:values.append(data[iy*info['width']+ix])
    return dict(footprint_occupancy_max=max(values) if values else None,footprint_unknown_fraction=sum(v<0 for v in values)/len(values) if values else None)
