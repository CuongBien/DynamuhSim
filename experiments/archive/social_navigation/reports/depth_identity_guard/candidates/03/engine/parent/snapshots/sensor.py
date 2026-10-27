"""Depth candidate tracking baseline: sensor arrays only, no simulator/GT access."""
import numpy as np
from scipy.ndimage import binary_dilation, label
from scipy.optimize import linear_sum_assignment

CONFIG = dict(stride=4, depth_min_m=.28, depth_max_m=5., cluster_cell_m=.08,
              min_points=10, min_top_m=.18, min_vertical_span_m=.10,
              max_xy_extent_m=1.20, max_track_gap_ns=500_000_000,
              min_confirmed_hits=3, match_base_m=.30, max_speed_mps=1.5)


def rot(yaw):
    c,s=np.cos(yaw),np.sin(yaw)
    return np.array([[c,-s],[s,c]])


def to_parent(xy,pose):
    return np.asarray(xy)@rot(pose[2]).T+pose[:2]


def to_local(xy,pose):
    return (np.asarray(xy)-pose[:2])@rot(pose[2])


def project(depth,k,transform,stride=4):
    v,u=np.mgrid[0:depth.shape[0]:stride,0:depth.shape[1]:stride]
    z=depth[::stride,::stride]
    good=np.isfinite(z)&(z>=.28)&(z<=5.)
    z=z[good]
    optical=np.column_stack(((u[good]-k[2])*z/k[0],(v[good]-k[5])*z/k[4],z))
    return optical@transform[:3,:3].T+transform[:3,3]


def candidates(points):
    # This is class-agnostic geometry, not a validated person detector.
    p=points[(points[:,0]>=0)&(points[:,0]<5.3)&(abs(points[:,1])<3.2)&
             (points[:,2]>=.04)&(points[:,2]<=1.8)]
    if not len(p):return []
    cells=np.floor((p[:,:2]+[0,3.2])/.08).astype(int)
    grid=np.zeros((68,82),dtype=bool);grid[cells[:,0],cells[:,1]]=True
    components,_=label(binary_dilation(grid,iterations=1),structure=np.ones((3,3)))
    components=components[cells[:,0],cells[:,1]]
    result=[]
    for ident in np.unique(components):
        q=p[components==ident];extent=np.ptp(q,axis=0)
        if len(q)<10 or q[:,2].max()<.18 or extent[2]<.10 or extent[:2].max()>1.20:continue
        result.append(dict(xy=np.median(q[:,:2],axis=0),points=len(q),extent=extent))
    return sorted(result,key=lambda q:tuple(q['xy']))


def odom_at(query,stamp,available,values,max_age_ns=150_000_000):
    valid=(stamp<=query)&(available<=query)&(query-stamp<=max_age_ns)
    indices=np.flatnonzero(valid)
    if not len(indices):return None
    # Latest measurement already delivered. No interpolation from future odometry.
    i=int(indices[np.argmax(stamp[indices])])
    return dict(pose=values[i,:3].astype(float),stamp_ns=int(stamp[i]),available_ns=int(available[i]))


class Tracker:
    def __init__(self):
        self.tracks={};self.next_id=1;self.last_stamp=None

    def update(self,stamp,detections,pose):
        if self.last_stamp is not None and stamp<=self.last_stamp:raise ValueError('Non-increasing sensor time')
        self.last_stamp=stamp
        self.tracks={k:v for k,v in self.tracks.items() if stamp-v['stamp_ns']<=500_000_000}
        keys=sorted(self.tracks);xy=to_parent(np.array([d['xy'] for d in detections]).reshape(-1,2),pose)
        matched={}
        if keys and len(xy):
            predicted=np.array([self.tracks[k]['xy']+self.tracks[k]['velocity']*((stamp-self.tracks[k]['stamp_ns'])/1e9) for k in keys])
            cost=np.linalg.norm(predicted[:,None,:]-xy[None,:,:],axis=-1)
            gates=np.array([.30+1.5*((stamp-self.tracks[k]['stamp_ns'])/1e9) for k in keys])
            rows,cols=linear_sum_assignment(np.where(cost<=gates[:,None],cost,1e6))
            matched={int(c):keys[r] for r,c in zip(rows,cols) if cost[r,c]<=gates[r]}
        out=[]
        for j,detection in enumerate(detections):
            if j in matched:
                ident=matched[j];old=self.tracks[ident];dt=(stamp-old['stamp_ns'])/1e9
                velocity=(xy[j]-old['xy'])/dt
                speed=np.linalg.norm(velocity)
                if speed>1.5:velocity*=1.5/speed
                velocity=.5*old['velocity']+.5*velocity;hits=old['hits']+1
            else:
                ident=self.next_id;self.next_id+=1;velocity=np.zeros(2);hits=1
            self.tracks[ident]=dict(xy=xy[j],velocity=velocity,stamp_ns=stamp,hits=hits)
            out.append(dict(track_id=ident,xy_base=detection['xy'].tolist(),xy_odom=xy[j].tolist(),
                velocity_odom=velocity.tolist(),hits=hits,confirmed=hits>=3,
                point_count=detection['points'],extent_m=detection['extent'].tolist(),
                semantic_class='unknown_geometry_candidate'))
        return out
