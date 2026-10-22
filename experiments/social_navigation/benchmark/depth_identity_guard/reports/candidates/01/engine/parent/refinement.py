"""Frozen geometry prior and conservative dormant-ID reuse; no label access."""
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_dilation,label
from scipy.optimize import linear_sum_assignment
from snapshots.sensor import candidates as baseline_candidates, to_parent

CONFIG=json.loads(Path(__file__).with_name('config.json').read_text())


def enrich(points):
    original=baseline_candidates(points)
    p=points[(points[:,0]>=0)&(points[:,0]<5.3)&(abs(points[:,1])<3.2)&(points[:,2]>=.04)&(points[:,2]<=1.8)]
    if not original:return [],[]
    cells=np.floor((p[:,:2]+[0,3.2])/.08).astype(int)
    grid=np.zeros((68,82),bool);grid[cells[:,0],cells[:,1]]=True
    cc,_=label(binary_dilation(grid,iterations=1),structure=np.ones((3,3)));ids=cc[cells[:,0],cells[:,1]]
    clusters=[p[ids==i] for i in np.unique(ids)]
    centers=np.array([np.median(q[:,:2],axis=0) for q in clusters])
    kept=[];rejected=[];c=CONFIG['shape_filter']
    for item in original:
        q=clusters[int(np.argmin(np.linalg.norm(centers-item['xy'],axis=1)))]
        eigen=np.sqrt(np.maximum(0,np.linalg.eigvalsh(np.cov(q[:,:2].T))))
        bins=np.linspace(q[:,2].min(),q[:,2].max()+1e-6,c['height_bins']+1)
        median=[]
        for j in range(c['height_bins']):
            level=q[(q[:,2]>=bins[j])&(q[:,2]<bins[j+1])]
            if len(level)>=2:median.append(np.median(level[:,:2],axis=0))
        shift=float(np.linalg.norm(np.array(median)-np.median(median,axis=0),axis=1).max()) if median else None
        item=dict(item,shape=np.array([np.ptp(q[:,2]),eigen[1],eigen[0]]),
            profile_shift_m=shift,profile_bins=len(median))
        reject=item['shape'][0]>=c['min_vertical_span_m'] and (
            item['shape'][2]<=c['thin_surface_minor_std_m'] or
            (len(median)>=c['min_profile_bins'] and shift<=c['constant_profile_shift_m']))
        (rejected if reject else kept).append(item)
    return kept,rejected


class DormantTracker:
    def __init__(self):self.tracks={};self.next_id=1;self.last_stamp=None

    def update(self,stamp,detections,pose):
        if self.last_stamp is not None and stamp<=self.last_stamp:raise ValueError('Non-increasing sensor time')
        self.last_stamp=stamp;c=CONFIG['tracking']
        self.tracks={k:v for k,v in self.tracks.items() if stamp-v['stamp_ns']<=c['dormant_ttl_ns']}
        xy=to_parent(np.array([d['xy'] for d in detections]).reshape(-1,2),pose)
        active=sorted(k for k,v in self.tracks.items() if stamp-v['stamp_ns']<=c['active_ttl_ns'])
        dormant=sorted(k for k,v in self.tracks.items() if stamp-v['stamp_ns']>c['active_ttl_ns'] and v['hits']>=c['dormant_min_hits'])
        matched={};reacquired=set()
        if active and len(xy):
            pred=np.array([self.tracks[k]['xy']+self.tracks[k]['velocity']*((stamp-self.tracks[k]['stamp_ns'])/1e9) for k in active])
            cost=np.linalg.norm(pred[:,None,:]-xy[None,:,:],axis=-1)
            gates=np.array([.30+1.5*((stamp-self.tracks[k]['stamp_ns'])/1e9) for k in active])
            rows,cols=linear_sum_assignment(np.where(cost<=gates[:,None],cost,1e6))
            matched={int(j):active[i] for i,j in zip(rows,cols) if cost[i,j]<=gates[i]}
        remaining=[j for j in range(len(xy)) if j not in matched]
        if dormant and remaining:
            # Do not let noisy long-gap velocity make a large extrapolated search.
            pred=np.array([self.tracks[k]['xy'] for k in dormant])
            distance=np.linalg.norm(pred[:,None,:]-xy[remaining][None,:,:],axis=-1)
            eligible=np.zeros(distance.shape,bool)
            for i,k in enumerate(dormant):
                old=self.tracks[k];dt=(stamp-old['stamp_ns'])/1e9
                for j,column in enumerate(remaining):
                    appearance=float(np.sqrt(np.mean(np.log((detections[column]['shape']+.05)/(old['shape']+.05))**2)))
                    eligible[i,j]=distance[i,j]<=c['dormant_gate_base_m']+c['dormant_gate_growth_mps']*dt and appearance<=c['dormant_max_logshape_distance']
            cost=np.where(eligible,distance,1e6)
            for i in range(len(dormant)):
                j=int(np.argmin(cost[i]))
                if not eligible[i,j] or int(np.argmin(cost[:,j]))!=i:continue
                # Even an ineligible nearby alternative makes positional identity ambiguous.
                alternatives=min(np.delete(distance[i],j).min(initial=np.inf),np.delete(distance[:,j],i).min(initial=np.inf))
                if alternatives-distance[i,j]<c['dormant_position_margin_m']:continue
                column=remaining[j];matched[column]=dormant[i];reacquired.add(column)
        out=[]
        for j,d in enumerate(detections):
            gap=0
            if j in matched:
                ident=matched[j];old=self.tracks[ident];gap=stamp-old['stamp_ns'];velocity=(xy[j]-old['xy'])/(gap/1e9)
                speed=np.linalg.norm(velocity)
                if speed>1.5:velocity*=1.5/speed
                velocity=.5*old['velocity']+.5*velocity;hits=old['hits']+1
            else:ident=self.next_id;self.next_id+=1;velocity=np.zeros(2);hits=1
            self.tracks[ident]=dict(xy=xy[j],velocity=velocity,stamp_ns=stamp,hits=hits,shape=d['shape'])
            out.append(dict(track_id=ident,xy_base=d['xy'].tolist(),xy_odom=xy[j].tolist(),velocity_odom=velocity.tolist(),
                hits=hits,confirmed=hits>=3,point_count=d['points'],extent_m=d['extent'].tolist(),
                semantic_class='unknown_geometry_candidate',reacquired_after_gap=j in reacquired,gap_since_last_measurement_ns=gap))
        return out
