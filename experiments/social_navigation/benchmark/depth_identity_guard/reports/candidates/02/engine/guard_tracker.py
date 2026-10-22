"""Causal conservative identity assignment; no GT/persona/backend inputs."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'parent'))
from refinement import CONFIG as PARENT
from snapshots.sensor import to_parent
CONFIG=json.loads((HERE/'config.json').read_text())


def shape_distance(a,b):return float(np.sqrt(np.mean(np.log((a+.05)/(b+.05))**2)))


def mutual_margin(distance,eligible,margin):
    if not distance.size:return {},set()
    cost=np.where(eligible,distance,np.inf);best_col=np.argmin(cost,axis=1);best_row=np.argmin(cost,axis=0)
    matched={};ambiguous=set()
    for i,j in enumerate(best_col):
        if not eligible[i,j] or best_row[j]!=i:continue
        row_alt=np.delete(cost[i],j).min(initial=np.inf);col_alt=np.delete(cost[:,j],i).min(initial=np.inf)
        if min(row_alt,col_alt)-cost[i,j]<margin:
            ambiguous.add(int(j))
            ambiguous.update(int(v) for v in np.flatnonzero(eligible[i]&(cost[i]<=cost[i,j]+margin)))
        else:matched[int(j)]=i
    return {j:i for j,i in matched.items() if j not in ambiguous},ambiguous


class IdentityGuard:
    def __init__(self):self.tracks={};self.next_id=1;self.last_stamp=None

    def update(self,stamp,detections,pose):
        if self.last_stamp is not None and stamp<=self.last_stamp:raise ValueError('Non-increasing sensor time')
        self.last_stamp=stamp;a=CONFIG['active'];p=PARENT['tracking']
        self.tracks={k:v for k,v in self.tracks.items() if stamp-v['stamp_ns']<=p['dormant_ttl_ns']}
        xy=to_parent(np.array([d['xy'] for d in detections]).reshape(-1,2),pose)
        active=sorted(k for k,v in self.tracks.items() if stamp-v['stamp_ns']<=p['active_ttl_ns'])
        dormant=sorted(k for k,v in self.tracks.items() if stamp-v['stamp_ns']>p['active_ttl_ns'] and v['hits']>=p['dormant_min_hits'])
        matched={};unknown=set();reacquired=set()
        if active and len(xy):
            residual=np.zeros((len(active),len(xy)));eligible=np.zeros(residual.shape,bool)
            for i,k in enumerate(active):
                old=self.tracks[k];dt=(stamp-old['stamp_ns'])/1e9
                pred=old['xy']+old['velocity']*dt;residual[i]=np.linalg.norm(xy-pred,axis=1)
                eligible[i]=residual[i]<=a['residual_base_m']+a['residual_growth_mps']*dt
                if stamp-old['stamp_ns']>a['gap_shape_check_after_ns']:
                    displacement=np.linalg.norm(xy-old['xy'],axis=1)
                    eligible[i]&=displacement<=a['position_slack_m']+a['max_displacement_speed_mps']*dt
                    eligible[i]&=np.array([shape_distance(d['shape'],old['shape'])<=a['max_gap_logshape_distance'] for d in detections])
            # Preserve regular measured tracking; stricter reassociation applies after gaps.
            rows,cols=linear_sum_assignment(np.where(eligible,residual,1e6))
            gap_pairs,gap_ambiguous=mutual_margin(residual,eligible,a['ambiguity_margin_m'])
            for i,j in zip(rows,cols):
                if not eligible[i,j]:continue
                old=self.tracks[active[i]]
                if stamp-old['stamp_ns']>a['gap_shape_check_after_ns']:
                    if gap_pairs.get(int(j))!=i:
                        if j in gap_ambiguous:unknown.add(int(j))
                        continue
                # Two established tracks competing for the same measurement are unresolved.
                peers=[k for k in range(len(active)) if k!=i and self.tracks[active[k]]['hits']>=3 and eligible[k,j]]
                competitor=min((residual[k,j] for k in peers),default=np.inf)
                if old['hits']>=3 and competitor-residual[i,j]<a['ambiguity_margin_m']:
                    unknown.add(int(j));continue
                matched[int(j)]=active[i]
        remaining=[j for j in range(len(xy)) if j not in matched and j not in unknown]
        if dormant and remaining:
            distance=np.array([np.linalg.norm(xy[remaining]-self.tracks[k]['xy'],axis=1) for k in dormant]);eligible=np.zeros(distance.shape,bool)
            for i,k in enumerate(dormant):
                old=self.tracks[k];dt=(stamp-old['stamp_ns'])/1e9
                for j,column in enumerate(remaining):
                    eligible[i,j]=distance[i,j]<=min(p['dormant_gate_base_m']+p['dormant_gate_growth_mps']*dt,a['position_slack_m']+a['max_displacement_speed_mps']*dt) and shape_distance(detections[column]['shape'],old['shape'])<=p['dormant_max_logshape_distance']
            cost=np.where(eligible,distance,1e6)
            for i in range(len(dormant)):
                j=int(np.argmin(cost[i]))
                if not eligible[i,j] or int(np.argmin(cost[:,j]))!=i:continue
                alternatives=min(np.delete(distance[i],j).min(initial=np.inf),np.delete(distance[:,j],i).min(initial=np.inf))
                if alternatives-distance[i,j]<p['dormant_position_margin_m']:continue
                column=remaining[j];matched[column]=dormant[i];reacquired.add(column)
        out=[]
        for j,d in enumerate(detections):
            if j in unknown:
                # Retain the actual measured proposal, but do not invent a person identity.
                out.append(dict(track_id=-1,proposal_index=j,xy_base=d['xy'].tolist(),xy_odom=xy[j].tolist(),velocity_odom=[0.,0.],velocity_valid=False,
                    hits=0,confirmed=False,identity_observed=False,association_ambiguous=True,point_count=d['points'],extent_m=d['extent'].tolist(),
                    semantic_class='unknown_geometry_candidate',reacquired_after_gap=False,gap_since_last_measurement_ns=0))
                continue
            gap=0
            if j in matched:
                ident=matched[j];old=self.tracks[ident];gap=stamp-old['stamp_ns'];velocity=(xy[j]-old['xy'])/(gap/1e9)
                speed=np.linalg.norm(velocity)
                if speed>1.5:velocity*=1.5/speed
                velocity=.5*old['velocity']+.5*velocity;hits=old['hits']+1
            else:ident=self.next_id;self.next_id+=1;velocity=np.zeros(2);hits=1
            self.tracks[ident]=dict(xy=xy[j],velocity=velocity,stamp_ns=stamp,hits=hits,shape=d['shape'])
            out.append(dict(track_id=ident,proposal_index=j,xy_base=d['xy'].tolist(),xy_odom=xy[j].tolist(),velocity_odom=velocity.tolist(),velocity_valid=hits>1,
                hits=hits,confirmed=hits>=3,identity_observed=True,association_ambiguous=False,point_count=d['points'],extent_m=d['extent'].tolist(),
                semantic_class='unknown_geometry_candidate',reacquired_after_gap=j in reacquired,gap_since_last_measurement_ns=gap))
        return out
