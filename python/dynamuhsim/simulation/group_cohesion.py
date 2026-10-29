"""Soft regrouping proposal before unchanged per-person ORCA/projection.

No position correction is applied after the safety guard. Native integration
runs once (retaining its RNG/noise); only its tentative velocity proposal is
adjusted for moving groups. Limits below define one frozen experimental model.
"""
import math
import numpy as np
from .group_intent import GroupGoal

PARAMETERS=dict(free_radius_m=.9,slow_radius_m=1.,stop_radius_m=1.6,attraction_gain_per_s=.8,max_attraction_mps=.4)
PROFILE=dict(schema="dynamuhsim_human_motion/3",profile="group_cohesion_v1",group_avoidance=False,per_person_guard="existing_response_v2",cohesion=PARAMETERS)


def cap(vector,speed):
    vector=np.asarray(vector,dtype=float);norm=float(np.linalg.norm(vector))
    if not np.all(np.isfinite(vector)) or not math.isfinite(speed) or speed<0:raise ValueError("Invalid cohesion proposal")
    return vector if norm<=speed or norm==0 else vector*(speed/norm)


def proposals(positions,native_velocities,speed):
    positions=np.asarray(positions,dtype=float);base=np.array([cap(v,speed) for v in native_velocities]);center=positions.mean(axis=0);delta=center-positions;distance=np.linalg.norm(delta,axis=1);spread=float(distance.max());gate=float(np.clip((PARAMETERS['stop_radius_m']-spread)/(PARAMETERS['stop_radius_m']-PARAMETERS['slow_radius_m']),0,1));attraction=[]
    for vector,d in zip(delta,distance):
        force=vector*(PARAMETERS['attraction_gain_per_s']*max(0.,d-PARAMETERS['free_radius_m'])/d) if d>0 else np.zeros(2)
        attraction.append(cap(force,PARAMETERS['max_attraction_mps']))
    adjusted=np.array([cap(gate*v+force,speed) for v,force in zip(base,attraction)])
    return adjusted,dict(center=center.tolist(),spread_m=spread,forward_gate=gate,attraction=[v.tolist() for v in attraction],capped_native_velocity=base.tolist())


class GroupCohesion(GroupGoal):
    profile='group_cohesion_v1'
    def __init__(self,kernel):
        super().__init__(kernel)
        self.native=kernel.motion.original['handle_collision_and_update']
        self.cohesion_last=[]
        kernel.motion.original['handle_collision_and_update']=self.propose

    def propose(self):
        self.native()
        motion=self.kernel.motion;e=self.kernel.env;dt=e.TIMESTEP;diagnostics=[]
        if motion.start is None:raise RuntimeError('Cohesion requires a native integration step')
        for group in e.moving_interactions:
            members=group.humans;positions=np.array([motion.start[int(h.id)][:2] for h in members]);native=np.array([[(h.x-pos[0])/dt,(h.y-pos[1])/dt] for h,pos in zip(members,positions)]);adjusted,detail=proposals(positions,native,e.MAX_ADVANCE_HUMAN)
            for i,(h,pos,v) in enumerate(zip(members,positions,adjusted)):
                if detail['forward_gate']<1 or np.linalg.norm(detail['attraction'][i])>1e-12:
                    h.x,h.y=pos+v*dt
                    if np.linalg.norm(v)>.05:h.orientation=math.atan2(v[1],v[0])
            diagnostics.append(dict(members=[int(h.id) for h in members],positions_before=positions.tolist(),native_velocity=native.tolist(),preferred_velocity=adjusted.tolist(),speed_cap_mps=float(e.MAX_ADVANCE_HUMAN),**detail))
        self.cohesion_last=diagnostics


def verify_cohesion_trace(rows):
    count=steps=0
    for row in rows:
        if not row.get('active_phase'):continue
        steps+=1
        if 'group_cohesion' not in row:return False,{}
        for value in row['group_cohesion']:
            expected,detail=proposals(value['positions_before'],value['native_velocity'],value['speed_cap_mps'])
            if not np.allclose(expected,value['preferred_velocity'],rtol=0,atol=1e-8) or abs(detail['forward_gate']-value['forward_gate'])>1e-8:return False,{}
            count+=1
    return steps>0 and count>0,dict(active_steps=steps,group_proposals=count)
