"""Heading-consistent version of pre-guard soft regrouping.

Same cohesion constants as v1; active regrouping uses the native shared goal
intent instead of reinforcing an already partially rotated displacement.
Large heading errors brake translation while body yaw remains bounded. No
post-guard position/yaw changes are made.
"""
import math
import numpy as np
from .group_cohesion import GroupCohesion,PARAMETERS,proposals as soft_proposals,cap
from .group_intent import preferred_velocity

PROFILE=dict(schema='dynamuhsim_human_motion/4',profile='group_cohesion_v2',group_avoidance=False,per_person_guard='existing_response_v2',cohesion=PARAMETERS,heading_controller=dict(model='bounded_heading_cosine_speed',low_speed_deadband_mps=.05))


def proposals(positions,native_velocities,speed,shared_velocity,old_yaws,native_yaws,dt,rotation_rate):
    _,detail=soft_proposals(positions,native_velocities,speed)
    shared=np.asarray(shared_velocity,dtype=float)
    if not np.all(np.isfinite(shared)) or not math.isfinite(dt) or dt<=0 or not math.isfinite(rotation_rate) or rotation_rate<0:raise ValueError('Invalid heading-cohesion inputs')
    norm=float(np.linalg.norm(shared));direction=shared/norm if norm>1e-12 else np.zeros(2);result=[];yaws=[];active=[];desired=[]
    for i,(v,old_yaw,native_yaw,force) in enumerate(zip(native_velocities,old_yaws,native_yaws,detail['attraction'])):
        change=detail['forward_gate']<1 or np.linalg.norm(force)>1e-12;active.append(bool(change))
        native=cap(v,speed)
        if not change:
            result.append(native);yaws.append(native_yaw);desired.append(native);continue
        target=cap(detail['forward_gate']*np.linalg.norm(native)*direction+force,speed);desired.append(target);target_speed=float(np.linalg.norm(target))
        if target_speed<=PROFILE['heading_controller']['low_speed_deadband_mps']:
            result.append(np.zeros(2));yaws.append(old_yaw);continue
        angle=math.atan2(target[1],target[0]);delta=math.atan2(math.sin(angle-old_yaw),math.cos(angle-old_yaw));turn=float(np.clip(delta,-rotation_rate*dt,rotation_rate*dt));yaw=math.atan2(math.sin(old_yaw+turn),math.cos(old_yaw+turn));advance=target_speed*max(0.,math.cos(delta));result.append(advance*np.array([math.cos(yaw),math.sin(yaw)]));yaws.append(yaw)
    detail.update(active_adjustment=active,desired_velocity=[v.tolist() for v in desired],target_yaws=yaws)
    return np.array(result),detail


class GroupCohesionHeading(GroupCohesion):
    profile='group_cohesion_v2'
    def propose(self):
        self.native();motion=self.kernel.motion;e=self.kernel.env;dt=e.TIMESTEP;diagnostics=[]
        if motion.start is None:raise RuntimeError('Cohesion requires a native integration step')
        for group in e.moving_interactions:
            members=group.humans;positions=np.array([motion.start[int(h.id)][:2] for h in members]);old=[motion.start[int(h.id)][2] for h in members];native_yaws=[float(h.orientation) for h in members];native=np.array([[(h.x-p[0])/dt,(h.y-p[1])/dt] for h,p in zip(members,positions)]);shared=group.prev_group_velocity
            if shared is None:shared=preferred_velocity((group.x,group.y),(group.goal_x,group.goal_y),e.MAX_ADVANCE_HUMAN)
            adjusted,detail=proposals(positions,native,e.MAX_ADVANCE_HUMAN,shared,old,native_yaws,dt,e.MAX_ROTATION_HUMAN)
            for i,(h,p,v) in enumerate(zip(members,positions,adjusted)):
                if detail['active_adjustment'][i]:h.x,h.y=p+v*dt;h.orientation=detail['target_yaws'][i]
            diagnostics.append(dict(members=[int(h.id) for h in members],positions_before=positions.tolist(),native_velocity=native.tolist(),preferred_velocity=adjusted.tolist(),old_yaws=old,native_yaws=native_yaws,shared_velocity=[float(v) for v in shared],dt_s=dt,rotation_rate_radps=e.MAX_ROTATION_HUMAN,speed_cap_mps=float(e.MAX_ADVANCE_HUMAN),**detail))
        self.cohesion_last=diagnostics


def verify_trace(rows):
    count=steps=0
    for row in rows:
        if not row.get('active_phase'):continue
        steps+=1
        if 'group_cohesion' not in row:return False,{}
        for v in row['group_cohesion']:
            expected,detail=proposals(v['positions_before'],v['native_velocity'],v['speed_cap_mps'],v['shared_velocity'],v['old_yaws'],v['native_yaws'],v['dt_s'],v['rotation_rate_radps'])
            if not np.allclose(expected,v['preferred_velocity'],rtol=0,atol=1e-8) or not np.allclose(detail['target_yaws'],v['target_yaws'],rtol=0,atol=1e-8) or detail['active_adjustment']!=v['active_adjustment']:return False,{}
            count+=1
    return steps>0 and count>0,dict(active_steps=steps,group_proposals=count)
