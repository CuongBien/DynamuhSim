"""Opt-in ablation: shared group goal intent, without a rigid group ORCA disc.

Native goals/smoothing/turn limits/RNG and the existing per-person ORCA plus
swept safety projection remain intact. Only the upper group-velocity proposal
changes. Reference group radii are logged but never used as constraints here.
"""
import math
import numpy as np
from .human_groups import group_state

PROFILE = dict(schema="dynamuhsim_human_motion/2", profile="group_goal_v1", group_avoidance=False, per_person_guard="existing_response_v2")


def preferred_velocity(center,goal,speed):
    v=np.array([goal[0]-center[0],goal[1]-center[1]],dtype=np.float32)
    if not np.all(np.isfinite(v)) or not math.isfinite(speed) or speed<0:
        raise ValueError("Invalid group goal intent")
    norm=np.linalg.norm(v)
    if not np.isfinite(norm):raise ValueError("Unbounded group goal intent")
    if norm!=0:v/=norm
    v*=speed
    return tuple(float(x) for x in v)


class GroupGoal:
    profile="group_goal_v1"
    def __init__(self,kernel):
        self.kernel=kernel
        self.last=[]

    def velocities(self):
        e=self.kernel.env;result=[];diagnostics=[]
        for group in e.moving_interactions:
            center=[float(group.x),float(group.y)];goal=[float(group.goal_x),float(group.goal_y)]
            velocity=preferred_velocity(center,goal,e.MAX_ADVANCE_HUMAN)
            radius,measured=group_state(group,self.kernel.motion.velocities)
            result.append(velocity)
            diagnostics.append(dict(members=[int(h.id) for h in group.humans],center=center,goal=goal,preferred_velocity=list(velocity),speed_cap_mps=float(e.MAX_ADVANCE_HUMAN),group_radius_reference_m=radius,measured_velocity=list(measured),control="shared_goal_only",group_avoidance=False))
        self.last=diagnostics
        return result


def verify_goal_intents(rows):
    count=steps=0
    for row in rows:
        if not row.get("active_phase"):continue
        steps+=1
        if "group_orca" in row or "group_intent" not in row:return False,{}
        for value in row["group_intent"]:
            if value["control"]!="shared_goal_only" or value["group_avoidance"] is not False:return False,{}
            expected=preferred_velocity(value["center"],value["goal"],value["speed_cap_mps"])
            if not np.allclose(expected,value["preferred_velocity"],rtol=0,atol=1e-8):return False,{}
            count+=1
    return steps>0 and count>0,dict(active_steps=steps,group_proposals=count)
