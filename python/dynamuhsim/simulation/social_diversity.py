"""Opt-in seeded social preferences BEFORE existing ORCA/swept projection.

GroupGoal avoids the unstable enclosing-disc upper solve. Native lifecycle,
arrival geometry and random draws still execute; native per-tick motion noise
is not used as this profile's preference. No pose is altered after projection.
Traits are engineering settings, not a fitted human population distribution.
"""
import copy
import hashlib
import math
import random
from .group_intent import GroupGoal
from .goal_clock import people

PROFILE = dict(schema="dynamuhsim_human_motion/5", profile="social_diversity_v1",
               group_avoidance=False, per_person_guard="existing_response_v2",
               traits="sha256_seed_id_v1", encounter_hold_s=.8, clear_hold_s=.6,
               maximum_wait_s=.8, wait_cooldown_s=2., cohesion_free_m=.9,
               cohesion_gain_per_s=.65, cohesion_cap_mps=.24)


def traits(seed, hid):
    rng = random.Random(int.from_bytes(hashlib.sha256(f"social_v1:{seed}:{hid}".encode()).digest()[:8], "big"))
    return dict(preferred_speed_mps=rng.uniform(.46,.70), acceleration_mps2=rng.uniform(.60,.80),
                deceleration_mps2=rng.uniform(1.2,1.6), reaction_distance_m=rng.uniform(1.6,2.4),
                personal_space_m=rng.uniform(.18,.36), turn_rate_radps=rng.uniform(1.05,1.55),
                goal_persistence_s=rng.uniform(.8,1.4), yielding_tendency=rng.random())


def cap(v, limit):
    n=math.hypot(*v)
    return tuple(x*min(1.,limit/n) for x in v) if n>1e-12 else (0.,0.)


def encounter(position, velocity, other, other_velocity, radius, reaction):
    delta=(other[0]-position[0],other[1]-position[1])
    relative=(velocity[0]-other_velocity[0],velocity[1]-other_velocity[1])
    closing=sum(a*b for a,b in zip(delta,relative));v2=sum(x*x for x in relative)
    t=max(0.,min(2.5,closing/v2)) if v2>1e-10 else 0.
    distance=math.hypot(*delta);miss=math.hypot(*(delta[i]-t*relative[i] for i in (0,1)))
    return distance<reaction and closing>0. and miss<radius, t, distance


class SocialDiversity(GroupGoal):
    profile="social_diversity_v1"
    def __init__(self, kernel, seed):
        super().__init__(kernel)
        self.traits={int(h.id):traits(seed,int(h.id)) for h in people(kernel)}
        self.states={hid:dict(state="walking",partner=None,side=1,until=0.,clear_since=None,
                              wait_end=0.,cooldown=0.) for hid in self.traits}
        self.events=[];self.current=[];self.group_detail=[];self.now=0.
        kernel.motion.behavior_controller=self

    def transition(self,hid,state,partner=None):
        old=self.states[hid]
        if old["state"]!=state or old["partner"]!=partner:
            self.events.append(dict(event="behavior_transition",time_active_s=self.now,id=hid,
                                    previous=old["state"],state=state,partner=partner,side=old["side"]))
            if old["state"] in ("yielding","waiting"):
                self.events.append(dict(event=old["state"]+"_end",time_active_s=self.now,id=hid,partner=old["partner"]))
            if state in ("yielding","waiting"):
                self.events.append(dict(event=state+"_start",time_active_s=self.now,id=hid,partner=partner))
            old.update(state=state,partner=partner,until=self.now+self.traits[hid]["goal_persistence_s"],clear_since=None)

    def preferences(self,hs,starts,native,static_ids):
        motion=self.kernel.motion;e=self.kernel.env;dt=e.TIMESTEP
        self.now=max(0.,(motion.tick-1)*dt);self.events=[];self.current=[];self.group_detail=[]
        positions={int(h.id):p for h,p in zip(hs,starts)}
        group_for={int(h.id):g for g in e.moving_interactions for h in g.humans}
        for g in e.moving_interactions:
            ps=[positions[int(h.id)] for h in g.humans];center=tuple(sum(p[i] for p in ps)/len(ps) for i in (0,1))
            spread=max(math.dist(p,center) for p in ps)
            self.group_detail.append(dict(members=[int(h.id) for h in g.humans],center=center,goal=[float(g.goal_x),float(g.goal_y)],
                                          envelope_m=spread+e.HUMAN_DIAMETER/2,spread_m=spread,
                                          preferred_velocity=list(cap((g.goal_x-center[0],g.goal_y-center[1]),sum(self.traits[int(h.id)]["preferred_speed_mps"] for h in g.humans)/len(ps)))))
        result=[]
        for h,pos,proposal in zip(hs,starts,native):
            hid=int(h.id);tr=self.traits[hid];s=self.states[hid];g=group_for.get(hid)
            if hid in static_ids:
                self.transition(hid,"conversation");result.append((0.,0.));self.current.append(dict(id=hid,state=s["state"],partner=None,preferred_velocity=[0.,0.]));continue
            target=(float(g.goal_x),float(g.goal_y)) if g is not None else (float(h.goal_x),float(h.goal_y))
            center=(sum(positions[int(z.id)][0] for z in g.humans)/len(g.humans),sum(positions[int(z.id)][1] for z in g.humans)/len(g.humans)) if g else pos
            direction=cap((target[0]-center[0],target[1]-center[1]),1.)
            spread=max(math.dist(positions[int(z.id)],center) for z in g.humans) if g else 0.
            # Flexible forward gate: laggards can catch up without a rigid layout.
            pace=tr["preferred_speed_mps"]*max(.35,min(1.,(1.8-spread)/.7))
            base=(direction[0]*pace,direction[1]*pace)
            toward=(center[0]-pos[0],center[1]-pos[1]);distance=math.hypot(*toward)
            cohesion=cap(tuple(v*.65*max(0.,distance-.9)/distance for v in toward),.24) if distance>0 else (0.,0.)
            desired=tuple(base[i]+cohesion[i] for i in (0,1))
            partners=[]
            for z in hs:
                zid=int(z.id)
                if zid==hid or (g is not None and z in g.humans):continue
                zv=motion.velocities.get(zid,(0.,0.));hit,ttc,d=encounter(pos,desired,positions[zid],zv,e.HUMAN_DIAMETER+tr["personal_space_m"],tr["reaction_distance_m"])
                if hit:partners.append((ttc,d,zid,zv,positions[zid]))
            hit,ttc,d=encounter(pos,desired,motion.robot_start,motion.robot_velocity,e.HUMAN_DIAMETER/2+e.robot.radius+tr["personal_space_m"],tr["reaction_distance_m"])
            if hit:partners.append((ttc,d,"robot",motion.robot_velocity,motion.robot_start))
            partners.sort(key=lambda z:(z[0],z[1],str(z[2])))
            chosen=next((z for z in partners if z[2]==s["partner"]),None) or (partners[0] if partners else None)
            if chosen is not None:
                _,d,partner,pv,pp=chosen;s["clear_since"]=None
                if s["partner"] is None or (partner!=s["partner"] and self.now>=s["until"]):
                    forward=sum(pv[i]*direction[i] for i in (0,1));lateral=abs(pv[0]*direction[1]-pv[1]*direction[0])
                    crossing=lateral>.12;can_yield=math.hypot(*pv)>.1 and crossing and tr["yielding_tendency"]>.45 and self.now>=s["cooldown"] and (partner=="robot" or hid>partner)
                    state="yielding" if can_yield else "overtaking" if forward>.05 and forward<pace-.05 and lateral<.12 else "passing"
                    self.transition(hid,state,partner)
                    self.events.append(dict(event="passing_side_selected",time_active_s=self.now,id=hid,partner=partner,side=1))
                    if can_yield:s["wait_end"]=self.now+.8;s["cooldown"]=self.now+2.8
                if s["state"] in ("yielding","waiting") and self.now>=s["wait_end"]:self.transition(hid,"passing",s["partner"])
                if s["state"]=="yielding" and math.hypot(*motion.velocities.get(hid,(0.,0.)))<.08:self.transition(hid,"waiting",s["partner"])
                if s["state"] in ("yielding","waiting"):desired=tuple(v*.12 for v in desired)
                else:
                    # Persistent right-side preference; ORCA/projection may override it for safety.
                    side=(direction[1],-direction[0]);desired=tuple(.8*desired[i]+.22*side[i] for i in (0,1))
            elif s["partner"] is not None:
                if s["clear_since"] is None:s["clear_since"]=self.now
                if self.now>=s["until"] and self.now-s["clear_since"]>=.6:self.transition(hid,"resuming")
            elif s["state"]=="resuming" and self.now<s["until"]:pass
            else:self.transition(hid,"regrouping" if distance>1.15 or (s["state"]=="regrouping" and distance>.95) else "walking")
            desired=cap(desired,tr["preferred_speed_mps"]);result.append(desired)
            self.current.append(dict(id=hid,state=s["state"],partner=s["partner"],passing_side=s["side"],goal=target,
                                     member_centroid_distance_m=distance,preferred_velocity=desired,traits=tr))
        return result

    def snapshot(self):
        return copy.deepcopy(dict(profile=self.profile,active_time_s=self.now,humans=self.current,groups=self.group_detail,events=self.events,traits=self.traits))
