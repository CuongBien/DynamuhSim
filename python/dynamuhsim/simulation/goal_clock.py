"""Opt-in, instance-local human goal expiry on Gazebo time.

Native geometric arrival, set/set_goal side effects and RNG remain intact.
Only the elapsed-time predicate is replaced. No vendor/global clock mutation.
Initial goals are born at the accepted navigation goal's start gate; later goals
are born at their native assignment step. Human object replacement is terminal
rather than silently falling back to a host-time predicate.
"""
import json
import math
from pathlib import Path
from types import MethodType

PROFILE = dict(schema="dynamuhsim_goal_clock/1", profile="sim_v1", timeout_sim_s=50., start="navigation_start_gate")


def validate_profile(value):
    if value != PROFILE:
        raise ValueError("Unsupported human goal clock")
    return dict(PROFILE)


def people(kernel):
    e=kernel.env
    if e.h_l_interactions:
        raise ValueError("Human/laptop lifecycle is outside the clock fixture scope")
    result=list(e.static_humans)+list(e.dynamic_humans)
    for g in e.moving_interactions+e.static_interactions:
        result.extend(g.humans)
    if len({int(h.id) for h in result}) != len(result):
        raise ValueError("Duplicate active human IDs")
    return sorted(result,key=lambda h:int(h.id))


class GoalClock:
    def __init__(self,kernel):
        self.kernel=kernel
        self.gate=self.now=self.last_end=None
        self.events=[]
        self.entries={}
        for h in people(kernel):
            hid=int(h.id)
            original_goal,original_set=h.set_goal,h.set
            self.entries[hid]=dict(human=h,assigned=None,target=[h.goal_x,h.goal_y])
            def set_goal(human,*args,_original=original_goal,**kwargs):
                result=_original(*args,**kwargs)
                self.assigned(human)
                return result
            def set_state(human,*args,_original=original_set,**kwargs):
                result=_original(*args,**kwargs)
                self.assigned(human)
                return result
            def reached(human,offset=None):
                return self.reached(human,offset)
            h.set_goal=MethodType(set_goal,h)
            h.set=MethodType(set_state,h)
            h.has_reached_goal=MethodType(reached,h)
        if not self.entries:
            raise ValueError("Goal clock requires active human objects")

    def entry(self,human):
        entry=self.entries.get(int(human.id))
        if entry is None or entry["human"] is not human:
            raise RuntimeError("Native human object replaced; restart clock fixture")
        return entry

    def check_population(self):
        humans=people(self.kernel)
        if {int(h.id) for h in humans} != set(self.entries):
            raise RuntimeError("Native human population changed; restart clock fixture")
        for h in humans:
            entry=self.entry(h)
            if [h.goal_x,h.goal_y] != entry["target"]:
                raise RuntimeError("Human goal changed outside set/set_goal")

    def assigned(self,human):
        entry=self.entry(human)
        old=entry["target"]
        entry["assigned"]=self.now
        entry["target"]=[human.goal_x,human.goal_y]
        self.events.append(dict(event="assigned",id=int(human.id),time_sim_s=self.now,previous=old,goal=entry["target"]))

    def age(self,entry):
        return 0. if self.now is None or entry["assigned"] is None else self.now-entry["assigned"]

    def reached(self,human,offset=None):
        entry=self.entry(human)
        if [human.goal_x,human.goal_y] != entry["target"]:
            raise RuntimeError("Untracked human goal mutation")
        age=self.age(entry)
        static=human.type=="static"
        if static:
            distance=threshold=None
            geometric=expired=False
        else:
            distance=math.sqrt((human.x-human.goal_x)**2+(human.y-human.goal_y)**2)
            threshold=(human.width/2 if offset is None else offset)+human.goal_radius
            geometric=distance<threshold
            expired=age>PROFILE["timeout_sim_s"]
        self.events.append(dict(event="checked",id=int(human.id),time_sim_s=self.now,goal=entry["target"],age_sim_s=age,static=static,distance_m=distance,threshold_m=threshold,geometric=geometric,expired=expired,reached=geometric or expired))
        return geometric or expired

    def begin_step(self,time_sim_s,gate_sim_s):
        if not all(math.isfinite(v) for v in [time_sim_s,gate_sim_s]):
            raise ValueError("Nonfinite goal clock time")
        self.events=[]
        if self.gate is None:
            if time_sim_s<gate_sim_s-1e-6:
                raise ValueError("Active worker precedes navigation gate")
            self.gate=gate_sim_s
            for entry in self.entries.values():entry["assigned"]=gate_sim_s
            self.events.append(dict(event="activated",time_sim_s=gate_sim_s))
        elif abs(gate_sim_s-self.gate)>1e-8:
            raise ValueError("Navigation gate changed during goal clock session")
        if self.last_end is not None and abs(time_sim_s-self.last_end)>1e-6:
            raise ValueError("Noncontiguous goal clock step")
        self.now=time_sim_s
        self.check_population()

    def end_step(self,state_time_sim_s):
        if self.now is None or not math.isfinite(state_time_sim_s) or state_time_sim_s<=self.now:
            raise ValueError("Invalid goal clock step end")
        self.now=self.last_end=state_time_sim_s
        self.check_population()

    def snapshot(self):
        return dict(profile="sim_v1",domain="gazebo_sim",gate_start_sim_s=self.gate,evaluation_time_sim_s=self.now,
                    elapsed_active_sim_s=0. if self.gate is None else self.now-self.gate,
                    humans={str(hid):dict(goal=entry["target"],assigned_sim_s=entry["assigned"],age_sim_s=self.age(entry)) for hid,entry in self.entries.items()},events=list(self.events))


def load_goal_clock(kernel,runtime):
    path=Path(runtime)/"human_goal_clock.json"
    if not path.exists():return None
    validate_profile(json.loads(path.read_text()))
    return GoalClock(kernel)


def verify_trace(rows,gate_sim_s,ready):
    """Reconstruct every goal age/expiry from gate+assignments, independently of host time."""
    targets={hid:value["goal"] for hid,value in ready["humans"].items()}
    assigned={hid:gate_sim_s for hid in targets}
    activated=False
    last_end=None
    evaluations=assignments=expiries=0
    for row in rows:
        snap=row["goal_clock"]
        if snap["profile"]!="sim_v1" or snap["domain"]!="gazebo_sim" or set(snap["humans"])!=set(targets):return False,{}
        if not row["active_phase"]:
            if activated or snap["gate_start_sim_s"] is not None or snap["evaluation_time_sim_s"] is not None or snap["elapsed_active_sim_s"]!=0 or any(h["age_sim_s"]!=0 or h["assigned_sim_s"] is not None for h in snap["humans"].values()):return False,{}
            continue
        if abs(snap["gate_start_sim_s"]-gate_sim_s)>1e-8 or abs(snap["evaluation_time_sim_s"]-row["state_time"])>1e-8 or abs(snap["elapsed_active_sim_s"]-(row["state_time"]-gate_sim_s))>1e-8:return False,{}
        if last_end is not None and abs(row["time"]-last_end)>1e-6:return False,{}
        for event in snap["events"]:
            if event["event"]=="activated":
                if activated or abs(event["time_sim_s"]-gate_sim_s)>1e-8:return False,{}
                activated=True
                continue
            if not activated or abs(event["time_sim_s"]-row["time"])>1e-8:return False,{}
            hid=str(event["id"])
            if hid not in targets:return False,{}
            if event["event"]=="assigned":
                if event["previous"]!=targets[hid]:return False,{}
                targets[hid]=event["goal"];assigned[hid]=event["time_sim_s"];assignments+=1
            elif event["event"]=="checked":
                age=event["time_sim_s"]-assigned[hid]
                if abs(age-event["age_sim_s"])>1e-8 or event["goal"]!=targets[hid]:return False,{}
                geometric=False if event["static"] else event["distance_m"]<event["threshold_m"]
                expired=False if event["static"] else age>PROFILE["timeout_sim_s"]
                if geometric!=event["geometric"] or expired!=event["expired"] or (geometric or expired)!=event["reached"]:return False,{}
                evaluations+=1;expiries+=expired
            else:return False,{}
        if not activated:return False,{}
        for hid,value in snap["humans"].items():
            if value["goal"]!=targets[hid] or abs(value["assigned_sim_s"]-assigned[hid])>1e-8 or abs(value["age_sim_s"]-(row["state_time"]-assigned[hid]))>1e-8:return False,{}
        last_end=row["state_time"]
    return activated and evaluations>0,dict(evaluations=evaluations,assignments=assignments,expired_evaluations=expiries,last_state_time_sim_s=last_end)
