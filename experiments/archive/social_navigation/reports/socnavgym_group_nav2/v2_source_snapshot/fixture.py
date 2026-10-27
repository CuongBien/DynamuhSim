"""Explicit initial native group objects; native subsequent lifecycle remains enabled."""
import math,sys,json
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;LIFE=BENCH/'socnavgym_warehouse_lifecycle'
sys.path.insert(0,str(LIFE))
from kernel import Kernel
from motion import people
sys.path.insert(0,str(BENCH/'socnavgym_response_fix'))
from responsive_motion import ResponsiveMotion
from socnavgym.envs.utils.human_human import Human_Human_Interaction
from socnavgym.envs.utils.plant import Plant

def make_kernel(scenario,seed):
    plan=json.loads((HERE/'protocol.json').read_text());layout=json.loads((BENCH.parent/'warehouse/layout.json').read_text());k=Kernel(LIFE/'config.yaml',layout,seed=seed);e=k.env
    motion=k.motion;motion.__class__=ResponsiveMotion;e.handle_collision_and_update=motion.integrate
    hs=people(e);assert [int(h.id) for h in hs]==list(range(1,7));e.dynamic_humans=[];e.static_humans=[];e.moving_interactions=[];e.static_interactions=[];e.goals={}
    for ids,center in zip(plan['initial_members'],plan['initial_group_centers']):
        x,y=center[0]-k.origin[0],center[1]-k.origin[1]
        g=Human_Human_Interaction(x,y,'moving' if scenario=='walking' else 'stationary',3,.72,.72,.8,e.INTERACTION_GOAL_RADIUS,e.INTERACTION_NOISE_VARIANCE)
        g.humans=[next(h for h in hs if int(h.id)==hid) for hid in ids]
        for i,h in enumerate(g.humans):
            angle=i*2*math.pi/3;h.x=h.initial_x=x+.72*math.cos(angle);h.y=h.initial_y=y+.72*math.sin(angle);h.orientation=h.initial_orientation=0. if scenario=='walking' else math.atan2(y-h.y,x-h.x);h.speed=0.;h.goal_radius=e.INTERACTION_GOAL_RADIUS;h.type='dynamic';h.position_history.clear()
        g.x=x;g.y=y  # actual centroid of the explicit equilateral initial positions
        if scenario=='walking':
            gx,gy=plan['walking_initial_goal'];g.set_goal(gx-k.origin[0],gy-k.origin[1]);e.moving_interactions.append(g)
            for h in g.humans:e.goals[h.id]=Plant(None,g.goal_x,g.goal_y,e.INTERACTION_GOAL_RADIUS)
        else:e.static_interactions.append(g)
    e.objects=e.walls+[e.robot]+hs;k.group_refs=[];k.initial=k.humans();k.groups()
    return k
