"""Opt-in standalone SocNavGym adapter: continuous lifecycle and disc-safe motion.
Changes behavior explicitly. Native vendor, fixed-group Gazebo kernel and robot code
are untouched. Supports the tested square room and a stationary robot only.
"""
import math
import numpy as np
import rvo2


def first_contact(p, displacement, radius):
    """Earliest linear swept-disc contact in [0,1], not just endpoint overlap."""
    px,py=p;dx,dy=displacement;c=px*px+py*py-radius*radius
    if c < -1e-8:raise ValueError('Already-overlapping initial discs')
    aa=dx*dx+dy*dy;bb=2*(px*dx+py*dy)
    if aa<1e-20 or bb>=0:return 1.
    discriminant=bb*bb-4*aa*max(c,0.)
    if discriminant<0:return 1.
    t=(-bb-math.sqrt(discriminant))/(2*aa)
    return min(1.,max(0.,t)) if t<=1. else 1.


def safe_fraction(positions,displacements,radii,robot,robot_radius,limits,margin=.002):
    fraction=1.;n=len(positions)
    for i,(x,y) in enumerate(positions):
        dx,dy=displacements[i]
        for axis,(coordinate,delta) in enumerate([(x,dx),(y,dy)]):
            limit=limits[axis]-radii[i]-margin
            if abs(coordinate)>limit+1e-8:raise ValueError('Initial disc intersects wall')
            if delta>0:fraction=min(fraction,max(0.,(limit-coordinate)/delta))
            elif delta<0:fraction=min(fraction,max(0.,(-limit-coordinate)/delta))
        fraction=min(fraction,first_contact((x-robot[0],y-robot[1]),(dx,dy),radii[i]+robot_radius+margin))
        for j in range(i):
            fraction=min(fraction,first_contact((x-positions[j][0],y-positions[j][1]),(dx-displacements[j][0],dy-displacements[j][1]),radii[i]+radii[j]+margin))
    return 1. if fraction>=1. else max(0.,fraction-1e-6)


def people(env):
    hs=list(env.static_humans)+list(env.dynamic_humans)
    for g in env.moving_interactions+env.static_interactions:hs.extend(g.humans)
    if env.h_l_interactions:raise ValueError('Human/laptop lifecycle is outside this adapter scope')
    return sorted(hs,key=lambda h:int(h.id))


def angle_delta(target,source):return math.atan2(math.sin(target-source),math.cos(target-source))


class ContinuousGroups:
    def __init__(self,env):
        self.env=env;self.events=[];self.steps=[];self.epoch=-1;self.tick=0;self.velocities={};self.start=None
        self.original={name:getattr(env,name) for name in ['reset','step','finish_human_crowd_formation','handle_collision_and_update']}
        env.reset=self.reset;env.step=self.step
        env.finish_human_crowd_formation=self.finish
        env.handle_collision_and_update=self.integrate

    def reset(self,*args,**kwargs):
        result=self.original['reset'](*args,**kwargs);self.epoch+=1;self.tick=0;self.velocities={}
        e=self.env
        if e.shape!='square' or e.tables or e.plants or e.laptops or e.chairs:
            raise ValueError('Only native square-room fixture without furniture is supported')
        return result

    def finish(self,*args,**kwargs):
        hs=people(self.env)
        before={int(h.id):(h.x,h.y,h.orientation,h.initial_x,h.initial_y,h.initial_orientation) for h in hs}
        # Preserve native selection, RNG draws, group type, membership and goals.
        result=self.original['finish_human_crowd_formation'](*args,**kwargs)
        snap=0.;yaw=0.
        for h in people(self.env):
            old=before[int(h.id)];snap=max(snap,math.hypot(h.x-old[0],h.y-old[1]));yaw=max(yaw,abs(angle_delta(h.orientation,old[2])))
            h.x,h.y,h.orientation,h.initial_x,h.initial_y,h.initial_orientation=old
        for g in self.env.moving_interactions+self.env.static_interactions:
            g.x=sum(h.x for h in g.humans)/len(g.humans);g.y=sum(h.y for h in g.humans)/len(g.humans)
        self.events.append(dict(epoch=self.epoch,step=self.tick,native_position_snap_removed_m=snap,native_yaw_snap_deferred_rad=yaw,approximate=kwargs.get('make_approx_crowd',False)))
        return result

    def step(self,action):
        hs=people(self.env);self.tick+=1
        self.start={int(h.id):(float(h.x),float(h.y),float(h.orientation)) for h in hs}
        self.robot_start=(float(self.env.robot.x),float(self.env.robot.y))
        result=self.original['step'](action)
        if math.hypot(self.env.robot.x-self.robot_start[0],self.env.robot.y-self.robot_start[1])>1e-9:
            raise ValueError('This prototype requires a stationary robot; moving robot not accepted')
        return result

    def integrate(self):
        e=self.env;dt=e.TIMESTEP;hs=people(e)
        if self.start is None:raise RuntimeError('Integrate must be called inside step')
        self.original['handle_collision_and_update']()
        static_ids={int(h.id) for h in e.static_humans}
        facing={}
        for g in e.static_interactions:
            for h in g.humans:
                static_ids.add(int(h.id));facing[int(h.id)]=math.atan2(g.y-self.start[int(h.id)][1],g.x-self.start[int(h.id)][0])
        starts=[self.start[int(h.id)][:2] for h in hs];preferred=[];target_yaw=[]
        for h,(x,y) in zip(hs,starts):
            hid=int(h.id);vx,vy=(h.x-x)/dt,(h.y-y)/dt
            speed=math.hypot(vx,vy)
            if speed>e.MAX_ADVANCE_HUMAN:vx*=e.MAX_ADVANCE_HUMAN/speed;vy*=e.MAX_ADVANCE_HUMAN/speed
            if hid in static_ids:vx=vy=0.
            preferred.append((vx,vy));target_yaw.append(facing.get(hid,float(h.orientation)))
        sim=rvo2.PyRVOSimulator(dt,max(e.orca_neighborDist,10.),len(hs)+1,e.orca_timeHorizon,e.orca_timeHorizonObst,e.HUMAN_DIAMETER/2+.005,e.MAX_ADVANCE_HUMAN)
        for wall in e.walls:sim.addObstacle(e.get_obstacle_corners(wall))
        for h,pos,pref in zip(hs,starts,preferred):
            i=sim.addAgent(pos);hid=int(h.id);sim.setAgentPrefVelocity(i,pref)
            sim.setAgentVelocity(i,self.velocities.get(hid,(0.,0.)))
            if hid in static_ids:sim.setAgentMaxSpeed(i,0.)
        rid=sim.addAgent(self.robot_start);sim.setAgentRadius(rid,e.robot.radius+.005)
        sim.setAgentMaxSpeed(rid,0.);sim.setAgentVelocity(rid,(0.,0.));sim.setAgentPrefVelocity(rid,(0.,0.))
        sim.processObstacles();sim.doStep()
        displacement=[(0.,0.) if int(h.id) in static_ids else tuple(v*dt for v in sim.getAgentVelocity(i)) for i,h in enumerate(hs)]
        fraction=safe_fraction(starts,displacement,[h.width/2 for h in hs],self.robot_start,e.robot.radius,(e.MAP_X/2-e.WALL_THICKNESS,e.MAP_Y/2-e.WALL_THICKNESS))
        max_yaw=0.;max_move=0.
        for h,(x,y),(dx,dy),yaw in zip(hs,starts,displacement,target_yaw):
            hid=int(h.id);old_yaw=self.start[hid][2];turn=max(-e.MAX_ROTATION_HUMAN*dt,min(e.MAX_ROTATION_HUMAN*dt,angle_delta(yaw,old_yaw)))
            h.x=x+fraction*dx;h.y=y+fraction*dy;h.orientation=math.atan2(math.sin(old_yaw+turn),math.cos(old_yaw+turn))
            h.vx=(h.x-x)/dt;h.vy=(h.y-y)/dt;h.speed=math.hypot(h.vx,h.vy)
            h.prev_x=x;h.prev_y=y;h.prev_orientation=old_yaw
            if h.type=='static':h.initial_x=h.x;h.initial_y=h.y;h.initial_orientation=h.orientation
            if h.position_history:
                stamp=h.position_history[-1][3];h.position_history[-1]=(h.x,h.y,h.orientation,stamp)
            self.velocities[hid]=(h.vx,h.vy);max_yaw=max(max_yaw,abs(turn));max_move=max(max_move,math.hypot(h.x-x,h.y-y))
        for g in e.moving_interactions+e.static_interactions:
            g.x=sum(h.x for h in g.humans)/len(g.humans);g.y=sum(h.y for h in g.humans)/len(g.humans)
        self.steps.append(dict(epoch=self.epoch,step=self.tick,safe_fraction=fraction,max_displacement_m=max_move,max_yaw_delta_rad=max_yaw,stationary_ids=sorted(static_ids)))
