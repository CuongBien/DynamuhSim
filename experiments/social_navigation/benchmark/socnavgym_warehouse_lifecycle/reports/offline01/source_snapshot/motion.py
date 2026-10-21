"""Warehouse variant: exact AABBs and measured-velocity robot prediction."""
import math,sys
from pathlib import Path
import rvo2
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'socnavgym_lifecycle_fix'))
from continuous_groups import ContinuousGroups,people,angle_delta
from geometry import admissible_fraction

class WarehouseMotion(ContinuousGroups):
    def __init__(self,env,boxes):
        self.boxes=boxes;self.robot_velocity=(0.,0.)
        super().__init__(env)
    def reset(self,*args,**kwargs):
        result=self.original['reset'](*args,**kwargs);self.epoch+=1;self.tick=0;self.velocities={};self.robot_velocity=(0.,0.)
        if self.env.tables or self.env.plants or self.env.laptops or self.env.chairs:raise ValueError('Use exact factory AABBs, not extra native furniture')
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
        sim.setAgentMaxSpeed(rid,max(e.MAX_ADVANCE_ROBOT,math.hypot(*self.robot_velocity)));sim.setAgentVelocity(rid,self.robot_velocity);sim.setAgentPrefVelocity(rid,self.robot_velocity)
        sim.processObstacles();sim.doStep()
        displacement=[(0.,0.) if int(h.id) in static_ids else tuple(v*dt for v in sim.getAgentVelocity(i)) for i,h in enumerate(hs)]
        fraction,gaps=admissible_fraction(starts,displacement,[h.width/2 for h in hs],self.robot_start,(self.robot_velocity[0]*dt,self.robot_velocity[1]*dt),e.robot.radius,self.boxes)
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
        self.steps.append(dict(predicted_clearance=gaps,robot_velocity=list(self.robot_velocity),epoch=self.epoch,step=self.tick,safe_fraction=fraction,max_displacement_m=max_move,max_yaw_delta_rad=max_yaw,stationary_ids=sorted(static_ids)))
