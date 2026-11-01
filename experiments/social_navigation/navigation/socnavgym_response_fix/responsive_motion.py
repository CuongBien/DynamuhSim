"""Warehouse variant: exact AABBs and measured-velocity robot prediction."""
import math,sys
from pathlib import Path
import rvo2
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'socnavgym_lifecycle_fix'))
from continuous_groups import ContinuousGroups,people,angle_delta
from projection import project
from motion_contract import preferred_velocity,velocity_change_limits,final_heading

class ResponsiveMotion(ContinuousGroups):
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
        contract=getattr(self,'motion_limits',None)
        starts=[self.start[int(h.id)][:2] for h in hs];preferred=[];target_yaw=[];native_proposals=[]
        previous=[self.velocities.get(int(h.id),(0.,0.)) for h in hs]
        for h,(x,y) in zip(hs,starts):
            hid=int(h.id);vx,vy=(h.x-x)/dt,(h.y-y)/dt
            speed=math.hypot(vx,vy)
            if speed>e.MAX_ADVANCE_HUMAN:vx*=e.MAX_ADVANCE_HUMAN/speed;vy*=e.MAX_ADVANCE_HUMAN/speed
            if hid in static_ids:vx=vy=0.
            native_proposals.append((vx,vy))
            preferred.append((vx,vy) if contract is None else preferred_velocity((vx,vy),self.start[hid][2],e.MAX_ROTATION_HUMAN,dt));target_yaw.append(facing.get(hid,float(h.orientation)))
        behavior=getattr(self,'behavior_controller',None)
        if behavior is not None:
            social=behavior.preferences(hs,starts,native_proposals,static_ids)
            preferred=[preferred_velocity(v,self.start[int(h.id)][2],behavior.traits[int(h.id)]['turn_rate_radps'],dt) for h,v in zip(hs,social)]
        sim=rvo2.PyRVOSimulator(dt,max(e.orca_neighborDist,10.),len(hs)+1,e.orca_timeHorizon,e.orca_timeHorizonObst,e.HUMAN_DIAMETER/2+.005,e.MAX_ADVANCE_HUMAN)
        for wall in e.walls:sim.addObstacle(e.get_obstacle_corners(wall))
        for h,pos,pref in zip(hs,starts,preferred):
            i=sim.addAgent(pos);hid=int(h.id);sim.setAgentPrefVelocity(i,pref)
            sim.setAgentVelocity(i,self.velocities.get(hid,(0.,0.)))
            if behavior is not None:
                sim.setAgentNeighborDist(i,behavior.traits[hid]['reaction_distance_m'])
                sim.setAgentMaxSpeed(i,behavior.traits[hid]['preferred_speed_mps'])
            if hid in static_ids:sim.setAgentMaxSpeed(i,0.)
        robot_safety_radius=e.robot.radius+(.12 if behavior is not None else 0.)
        rid=sim.addAgent(self.robot_start);sim.setAgentRadius(rid,robot_safety_radius+.005)
        sim.setAgentMaxSpeed(rid,max(e.MAX_ADVANCE_ROBOT,math.hypot(*self.robot_velocity)));sim.setAgentVelocity(rid,self.robot_velocity);sim.setAgentPrefVelocity(rid,self.robot_velocity)
        sim.processObstacles();sim.doStep()
        displacement=[(0.,0.) if int(h.id) in static_ids else tuple(v*dt for v in sim.getAgentVelocity(i)) for i,h in enumerate(hs)]
        orca_displacement=list(displacement)
        motion_options={} if contract is None else dict(previous_velocities=previous,acceleration_limits=velocity_change_limits(previous,[(dx/dt,dy/dt) for dx,dy in displacement],contract))
        if behavior is not None and contract is not None:
            motion_options['acceleration_limits']=[velocity_change_limits([old],[tuple(v/dt for v in delta)],{key:behavior.traits[int(h.id)][key] for key in contract})[0] for h,old,delta in zip(hs,previous,displacement)]
        displacement,projection=project(starts,displacement,[h.width/2 for h in hs],self.robot_start,(self.robot_velocity[0]*dt,self.robot_velocity[1]*dt),robot_safety_radius,self.boxes,dt,e.MAX_ADVANCE_HUMAN,{i for i,h in enumerate(hs) if int(h.id) in static_ids},**motion_options);fraction=1.;gaps=projection['predicted_clearance']
        motion_audit=[]
        max_yaw=0.;max_move=0.
        for h,(x,y),(dx,dy),yaw in zip(hs,starts,displacement,target_yaw):
            hid=int(h.id);old_yaw=self.start[hid][2]
            if contract is not None:yaw=final_heading(old_yaw,(dx/dt,dy/dt),yaw,e.MAX_ROTATION_HUMAN if behavior is None else behavior.traits[hid]['turn_rate_radps'],dt,hid in static_ids)
            if behavior is not None and hid not in static_ids and math.hypot(dx,dy)/dt<.01 and math.hypot(*social[len(motion_audit)])>.05:
                # A reversed goal must permit turning at rest; XY remains projected.
                desired_yaw=math.atan2(social[len(motion_audit)][1],social[len(motion_audit)][0])
                yaw=final_heading(old_yaw,(0.,0.),desired_yaw,behavior.traits[hid]['turn_rate_radps'],dt,True)
            turn=max(-e.MAX_ROTATION_HUMAN*dt,min(e.MAX_ROTATION_HUMAN*dt,angle_delta(yaw,old_yaw)))
            i=len(motion_audit);travel=math.atan2(dy,dx) if math.hypot(dx,dy)>1e-10 else None
            motion_audit.append(dict(id=hid,previous_executed_velocity=list(previous[i]),native_proposed_velocity=list(native_proposals[i]),
                                     heading_preferred_velocity=list(preferred[i]),post_orca_velocity=[v/dt for v in orca_displacement[i]],
                                     final_velocity=[dx/dt,dy/dt],final_yaw=yaw,
                                     heading_travel_error_rad=None if travel is None else abs(angle_delta(travel,yaw)),
                                     interval_start_heading_error_rad=None if travel is None else abs(angle_delta(travel,old_yaw))))
            h.x=x+fraction*dx;h.y=y+fraction*dy;h.orientation=math.atan2(math.sin(old_yaw+turn),math.cos(old_yaw+turn))
            h.vx=(h.x-x)/dt;h.vy=(h.y-y)/dt;h.speed=math.hypot(h.vx,h.vy)
            h.prev_x=x;h.prev_y=y;h.prev_orientation=old_yaw
            if h.type=='static':h.initial_x=h.x;h.initial_y=h.y;h.initial_orientation=h.orientation
            if h.position_history:
                stamp=h.position_history[-1][3];h.position_history[-1]=(h.x,h.y,h.orientation,stamp)
            self.velocities[hid]=(h.vx,h.vy);max_yaw=max(max_yaw,abs(turn));max_move=max(max_move,math.hypot(h.x-x,h.y-y))
        for g in e.moving_interactions+e.static_interactions:
            g.x=sum(h.x for h in g.humans)/len(g.humans);g.y=sum(h.y for h in g.humans)/len(g.humans)
        self.steps.append(dict(motion_contract=motion_audit,projection=projection,predicted_clearance=gaps,robot_velocity=list(self.robot_velocity),epoch=self.epoch,step=self.tick,safe_fraction=fraction,max_displacement_m=max_move,max_yaw_delta_rad=max_yaw,stationary_ids=sorted(static_ids),robot_projection_radius_m=robot_safety_radius))

def enable(runner):
    motion=runner.k.motion;motion.__class__=ResponsiveMotion
    motion.env.handle_collision_and_update=motion.integrate
    return runner
