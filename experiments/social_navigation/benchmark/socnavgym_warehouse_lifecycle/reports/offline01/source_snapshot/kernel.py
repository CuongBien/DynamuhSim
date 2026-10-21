"""Native lifecycle with external local-aisle fixture and full factory geometry."""
import json,math,random
from pathlib import Path
import gymnasium as gym,numpy as np,socnavgym
from socnavgym.envs.utils.wall import Wall
from motion import WarehouseMotion,people
from geometry import clearance

class Kernel:
    origin=(-24.,0.)
    def __init__(self,config,layout,seed=42):
        random.seed(seed);np.random.seed(seed);self.wrapper=gym.make('SocNavGym-v2',config=str(config));self.wrapper.action_space.seed(seed);self.env=self.wrapper.unwrapped
        self.boxes=[dict(b,x=b['x']-self.origin[0],y=b['y']-self.origin[1]) for b in layout['obstacles']]
        e=self.env
        def walls():
            for i,b in enumerate(self.boxes,100):
                wall=Wall(id=i,x=b['x'],y=b['y'],orientation=0. if b['sx']>=b['sy'] else math.pi/2,length=max(b['sx'],b['sy']),thickness=min(b['sx'],b['sy']))
                e.walls.append(wall);e.objects.append(wall)
        e._add_walls=walls
        self.motion=WarehouseMotion(e,self.boxes);self.wrapper.reset(seed=seed)
        e.robot.x=-3.;e.robot.y=0.;e.robot.orientation=0.;e.robot.radius=e.ROBOT_RADIUS=.25;e.robot.goal_x=50.;e.robot.goal_y=0.
        hs=people(e);assert len(hs)==6
        for h,(x,y) in zip(hs,[(-1.,-1.),(-1.,1.),(1.,-1.),(1.,1.),(3.,-1.),(3.,1.)]):
            h.x=h.initial_x=x;h.y=h.initial_y=y;h.orientation=h.initial_orientation=0.;h.speed=0.;h.position_history.clear()
        e.objects=e.walls+[e.robot]+hs
        for h in hs:
            goal=e.sample_goal(e.HUMAN_GOAL_RADIUS,e.MAP_X/2-e.MARGIN,e.MAP_Y/2-e.MARGIN)
            if goal is None:raise RuntimeError('Native local-aisle goal sampling failed')
            e.goals[h.id]=goal;h.set_goal(goal.x,goal.y)
        starts=[(h.x,h.y) for h in hs]
        assert min(clearance(starts,[(0.,0.)]*6,[.36]*6,(-3.,0.),(0.,0.),.25,self.boxes).values())>.002
        self.group_refs=[];self.steps=0;self.initial=self.humans()
    def humans(self):
        return [dict(id=int(h.id),x=float(h.x)+self.origin[0],y=float(h.y)+self.origin[1],yaw=float(h.orientation),speed=float(h.speed),radius=h.width/2,vx=self.motion.velocities.get(int(h.id),(0.,0.))[0],vy=self.motion.velocities.get(int(h.id),(0.,0.))[1]) for h in people(self.env)]
    def groups(self):
        result=[]
        for kind,gs in [('moving',self.env.moving_interactions),('standing',self.env.static_interactions)]:
            for g in gs:
                if not any(g is old for old in self.group_refs):self.group_refs.append(g)
                result.append(dict(id=next(i+1 for i,old in enumerate(self.group_refs) if g is old),type=kind,members=sorted(int(h.id) for h in g.humans),center=[float(g.x)+self.origin[0],float(g.y)+self.origin[1]]))
        return result
    def advance(self,robot):
        e=self.env;e.robot.x=robot['x']-self.origin[0];e.robot.y=robot['y']-self.origin[1];e.robot.orientation=robot['yaw']
        self.motion.robot_velocity=(robot['vx'],robot['vy'])
        _,_,terminated,truncated,_=self.wrapper.step(np.zeros(3,dtype=np.float32));self.steps+=1
        if terminated or truncated:raise RuntimeError('Native episode ended; session restart required')
    def close(self):self.wrapper.close()
