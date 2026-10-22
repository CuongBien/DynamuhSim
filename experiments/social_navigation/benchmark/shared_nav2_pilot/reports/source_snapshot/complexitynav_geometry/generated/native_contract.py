"""Opt-in ComplexityNav human-step contract. No geometry repair or ROS imports."""
import copy
import math
import random
from types import SimpleNamespace
import numpy as np
from crowd_nav.configs.icra_benchmark.config import BaseEnvConfig
from crowd_sim.envs.crowd_sim import CrowdSim
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA
from crowd_sim.envs.utils.action import ActionXY

DT = .25
ORIGIN = (-24., 0.)

def fixture():
    return [dict(id=i+1,policy='orca' if i<3 else 'socialforce',x=x,y=y,
                 gx=(-21. if x < -23. else -27.),gy=y)
            for i,(x,y) in enumerate([(-25.,-1.),(-23.,-1.),(-21.,-1.),
                                     (-25.,1.),(-23.,1.),(-21.,1.)])]

def make_native(humans, origin=ORIGIN, visible=True):
    cfg=SimpleNamespace(**{s:copy.deepcopy(getattr(BaseEnvConfig,s))
                         for s in ['env','reward','sim','humans','robot']})
    cfg.env.time_step=DT
    cfg.env.time_limit=3600
    # Exact inner faces of the real perimeter walls, not the local sampling box.
    cfg.env.dx_range=[-29.85-origin[0],29.85-origin[0]]
    cfg.env.dy_range=[-19.85-origin[1],19.85-origin[1]]
    cfg.sim.human_num=len(humans)
    cfg.robot.radius=.25
    cfg.robot.visible=visible
    env=CrowdSim();env.configure(cfg)
    robot=Robot(cfg,'robot');robot.time_step=DT;robot.set_policy(ORCA())
    env.set_robot(robot)
    scene={};goals={};ordered=[]
    for h in humans:
        k=h['policy'];scene.setdefault(k,[]);goals.setdefault(k,[])
        p=(h['x']-origin[0],h['y']-origin[1]);g=(h['gx']-origin[0],h['gy']-origin[1])
        scene[k].append((*p,*g,0.,0.,0.))
        goals[k].append([p,g]*1000)
    for k in scene:ordered.extend(h for h in humans if h['policy']==k)
    env.reset('test',scene,goals,test_case=42)
    return env,ordered

class ContractKernel:
    def __init__(self, humans=None, visible=True):
        self.fixture=copy.deepcopy(humans or fixture());self.visible=visible;self.epoch=-1
        self.reset()

    def reset(self):
        self.epoch+=1;self.step=0;self.time=0.
        random.seed(42);np.random.seed(42)
        self.env,self.ordered=make_native(self.fixture,visible=self.visible)
        self.identity=tuple(id(h) for h in self.env.humans)
        self.last_robot=None
        return self.state()

    def state(self):
        humans=[]
        for f,h in zip(self.ordered,self.env.humans):
            humans.append(dict(id=f['id'],native_id=str(h.id),policy=f['policy'],
                x=float(h.px+ORIGIN[0]),y=float(h.py+ORIGIN[1]),vx=float(h.vx),vy=float(h.vy),
                radius=float(h.radius),gx=float(h.gx+ORIGIN[0]),gy=float(h.gy+ORIGIN[1])))
        return dict(epoch=self.epoch,step=self.step,time=self.time,frame='world',humans=humans)

    def request(self, req):
        # Reject before mutation, so a bad clock/frame/epoch cannot advance native state.
        if req['frame']!='world':raise ValueError('Expected world frame')
        if req['epoch']!=self.epoch:raise ValueError('Stale reset epoch')
        if not math.isfinite(req['time']) or abs(req['time']-self.time)>1e-9:
            raise ValueError('Clock must equal current interval start')
        if not isinstance(req['advance'],bool):raise ValueError('advance must be boolean')
        robot=req['robot']
        if not all(math.isfinite(robot[k]) for k in ['x','y','vx','vy','yaw']):
            raise ValueError('Non-finite robot feedback')
        if not req['advance']:return self.state()
        r=self.env.robot
        r.px=robot['x']-ORIGIN[0];r.py=robot['y']-ORIGIN[1]
        r.vx=robot['vx'];r.vy=robot['vy'];r.theta=robot['yaw']
        r.gx=26.-ORIGIN[0];r.gy=0.-ORIGIN[1]
        _,reward,done,info=self.env.step(ActionXY(r.vx,r.vy))
        self.last_robot=copy.deepcopy(robot)
        self.step+=1;self.time=self.step*DT
        if tuple(id(h) for h in self.env.humans)!=self.identity:
            raise RuntimeError('Native population identity changed')
        row=self.state();row.update(interval_start=req['time'],robot_input=self.last_robot,
                                   native_done=bool(done),native_info=str(info),reward=float(reward))
        return row
