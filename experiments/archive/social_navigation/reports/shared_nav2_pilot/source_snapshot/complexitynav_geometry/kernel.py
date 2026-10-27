"""Explicit variant: native intent at roadmap goals + obstacle-aware ORCA projection."""
import math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent/'generated'))
from native_contract import ContractKernel,DT,ORIGIN
from swept import admissible_fraction
from roadmap import Roadmap
from crowd_sim.envs.policy.multi_policy import CentralizedMultiPolicy
from crowd_sim.envs.utils.state import FullState
from crowd_sim.envs.utils.action import ActionXY
import rvo2

class ObstaclePlanner:
    def __init__(self,layout,roadmap,ordered):
        self.layout=layout;self.roadmap=roadmap;self.ordered=ordered
        self.native=CentralizedMultiPolicy();self.log=[];self.max_speed=1.

    def predict(self,state,policies=None,orca_border=None,sfm_border=None):
        if len(state)!=len(policies)+1:raise ValueError('Visible measured robot required')
        hs=state[:-1];robot=state[-1];guided=[];waypoints=[];route_lengths=[]
        for h in hs:
            point=(h.px+ORIGIN[0],h.py+ORIGIN[1]);goal=(h.gx+ORIGIN[0],h.gy+ORIGIN[1])
            waypoint,length=self.roadmap.waypoint(point,goal)
            guided.append(FullState(h.px,h.py,h.vx,h.vy,h.radius,waypoint[0]-ORIGIN[0],waypoint[1]-ORIGIN[1],h.v_pref,h.theta))
            waypoints.append(waypoint);route_lengths.append(length)
        # Stock policy functions are unchanged, but they receive the route waypoint.
        intent,robot_actions=self.native.predict(guided+[robot],policies,orca_border,sfm_border)
        count=len(state);sim=rvo2.PyRVOSimulator(DT,10.,count,5.,5.,.32,self.max_speed)
        for b in self.layout['obstacles']:
            x0=b['x']-b['sx']/2-ORIGIN[0];x1=b['x']+b['sx']/2-ORIGIN[0]
            y0=b['y']-b['sy']/2-ORIGIN[1];y1=b['y']+b['sy']/2-ORIGIN[1]
            sim.addObstacle([(x0,y0),(x1,y0),(x1,y1),(x0,y1)])
        for h,a in zip(hs,intent):
            i=sim.addAgent((h.px,h.py));sim.setAgentRadius(i,h.radius+.02)
            sim.setAgentVelocity(i,(h.vx,h.vy));speed=math.hypot(a.vx,a.vy);factor=min(1.,self.max_speed/max(speed,1e-12))
            sim.setAgentPrefVelocity(i,(a.vx*factor,a.vy*factor))
        rid=sim.addAgent((robot.px,robot.py));sim.setAgentRadius(rid,robot.radius+.02)
        sim.setAgentMaxSpeed(rid,max(self.max_speed,math.hypot(robot.vx,robot.vy)))
        sim.setAgentVelocity(rid,(robot.vx,robot.vy));sim.setAgentPrefVelocity(rid,(robot.vx,robot.vy))
        sim.processObstacles();sim.doStep()
        starts=[(h.px+ORIGIN[0],h.py+ORIGIN[1]) for h in hs]
        deltas=[tuple(v*DT for v in sim.getAgentVelocity(i)) for i in range(len(hs))]
        fraction,gaps=admissible_fraction(starts,deltas,[h.radius for h in hs],(robot.px+ORIGIN[0],robot.py+ORIGIN[1]),(robot.vx*DT,robot.vy*DT),robot.radius,self.layout['obstacles'])
        actions=[ActionXY(d[0]*fraction/DT,d[1]*fraction/DT) for d in deltas]
        self.log.append(dict(safe_fraction=fraction,predicted_swept_gap_m=gaps,waypoints_world=waypoints,route_lengths_m=route_lengths,
            native_intent_xy=[tuple(a) for a in intent],projected_actions_xy=[tuple(a) for a in actions],robot_predicted_delta=[robot.vx*DT,robot.vy*DT]))
        return actions,robot_actions

class GeometryKernel(ContractKernel):
    def __init__(self,layout,humans=None):
        self.layout=layout;self.roadmap=Roadmap(layout,.3)
        super().__init__(humans,visible=True)
    def reset(self):
        state=super().reset()
        self.projector=ObstaclePlanner(self.layout,self.roadmap,self.ordered)
        self.env.centralized_planner=self.projector
        return state
    def state(self):
        result=super().state()
        for row,h in zip(result['humans'],self.env.humans):row['completed_legs']=int(h.current_goal)
        return result
    def request(self,req):
        result=super().request(req)
        if req['advance']:result['geometry']=self.projector.log[-1]
        return result
