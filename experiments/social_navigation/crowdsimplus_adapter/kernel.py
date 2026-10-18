"""Native CrowdSimPlus human kernel, executed only in its pinned Python 3.8 image.
Robot state is injected, never stepped here. All human actions are computed on
one snapshot, constrained by native static collision code, then applied together.
"""
import configparser
import hashlib
import math
from pathlib import Path
import subprocess
from crowd_sim_plus.envs.crowd_sim_plus import CrowdSimPlus
from crowd_sim_plus.envs.utils.human_plus import Human
from crowd_sim_plus.envs.utils.robot_plus import Robot
from crowd_sim_plus.envs.utils.action import ActionXY
ROOT=Path('/opt/CrowdSimPlus')

class CrowdKernel:
    def __init__(self,scene,layout,visible=True,human_policy='orca_plus'):
        if human_policy not in ('orca_plus','sfm'):raise ValueError('Unsupported human policy')
        self.scene=scene;self.steps=0;self.visible=visible;self.targets=[0,0,0];self.human_policy=human_policy
        cfg=configparser.RawConfigParser();cfg.read(str(ROOT/'sicnav/configs/env.config'))
        for section,values in dict(env=dict(time_step=.1,randomize_attributes='false',time_limit=1000),
                sim=dict(human_num=3,starts_moving=0),humans=dict(policy=human_policy,radius=.36,v_pref=.8),
                robot=dict(policy='orca_plus',radius=.25,visible=str(visible).lower())).items():
            for key,value in values.items():cfg.set(section,key,str(value))
        self.config=cfg;self.env=e=CrowdSimPlus();e.configure(cfg);e.sim_env='social_warehouse';e.mode='test'
        e.global_time=0.;e.global_time_step=0;e.static_obstacles=[]
        for b in layout['obstacles']:
            x0,x1=b['x']-b['sx']/2,b['x']+b['sx']/2;y0,y1=b['y']-b['sy']/2,b['y']+b['sy']/2
            corners=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
            e.static_obstacles.extend([[corners[i],corners[(i+1)%4]] for i in range(4)])
        self.robot_start=scene.get('robot_start',[-27.,0.,0.]);self.robot_goal=scene.get('robot_goal',[-14.,0.,0.])
        robot=Robot(cfg,'robot');robot.time_step=e.time_step;robot.set(*self.robot_start[:2],*self.robot_goal[:2],0.,0.,self.robot_start[2]);e.set_robot(robot)
        e.humans=[];self.yaws=[];self.arrivals=[0,0,0]
        route_map=scene.get('human_routes')
        if route_map is not None and set(route_map)!={str(h['id']) for h in scene['initial']}:
            raise ValueError('Per-human routes must cover all human IDs exactly')
        self.routes=[route_map[str(h['id'])] if route_map is not None else scene['group_route'] for h in scene['initial']]
        for route in self.routes:
            if len(route)!=2 or any(len(point)!=2 or not all(math.isfinite(v) for v in point) for point in route):
                raise ValueError('Patrol requires two finite XY endpoints per human')
        for index,state in enumerate(scene['initial']):
            h=Human(cfg,'humans',env=e);h.time_step=e.time_step
            # Human catches configure errors upstream; reject incomplete policies here.
            required=('radius','A','B','KI','A_static','B_static','A_bottleneck','B_bottleneck') if human_policy=='sfm' else ('radius','safety_space','time_horizon')
            for key in required:
                if not hasattr(h.policy,key):raise RuntimeError('Native policy configuration missing '+key)
            if h.policy.time_step!=e.time_step:raise RuntimeError('Human policy timestep mismatch')
            gx,gy=self.routes[index][0]
            h.set(state['x'],state['y'],gx,gy,0.,0.,state['yaw']);e.humans.append(h);self.yaws.append(state['yaw'])
        self.last_actions=[]

    def advance(self,robot):
        e=self.env;r=e.robot
        r.set(robot['x'],robot['y'],*self.robot_goal[:2],robot['vx'],robot['vy'],robot['yaw'])
        actions=[]
        if self.scene['condition']=='standing':
            # External stationary fixture, not a native conversation/group model.
            self.last_actions=[dict(vx=0.,vy=0.) for h in e.humans]
        else:
            for h in e.humans:
                ob=[j.get_observable_state() for j in e.humans if j is not h]
                if r.visible:ob.append(r.get_observable_state(original=True))
                action=h.act(ob,e.static_obstacles)
                actions.append(e.constrain_agent_action_exact(h,action))
            for i,(h,action) in enumerate(zip(e.humans,actions)):
                h.step(action)
                if math.hypot(h.vx,h.vy)>1e-5:self.yaws[i]=math.atan2(h.vy,h.vx)
            # Native reached_destination criterion; external patrol instead of episode end.
            for i,h in enumerate(e.humans):
                if h.reached_destination():
                    self.arrivals[i]+=1
                    self.targets[i]=1-self.targets[i];h.final_gx,h.final_gy=self.routes[i][self.targets[i]];h.set_g_xy(h.px,h.py)
            self.last_actions=[dict(vx=float(a.vx),vy=float(a.vy)) for a in actions]
        e.global_time+=e.time_step;e.global_time_step+=1;self.steps+=1

    def humans(self):
        return [dict(id=i+1,x=float(h.px),y=float(h.py),yaw=float(self.yaws[i]),speed=float(math.hypot(h.vx,h.vy)),
            vx=float(h.vx),vy=float(h.vy),radius=float(h.radius),goal=[float(h.gx),float(h.gy)],completed_legs=self.arrivals[i],route_target_index=self.targets[i]) for i,h in enumerate(self.env.humans)]

    def metadata(self):
        e=self.env;p=e.humans[0].policy
        files=['crowd_sim_plus/envs/crowd_sim_plus.py','crowd_sim_plus/envs/policy/orca_plus.py','crowd_sim_plus/envs/policy/social_force.py','crowd_sim_plus/envs/utils/human_plus.py','crowd_sim_plus/envs/utils/agent_plus.py']
        return dict(framework='CrowdSimPlus',source_commit=subprocess.check_output(['git','-c','safe.directory='+str(ROOT),'-C',str(ROOT),'rev-parse','HEAD'],universal_newlines=True).strip(),
            source_sha256={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},
            time_step=e.time_step,original_demo_time_step=.25,robot_visible=e.robot.visible,
            human_policy=self.human_policy,human_radius=.36,human_v_pref=.8,robot_radius=.25,
            orca_safety_space=p.safety_space if self.human_policy=='orca_plus' else None,
            orca_added_radius=.01+p.safety_space if self.human_policy=='orca_plus' else None,
            neighbor_dist=getattr(p,'neighbor_dist',None),time_horizon=getattr(p,'time_horizon',None),time_horizon_obst=getattr(p,'time_horizon_obst',None),
            sfm_parameters={key:getattr(p,key) for key in ['radius','A','B','KI','A_static','B_static','A_bottleneck','B_bottleneck','is_bottleneck']} if self.human_policy=='sfm' else None,
            static_boxes=len(e.static_obstacles)//4,static_line_segments=len(e.static_obstacles),group_model=False,conversation_model=False,
            movement_methods=['Human.act','CrowdSimPlus.constrain_agent_action_exact','Human.step'],
            goal_layout=self.scene.get('goal_layout','shared'),human_routes=self.routes,
            arrival_rule='upstream Human.reached_destination; external patrol switches endpoints only on arrival',
            external_scenario=(self.scene['condition']+' initial poses; ' if self.scene['condition'] in ('crossing','bottleneck') else 'shared initial poses; ')+('separate per-human endpoints' if self.scene.get('goal_layout')=='individual' else 'shared endpoints')+'; standing fixture held externally',
            robot_controller='ROS Nav2 only; native robot act/step and full env reward/episode loop not called',
            visual_heading='last nonzero actual velocity; initial scene yaw while stationary')
