"""Replay deterministic ROS callback traces against golden outputs from old runners."""
import contextlib,hashlib,importlib.util,io,json,math,sys,types,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
FIXTURE=ROOT/'tests/fixtures/mppi_monitor.json'


class FakeNode:
    def __init__(self,name):self.name=name;self.subscriptions=[]
    def set_parameters(self,values):self.parameters=values
    def create_subscription(self,kind,topic,callback,qos):self.subscriptions.append((topic,qos));return None
    def get_clock(self):return NS(now=lambda:NS(to_msg=lambda:NS(sec=10,nanosec=0)))


class FakeGoal:
    def __init__(self):self.pose=NS(header=NS(),pose=NS(position=NS(),orientation=NS()))


class FakeParameter:
    Type=NS(BOOL=1)
    def __init__(self,*args):self.values=args


class Scenario:
    def __init__(self,name):
        self.name=name;self.wall=0.;self.index=0;self.actions=[];self.last_sim=10.
        self.limit=3. if name=='stopped' else 4.
        self.frames=12 if name=='stopped' else 8
    def time(self):self.wall+=.1;return self.wall
    def sleep(self,seconds):self.wall+=seconds
    def ok(self):return self.index<self.frames
    def spin_until_future_complete(self,node,future,timeout_sec):self.actions.append(('spin_future',timeout_sec))
    def action(self,node,*args):
        scene=self
        class Client:
            def wait_for_server(self,timeout_sec):scene.actions.append(('wait_server',timeout_sec));return scene.name!='no_server'
            def send_goal_async(self,goal):
                scene.actions.append(('send_goal',goal.pose.header.frame_id,goal.pose.pose.position.x,goal.pose.pose.position.y))
                return NS(done=lambda:True,result=lambda:NS(accepted=scene.name!='rejected'))
        return Client()
    def spin_once(self,node,timeout_sec):
        if node.t_start is None:return
        self.index+=1;i=self.index;t=10+i*.25
        if self.name=='watchdog':t=10.;self.wall+=15.;self.frames=20
        self.last_sim=t
        node._clock_cb(NS(clock=NS(sec=int(t),nanosec=int(round((t-int(t))*1e9)))))
        x=-13.+i*.8;y=0.;yaw=0.;speed=.3;human=None
        if self.name in ['stopped','watchdog']:x=-13.;speed=0.;human=(-8.,0.)
        if self.name=='wall':y=.30;yaw=math.pi/4
        if self.name in ['near_miss','collision']:
            y=.31 if i>=3 else 0.;human=(x+.41,y) if i>=3 else (x+2.,y)
            if self.name=='collision' and i>=4:human=(x+.35,y)
        if self.name=='goal':x=-13.+i*.8
        if self.name in ['near_miss','wall'] and i==8:x=-7.1
        gt=dict(robot=dict(x=x,y=y,vx=speed,vy=0.),humans=[] if human is None else [dict(x=human[0],y=human[1],vx=0.,vy=0.)])
        node._gt_cb(NS(data=json.dumps(gt)))
        node._odom_cb(NS(pose=NS(pose=NS(orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))))
        stamp=NS(sec=0,nanosec=0) if i%2 else NS(sec=int(t),nanosec=int(round((t-int(t))*1e9)))
        node._cmd_cb(NS(header=NS(stamp=stamp),twist=NS(linear=NS(x=speed,y=.06 if i>=3 else 0.),angular=NS(z=0.))))
        data=[50]*(100*60)
        # Human core is lethal; both lane queries remain valid and non-lethal.
        if human:
            mx=int((human[0]+13.)/.1);my=int((human[1]+3.)/.1)
            if 0<=mx<100 and 0<=my<60:data[my*100+mx]=100
        node._costmap_cb(NS(info=NS(origin=NS(position=NS(x=0.,y=-3.)),resolution=.1,width=100,height=60),data=data))
        if hasattr(node,'_plan_cb'):node._plan_cb(NS(poses=[NS(pose=NS(position=NS(x=-10.,y=.22))),NS(pose=NS(position=NS(x=-7.,y=-.1)))]))


def environment(scene):
    ros=types.ModuleType('rclpy');ros.parameter=NS(Parameter=FakeParameter);ros.Parameter=FakeParameter
    ros.ok=scene.ok;ros.spin_once=scene.spin_once;ros.spin_until_future_complete=scene.spin_until_future_complete
    modules={'rclpy':ros,'rclpy.node':NS(Node=FakeNode),'rclpy.action':NS(ActionClient=scene.action),
             'rosgraph_msgs.msg':NS(Clock=NS),'nav_msgs.msg':NS(Odometry=NS,OccupancyGrid=NS,Path=NS),
             'geometry_msgs.msg':NS(TwistStamped=NS),'std_msgs.msg':NS(String=NS),'nav2_msgs.action':NS(NavigateToPose=NS(Goal=FakeGoal))}
    return ros,modules


def load_current(scene):
    ros,modules=environment(scene)
    name='experiments.gazebo_benchmark.core._monitor_test'
    with patch.dict(sys.modules,modules):
        spec=importlib.util.spec_from_file_location(name,ROOT/'experiments/gazebo_benchmark/core/mppi_monitor.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{name:module}):spec.loader.exec_module(module)
    module.time=scene
    return module


def run_monitor(module,scene,step):
    monitor=getattr(module,'Step'+str(step)+'InteractionMonitor')(max_duration_s=scene.limit)
    with contextlib.redirect_stdout(io.StringIO()):result=monitor.run_navigation()
    return dict(result=result,actions=scene.actions,subscriptions=monitor.subscriptions,steps=scene.index,sim_time=scene.last_sim,node_name=monitor.name)


def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


class MppiMonitorTests(unittest.TestCase):
    def test_all_metrics_stop_conditions_actions_and_subscriptions_match_old_runners(self):
        golden=json.loads(FIXTURE.read_text())
        for key,expected in golden['cases'].items():
            step,case=key.split(':');scene=Scenario(case);module=load_current(scene)
            actual=run_monitor(module,scene,int(step))
            with self.subTest(step=step,case=case):self.assertEqual(digest(actual),expected,json.dumps(actual,sort_keys=True))

    def test_profile_defaults_and_plan_subscription_preserved(self):
        for step,duration in [(1,25.),(2,25.),(3,40.)]:
            scene=Scenario('goal');module=load_current(scene);monitor=getattr(module,'Step'+str(step)+'InteractionMonitor')()
            self.assertEqual(monitor.max_duration_s,duration)
            self.assertEqual(any(t=='/plan' for t,q in monitor.subscriptions),step==3)

    def test_costmap_lookup_and_lane_unknown_cells(self):
        scene=Scenario('goal');module=load_current(scene);monitor=module.Step2InteractionMonitor()
        grid=NS(info=NS(origin=NS(position=NS(x=0.,y=-1.)),resolution=.1,width=4,height=4),data=[-1]*16)
        self.assertEqual(monitor.get_cost_at_point(grid,0.,-1.),-1)
        self.assertIsNone(monitor.get_cost_at_point(grid,5.,5.))
        self.assertEqual(monitor.evaluate_lanes_at_2m(grid,-13.,0.,-12.,0.),dict(cost_left_lane_at_human=None,cost_right_lane_at_human=None,min_cost_left_lane_profile=None,min_cost_right_lane_profile=None))

    def test_runners_import_shared_classes_and_resolve_their_own_checkout(self):
        import ast
        for step,name in [(1,'step1_vision'),(2,'step2'),(3,'step3')]:
            path=ROOT/'experiments/gazebo_benchmark/suites/mppi_ablation'/('benchmark_a3_3_'+name+'.py')
            text=path.read_text();tree=ast.parse(text)
            self.assertFalse(any(isinstance(n,ast.ClassDef) and n.name.endswith('InteractionMonitor') for n in tree.body))
            self.assertNotIn('/home/cuongbien/DynamuhSim',text)
            self.assertIn('from experiments.gazebo_benchmark.core.mppi_monitor import Step'+str(step)+'InteractionMonitor',text)


if __name__=='__main__':unittest.main()
