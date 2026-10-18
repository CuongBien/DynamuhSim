#!/usr/bin/env python3
"""One isolated Nav2 pilot. Ground truth is measured from Gazebo for both backends."""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image

HERE=Path(__file__).resolve().parent
SOCIAL=HERE.parent
REPO=SOCIAL.parent.parent
sys.path.insert(0,str(SOCIAL/'gazebo_adapter'))
from run import make_world, element, stop
sys.path.insert(0,str(SOCIAL/'crowdsimplus_adapter'))
from crowd_host import prepare_scene as prepare_crowd_scene, convert_world as convert_crowd_world, docker_command, cleanup_container

p=argparse.ArgumentParser()
p.add_argument('--backend',choices=['empty','rvo2','socnavgym','crowdsimplus'],required=True)
p.add_argument('--condition',choices=['standing','counterflow','crossing'],default='standing')
p.add_argument('--seed',type=int,default=42)
p.add_argument('--gui',action='store_true')
p.add_argument('--report-dir',type=Path,help='Separate output directory; never reuse a completed trial directory')
p.add_argument('--diagnostics',action='store_true',help='Capture raw sensor, TF, costmap, commands and world poses')
p.add_argument('--geometry',choices=['native','circle_contact'],default='native')
p.add_argument('--robot-avoidance',action='store_true',help='Experimental robot-aware group ORCA; requires circle_contact')
p.add_argument('--invisible-robot',action='store_true',help='CrowdSimPlus observation ablation only')
p.add_argument('--human-policy',choices=['orca_plus','sfm'],default='orca_plus',help='CrowdSimPlus native human policy')
p.add_argument('--goal-layout',choices=['shared','individual'],default='shared',help='CrowdSimPlus waypoint fixture; native policy unchanged')
p.add_argument('--sim-budget',type=float,default=75.)
a=p.parse_args()
if a.backend=='crowdsimplus' and not a.report_dir:p.error('CrowdSimPlus requires a separate --report-dir')
if a.invisible_robot and a.backend!='crowdsimplus':p.error('--invisible-robot requires CrowdSimPlus')
if a.robot_avoidance and (a.geometry!='circle_contact' or a.backend!='socnavgym' or not a.report_dir):
    p.error('--robot-avoidance requires SocNavGym, circle_contact, and separate --report-dir')
if a.human_policy!='orca_plus' and a.backend!='crowdsimplus':p.error('--human-policy sfm requires CrowdSimPlus')
if a.goal_layout!='shared' and (a.backend!='crowdsimplus' or a.condition not in ('counterflow','crossing')):p.error('--goal-layout individual requires a moving CrowdSimPlus fixture')
if a.condition=='crossing' and (a.backend!='crowdsimplus' or a.goal_layout!='individual'):p.error('Crossing requires CrowdSimPlus and --goal-layout individual')
worker_flags=['--robot-avoidance'] if a.robot_avoidance else []
if a.geometry!='native' and (a.backend!='socnavgym' or not a.report_dir):
    p.error('--geometry circle_contact requires --backend socnavgym and a separate --report-dir')
if a.diagnostics and not a.report_dir:
    p.error('--diagnostics requires a separate --report-dir')
label=a.backend if a.geometry=='native' else f'{a.backend}_{a.geometry}'
if a.robot_avoidance:label+='_robot_orca'
if a.backend=='crowdsimplus' and a.human_policy!='orca_plus':label+='_'+a.human_policy
if a.goal_layout!='shared':label+='_individual_goals'
if a.invisible_robot:label+='_blind'
name=f'{label}_{a.condition}_s{a.seed}'
(HERE/'generated').mkdir(exist_ok=True)
runtime=Path(tempfile.mkdtemp(prefix=name+'_',dir=HERE/'generated'))
os.environ.setdefault('ROS_DOMAIN_ID','75')
os.environ['GZ_PARTITION']=f'dynamuhsim_{name}_{os.getpid()}'
os.environ['ROS_LOG_DIR']=str(runtime/'ros_logs')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS','UDPv4')
report=a.report_dir.resolve() if a.report_dir else HERE/'reports'
report.mkdir(exist_ok=True,parents=True)
if a.report_dir and (report/f'{name}.json').exists():
    p.error('Trial already exists in --report-dir; choose a new directory')
diag=None
if a.diagnostics:
    from diagnostics import Capture
    diag=Capture(runtime/'diagnostics.jsonl')
processes=[]
node=None
socket_dir=None
container_name=None
result=dict(name=name,backend=a.backend,goal_layout=a.goal_layout if a.backend=='crowdsimplus' else None,human_policy=a.human_policy if a.backend=='crowdsimplus' else None,condition=a.condition,seed=a.seed,geometry=a.geometry,robot_avoidance=a.robot_avoidance,robot_visible=not a.invisible_robot,status='ERROR',
    protocol_valid=False,runtime=str(runtime.relative_to(HERE)),metrics_clock='simulation',
    commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip())
rows=[]
frames=deque(maxlen=240)
phase='warmup'
latest_poses={}
pose_time=None
last_row=None
native_messages=0
counts={key:0 for key in ['scan','imu','odom','clock','amcl']}
latest={}
files=[]
try:
    if a.backend=='crowdsimplus':
        prepare_crowd_scene(runtime,a.condition,a.seed,a.goal_layout)
    else:
        prepare=subprocess.run([str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'worker.py'),
            '--runtime',str(runtime),'--condition',a.condition,'--seed',str(a.seed),'--geometry',a.geometry,'--prepare',*worker_flags],
            capture_output=True,text=True,timeout=90)
        (runtime/'prepare.log').write_text(prepare.stdout+prepare.stderr)
        if prepare.returncode:
            raise RuntimeError('Scene preparation failed')
    scene=json.loads((runtime/'scene.json').read_text())
    if a.backend=='empty':
        scene['initial']=[]
        (runtime/'scene.json').write_text(json.dumps(scene,indent=2))
    result['contact_geometry']=json.loads((runtime/'geometry.json').read_text()) if a.backend=='socnavgym' else None
    result['group_behavior']=json.loads((runtime/'behavior.json').read_text()) if a.backend=='socnavgym' else None
    result['scene']=scene
    result['scene_sha256']=hashlib.sha256((runtime/'scene.json').read_bytes()).hexdigest()
    result['map_sha256']=hashlib.sha256((SOCIAL/'warehouse/warehouse.pgm').read_bytes()).hexdigest()
    result['nav_params_sha256']=hashlib.sha256((SOCIAL/'warehouse/nav2_warehouse.yaml').read_bytes()).hexdigest()
    socket_path=''
    if a.backend=='socnavgym':
        socket_dir=tempfile.TemporaryDirectory(prefix='social_bench_')
        socket_path=str(Path(socket_dir.name)/'worker.sock')
        workerlog=(runtime/'worker.log').open('w'); files.append(workerlog)
        worker=subprocess.Popen([str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'worker.py'),
            '--runtime',str(runtime),'--condition',a.condition,'--seed',str(a.seed),'--geometry',a.geometry,'--socket',socket_path,*worker_flags],
            stdout=workerlog,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(worker)
        deadline=time.monotonic()+90
        while not (runtime/'worker_ready').exists():
            if worker.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('SocNavGym worker not ready')
            time.sleep(.1)
    if a.backend=='crowdsimplus':
        socket_dir=tempfile.TemporaryDirectory(prefix='crowdsimplus_')
        socket_path=str(Path(socket_dir.name)/'worker.sock')
        container_name=f'dynamuhsim_crowd_{os.getpid()}'
        workerlog=(runtime/'worker.log').open('w'); files.append(workerlog)
        worker=subprocess.Popen(docker_command(runtime,Path(socket_dir.name),container_name,a.invisible_robot,a.human_policy),
            stdout=workerlog,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(worker)
        deadline=time.monotonic()+90
        while not (runtime/'worker_ready').exists():
            if worker.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('CrowdSimPlus container worker not ready: '+str(runtime/'worker.log'))
            time.sleep(.1)
        result['crowdsim_metadata']=json.loads((runtime/'crowdsim_metadata.json').read_text())
        if result['crowdsim_metadata']['goal_layout']!=a.goal_layout:raise RuntimeError('Unexpected goal layout')
        if result['crowdsim_metadata']['human_policy']!=a.human_policy:raise RuntimeError('Unexpected native human policy')
        if result['crowdsim_metadata']['source_commit']!='d526a07c013c621fa1ad50894b6ba639f61063de':
            raise RuntimeError('Unexpected CrowdSimPlus source revision')
        result['crowdsim_image_id']=subprocess.check_output(['docker','image','inspect','dynamuhsim-crowdsimplus:round2','--format','{{.Id}}'],text=True).strip()
    make_world(runtime,socket_path,scene['initial'],'conversation' if a.condition=='standing' else 'walking')
    if a.backend=='crowdsimplus':convert_crowd_world(runtime)
    tree=ET.parse(runtime/'warehouse.sdf')
    world=tree.getroot().find('world')
    if a.backend not in ('socnavgym','crowdsimplus'):
        world.remove(world.find("plugin[@name='social_experiment::Adapter']"))
    names={}
    for h in scene['initial']:
        names[h['id']]=f"{a.backend}_{h['id']}_proxy" if a.backend in ('socnavgym','crowdsimplus') else f"human_{h['id']:02d}_proxy"
    if a.backend=='rvo2':
        for h in scene['initial']:
            for tag,suffix in [('model','proxy'),('actor','visual')]:
                entity=world.find(f"{tag}[@name='socnavgym_{h['id']}_{suffix}']")
                entity.set('name',f"human_{h['id']:02d}_{suffix}")
        people=[]
        for h in scene['initial']:
            people.append(dict(id=h['id'],name=f"human_{h['id']:02d}",
                mode='stationary' if a.condition=='standing' else 'autonomous',
                persona='H4' if a.condition=='standing' else 'H1',
                speed=0. if a.condition=='standing' else .8,max_speed=.8,
                visual_z=1.,proxy_z=.85,scale=1.,loop=True,wait_for_robot=False,
                start=dict(x=h['x'],y=h['y'],yaw=h['yaw']),
                waypoints=[[h['x'],h['y']],*scene['group_route']]))
        scenario=runtime/'rvo2_scenario.json'
        scenario.write_text(json.dumps(dict(humans=people),indent=2))
        library=(REPO/'install/custom_corridor_plugins/lib/librvo2_human_system.so').resolve(strict=True)
        result['rvo2_binary_sha256']=hashlib.sha256(library.read_bytes()).hexdigest()
        plugin=element(world,'plugin',filename=str(library),name='custom_corridor::RVO2HumanSystem')
        for key,value in dict(human_count=3,robot_name='mecanum_robot',robot_radius=.25,human_radius=.36,
            randomize_politeness='false',auto_start='false',corridor_mode='false',
            seed=a.seed,default_scenario_file=str(scenario),max_speed=.8).items():
            element(plugin,key,value)
    ET.indent(tree)
    tree.write(runtime/'warehouse.sdf',encoding='unicode')
    os.environ['SOCIAL_BENCHMARK_WORLD']=str(runtime/'warehouse.sdf')
    rclpy.init()
    node=Node('social_navigation_pilot')
    def receive(key,msg):
        counts[key]+=1; latest[key]=msg
        if diag and key in ('scan','odom','amcl'):
            diag.ros(key,msg,latest.get('clock'))
    for key,topic,kind in [('scan','/scan',LaserScan),('imu','/imu',Imu),('odom','/odom',Odometry),
            ('clock','/clock',Clock),('amcl','/amcl_pose',PoseWithCovarianceStamped)]:
        node.create_subscription(kind,topic,lambda msg,key=key:receive(key,msg),qos_profile_sensor_data)
    def native_message(msg):
        global native_messages
        state=json.loads(msg.data)
        if state.get('adapter_ok') is False:
            raise RuntimeError('Adapter failure: '+state.get('error','unknown'))
        native_messages+=1
        if diag: diag.write('native',state,latest.get('clock'))
    topic=f'/{a.backend}/ground_truth' if a.backend in ('socnavgym','crowdsimplus') else '/rvo2_human/ground_truth'
    if a.backend!='empty':
        node.create_subscription(String,topic,native_message,qos_profile_sensor_data)
    if diag:
        diag.subscribe(node,lambda:latest.get('clock'))
        result['diagnostics']=str(diag.path)
    command=node.create_publisher(String,'/rvo2_human/command',10)
    gz=GzNode()
    def pose_message(msg):
        global pose_time,last_row
        t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9
        pose_time=t
        for pose in msg.pose:
            if pose.name=='mecanum_robot' or pose.name in names.values():
                latest_poses[pose.name]=dict(x=pose.position.x,y=pose.position.y,z=pose.position.z,
                    yaw=math.atan2(2*(pose.orientation.w*pose.orientation.z+pose.orientation.x*pose.orientation.y),
                                  1-2*(pose.orientation.y**2+pose.orientation.z**2)))
        if diag and 'mecanum_robot' in latest_poses:
            diag.gazebo(t,latest_poses)
        if phase!='active' or 'mecanum_robot' not in latest_poses or not all(n in latest_poses for n in names.values()):
            return
        if last_row is not None and t-last_row<.099:
            return
        last_row=t
        robot=latest_poses['mecanum_robot'].copy()
        humans=[dict(id=i,radius=.36,**latest_poses[n]) for i,n in names.items()]
        clearance=min((math.hypot(h['x']-robot['x'],h['y']-robot['y'])-.36-.25 for h in humans),default=None)
        pair=min((math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(humans) for j in humans[i+1:]),default=None)
        rows.append(dict(time=t,robot=robot,humans=humans,robot_human_clearance=clearance,human_pair_clearance=pair))
    def image_message(msg):
        if phase=='active':
            frames.append((msg.header.stamp.sec+msg.header.stamp.nsec/1e9,msg.width,msg.height,msg.step,bytes(msg.data)))
    gz.subscribe(Pose_V,'/world/social_warehouse/pose/info',pose_message)
    gz.subscribe(GzImage,'/crowdsimplus/observer' if a.backend=='crowdsimplus' else '/socnavgym/observer',image_message)
    launchlog=(report/f'{name}.log').open('w'); files.append(launchlog)
    launch=subprocess.Popen(['ros2','launch',str(HERE/'warehouse.launch.py'),f'gui:={str(a.gui).lower()}'],
        stdout=launchlog,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(launch)
    def wait(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if any(process.poll() is not None for process in processes):
                raise RuntimeError('Owned simulator or worker exited')
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate(): return True
        return False
    if not wait(lambda:all(counts[k]>=5 for k in ['scan','imu','odom','clock']) and counts['amcl']>0
            and 'mecanum_robot' in latest_poses and all(n in latest_poses for n in names.values())
            and (a.backend not in ('socnavgym','crowdsimplus') or native_messages>=5),100):
        raise RuntimeError(f'Sensors or initial scene not ready: {counts}')
    lifecycle=node.create_client(GetState,'/bt_navigator/get_state')
    client=ActionClient(node,NavigateToPose,'navigate_to_pose')
    deadline=time.monotonic()+90
    ready=False
    while time.monotonic()<deadline:
        if lifecycle.service_is_ready() and client.server_is_ready():
            future=lifecycle.call_async(GetState.Request())
            if wait(future.done,3) and future.result().current_state.id==3:
                ready=True; break
        wait(lambda:False,.2)
    if not ready: raise RuntimeError('bt_navigator not active')
    errors=[math.hypot(latest_poses[names[h['id']]]['x']-h['x'],latest_poses[names[h['id']]]['y']-h['y']) for h in scene['initial']]
    result['initial_position_error_m_max']=max(errors,default=0.)
    if max(errors,default=0.)>.01: raise RuntimeError('People moved during Nav2 warmup')
    initial_robot=latest_poses['mecanum_robot']
    if math.hypot(initial_robot['x']+27.,initial_robot['y'])>.02:
        raise RuntimeError('Robot moved during warmup')
    goal=NavigateToPose.Goal()
    goal.pose.header.frame_id='map'; goal.pose.header.stamp=latest['clock'].clock
    goal.pose.pose.position.x=-14.; goal.pose.pose.orientation.w=1.
    future=client.send_goal_async(goal)
    if not wait(future.done,10): raise RuntimeError('No Nav2 goal response')
    handle=future.result()
    if not handle.accepted: raise RuntimeError('Nav2 goal rejected')
    if a.backend=='rvo2' and command.get_subscription_count()==0:
        raise RuntimeError('RVO2 command bridge not connected')
    start=latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9
    wall_start=time.monotonic()
    phase='active'
    (runtime/'start_gate').write_text(str(start))
    command.publish(String(data='start'))
    pending=handle.get_result_async()
    timed_out=False
    deadline=time.monotonic()+max(180.,4*a.sim_budget)
    while not pending.done():
        if not wait(lambda:pending.done(),.1):
            now=latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9
            if now-start>a.sim_budget or time.monotonic()>deadline:
                timed_out=True; handle.cancel_goal_async(); break
    end=latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9
    phase='finished'
    result['sim_seconds']=end-start
    result['wall_seconds']=time.monotonic()-wall_start
    result['observed_rtf']=result['sim_seconds']/result['wall_seconds']
    result['action_status']=None if timed_out else pending.result().status
    result['status']='TIMEOUT' if timed_out else {4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(result['action_status'],'UNKNOWN')
    robot=latest_poses['mecanum_robot']
    amcl=latest['amcl'].pose.pose.position
    result['final_robot_xy']=[robot['x'],robot['y']]
    result['goal_error_world_m']=math.hypot(robot['x']+14.,robot['y'])
    result['goal_error_amcl_m']=math.hypot(amcl.x+14.,amcl.y)
    result['native_messages']=native_messages
    result['sensors']=counts
    if len(rows)<10 or (a.backend!='empty' and native_messages<3):
        raise RuntimeError('Insufficient measurement/behavior telemetry')
    if a.backend!='empty':
        result['max_human_displacement_m']=max(math.hypot(h['x']-scene['initial'][h['id']-1]['x'],h['y']-scene['initial'][h['id']-1]['y']) for row in rows for h in row['humans'])
        if a.condition in ('counterflow','crossing') and result['max_human_displacement_m']<.3:
            raise RuntimeError('Moving backend never moved after start gate')
    result['robot_path_m']=sum(math.hypot(b['robot']['x']-c['robot']['x'],b['robot']['y']-c['robot']['y']) for c,b in zip(rows,rows[1:]))
    result['min_robot_human_clearance_m']=min((row['robot_human_clearance'] for row in rows if row['robot_human_clearance'] is not None),default=None)
    result['min_human_pair_clearance_m']=min((row['human_pair_clearance'] for row in rows if row['human_pair_clearance'] is not None),default=None)
    result['robot_human_overlap_s']=sum(b['time']-c['time'] for c,b in zip(rows,rows[1:]) if b['robot_human_clearance'] is not None and b['robot_human_clearance']<0.)
    result['human_pair_overlap_s']=sum(b['time']-c['time'] for c,b in zip(rows,rows[1:]) if b['human_pair_clearance'] is not None and b['human_pair_clearance']<0.)
    result['robot_stopped_s']=sum(b['time']-c['time'] for c,b in zip(rows,rows[1:]) if math.hypot(b['robot']['x']-c['robot']['x'],b['robot']['y']-c['robot']['y'])/(b['time']-c['time'])<.03)
    result['protocol_valid']=True
except Exception as ex:
    result['status']='ERROR'; result['error']=str(ex)
    import traceback
    traceback.print_exc()
finally:
    phase='finished'
    if node:
        node.destroy_node(); rclpy.shutdown()
    for process in reversed(processes): stop(process)
    cleanup_container(container_name)
    if socket_dir: socket_dir.cleanup()
    for f in files: f.close()
    if diag:
        result['diagnostic_counts']=diag.counts
        diag.close()
    (report/f'{name}_trajectory.json').write_text(json.dumps(rows))
    result['samples']=len(rows)
    result['observer_frames']=len(frames)
    if frames:
        target=min(rows,key=lambda r:r['robot_human_clearance'] if r['robot_human_clearance'] is not None else 1e9)['time'] if rows else frames[0][0]
        frame=min(frames,key=lambda f:abs(f[0]-target))
        _,width,height,stride,data=frame
        Image.frombytes('RGB',(width,height),data,'raw','RGB',stride).save(report/f'{name}.png')
        preview=[Image.frombytes('RGB',(f[1],f[2]),f[4],'raw','RGB',f[3]).quantize(colors=96) for f in list(frames)[::2]]
        preview[0].save(report/f'{name}.gif',save_all=True,append_images=preview[1:],duration=667,loop=0)
    (report/f'{name}.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='scene'},indent=2),flush=True)
raise SystemExit(0 if result['protocol_valid'] else 1)
