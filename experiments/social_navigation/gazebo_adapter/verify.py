#!/usr/bin/env python3
"""Bounded integration checks against real ROS sensors and Gazebo model poses."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image

HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser()
p.add_argument('--mode',choices=['walking','conversation'],default='walking')
p.add_argument('--gui',action='store_true')
p.add_argument('--observe-sim-seconds',type=float,default=8.)
a=p.parse_args()
os.environ.setdefault('ROS_DOMAIN_ID','74')
os.environ.setdefault('GZ_PARTITION','dynamuhsim_socnavgym_verify')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS','UDPv4')
name=f'{a.mode}{"_long" if a.observe_sim_seconds>8 else ""}_{"gui" if a.gui else "headless"}'
report=HERE/'reports'
report.mkdir(exist_ok=True)
command=['python3',str(HERE/'run.py'),'--mode',a.mode]+(['--gui'] if a.gui else [])
log=(report/f'{name}.log').open('w')
process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
result=dict(status='FAIL',mode=a.mode,gui=a.gui,command_duration_clock='simulation')
node=None
gz=None
try:
    rclpy.init()
    node=Node('socnavgym_integration_verifier')
    latest={}
    counts={k:0 for k in ['gt','scan','imu','odom','clock']}
    states=[]
    scans=[]
    gzposes=deque(maxlen=12000)
    gz=GzNode()
    frames=deque(maxlen=180)
    def image_message(msg):
        frames.append((msg.header.stamp.sec+msg.header.stamp.nsec/1e9,msg.width,msg.height,msg.step,bytes(msg.data)))
    gz.subscribe(GzImage,'/socnavgym/observer',image_message)
    def pose_message(msg):
        t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9
        proxies={pose.name:[pose.position.x,pose.position.y] for pose in msg.pose if pose.name.endswith('_proxy') and pose.name.startswith('socnavgym_')}
        if proxies:
            gzposes.append((t,proxies))
    gz.subscribe(Pose_V,'/world/social_warehouse/pose/info',pose_message)
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    def receive(key,msg):
        counts[key]+=1
        latest[key]=json.loads(msg.data) if key=='gt' else msg
        if key=='gt':
            if not latest[key].get('adapter_ok'):
                raise RuntimeError(latest[key].get('error','Adapter failed'))
            states.append(latest[key])
        elif key=='scan':
            scans.append(dict(time=stamp(msg),ranges=list(msg.ranges),angle_min=msg.angle_min,angle_increment=msg.angle_increment))
    for key,topic,kind in [('gt','/socnavgym/ground_truth',String),('scan','/scan',LaserScan),
            ('imu','/imu',Imu),('odom','/odom',Odometry),('clock','/clock',Clock)]:
        node.create_subscription(kind,topic,lambda msg,key=key:receive(key,msg),qos_profile_sensor_data)
    pub=node.create_publisher(TwistStamped,'/cmd_vel',10)
    def wait(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if process.poll() is not None:
                raise RuntimeError('Experiment process exited')
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():
                return True
        return False
    def drive(vx,vy,seconds):
        start_sim=latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9
        deadline=time.monotonic()+max(30.,4*seconds)
        while latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9-start_sim<seconds:
            if time.monotonic()>deadline or process.poll() is not None:
                raise RuntimeError('Simulation stalled during velocity command')
            msg=TwistStamped()
            msg.header.stamp=latest['clock'].clock
            msg.twist.linear.x=float(vx)
            msg.twist.linear.y=float(vy)
            pub.publish(msg)
            rclpy.spin_once(node,timeout_sec=.025)
    if not wait(lambda:all(v>=5 for v in counts.values()),90):
        raise RuntimeError(f'Topics did not become ready: {counts}')
    def xy():
        pose=latest['odom'].pose.pose.position
        return (pose.x,pose.y)
    start=xy()
    drive(.12,0.,3.)
    drive(0.,0.,.5)
    forward=xy()
    drive(0.,.12,2.)
    drive(0.,0.,.5)
    side=xy()
    result['forward_displacement_m']=forward[0]-start[0]
    result['lateral_displacement_m']=side[1]-forward[1]
    assert .2<result['forward_displacement_m']<.55
    assert .12<result['lateral_displacement_m']<.4
    robot=latest['gt']['robot']
    result['robot_feedback_error_m']=math.hypot(robot['x']-(-27+side[0]),robot['y']-side[1])
    assert result['robot_feedback_error_m']<.03
    result['max_feedback_vx']=max(s['robot']['vx'] for s in states)
    result['max_feedback_vy']=max(s['robot']['vy'] for s in states)
    assert result['max_feedback_vx']>.08 and result['max_feedback_vy']>.08
    if frames:
        _,width,height,stride,data=frames[0]
        Image.frombytes('RGB',(width,height),data,'raw','RGB',stride).save(report/f'{name}_initial.png')
    def control(paused):
        completed=subprocess.run(['gz','service','-s','/world/social_warehouse/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
            '--timeout','5000','--req','pause: '+str(paused).lower()],capture_output=True,text=True,timeout=8)
        if completed.returncode or 'data: true' not in completed.stdout:
            raise RuntimeError(f'Gazebo pause/resume request failed: {completed.stdout} {completed.stderr}')
    control(True)
    wait(lambda:False,.4)
    frozen=latest['gt']
    t0=latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9
    wait(lambda:False,1.5)
    t1=latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9
    result['pause_clock_delta_s']=t1-t0
    result['pause_human_step_delta']=latest['gt']['step']-frozen['step']
    assert abs(t1-t0)<1e-8 and result['pause_human_step_delta']==0
    control(False)
    assert wait(lambda:latest['gt']['step']>frozen['step']+5,8)
    observe_start=latest['gt']['time']
    assert wait(lambda:latest['gt']['time']>=observe_start+a.observe_sim_seconds, max(30.,4*a.observe_sim_seconds))
    assert all([h['id'] for h in s['humans']]==[1,2,3] for s in states)
    result['native_steps_observed']=states[-1]['step']-states[0]['step']
    result['sim_seconds_observed']=states[-1]['time']-states[0]['time']
    result['step_time_error_s']=abs(result['sim_seconds_observed']-.1*result['native_steps_observed'])
    assert result['step_time_error_s']<1e-6
    initial=states[0]['humans']
    final=states[-1]['humans']
    result['max_human_displacement_m']=max(math.hypot(h['x']-j['x'],h['y']-j['y']) for h,j in zip(initial,final))
    if a.mode=='walking':
        assert result['max_human_displacement_m']>2
    else:
        assert result['max_human_displacement_m']<1e-8
    result['max_group_radius_m']=max(max(math.hypot(h['x']-sum(j['x'] for j in s['humans'])/3,h['y']-sum(j['y'] for j in s['humans'])/3) for h in s['humans']) for s in states)
    result['min_robot_human_clearance_m']=min(math.hypot(h['x']-s['robot']['x'],h['y']-s['robot']['y'])-h['radius']-.25 for s in states for h in s['humans'])
    result['compute_ms_max']=max(s['compute_ms'] for s in states)
    result['compute_ms_mean']=sum(s['compute_ms'] for s in states)/len(states)
    # Compare real model poses published by SceneBroadcaster to worker state at
    # the same simulation time. Do not treat our desired-pose command as evidence.
    pose_errors=[]
    for t,proxies in list(gzposes):
        state=min(states,key=lambda s:abs(s['time']-t))
        if abs(state['time']-t)>.011:
            continue
        for h in state['humans']:
            actual=proxies.get(f"socnavgym_{h['id']}_proxy")
            if actual:
                pose_errors.append(math.hypot(actual[0]-h['x'],actual[1]-h['y']))
    result['gazebo_pose_samples']=len(pose_errors)
    result['gazebo_proxy_error_m_max']=max(pose_errors,default=None)
    assert len(pose_errors)>20 and max(pose_errors)<.15
    # GPU LiDAR must see the human proxy, in beams with no factory obstacle in
    # front. The test group is in the empty main aisle near x=-24.
    errors=[]
    matched_humans=set()
    for scan in scans:
        state=min(states,key=lambda s:abs(s['time']-scan['time']))
        if abs(state['time']-scan['time'])>.061:
            continue
        r=state['robot']
        for h in state['humans']:
            dx,dy=h['x']-r['x'],h['y']-r['y']
            bearing=(math.atan2(dy,dx)-r['yaw'])%(2*math.pi)
            distance=math.hypot(dx,dy)
            if not (.8<distance<3.8 and min(bearing,2*math.pi-bearing)<.7):
                continue
            index=round((bearing-scan['angle_min'])/scan['angle_increment'])
            if not 0<=index<len(scan['ranges']):
                continue
            # Exact ray-circle intersection, including finite beam angle.
            angle=scan['angle_min']+index*scan['angle_increment']+r['yaw']
            along=dx*math.cos(angle)+dy*math.sin(angle)
            cross=dx*math.sin(angle)-dy*math.cos(angle)
            if abs(cross)>=h['radius']:
                continue
            predicted=along-math.sqrt(h['radius']**2-cross**2)
            if predicted>3.4:
                continue
            # Other people can occlude this person; exclude such rays explicitly.
            occluded=False
            for other in state['humans']:
                if other['id']==h['id']:
                    continue
                ox,oy=other['x']-r['x'],other['y']-r['y']
                oa=ox*math.cos(angle)+oy*math.sin(angle)
                oc=ox*math.sin(angle)-oy*math.cos(angle)
                if abs(oc)<other['radius'] and 0<oa-math.sqrt(other['radius']**2-oc**2)<predicted-.02:
                    occluded=True
            if occluded:
                continue
            measured=scan['ranges'][index]
            error=abs(measured-predicted) if math.isfinite(measured) else 10.
            errors.append(error)
            if error<.15:
                matched_humans.add(h['id'])
    result['lidar_human_samples']=len(errors)
    result['lidar_samples_within_15cm']=sum(e<.15 for e in errors)
    result['lidar_match_fraction']=sum(e<.15 for e in errors)/max(1,len(errors))
    result['lidar_human_ids_matched']=sorted(matched_humans)
    assert len(errors)>=5 and result['lidar_match_fraction']>.7
    result['sensor_messages']=counts
    frames=[f for f in frames if f[0]>states[0]['time']+.2]
    result['observer_frames']=len(frames)
    result['observer_sim_time_range']=[frames[0][0],frames[-1][0]] if frames else []
    assert len(frames)>3
    for index,label in [(0,'initial'),(len(frames)//2,'middle'),(-1,'final')]:
        _,width,height,stride,data=frames[index]
        Image.frombytes('RGB',(width,height),data,'raw','RGB',stride).save(report/f'{name}_{label}.png')
    preview=[]
    for _,width,height,stride,data in frames:
        preview.append(Image.frombytes('RGB',(width,height),data,'raw','RGB',stride).quantize(colors=96))
    durations=[max(40,round(1000*(frames[i+1][0]-frames[i][0]))) for i in range(len(frames)-1)]+[333]
    preview[0].save(report/f'{name}.gif',save_all=True,append_images=preview[1:],duration=durations,loop=0)
    result['status']='PASS'
except Exception as ex:
    result['error']=str(ex) or type(ex).__name__
    import traceback
    traceback.print_exc()
finally:
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:
            process.wait(30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            process.wait(5)
    log.close()
    (report/f'{name}.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
raise SystemExit(0 if result['status']=='PASS' else 1)
