#!/usr/bin/env python3
"""Bounded ROS/Gazebo smoke and navigation checks; stops only its own process group."""
import argparse
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
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState

HERE=Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument('--world',choices=['baseline','warehouse'],default='warehouse')
parser.add_argument('--nav',action='store_true')
parser.add_argument('--gui',action='store_true')
parser.add_argument('--goal-x',type=float,default=-23.)
parser.add_argument('--timeout',type=float,default=150.)
args=parser.parse_args()
os.environ.setdefault('ROS_DOMAIN_ID','73')
os.environ.setdefault('GZ_PARTITION','dynamuhsim_warehouse_checks')
os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS','UDPv4')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
name=f'{args.world}_{"nav" if args.nav else "sensors"}_{"gui" if args.gui else "headless"}'
if args.nav: name+=f'_goal{args.goal_x:g}'
if args.world=='warehouse':
    command=['ros2','launch',str(HERE/'warehouse/warehouse.launch.py'),f'nav:={str(args.nav).lower()}',
             f'gui:={str(args.gui).lower()}','rviz:=false']
else:
    command=['ros2','launch','custom_corridor','corridor_tb3.launch.py','width:=rvo2',
             'robot_model:=mecanum',f'gui:={str(args.gui).lower()}','rviz:=false']
log=(HERE/'reports'/f'{name}.log').open('w')
process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
result={'world':args.world,'gui':args.gui,'nav_requested':args.nav,'command':command,'status':'FAIL'}
node=None
try:
    rclpy.init()
    node=Node('warehouse_verifier')
    latest={}
    counts={k:0 for k in ['scan','imu','odom','clock','amcl']}
    clock_samples=[]
    def receive(key,msg):
        counts[key]+=1
        latest[key]=msg
        if key=='clock':
            clock_samples.append((time.monotonic(),msg.clock.sec+msg.clock.nanosec/1e9))
    for key,topic,kind in [('scan','/scan',LaserScan),('imu','/imu',Imu),('odom','/odom',Odometry),
                           ('clock','/clock',Clock),('amcl','/amcl_pose',PoseWithCovarianceStamped)]:
        node.create_subscription(kind,topic,lambda msg,key=key:receive(key,msg),qos_profile_sensor_data)
    pub=node.create_publisher(TwistStamped,'/cmd_vel',10)
    def spin_until(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():
                return True
        return False
    def command_velocity(vx,vy,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            msg=TwistStamped()
            if 'clock' in latest:
                msg.header.stamp=latest['clock'].clock
            msg.twist.linear.x=float(vx)
            msg.twist.linear.y=float(vy)
            pub.publish(msg)
            rclpy.spin_once(node,timeout_sec=.05)
    if not spin_until(lambda:all(counts[k]>=5 for k in ['scan','imu','odom','clock']),75.):
        raise RuntimeError(f'Sensors not ready: {counts}')
    pose=lambda:(latest['odom'].pose.pose.position.x,latest['odom'].pose.pose.position.y)
    start=pose()
    result['initial_odom']=start
    if args.nav:
        client=ActionClient(node,NavigateToPose,'navigate_to_pose')
        if not spin_until(lambda:client.server_is_ready() and counts['amcl']>0,75.):
            raise RuntimeError('Nav2 action/AMCL not ready')
        lifecycle=node.create_client(GetState,'/bt_navigator/get_state')
        ready=False
        deadline=time.monotonic()+90.
        while time.monotonic()<deadline and process.poll() is None:
            if not lifecycle.service_is_ready():
                rclpy.spin_once(node,timeout_sec=.1)
                continue
            request=lifecycle.call_async(GetState.Request())
            if spin_until(request.done,3.) and request.result().current_state.id==3:
                ready=True
                break
            spin_until(lambda:False,.5)
        if not ready:
            raise RuntimeError('bt_navigator lifecycle did not become active')
        goal=NavigateToPose.Goal()
        goal.pose.header.frame_id='map'
        goal.pose.header.stamp=latest['clock'].clock
        goal.pose.pose.position.x=args.goal_x
        goal.pose.pose.orientation.w=1.
        pending=client.send_goal_async(goal)
        if not spin_until(pending.done,10.):
            raise RuntimeError('No action response')
        handle=pending.result()
        if not handle.accepted:
            raise RuntimeError('Navigation rejected')
        pending_result=handle.get_result_async()
        if not spin_until(pending_result.done,args.timeout):
            handle.cancel_goal_async()
            raise RuntimeError('Navigation timed out')
        result['action_status']=pending_result.result().status
        result['goal_x']=args.goal_x
        if result['action_status']!=4:
            raise RuntimeError(f'Navigation ended with status {result["action_status"]}')
        amcl=latest['amcl'].pose.pose.position
        result['final_amcl']=[amcl.x,amcl.y]
        result['goal_error_m']=math.hypot(amcl.x-args.goal_x,amcl.y)
        if result['goal_error_m']>.5:
            raise RuntimeError('Action succeeded but AMCL goal error exceeds 0.5m')
    else:
        command_velocity(.15,0,3.)
        command_velocity(0,0,.5)
        forward=pose()
        command_velocity(0,.12,3.)
        command_velocity(0,0,.5)
        lateral=pose()
        result['forward_displacement_m']=math.dist(start,forward)
        result['lateral_displacement_m']=math.dist(forward,lateral)
        if result['forward_displacement_m']<.1 or result['lateral_displacement_m']<.1:
            raise RuntimeError('Mecanum motion below threshold')
    scan=latest['scan']
    valid=[float(x) for x in scan.ranges if math.isfinite(x) and scan.range_min<=x<=scan.range_max]
    result.update(sensor_messages=counts,scan_beams=len(scan.ranges),scan_finite_returns=len(valid),
                  min_range_m=min(valid) if valid else None,final_odom=pose())
    samples=clock_samples[len(clock_samples)//2:]
    if len(samples)>1:
        result['observed_rtf']=(samples[-1][1]-samples[0][1])/(samples[-1][0]-samples[0][0])
    if not valid:
        raise RuntimeError('No finite LiDAR returns')
    result['status']='PASS'
except Exception as exc:
    result['error']=str(exc)
finally:
    if node:
        try:
            command_velocity(0,0,.2)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    try:
        os.killpg(process.pid,signal.SIGINT)
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL)
    except ProcessLookupError:
        pass
    log.close()
    (HERE/'reports'/f'{name}.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
raise SystemExit(0 if result['status']=='PASS' else 1)


