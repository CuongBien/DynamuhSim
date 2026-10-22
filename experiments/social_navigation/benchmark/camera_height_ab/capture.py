#!/usr/bin/env python3
"""Paired GPU depth observations in bounded frozen scenes, with canonical local TF."""
import argparse
import collections
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from fixture import create_fixture,REPO,SOCIAL,BENCH,HERE
sys.path.insert(0,str(BENCH/'gpu_runtime'))
from runtime_guard import RuntimeGuard,gpu_preflight
sys.path.insert(0,str(BENCH))
from shutdown_fix.ordered_stop import launch_environment,stop_server
sys.path.insert(0,str(SOCIAL/'gazebo_adapter'))
from run import stop


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ns(stamp):return stamp.sec*1000000000+stamp.nanosec

def capture(index,snapshot,root,protocol):
    out=root/f'snapshot{index:02d}';out.mkdir()
    runtime=HERE/'generated'/root.name/out.name;runtime.mkdir(parents=True,exist_ok=False)
    result=dict(status='ERROR',snapshot_index=index,source_snapshot=snapshot,paired=True,
                fixture='Frozen world; no native human step or Nav2 action',
                runtime=str(runtime.relative_to(REPO)),frames_target=10)
    os.environ.update(ROS_DOMAIN_ID='91',GZ_PARTITION=f'camera_height_{os.getpid()}_{index}',
                      ROS_LOG_DIR=str(runtime/'ros_logs'),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      FASTDDS_BUILTIN_TRANSPORTS='UDPv4',QT_QPA_PLATFORM='xcb')
    guard=RuntimeGuard(runtime);processes=[];files=[];node=None
    images={k:{} for k in ['low','high']};infos={};tf=[]
    try:
        world=create_fixture(runtime,snapshot)
        result['rendering']=gpu_preflight(runtime)
        env,result['shutdown_mode']=launch_environment(runtime)
        rclpy.init();node=Node('camera_height_capture')
        broadcaster=StaticTransformBroadcaster(node)
        r=snapshot['robot']
        def transform(parent,child,xyz,q):
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
            return t
        tf.append(transform('world','base_link',[r['x'],r['y'],.04],[0,0,math.sin(r['yaw']/2),math.cos(r['yaw']/2)]))
        for label,z in [('low',.08),('high',.36)]:
            tf.append(transform('base_link',f'camera_{label}_link',[.12,0,z],[0,0,0,1]))
            tf.append(transform(f'camera_{label}_link',f'camera_{label}_optical_frame',[0,0,0],[-.5,.5,-.5,.5]))
        broadcaster.sendTransform(tf)
        def receive(label,msg):
            stamp=ns(msg.header.stamp)
            if stamp<protocol['validity']['min_stamp_ns']:return
            assert msg.encoding=='32FC1' and msg.width==424 and msg.height==240
            assert msg.header.frame_id==f'camera_{label}_optical_frame'
            d=np.frombuffer(bytes(msg.data),dtype='>f4' if msg.is_bigendian else '<f4').reshape(msg.height,msg.step//4)[:,:msg.width]
            images[label][stamp]=d.astype(np.float32,copy=True)
        for label in ['low','high']:
            node.create_subscription(Image,f'/height/{label}/depth',lambda m,label=label:receive(label,m),qos_profile_sensor_data)
            node.create_subscription(CameraInfo,f'/height/{label}/camera_info',lambda m,label=label:infos.update({label:m}),qos_profile_sensor_data)
        for name,command in [('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
                '/height/low/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                '/height/high/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                '/height/low/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/height/high/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']),
                ('gazebo',['gz','sim','-r','-s',str(world)])]:
            log=(out/(name+'.log')).open('w');files.append(log)
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env))
        deadline=time.monotonic()+protocol['validity']['max_wall_s_per_snapshot']
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in processes):raise RuntimeError('Owned process exited early')
            rclpy.spin_once(node,timeout_sec=.05)
            common=sorted(set(images['low'])&set(images['high']))
            if len(common)>=10 and len(infos)==2:break
        else:raise RuntimeError('Paired depth not ready within wall budget')
        common=common[:10]
        assert len(common)==10
        for label in ['low','high']:
            stack=np.stack([images[label][t] for t in common])
            same_finite=np.isfinite(stack)==np.isfinite(stack[0])
            finite=np.isfinite(stack)&np.isfinite(stack[0])
            delta=np.zeros_like(stack);np.subtract(stack,stack[0],out=delta,where=finite)
            assert same_finite.all() and np.max(np.abs(delta))<=1e-5, 'Scene is not frozen at sensor: '+label
        assert np.allclose(infos['low'].k,infos['high'].k)
        assert all(m.width==424 and m.height==240 and not any(abs(v)>1e-12 for v in m.d) for m in infos.values())
        for label in ['low','high']:
            np.save(out/f'depth_{label}.npy',np.stack([images[label][t] for t in common]),allow_pickle=False)
        result.update(status='MEASURED',paired_stamp_ns=common,
                      calibration={label:dict(k=list(m.k),p=list(m.p),d=list(m.d),frame_id=m.header.frame_id,width=m.width,height=m.height) for label,m in infos.items()},
                      tf_static=[dict(parent=t.header.frame_id,child=t.child_frame_id,
                          xyz=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                          q=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]) for t in tf])
        result['world_sha256']=sha(world)
        for name in ['warehouse.sdf','robot_low.urdf','robot_high.urdf','snapshot.json']:
            (out/name).write_bytes((runtime/name).read_bytes())
    except Exception as e:
        result['error']=str(e);result['received_counts']={k:len(v) for k,v in images.items()};result['camera_info_received']=sorted(infos);traceback.print_exc()
    finally:
        try:result['server_shutdown']=stop_server(guard)
        except Exception as e:result['shutdown_error']=str(e)
        for p in reversed(processes):stop(p)
        if node:
            node.destroy_node();rclpy.shutdown()
        for f in files:f.close()
        result['cleanup']=guard.cleanup()
        result['gazebo_returncode']=processes[-1].returncode if processes else None
        result['source_sha256']={str(p.relative_to(REPO)):sha(p) for p in [Path(__file__),HERE/'fixture.py',HERE/'protocol.json',HERE/'run.sh',HERE/'freeze_actors.cc',HERE/'generated/build/libfreeze_actors.so',REPO/'src/custom_corridor/models/mecanum_robot/model.sdf',REPO/'src/custom_corridor/urdf/mecanum_robot.urdf',SOCIAL/'gazebo_adapter/run.py',SOCIAL/'warehouse/warehouse.sdf']}
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:result.get(k) for k in ['status','snapshot_index','error','gazebo_returncode','server_shutdown']},indent=2),flush=True)
    if result['status']!='MEASURED' or result['cleanup']['status']!='PASS' or result['gazebo_returncode']!=0:
        raise RuntimeError('Snapshot failed; retain output and stop cohort')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    lock=(BENCH/'common_gazebo/generated/domain91.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    protocol=json.loads((HERE/'protocol.json').read_text())
    rows=[json.loads(l) for l in (REPO/protocol['source_episode']).open()]
    snapshots=[min(rows,key=lambda s:abs(s['stamp_ns']-t)) for t in protocol['target_snapshot_ns']]
    (out/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    results=[capture(i,snapshot,out,protocol) for i,snapshot in enumerate(snapshots,1)]
    (out/'capture.json').write_text(json.dumps(dict(status='PASS',snapshots=len(results),source_episode_sha256=sha(REPO/protocol['source_episode'])),indent=2)+'\n')
