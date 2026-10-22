#!/usr/bin/env python3
"""Bounded depth/mask/LiDAR capture; GPU and cleanup required for every fixture."""
import argparse
import fcntl
import hashlib
import json
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
from sensor_msgs.msg import Image,CameraInfo,LaserScan
from fixture import create_fixture,labels,REPO,SOCIAL,BENCH,HERE,HEIGHT
sys.path.insert(0,str(BENCH/'gpu_runtime'));from runtime_guard import RuntimeGuard,gpu_preflight
sys.path.insert(0,str(BENCH));from shutdown_fix.ordered_stop import launch_environment,stop_server
sys.path.insert(0,str(SOCIAL/'gazebo_adapter'));from run import stop

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ns(s):return s.sec*1000000000+s.nanosec

def capture(index,snapshot,condition,root,protocol):
    out=root/f'snapshot{index:02d}'/condition;out.mkdir(parents=True)
    runtime=HERE/'generated/runtime'/root.name/f'snapshot{index:02d}'/condition;runtime.mkdir(parents=True,exist_ok=False)
    result=dict(status='ERROR',snapshot_index=index,condition=condition,snapshot=snapshot,
                runtime=str(runtime.relative_to(REPO)),native_human_steps=0,nav2_actions=0)
    os.environ.update(ROS_DOMAIN_ID='91',GZ_PARTITION=f'depth_proxy_{os.getpid()}_{index}_{condition}',ROS_LOG_DIR=str(runtime/'ros_logs'),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4',QT_QPA_PLATFORM='xcb')
    guard=RuntimeGuard(runtime);processes=[];logs=[];node=None
    image_rows={k:{} for k in labels(condition)};info={};scans={k:{} for k in ['native','geometry']}
    try:
        world,meta=create_fixture(runtime,snapshot,condition);result['fixture']=meta
        result['rendering']=gpu_preflight(runtime);env,result['shutdown_mode']=launch_environment(runtime)
        rclpy.init();node=Node('depth_proxy_capture')
        def image(label,m):
            t=ns(m.header.stamp)
            if t<protocol['validity']['min_stamp_ns']:return
            level=label.split('_')[0]
            assert m.encoding=='32FC1' and m.width==424 and m.height==240
            assert m.header.frame_id==f'camera_{level}_optical_frame'
            a=np.frombuffer(bytes(m.data),dtype='>f4' if m.is_bigendian else '<f4').reshape(m.height,m.step//4)[:,:m.width]
            image_rows[label][t]=a.astype(np.float32,copy=True)
        def scan(name,m):
            t=ns(m.header.stamp)
            if t<protocol['validity']['min_stamp_ns']:return
            assert len(m.ranges)==360 and m.header.frame_id=='base_scan'
            scans[name][t]=m
        bridge_args=[]
        for label in labels(condition):
            prefix=f'/ablation/{condition}/{label}'
            node.create_subscription(Image,prefix+'/depth',lambda m,k=label:image(k,m),qos_profile_sensor_data)
            node.create_subscription(CameraInfo,prefix+'/camera_info',lambda m,k=label:info.update({k:m}),qos_profile_sensor_data)
            bridge_args.extend([prefix+'/depth@sensor_msgs/msg/Image[gz.msgs.Image',prefix+'/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'])
        for name in scans:
            topic=f'/ablation/{condition}/scan_{name}'
            node.create_subscription(LaserScan,topic,lambda m,k=name:scan(k,m),qos_profile_sensor_data)
            bridge_args.append(topic+'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')
        commands=[('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',*bridge_args]),('gazebo',['gz','sim','-r','-s',str(world)])]
        for name,cmd in commands:
            log=(out/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env))
        deadline=time.monotonic()+protocol['validity']['max_wall_s']
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in processes):raise RuntimeError('Owned process exited')
            rclpy.spin_once(node,timeout_sec=.05)
            common=sorted(set.intersection(*(set(v) for v in image_rows.values())))
            if len(common)>=10 and len(info)==len(image_rows) and all(len(v)>=5 for v in scans.values()):break
        else:raise RuntimeError('Depth/info/LiDAR did not become ready')
        common=common[:10]
        stability={}
        for label in image_rows:
            stack=np.stack([image_rows[label][t] for t in common])
            np.save(out/f'depth_{label}.npy',stack,allow_pickle=False)
            finite=np.isfinite(stack)&np.isfinite(stack[0]);delta=np.zeros_like(stack)
            np.subtract(stack,stack[0],out=delta,where=finite)
            stability[label]=dict(max_abs_diff_m=float(np.max(np.abs(delta))),finite_mask_changes=int(np.sum(np.isfinite(stack)!=np.isfinite(stack[0]))),bitwise_static=all(frame.tobytes()==stack[0].tobytes() for frame in stack))
        result['depth_stability']=stability
        calibration={k:dict(k=list(m.k),d=list(m.d),p=list(m.p),width=m.width,height=m.height,frame_id=m.header.frame_id) for k,m in info.items()}
        assert all(np.allclose(v['k'],next(iter(calibration.values()))['k']) for v in calibration.values())
        for name,values in scans.items():
            ts=sorted(values)[:5];messages=[values[t] for t in ts]
            m=messages[0]
            np.savez(out/f'scan_{name}.npz',stamp_ns=np.array(ts,dtype=np.int64),ranges=np.array([x.ranges for x in messages],dtype=np.float32),intensities=np.array([x.intensities for x in messages],dtype=np.float32))
            result.setdefault('lidar_geometry',{})[name]=dict(frame_id=m.header.frame_id,angle_min=m.angle_min,angle_max=m.angle_max,angle_increment=m.angle_increment,range_min=m.range_min,range_max=m.range_max,rays=len(m.ranges))
        result.update(paired_stamp_ns=common,calibration=calibration)
        for name in ['warehouse.sdf','fixture.json','snapshot.json','robot_low.urdf','robot_high.urdf']:(out/name).write_bytes((runtime/name).read_bytes())
        assert all(v['finite_mask_changes']==0 and v['max_abs_diff_m']<=protocol['validity']['depth_static_tolerance_m'] for v in stability.values()), 'Depth repeat tolerance failed'
        if condition=='masked' and index<=2:
            effects={}
            for level in ['low','high']:
                a=image_rows[level+'_on'][common[5]];b=image_rows[level+'_off'][common[5]]
                good=np.isfinite(a)&np.isfinite(b);delta=np.zeros_like(a);np.subtract(b,a,out=delta,where=good)
                effects[level]=int(np.sum((good&(np.abs(delta)>.02))|(np.isfinite(a)!=np.isfinite(b))))
            result['mask_effect_pixels']=effects
            assert all(v>0 for v in effects.values()), 'Depth proxy mask has no measured effect'
        result['status']='MEASURED'
    except Exception as e:
        result.update(error=str(e),depth_counts={k:len(v) for k,v in image_rows.items()},info_received=sorted(info),scan_counts={k:len(v) for k,v in scans.items()});traceback.print_exc()
    finally:
        try:result['server_shutdown']=stop_server(guard)
        except Exception as e:result['shutdown_error']=str(e)
        for p in reversed(processes):stop(p)
        if node:node.destroy_node();rclpy.shutdown()
        for f in logs:f.close()
        gazebo_log=out/'gazebo.log'
        result['render_hook_log']=[line for line in (gazebo_log.read_text(errors='replace') if gazebo_log.exists() else '').splitlines() if '[DepthProxyFilter]' in line]
        result['cleanup']=guard.cleanup();result['gazebo_returncode']=processes[-1].returncode if processes else None
        result['accepted']=result['status']=='MEASURED' and result['gazebo_returncode']==0 and result['cleanup']['status']=='PASS'
        if result['status']=='MEASURED' and not result['accepted']:result['status']='MEASURED_UNCLEAN_SHUTDOWN'
        sources=[Path(__file__),HERE/'depth_proxy_filter.cc',HERE/'generated/build_jazzy/libdepth_proxy_filter.so',HERE/'fixture.py',HERE/'protocol.json',HERE/'run.sh',HERE/'snapshots.json',HEIGHT/'fixture.py',HEIGHT/'freeze_actors.cc',HEIGHT/'generated/build/libfreeze_actors.so',REPO/'src/custom_corridor/models/mecanum_robot/model.sdf',REPO/'src/custom_corridor/urdf/mecanum_robot.urdf',SOCIAL/'gazebo_adapter/run.py',SOCIAL/'warehouse/warehouse.sdf']
        result['source_sha256']={str(p.relative_to(REPO)):sha(p) for p in sources}
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:result.get(k) for k in ['status','snapshot_index','condition','error','depth_counts','info_received','scan_counts','gazebo_returncode']},indent=2),flush=True)
    assert result['status']=='MEASURED' and result['gazebo_returncode']==0 and result['cleanup']['status']=='PASS', 'Stop cohort, preserve failed output'
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--snapshot-indices',type=int,nargs='+',default=[1,2,3]);a=p.parse_args()
    assert (HEIGHT/'generated/build/libfreeze_actors.so').is_file(), 'Build camera_height_ab freezer first'
    assert (HERE/'generated/build_jazzy/libdepth_proxy_filter.so').is_file(), 'Build depth proxy render plugin first'
    root=a.output.resolve();root.mkdir(parents=True,exist_ok=False)
    lock=(BENCH/'common_gazebo/generated/domain91.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    protocol=json.loads((HERE/'protocol.json').read_text());snapshots=json.loads((HERE/'snapshots.json').read_text())
    (root/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    result=[]
    for i,s in enumerate(snapshots,1):
        if i not in a.snapshot_indices:continue
        for condition in protocol['conditions']:
            if result:time.sleep(protocol.get('gpu_cooldown_wall_s',0))
            result.append(capture(i,s,condition,root,protocol))
    (root/'capture.json').write_text(json.dumps(dict(status='PASS',fixtures=len(result)),indent=2)+'\n')
