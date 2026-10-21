#!/usr/bin/env python3
"""One bounded live Gazebo integration; manual robot motion, no Nav2 benchmark."""
import argparse,collections,hashlib,json,math,os,signal,subprocess,sys,tempfile,time,traceback,xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan,Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent;REPO=SOCIAL.parent.parent
sys.path.insert(0,str(SOCIAL/'gazebo_adapter'));from run import make_world,stop
sys.path.insert(0,str(BENCH/'gpu_runtime'));from runtime_guard import RuntimeGuard,gpu_preflight
sys.path.insert(0,str(BENCH));from shutdown_fix.ordered_stop import launch_environment,stop_server
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);p.add_argument('--gui',action='store_true');p.add_argument('--sim-seconds',type=float,default=120.);a=p.parse_args()
out=a.report_dir.resolve();out.mkdir(parents=True,exist_ok=False);runtime=HERE/'generated'/out.name;runtime.mkdir(parents=True,exist_ok=False)
preflight=json.loads((HERE/'reports/offline02/summary.json').read_text());assert len(preflight)==3 and all(r['status']=='PASS' for r in preflight)
os.environ.update(ROS_DOMAIN_ID='90',GZ_PARTITION='dynamuhsim_lifecycle_'+str(os.getpid()),QT_QPA_PLATFORM='xcb',ROS_LOG_DIR=str(runtime/'ros_logs'),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
guard=RuntimeGuard(runtime);processes=[];files=[];node=None;gz=None;frames=[];states=[];poses=[];scans=[];odom=[];latest={};counts=collections.Counter();sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
result=dict(status='ERROR',gui=a.gui,sim_budget_s=a.sim_seconds,runtime=str(runtime.relative_to(REPO)),scope='Live factory6-person lifecycle integration; own velocity commands, no Nav2 or navigation-success claim',head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip())
result['source_sha256']={str(p.relative_to(REPO)):sha(p) for p in [HERE/'kernel.py',HERE/'motion.py',HERE/'geometry.py',HERE/'worker.py',HERE/'adapter.cc',HERE/'warehouse.launch.py',HERE/'config.yaml',Path(__file__),HERE/'build/liblifecycle_adapter.so',SOCIAL/'warehouse/layout.json',SOCIAL/'gazebo_adapter/run.py',SOCIAL/'benchmark/socnavgym_lifecycle_fix/continuous_groups.py']}
(out/'protocol.json').write_text(json.dumps(result,indent=2))
def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
try:
    result['rendering']=gpu_preflight(runtime);launch_env,result['shutdown_preload']=launch_environment(runtime)
    sockdir=tempfile.TemporaryDirectory(prefix='lifecycle_');socket_path=str(Path(sockdir.name)/'worker.sock')
    f=(runtime/'worker.log').open('w');files.append(f)
    worker=subprocess.Popen([str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'worker.py'),'--runtime',str(runtime),'--socket',socket_path],stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(worker)
    deadline=time.monotonic()+90
    while not (runtime/'ready.json').exists():
        if worker.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Lifecycle worker not ready')
        time.sleep(.1)
    ready=json.loads((runtime/'ready.json').read_text());result['ready']=ready;assert ready['obstacles']==42 and ready['wall_observation_shape']==[336]
    make_world(runtime,socket_path,ready['humans'],'walking');tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world');plugin=world.find("plugin[@name='social_experiment::Adapter']")
    plugin.set('filename',str(HERE/'build/liblifecycle_adapter.so'));plugin.set('name','social_lifecycle::Adapter');ET.SubElement(plugin,'population').text='6'
    for item in list(world):
        name=item.get('name','')
        if name.startswith('socnavgym_') and name!='socnavgym_observer':item.set('name',name.replace('socnavgym_','lifecycle_',1))
    observer=world.find("model[@name='socnavgym_observer']");observer.find('.//topic').text='/lifecycle/observer';observer.find('.//update_rate').text='1'
    observer.find('.//image/width').text='640';observer.find('.//image/height').text='480'
    ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode');os.environ['SOCNAVGYM_EXPERIMENT_WORLD']=str(runtime/'warehouse.sdf');launch_env['SOCNAVGYM_EXPERIMENT_WORLD']=str(runtime/'warehouse.sdf')
    rclpy.init();node=Node('lifecycle_integration_verifier');gz=GzNode()
    def receive(key,msg):
        counts[key]+=1
        if key=='gt':
            value=json.loads(msg.data)
            if not value.get('adapter_ok'):raise RuntimeError('Adapter failed: '+value.get('error','unknown'))
            states.append(value);latest[key]=value
        else:latest[key]=msg
        if key=='scan':scans.append(dict(time=stamp(msg),ranges=[float(r) if math.isfinite(r) else None for r in msg.ranges],angle_min=msg.angle_min,angle_increment=msg.angle_increment,range_min=msg.range_min,range_max=msg.range_max))
        if key=='odom':odom.append(dict(time=stamp(msg),x=msg.pose.pose.position.x,y=msg.pose.pose.position.y))
    for key,topic,kind in [('gt','/socnavgym_lifecycle/ground_truth',String),('scan','/scan',LaserScan),('imu','/imu',Imu),('odom','/odom',Odometry),('clock','/clock',Clock)]:node.create_subscription(kind,topic,lambda msg,key=key:receive(key,msg),qos_profile_sensor_data)
    def scene(msg):
        t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9;entities={}
        for p in msg.pose:
            if p.name=='mecanum_robot' or p.name.startswith('lifecycle_') and p.name.endswith('_proxy'):
                q=p.orientation;entities[p.name]=dict(x=p.position.x,y=p.position.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        if entities:poses.append(dict(time=t,poses=entities))
    def camera(msg):frames.append((msg.header.stamp.sec+msg.header.stamp.nsec/1e9,msg.width,msg.height,msg.step,bytes(msg.data)))
    gz.subscribe(Pose_V,'/world/social_warehouse/pose/info',scene);gz.subscribe(GzImage,'/lifecycle/observer',camera)
    pub=node.create_publisher(TwistStamped,'/cmd_vel',10)
    f=(out/'launch.log').open('w');files.append(f)
    launch=subprocess.Popen(['ros2','launch',str(HERE/'warehouse.launch.py'),f'gui:={str(a.gui).lower()}'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=launch_env);processes.append(launch)
    def spin(seconds,predicate=lambda:False):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in processes):raise RuntimeError('Owned launch/worker exited')
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return True
        return False
    assert spin(100,lambda:all(counts[k]>=5 for k in ['gt','scan','imu','odom','clock']) and len(poses)>10),'Sensors/poses not ready'
    start=latest['gt']['time'];result['measurement_start_sim']=start;paused=False;deadline=time.monotonic()+max(360.,a.sim_seconds*5)
    while latest['gt']['time']<start+a.sim_seconds:
        if time.monotonic()>deadline:raise RuntimeError('Simulation wall-time budget exceeded')
        elapsed=latest['gt']['time']-start;vx=.12 if elapsed<3 else 0.;vy=.12 if 3.5<=elapsed<5.5 else 0.
        cmd=TwistStamped();cmd.header.stamp=latest['clock'].clock;cmd.twist.linear.x=vx;cmd.twist.linear.y=vy;pub.publish(cmd)
        spin(.025)
        if not paused and elapsed>8.:
            def control(value):
                response=subprocess.run(['gz','service','-s','/world/social_warehouse/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: '+str(value).lower()],capture_output=True,text=True,timeout=8)
                if response.returncode or 'data: true' not in response.stdout:raise RuntimeError('Pause/resume service failed')
            control(True);spin(.4);before=latest['gt'];spin(1.2);after=latest['gt'];result['pause_check']=dict(time_delta=after['time']-before['time'],step_delta=after['step']-before['step']);assert result['pause_check']==dict(time_delta=0.,step_delta=0)
            control(False);paused=True
    cmd=TwistStamped();cmd.header.stamp=latest['clock'].clock;pub.publish(cmd);spin(.3)
    result['sensor_counts']=dict(counts);result['sim_seconds_observed']=states[-1]['time']-states[0]['time'];result['cmd_vel_publishers']=[p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
    result['status']='RECORDED'
except Exception as error:result['error']=str(error);traceback.print_exc()
finally:
    try:result['server_shutdown']=stop_server(guard)
    except Exception as error:result['shutdown_error']=str(error)
    for process in reversed(processes):stop(process)
    if node:
        try:node.destroy_node();rclpy.shutdown()
        except Exception as error:result['ros_teardown_error']=str(error)
    result['cleanup']=guard.cleanup()
    for f in files:f.close()
    if 'sockdir' in globals():sockdir.cleanup()
    log=(out/'launch.log').read_text() if (out/'launch.log').exists() else '';result['gazebo_clean_exit']='[INFO] [gz-1]: process has finished cleanly' in log
    for name,rows in [('ground_truth',states),('gazebo_poses',poses),('scans',scans),('odom',odom)]:
        with (out/(name+'.jsonl')).open('w') as f:
            for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')
    if frames:
        frame_meta=[];images=[]
        for t,w,h,stride,data in frames:images.append(Image.frombytes('RGB',(w,h),data,'raw','RGB',stride).quantize(colors=96));frame_meta.append(t)
        for i,label in [(0,'initial'),(len(frames)//2,'middle'),(len(frames)-1,'final')]:images[i].convert('RGB').save(out/(label+'.png'))
        durations=[max(20,round((b-a)*250/10)*10) for a,b in zip(frame_meta,frame_meta[1:])]+[250]
        images[0].save(out/'camera_preview_4x.gif',save_all=True,append_images=images[1:],duration=durations,loop=0)
        result['media']=dict(source='Live Gazebo RGB observer camera, not screen recording or AI',frame_times=frame_meta,playback_speed=4,gif_duration_ms=sum(durations))
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
raise SystemExit(0 if result['status']=='RECORDED' else 1)
