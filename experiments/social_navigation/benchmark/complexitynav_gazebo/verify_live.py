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
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);p.add_argument('--gui',action='store_true');p.add_argument('--sim-seconds',type=float,default=60.);p.add_argument('--fault-after-updates',type=int);a=p.parse_args()
out=a.report_dir.resolve();out.mkdir(parents=True,exist_ok=False);runtime=HERE/'generated'/out.name;runtime.mkdir(parents=True,exist_ok=False)
GEOM=BENCH/'complexitynav_geometry';IMAGE='sha256:edfd591ea7551813419e511b341089ad956b6d3e4c1bd395a326035865ef1d5d'
preflight=json.loads((GEOM/'reports/run01/independent_validation.json').read_text());assert preflight['status']=='PASS_OFFLINE_ONLY'
for name,digest in json.loads((GEOM/'reports/run01/protocol.json').read_text())['source_sha256'].items():assert hashlib.sha256((GEOM/name).read_bytes()).hexdigest()==digest,name
transport=json.loads((HERE/'reports/transport01/validation.json').read_text());assert transport['status']=='PASS' and transport['worker_sha256']==hashlib.sha256((HERE/'worker.py').read_bytes()).hexdigest()
os.environ.update(ROS_DOMAIN_ID='91',GZ_PARTITION='dynamuhsim_complexitynav_'+str(os.getpid()),QT_QPA_PLATFORM='xcb',ROS_LOG_DIR=str(runtime/'ros_logs'),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
guard=RuntimeGuard(runtime);processes=[];files=[];node=None;gz=None;frames=[];states=[];poses=[];scans=[];odom=[];faults=[];latest={};counts=collections.Counter();container_name='dynamuhsim-cnav-live-'+guard.token[:12];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
result=dict(status='ERROR',gui=a.gui,sim_budget_s=a.sim_seconds,runtime=str(runtime.relative_to(REPO)),scope='Live4-person ComplexityNav geometry variant; own bounded robot commands, no Nav2/ranking claim',fault_after_updates=a.fault_after_updates,head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip())
result['source_sha256']={str(p.relative_to(REPO)):sha(p) for p in [HERE/'fixture.json',HERE/'worker.py',HERE/'adapter.cc',HERE/'warehouse.launch.py',Path(__file__),HERE/'build/libcomplexitynav_adapter.so',GEOM/'kernel.py',GEOM/'roadmap.py',GEOM/'generated/native_contract.py',GEOM/'generated/swept.py',GEOM/'generated/layout.json',SOCIAL/'warehouse/layout.json',SOCIAL/'gazebo_adapter/run.py']}
(out/'protocol.json').write_text(json.dumps(dict(result,status='PLANNED'),indent=2)+'\n')
def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
try:
    result['rendering']=gpu_preflight(runtime);launch_env,result['shutdown_preload']=launch_environment(runtime)
    sockdir=tempfile.TemporaryDirectory(prefix='cnav_live_');socket_path=str(Path(sockdir.name)/'worker.sock')
    f=(runtime/'worker.log').open('w');files.append(f)
    worker_command=['docker','run','--rm','--name',container_name,'--network','none','--cpus','2','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-e','DYNAMUHSIM_EXPERIMENT_SESSION='+guard.token,'-v',f'{HERE}:/work:ro','-v',f'{GEOM}:/geometry:ro','-v',f'{runtime}:/runtime','-v',f'{sockdir.name}:/socket',IMAGE,'python','/work/worker.py','--runtime','/runtime','--socket','/socket/worker.sock']
    if a.fault_after_updates is not None:worker_command+=['--fault-after-updates',str(a.fault_after_updates)]
    result['worker_command']=worker_command;result['container_image']=IMAGE
    worker=subprocess.Popen(worker_command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(worker)
    deadline=time.monotonic()+90
    while not (runtime/'ready.json').exists():
        if worker.poll() is not None or time.monotonic()>deadline:raise RuntimeError('ComplexityNav worker not ready')
        time.sleep(.1)
    ready=json.loads((runtime/'ready.json').read_text());result['ready']=ready;assert ready['obstacles']==42 and ready['native_dt']==.25 and len(ready['humans'])==4
    make_world(runtime,socket_path,ready['humans'],'walking');tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world');plugin=world.find("plugin[@name='social_experiment::Adapter']")
    plugin.set('filename',str(HERE/'build/libcomplexitynav_adapter.so'));plugin.set('name','complexity_experiment::Adapter');ET.SubElement(plugin,'population').text='4'
    physics=world.find('physics');physics.find('max_step_size').text='0.0025'
    physics.find('real_time_factor').text='1.0'
    for item in list(world):
        name=item.get('name','')
        if name.startswith('socnavgym_'):item.set('name',name.replace('socnavgym_','complexitynav_',1))
    observer=world.find("model[@name='complexitynav_observer']");observer.find('.//topic').text='/complexitynav/observer';observer.find('.//update_rate').text='1'
    observer.find('.//image/width').text='640';observer.find('.//image/height').text='480'
    ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode');os.environ['COMPLEXITYNAV_EXPERIMENT_WORLD']=str(runtime/'warehouse.sdf');launch_env['COMPLEXITYNAV_EXPERIMENT_WORLD']=str(runtime/'warehouse.sdf')
    rclpy.init();node=Node('complexitynav_integration_verifier');gz=GzNode()
    def receive(key,msg):
        counts[key]+=1
        if key=='gt':
            value=json.loads(msg.data)
            if not value.get('adapter_ok'):
                faults.append(value)
                if a.fault_after_updates is None:raise RuntimeError('Adapter failed: '+value.get('error','unknown'))
                return
            states.append(value);latest[key]=value
        else:latest[key]=msg
        if key=='scan':scans.append(dict(time=stamp(msg),ranges=[float(r) if math.isfinite(r) else None for r in msg.ranges],angle_min=msg.angle_min,angle_increment=msg.angle_increment,range_min=msg.range_min,range_max=msg.range_max))
        if key=='odom':odom.append(dict(time=stamp(msg),x=msg.pose.pose.position.x,y=msg.pose.pose.position.y))
    for key,topic,kind in [('gt','/complexitynav/ground_truth',String),('scan','/scan',LaserScan),('imu','/imu',Imu),('odom','/odom',Odometry),('clock','/clock',Clock)]:node.create_subscription(kind,topic,lambda msg,key=key:receive(key,msg),qos_profile_sensor_data)
    def scene(msg):
        t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9;entities={}
        for p in msg.pose:
            if p.name=='mecanum_robot' or p.name.startswith('complexitynav_') and p.name.endswith('_proxy'):
                q=p.orientation;entities[p.name]=dict(x=p.position.x,y=p.position.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        if entities:poses.append(dict(time=t,poses=entities))
    def camera(msg):frames.append((msg.header.stamp.sec+msg.header.stamp.nsec/1e9,msg.width,msg.height,msg.step,bytes(msg.data)))
    gz.subscribe(Pose_V,'/world/social_warehouse/pose/info',scene);gz.subscribe(GzImage,'/complexitynav/observer',camera)
    pub=node.create_publisher(TwistStamped,'/cmd_vel',10)
    f=(out/'launch.log').open('w');files.append(f)
    launch=subprocess.Popen(['ros2','launch',str(HERE/'warehouse.launch.py'),f'gui:={str(a.gui).lower()}'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=launch_env);processes.append(launch)
    def spin(seconds,predicate=lambda:False):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if launch.poll() is not None:raise RuntimeError('Owned launch exited')
            if worker.poll() is not None and a.fault_after_updates is None:raise RuntimeError('Owned worker exited')
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return True
        return False
    assert spin(100,lambda:all(counts[k]>=5 for k in ['gt','scan','imu','odom','clock']) and len(poses)>10),'Sensors/poses not ready'
    start=latest['gt']['time'];result['measurement_start_sim']=start;paused=False;deadline=time.monotonic()+max(360.,a.sim_seconds*5)
    while latest['gt']['time']<start+a.sim_seconds:
        if time.monotonic()>deadline:raise RuntimeError('Simulation wall-time budget exceeded')
        elapsed=latest['gt']['time']-start;vx=.1 if elapsed<6 else 0.;vy=.1 if 6<=elapsed<9 else 0.
        if a.fault_after_updates is not None:vx=vy=0.
        if faults and latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9>=faults[0]['time']+1.5:
            result['freeze_window']=dict(start_sim=faults[0]['time'],end_sim=latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9);break
        cmd=TwistStamped();cmd.header.stamp=latest['clock'].clock;cmd.twist.linear.x=vx;cmd.twist.linear.y=vy;pub.publish(cmd)
        spin(.025)
        if not paused and elapsed>11. and a.fault_after_updates is None:
            def control(value):
                response=subprocess.run(['gz','service','-s','/world/social_warehouse/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: '+str(value).lower()],capture_output=True,text=True,timeout=8)
                if response.returncode or 'data: true' not in response.stdout:raise RuntimeError('Pause/resume service failed')
            control(True);spin(.4);before=latest['gt'];spin(1.2);after=latest['gt'];result['pause_check']=dict(time_delta=after['time']-before['time'],step_delta=after['step']-before['step']);assert result['pause_check']==dict(time_delta=0.,step_delta=0)
            control(False);paused=True
    cmd=TwistStamped();cmd.header.stamp=latest['clock'].clock;pub.publish(cmd);spin(.3)
    result['sensor_counts']=dict(counts);result['sim_seconds_observed']=states[-1]['time']-states[0]['time'];result['cmd_vel_publishers']=[p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
    if a.fault_after_updates is not None:
        assert faults and 'freeze_window' in result,'Expected injected fault not observed'
        result['status']='EXPECTED_FAULT_RECORDED'
    else:result['status']='RECORDED'
except Exception as error:result['error']=str(error);traceback.print_exc()
finally:
    try:result['server_shutdown']=stop_server(guard)
    except Exception as error:result['shutdown_error']=str(error)
    remaining=subprocess.check_output(['docker','ps','-aq','--filter','name=^/'+container_name+'$'],text=True).strip()
    if remaining:
        response=subprocess.run(['docker','stop','-t','3',container_name],capture_output=True,text=True,timeout=12);result['container_stop']=dict(status='STOP_REQUESTED',returncode=response.returncode)
    else:result['container_stop']=dict(status='ALREADY_EXITED')
    for process in reversed(processes):stop(process)
    result['container_remaining']=subprocess.check_output(['docker','ps','-aq','--filter','name=^/'+container_name+'$'],text=True).strip()
    result['faults']=faults
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
raise SystemExit(0 if result['status'] in ['RECORDED','EXPECTED_FAULT_RECORDED'] else 1)
