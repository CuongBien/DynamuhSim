#!/usr/bin/env python3
"""Bounded Nav2 action pilot; no manual cmd_vel publisher or robot source edits."""
import argparse,base64,collections,copy,fcntl,hashlib,json,math,os,subprocess,sys,tempfile,time,traceback,xml.etree.ElementTree as ET,zlib
from pathlib import Path
import yaml,rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from nav_msgs.msg import Odometry,Path as RosPath,OccupancyGrid
from sensor_msgs.msg import LaserScan,Imu
from rosgraph_msgs.msg import Clock
from rcl_interfaces.msg import Log
from geometry_msgs.msg import PoseWithCovarianceStamped,TwistStamped
from tf2_ros import Buffer,TransformListener
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image
HERE=Path('/home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/dataset_pilot');BENCH=HERE.parent;BASE=BENCH/'socnavgym_group_nav2';SOCIAL=BENCH.parent;REPO=SOCIAL.parent.parent;FIX=BENCH/'socnavgym_response_fix';COMMON=BENCH/'common_response'
sys.path.insert(0,str(SOCIAL/'gazebo_adapter'));from run import make_world,stop
sys.path.insert(0,str(BENCH/'gpu_runtime'));from runtime_guard import RuntimeGuard,gpu_preflight
sys.path.insert(0,str(BENCH));from shutdown_fix.ordered_stop import launch_environment,stop_server
from amcl_observer import observer_qos,publisher_evidence
from recording import Recording
from motion_prepare import prepare_camera,add_filter
from pair_recording import Pairs
from quiesce import stop_paused
from config_runtime import apply_runtime_config,changes,capture_live_parameters
original_stop=stop_server
def stop_server(guard):return stop_paused(guard,original_stop)
p=argparse.ArgumentParser();p.add_argument('--scenario',choices=['walking','standing'],required=True);p.add_argument('--seed',type=int,choices=[42,43],required=True);p.add_argument('--report-dir',type=Path,required=True);p.add_argument('--gui',action='store_true');a=p.parse_args();a.condition='people'
plan=json.loads((BASE/'protocol.json').read_text());plan['sim_budget']=90.;out=a.report_dir.resolve();out.mkdir(parents=True,exist_ok=False);runtime=HERE/'generated'/out.name;runtime.mkdir(exist_ok=False)
lock=(BENCH/'common_gazebo/generated/domain91.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
os.environ.update(ROS_DOMAIN_ID='91',GZ_PARTITION='dynamuhsim_group_nav2_'+str(os.getpid()),ROS_LOG_DIR=str(runtime/'ros_logs'),QT_QPA_PLATFORM='xcb',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();guard=RuntimeGuard(runtime);processes=[];files=[];node=None;frames=[];poses=[];states=[];scans=[];odom=[];amcl=[];commands=[];paths=[];costmaps=[];feedback=[];roslogs=[];tfrows=[];faults=[];latest={};counts=collections.Counter();recording=None;pairs=None;worker=None;gui_process=None;handle=None;phase='warmup';client=None;tfbuffer=None
result=dict(status='ERROR',condition=a.condition,scenario=a.scenario,seed=a.seed,model='socnavgym_initial_groups_v2',native_dt=.1,gui=a.gui,head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),runtime=str(runtime.relative_to(REPO)),robot_start=plan['start'],robot_goal=plan['goal'],sim_budget_s=plan['sim_budget'],response_variant='bounded_reactive_standing_v3' if a.scenario=='standing' else 'walking_response_v2',no_scripted_robot_velocity_publisher=True)
source=[Path(__file__),BASE/'worker.py',BASE/'fixture.py',BASE/'reactive_motion.py',BASE/'standing_projection.py',BASE/'config.yaml',HERE/'warehouse.launch.py',BASE/'protocol.json',FIX/'projection.py',FIX/'responsive_motion.py',FIX/'adapter.cc',FIX/'build/libresponse_adapter.so',COMMON/'model_worker.py',COMMON/'inputs/crossing_near.json',SOCIAL/'warehouse/nav2_warehouse.yaml',SOCIAL/'warehouse/warehouse.yaml',SOCIAL/'warehouse/warehouse.pgm',SOCIAL/'warehouse/warehouse.sdf',REPO/'src/custom_corridor/launch/nav2_corridor.launch.py',REPO/'src/custom_corridor/launch/publish_episode_initial_pose.py']
source += [BENCH/'socnavgym_warehouse_lifecycle'/n for n in ['kernel.py','motion.py','geometry.py','config.yaml']]+[BENCH/'socnavgym_lifecycle_fix/continuous_groups.py',SOCIAL/'vendor/SocNavGym/socnavgym/envs/socnavenv_v2.py',SOCIAL/'vendor/SocNavGym/socnavgym/envs/utils/human_human.py']
source += [HERE/n for n in ['recording.py','prepare.py','storage.yaml','qos.yaml','run.sh']] + [REPO/'src/custom_corridor/launch/corridor_tb3.launch.py',REPO/'src/custom_corridor/models/mecanum_robot/model.sdf',REPO/'src/custom_corridor/urdf/mecanum_robot.urdf']
result['source_sha256']={str(q.relative_to(REPO)):sha(q) for q in source};(out/'protocol.json').write_text(json.dumps(result,indent=2)+'\n')
def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
def simtime():return latest['clock'].clock.sec+latest['clock'].clock.nanosec/1e9 if 'clock' in latest else 0.
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def endpoint(msg):return dict(x=msg.position.x,y=msg.position.y,yaw=yaw(msg.orientation))
try:
    assert json.loads((BASE/'reports/reaction_gate.json').read_text())['status']=='PASS'
    assert json.loads((BASE/'reports/gate03/validation.json').read_text())['status']=='PASS'
    assert json.loads((BASE/'reports/offline05/validation.json').read_text())['status']=='PASS'
    assert json.loads((BENCH/'socnavgym_nav2_pilot/reports/empty01/validation.json').read_text())['actual_goal_reached']
    result['camera_runtime_corrections']=prepare_camera(runtime,REPO);result['rendering']=gpu_preflight(runtime);launch_env,result['shutdown_preload']=launch_environment(runtime)
    sockdir=tempfile.TemporaryDirectory(prefix='nav2v2_');socket_path=str(Path(sockdir.name)/'worker.sock');initial=[]
    if a.condition=='people':
        f=(runtime/'worker.log').open('w');files.append(f);cmd=[str(SOCIAL/'.venv-socnavgym/bin/python'),str(BASE/'worker.py'),'--runtime',str(runtime),'--socket',socket_path,'--scenario',a.scenario,'--seed',str(a.seed)];result['worker_command']=cmd
        worker=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(worker);deadline=time.monotonic()+90
        while not (runtime/'ready.json').exists():
            if worker.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Worker not ready')
            time.sleep(.1)
        ready=json.loads((runtime/'ready.json').read_text());initial=ready['humans'];result['ready']=ready;assert len(initial)==6 and ready['requires_start_gate']
    make_world(runtime,socket_path,initial,'walking');tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world');plugin=world.find("plugin[@name='social_experiment::Adapter']")
    if a.condition=='empty':world.remove(plugin)
    else:
        plugin.set('filename',str(FIX/'build/libresponse_adapter.so'));plugin.set('name','response_experiment::Adapter');ET.SubElement(plugin,'population').text='6';ET.SubElement(plugin,'planning_dt').text='.1'
    world.find('physics/max_step_size').text='.0025';world.find('physics/real_time_factor').text='1.0'
    for item in list(world):
        name=item.get('name','')
        if name.startswith('socnavgym_'):item.set('name',name.replace('socnavgym_','common_',1))
    observer=world.find("model[@name='common_observer']");observer.find('pose').text='-24 -5 12 0 1.17 1.57079632679';observer.find('.//topic').text='/common/observer';observer.find('.//update_rate').text='1';observer.find('.//image/width').text='640';observer.find('.//image/height').text='480'
    add_filter(world);ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode');config=yaml.safe_load((SOCIAL/'warehouse/nav2_warehouse.yaml').read_text());original=copy.deepcopy(config);config['amcl']['ros__parameters']['initial_pose'].update(x=-28.,y=0.,yaw=0.)
    changed=copy.deepcopy(config);changed['amcl']['ros__parameters']['initial_pose']=original['amcl']['ros__parameters']['initial_pose'];assert changed==original
    before_treatment=copy.deepcopy(config);apply_runtime_config(config);(out/'config_changes.json').write_text(json.dumps(changes(before_treatment,config),indent=2)+'\n')
    (runtime/'nav2_runtime.yaml').write_text(yaml.safe_dump(config,sort_keys=False));result['runtime_nav_params_sha256']=sha(runtime/'nav2_runtime.yaml');result['runtime_world_sha256']=sha(runtime/'warehouse.sdf')
    launch_env.update(NAV2_PILOT_WORLD=str(runtime/'warehouse.sdf'),NAV2_PILOT_PARAMS=str(runtime/'nav2_runtime.yaml'),DATASET_CAMERA_LAUNCH=str(runtime/'mecanum_camera.launch.py'),DATASET_ROBOT_SDF=str(runtime/'mecanum_camera.sdf'))
    rclpy.init();node=Node('dataset_pilot',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);gz=GzNode();tfbuffer=Buffer();tflistener=TransformListener(tfbuffer,node);recording=Recording(node,out);pairs=Pairs(node,out,lambda:phase)
    def receive(key,msg):
        counts[key]+=1;latest[key]=msg
        if key=='gt':
            value=json.loads(msg.data)
            if not value.get('adapter_ok'):faults.append(value);raise RuntimeError('Terminal adapter fault: '+value.get('error','unknown'))
            states.append(value);latest[key]=value
        elif key=='scan':scans.append(dict(time=stamp(msg),ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges],angle_min=msg.angle_min,angle_increment=msg.angle_increment,range_min=msg.range_min,range_max=msg.range_max))
        elif key=='odom':odom.append(dict(time=stamp(msg),**endpoint(msg.pose.pose)))
        elif key=='amcl':amcl.append(dict(time=stamp(msg),**endpoint(msg.pose.pose),covariance=list(msg.pose.covariance)))
        elif key=='cmd':commands.append(dict(time=stamp(msg),observed_sim=simtime(),vx=msg.twist.linear.x,vy=msg.twist.linear.y,wz=msg.twist.angular.z,phase=phase))
        elif key=='plan':paths.append(dict(time=stamp(msg),poses=[endpoint(p.pose) for p in msg.poses],phase=phase))
        elif key=='local_costmap':costmaps.append(dict(time=stamp(msg),frame=msg.header.frame_id,width=msg.info.width,height=msg.info.height,resolution=msg.info.resolution,origin=endpoint(msg.info.origin),data=base64.b64encode(zlib.compress(bytes((int(v)+256)%256 for v in msg.data))).decode(),phase=phase))
        elif key=='log' and msg.level>=30:roslogs.append(dict(time=simtime(),level=msg.level,node=msg.name,text=msg.msg))
    for key,topic,kind in [('gt','/common/ground_truth',String),('scan','/scan',LaserScan),('imu','/imu',Imu),('odom','/odom',Odometry),('clock','/clock',Clock),('amcl','/amcl_pose',PoseWithCovarianceStamped),('cmd','/cmd_vel',TwistStamped),('plan','/plan',RosPath),('local_costmap','/local_costmap/costmap',OccupancyGrid),('log','/rosout',Log)]:node.create_subscription(kind,topic,lambda msg,key=key:receive(key,msg),observer_qos('transient_local') if key=='amcl' else qos_profile_sensor_data)
    def scene(msg):
        entities={}
        for p in msg.pose:
            if p.name=='mecanum_robot' or p.name.startswith('common_') and p.name.endswith('_proxy'):entities[p.name]=endpoint(p)
        if entities:
            poses.append(dict(time=msg.header.stamp.sec+msg.header.stamp.nsec/1e9,poses=entities,phase=phase))
            recording.scene(msg.header.stamp.sec*1000000000+msg.header.stamp.nsec,entities,phase)
    def camera(msg):
        if phase=='active':frames.append((msg.header.stamp.sec+msg.header.stamp.nsec/1e9,msg.width,msg.height,msg.step,bytes(msg.data)))
    gz.subscribe(Pose_V,'/world/social_warehouse/pose/info',scene);gz.subscribe(GzImage,'/common/observer',camera)
    f=(out/'launch.log').open('w');files.append(f);pair_log=(out/'pair_bridge.log').open('w');files.append(pair_log);processes.append(subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','/proxy_check/on/depth@sensor_msgs/msg/Image[gz.msgs.Image'],stdout=pair_log,stderr=subprocess.STDOUT,start_new_session=True,env=launch_env));launch=subprocess.Popen(['ros2','launch',str(HERE/'warehouse.launch.py'),'gui:=false'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=launch_env);processes.append(launch)
    def capture_tf():
        if not poses:return
        try:
            m=tfbuffer.lookup_transform('map','base_footprint',rclpy.time.Time());o=tfbuffer.lookup_transform('map','odom',rclpy.time.Time());tfrows.append(dict(time=simtime(),map_base=dict(x=m.transform.translation.x,y=m.transform.translation.y,yaw=yaw(m.transform.rotation)),map_odom=dict(x=o.transform.translation.x,y=o.transform.translation.y,yaw=yaw(o.transform.rotation))))
        except Exception:pass
    def wait(predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if any(proc.poll() is not None for proc in processes):raise RuntimeError('Owned launch or worker exited')
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return True
        return False
    assert wait(lambda:all(counts[k]>=5 for k in ['scan','imu','odom','clock']) and counts['amcl']>0 and poses and 'mecanum_robot' in poses[-1]['poses'] and (a.condition=='empty' or len(states)>5),100),'Sensors/scene/localization missing'
    lifecycle=node.create_client(GetState,'/bt_navigator/get_state');client=ActionClient(node,NavigateToPose,'/navigate_to_pose');deadline=time.monotonic()+100;ready=False
    while time.monotonic()<deadline:
        if lifecycle.service_is_ready() and client.server_is_ready():
            f=lifecycle.call_async(GetState.Request())
            if wait(f.done,4) and f.result().current_state.id==3:ready=True;break
        wait(lambda:False,.1)
    assert ready,'bt_navigator not active';assert wait(lambda:counts['local_costmap']>0,10),'Local costmap missing';capture_tf();assert tfrows,'map->base TF missing'
    if a.gui:
        gui_log=(runtime/'gui.log').open('w');files.append(gui_log);gui_process=subprocess.Popen(['gz','sim','-g'],stdout=gui_log,stderr=subprocess.STDOUT,start_new_session=True,env=launch_env);processes.append(gui_process);wait(lambda:False,3.);result['gui_start_mode']='Separate GUI after server sensors/AMCL/Nav2 ready'
    assert wait(lambda:recording.ready() and pairs.ready(),30),'Full recorder/depth/calibration not ready'
    result['live_parameters']=capture_live_parameters(node,wait,out)
    recording.event('ready',round(simtime()*1e9),scenario=a.scenario,seed=a.seed)
    result['amcl_publishers']=publisher_evidence(node);result['cmd_vel_publishers_before_goal']=[dict(node=e.node_name,namespace=e.node_namespace) for e in node.get_publishers_info_by_topic('/cmd_vel')];assert all(e['node'] in ['controller_server','behavior_server'] for e in result['cmd_vel_publishers_before_goal'])
    robot=poses[-1]['poses']['mecanum_robot'];assert math.hypot(robot['x']+28,robot['y'])<.02,'Robot moved in warmup'
    if a.condition=='people':
        assert max(math.hypot(poses[-1]['poses'][f'common_{h["id"]}_proxy']['x']-h['x'],poses[-1]['poses'][f'common_{h["id"]}_proxy']['y']-h['y']) for h in initial)<.01,'Humans moved before start gate'
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=latest['clock'].clock;goal.pose.pose.position.x=-20.;goal.pose.pose.position.y=0.;goal.pose.pose.orientation.w=1.
    def feedback_cb(msg):
        f=msg.feedback;feedback.append(dict(time=simtime(),distance_remaining=f.distance_remaining,recoveries=f.number_of_recoveries))
    future=client.send_goal_async(goal,feedback_callback=feedback_cb);assert wait(future.done,10),'No goal response';handle=future.result();assert handle.accepted,'Goal rejected'
    start=simtime();phase='active';recording.event('goal_accepted',round(start*1e9),goal_world=[-20.,0.,0.]);(runtime/'start_gate').write_text(str(start));result['goal_start_sim']=start;result['before_goal_world_pose']=robot;pending=handle.get_result_async();wall_start=time.monotonic();deadline=wall_start+max(240,plan['sim_budget']*6);last_tf=-1.;timed_out=False
    while not pending.done():
        wait(pending.done,.04)
        if simtime()-last_tf>.2:capture_tf();last_tf=simtime()
        if simtime()-start>=plan['sim_budget'] or time.monotonic()>deadline:
            timed_out=True;cancel=handle.cancel_goal_async();wait(cancel.done,5);break
    result['action_end_sim']=simtime();result['active_sim_seconds']=simtime()-start;result['wall_seconds']=time.monotonic()-wall_start;result['action_status']=None if timed_out else pending.result().status
    result['navigation_status']='TIMEOUT' if timed_out else {4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(result['action_status'],'UNKNOWN')
    if not timed_out:
        payload=pending.result().result;result['action_error_code']=getattr(payload,'error_code',None);result['action_error_msg']=getattr(payload,'error_msg',None)
    wait(lambda:False,.4);capture_tf();phase='finished';robot=poses[-1]['poses']['mecanum_robot'];result['final_world_pose']=robot;result['goal_xy_error_world_m']=math.hypot(robot['x']+20,robot['y']);result['goal_yaw_error_world_rad']=abs(math.atan2(math.sin(robot['yaw']),math.cos(robot['yaw'])));result['actual_goal_reached']=result['action_status']==4 and result['goal_xy_error_world_m']<=.2 and result['goal_yaw_error_world_rad']<=.25
    recording.event('action_end',round(result['action_end_sim']*1e9),navigation_status=result['navigation_status'],actual_goal_reached=result['actual_goal_reached'])
    tail_start=simtime();phase='label_tail';assert wait(lambda:simtime()-tail_start>=3.2,45),'Future label tail incomplete'
    recording.event('observation_end',round(simtime()*1e9))
    wait(lambda:False,.5)
    result['counts']=dict(counts);result['experiment']='Conservative Nav2 footprint/inflation/speed pilot; native sensors/standingv3 unchanged';result['cmd_vel_publishers_after_goal']=[dict(node=e.node_name,namespace=e.node_namespace) for e in node.get_publishers_info_by_topic('/cmd_vel')];result['status']='MEASURED';result['amcl_final_pose']=endpoint(latest['amcl'].pose.pose)
except Exception as error:result['error']=str(error);traceback.print_exc()
finally:
    phase='finished'
    if result['status']=='ERROR' and handle:
        try:handle.cancel_goal_async()
        except Exception:pass
    if pairs:result['depth_pairs']=pairs.close()
    if recording:
        try:result['recording']=recording.close()
        except Exception as error:result['recording_error']=str(error)
    if gui_process:
        stop(gui_process);result['gui_returncode']=gui_process.returncode
    try:result['server_shutdown']=stop_server(guard)
    except Exception as error:result['shutdown_error']=str(error)
    for proc in reversed(processes):stop(proc)
    if node:
        try:node.destroy_node();rclpy.shutdown()
        except Exception as error:result['ros_teardown_error']=str(error)
    result['cleanup']=guard.cleanup()
    for f in files:f.close()
    if 'sockdir' in globals():sockdir.cleanup()
    result['faults']=faults;result['gazebo_clean_exit']='[INFO] [gz-1]: process has finished cleanly' in (out/'launch.log').read_text(errors='replace') if (out/'launch.log').exists() else False
    for name,rows in [('gazebo_poses',poses),('ground_truth',states),('scans',scans),('odom',odom),('amcl',amcl),('commands',commands),('plans',paths),('local_costmaps',costmaps),('feedback',feedback),('roslogs',roslogs),('tf',tfrows)]:
        with (out/(name+'.jsonl')).open('w') as f:
            for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')
    if frames:
        images=[Image.frombytes('RGB',(w,h),data,'raw','RGB',stride).quantize(colors=96) for t,w,h,stride,data in frames];times=[frame[0] for frame in frames]
        for i,label in [(0,'initial'),(len(images)//2,'middle'),(len(images)-1,'final')]:images[i].convert('RGB').save(out/(label+'.png'))
        durations=[max(20,round((b-a)*250/10)*10) for a,b in zip(times,times[1:])]+[250];images[0].save(out/'camera_preview_4x.gif',save_all=True,append_images=images[1:],duration=durations,loop=0);result['media']=dict(source='Live Gazebo observer RGB, not AI',frame_times=times,playback_speed=4,gif_duration_ms=sum(durations))
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
raise SystemExit(0 if result['status']=='MEASURED' else 1)
