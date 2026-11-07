"""Causal OBS / explicitly unavailable ACT fields / separate Gazebo GT, TASK03.

Source banks are written once. Windows carry indices, not copied sensor tensors.
No detector, contact event, candidate action or simulator state is synthesized.
"""
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from dynamuhsim.analysis.mcap_records import McapFile
from dynamuhsim.datasets.normalization import rotation

NS=1_000_000_000
STEP=100_000_000
HISTORY_STEPS=20
FUTURE_STEPS=30
TASK_FIELDS = dict(SENSOR_FACTUAL_WM='sensor_factual_valid',
    PAST_ACTION_FACTUAL_WM='past_action_factual_valid',
    FUTURE_ACTION_CONDITIONED_WM='future_recorded_action_valid',
    SEMANTIC_OBSERVED_TRACK_WM='semantic_observed_tracks_valid',COUNTERFACTUAL_WM='counterfactual_valid')
TOPICS={'/camera/depth/image_rect_raw','/camera/camera_info','/scan','/odom','/imu',
        '/tf','/tf_static','/cmd_vel','/clock','/dataset/scene_poses','/dataset/events','/map'}
MAX_AGE=dict(depth=150_000_000,camera_info=150_000_000,lidar=300_000_000,
             odom=52_500_000,imu=50_000_000,tf_map_odom=600_000_000,
             tf_odom_base=105_000_000,control=100_000_000)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def causal(stamp_ns,record_ns,queries,max_age_ns,publish_ns=None):
    """Newest source sample whose source AND recorder-availability time are past.

    Stable ties select the last publication. A delayed old sample cannot replace
    a newer one, and an unavailable sample always returns index -1.
    """
    stamp=np.asarray(stamp_ns,dtype=np.int64);record=np.asarray(record_ns,dtype=np.int64)
    queries=np.asarray(queries,dtype=np.int64);out=np.full(queries.shape,-1,dtype=np.int64)
    if not len(stamp):return out,np.zeros(queries.shape,dtype=bool)
    publish=np.asarray(publish_ns if publish_ns is not None else np.arange(len(stamp)),dtype=np.int64)
    available=np.maximum(stamp,record);order=np.argsort(available,kind='stable');prefix=[];best=None
    for i in order:
        if best is None or (stamp[i],publish[i],i)>(stamp[best],publish[best],best):best=int(i)
        prefix.append(best)
    j=np.searchsorted(available[order],queries,side='right')-1;has=j>=0
    out[has]=np.asarray(prefix)[j[has]]
    good=has.copy();good[has]&=(queries[has]-stamp[out[has]]<=max_age_ns)
    out[~good]=-1
    return out,good


def time_window(anchor_index,n):
    history=np.arange(anchor_index-HISTORY_STEPS,anchor_index+1,dtype=np.int64)
    future=np.arange(anchor_index+1,anchor_index+FUTURE_STEPS+1,dtype=np.int64)
    return np.where((history>=0)&(history<n),history,-1),np.where((future>=0)&(future<n),future,-1)


def transform(xyz,quat):
    value=np.eye(4);value[:3,:3]=rotation(quat);value[:3,3]=xyz
    return value


def gather(values,indices,mask=None):
    indices=np.asarray(indices,dtype=np.int64);good=indices>=0
    if mask is not None:good&=np.asarray(mask,dtype=bool)
    shape=(*indices.shape,*values.shape[1:])
    out=np.full(shape,np.nan,dtype=values.dtype) if np.issubdtype(values.dtype,np.floating) else np.zeros(shape,dtype=values.dtype)
    if good.any():out[good]=values[indices[good]]
    return out


def check_export_gate(audit_path):
    audit=json.loads(Path(audit_path).read_text())
    if audit['verdict'] not in ['PARTIAL','RAW_CAPTURE_PASS']:raise ValueError('TASK02 did not prove export prerequisites')
    failures=[k for k,v in audit['gates'].items() if v is not True and k!='embedded_schemas']
    if failures:raise ValueError('TASK02 prerequisites not proven: '+', '.join(failures))
    used_types={audit['topic_statistics'][t]['type'] for t in TOPICS}
    missing_used=set(audit.get('missing_embedded_schemas',[]))&used_types
    if missing_used:raise ValueError('An exported topic lacks embedded schema: '+str(sorted(missing_used)))
    for t in TOPICS:
        s=audit['topic_statistics'][t]
        if s['status']!='MEASURED' or s['decoded']!=s['count']:raise ValueError('Unverified required export topic '+t)
    for p,digest in audit['input_sha256'].items():
        if sha(p)!=digest:raise ValueError('Audited input changed: '+p)
    manifest=json.loads((Path(audit['run_dir'])/'manifest.json').read_text());plan=manifest['plan']
    if (plan.get('human_policy'),plan.get('scenario'),plan.get('seed'))!=('crowd_orca_plus','crossing',42):
        raise ValueError('Only the TASK01 ORCA+/crossing/42 episode is authorized')
    return audit


def decode_sources(run):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    values=defaultdict(list);stamp=defaultdict(list);record=defaultdict(list);publish=defaultdict(list)
    frames=defaultdict(set);statics={};scenes=[];events=[];controls=[];map_msg=None;classes={}
    for path in sorted((run/'episode/raw_bag').glob('*.mcap')):
        for row in McapFile(path).messages():
            topic=row['topic']
            if topic not in TOPICS:continue
            typename=row['schema']['name']
            if typename not in classes:classes[typename]=get_message(typename)
            msg=deserialize_message(row['payload'],classes[typename])
            def add(key,t,value,frame=None):
                values[key].append(value);stamp[key].append(int(t));record[key].append(row['record_ns']);publish[key].append(row['publish_ns'])
                if frame is not None:frames[key].add(frame)
            def ts(s):return int(s.sec)*NS+int(s.nanosec)
            if topic=='/camera/depth/image_rect_raw':
                if msg.encoding!='32FC1':raise ValueError('Unsupported depth encoding')
                data=np.frombuffer(bytes(msg.data),dtype='>f4' if msg.is_bigendian else '<f4').reshape(msg.height,msg.step//4)[:,:msg.width]
                add('depth',ts(msg.header.stamp),data.astype(np.float32,copy=True),msg.header.frame_id)
            elif topic=='/camera/camera_info':
                add('camera_info',ts(msg.header.stamp),dict(k=list(msg.k),p=list(msg.p),d=list(msg.d),width=msg.width,height=msg.height),msg.header.frame_id)
            elif topic=='/scan':
                add('lidar',ts(msg.header.stamp),dict(ranges=list(msg.ranges),meta=[msg.angle_min,msg.angle_max,msg.angle_increment,msg.range_min,msg.range_max,msg.scan_time,msg.time_increment]),msg.header.frame_id)
            elif topic=='/odom':
                p=msg.pose.pose;q=p.orientation;v=msg.twist.twist
                add('odom',ts(msg.header.stamp),dict(pose=[p.position.x,p.position.y,p.position.z,q.x,q.y,q.z,q.w],twist=[v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]),msg.header.frame_id)
                frames['odom_child'].add(msg.child_frame_id)
            elif topic=='/imu':
                q=msg.orientation;w=msg.angular_velocity;a=msg.linear_acceleration
                add('imu',ts(msg.header.stamp),dict(value=[q.x,q.y,q.z,q.w,w.x,w.y,w.z,a.x,a.y,a.z],cov=[list(msg.orientation_covariance),list(msg.angular_velocity_covariance),list(msg.linear_acceleration_covariance)]),msg.header.frame_id)
            elif topic=='/clock':add('clock',ts(msg.clock),ts(msg.clock))
            elif topic=='/cmd_vel':
                if msg.header.frame_id!='base_footprint':raise ValueError('Unexpected command frame '+msg.header.frame_id)
                controls.append(dict(control_id=len(controls),record_type='observed_published_command',
                    source_stamp_ns=ts(msg.header.stamp),record_stamp_ns=row['record_ns'],publish_wall_ns=row['publish_ns'],
                    frame='base_footprint',value=[msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z],
                    requested_internal_available=False,executed_action=False))
            elif topic in ['/tf','/tf_static']:
                for tr in msg.transforms:
                    edge=(tr.header.frame_id,tr.child_frame_id);xyz=tr.transform.translation;q=tr.transform.rotation
                    matrix=transform([xyz.x,xyz.y,xyz.z],[q.x,q.y,q.z,q.w])
                    if topic=='/tf_static':statics[edge]=dict(matrix=matrix,stamp_ns=ts(tr.header.stamp),record_ns=row['record_ns'])
                    elif edge==('map','odom'):add('tf_map_odom',ts(tr.header.stamp),matrix)
                    elif edge==('odom','base_footprint'):add('tf_odom_base',ts(tr.header.stamp),matrix)
            elif topic=='/dataset/scene_poses':scenes.append(json.loads(msg.data))
            elif topic=='/dataset/events':events.append(dict(**json.loads(msg.data),record_stamp_ns=row['record_ns']))
            elif topic=='/map':map_msg=msg
    return dict(values=values,stamp=stamp,record=record,publish=publish,frames=frames,statics=statics,scenes=scenes,events=events,controls=controls,map=map_msg)


def observation_banks(raw,grid):
    v=raw['values'];bank={}
    for key in v:
        bank[key+'_stamp_ns']=np.asarray(raw['stamp'][key],dtype=np.int64)
        bank[key+'_record_ns']=np.asarray(raw['record'][key],dtype=np.int64)
        bank[key+'_publish_wall_ns']=np.asarray(raw['publish'][key],dtype=np.int64)
    bank['depth']=np.stack(v['depth']);bank['depth_pixel_available']=np.isfinite(bank['depth'])&(bank['depth']>0)
    bank['camera_k']=np.asarray([r['k'] for r in v['camera_info']],dtype=np.float64).reshape(-1,3,3)
    bank['camera_p']=np.asarray([r['p'] for r in v['camera_info']],dtype=np.float64).reshape(-1,3,4)
    bank['camera_d']=np.asarray([r['d'] for r in v['camera_info']],dtype=np.float64)
    bank['lidar']=np.asarray([r['ranges'] for r in v['lidar']],dtype=np.float32)
    bank['lidar_metadata']=np.asarray([r['meta'] for r in v['lidar']],dtype=np.float64)
    bank['lidar_ray_available']=np.isfinite(bank['lidar'])&(bank['lidar']>=bank['lidar_metadata'][:,3,None])&(bank['lidar']<=bank['lidar_metadata'][:,4,None])
    bank['odom_pose']=np.asarray([r['pose'] for r in v['odom']],dtype=np.float64)
    bank['odom_twist_body']=np.asarray([r['twist'] for r in v['odom']],dtype=np.float64)
    bank['imu']=np.asarray([r['value'] for r in v['imu']],dtype=np.float64)
    bank['imu_covariance']=np.asarray([r['cov'] for r in v['imu']],dtype=np.float64)
    bank['imu_field_available']=bank['imu_covariance'][:,:,0]>=0
    bank['imu_covariance_available']=bank['imu_field_available']&np.any(bank['imu_covariance']!=0,axis=2)
    for key in ['tf_map_odom','tf_odom_base']:bank[key]=np.asarray(v[key],dtype=np.float64)
    edges=sorted(raw['statics']);bank['static_parent']=np.asarray([e[0] for e in edges],dtype='U64');bank['static_child']=np.asarray([e[1] for e in edges],dtype='U64')
    bank['static_transform']=np.asarray([raw['statics'][e]['matrix'] for e in edges])
    bank['static_record_ns']=np.asarray([raw['statics'][e]['record_ns'] for e in edges],dtype=np.int64)
    bank['grid_query_ns']=grid
    # No GT IDs, positions, visibility or detector outputs enter this bank.
    bank['observed_tracks']=np.empty((len(grid),0,5),dtype=np.float32)
    bank['observed_track_ids']=np.empty((0,),dtype=np.int64)
    bank['observed_tracks_available']=np.zeros(len(grid),dtype=bool)
    bank['observed_occlusion_known']=np.zeros(len(grid),dtype=bool)
    bank['observed_tracks_stamp_ns']=np.full(len(grid),-1,dtype=np.int64)
    return bank


def disc_occupancy(xy,radii,height,width,origin,resolution):
    grid=np.zeros((height,width),dtype=bool)
    for (x,y),radius in zip(xy,radii):
        lo=np.floor((np.array([x-radius,y-radius])-origin)/resolution).astype(int)-1
        hi=np.ceil((np.array([x+radius,y+radius])-origin)/resolution).astype(int)+1
        x0,x1=max(0,lo[0]),min(width,hi[0]);y0,y1=max(0,lo[1]),min(height,hi[1])
        if x1<=x0 or y1<=y0:continue
        xs=origin[0]+(np.arange(x0,x1)+.5)*resolution;ys=origin[1]+(np.arange(y0,y1)+.5)*resolution
        dx=np.maximum(np.abs(xs-x)-resolution/2,0);dy=np.maximum(np.abs(ys-y)-resolution/2,0)
        grid[y0:y1,x0:x1]|=dy[:,None]**2+dx[None,:]**2<=radius**2
    return grid


def gt_banks(raw,grid,run):
    manifest=json.loads((run/'manifest.json').read_text());radius=manifest['result']['ready']['metadata']['human_radius']
    scenes=sorted(raw['scenes'],key=lambda r:r['stamp_ns']);times=np.asarray([r['stamp_ns'] for r in scenes],dtype=np.int64)
    ids=sorted({h['id'] for row in scenes for h in row['humans']});n=len(grid);h=len(ids)
    native=ET.parse(run/'runtime/warehouse.sdf').getroot()
    for hid in ids:
        cylinders=native.findall(f".//model[@name='common_{hid}_proxy']/link/collision/geometry/cylinder/radius")
        if len(cylinders)!=1 or not math.isclose(float(cylinders[0].text),radius,abs_tol=1e-9):
            raise ValueError('Occupancy radius not verified against native human collision geometry: '+str(hid))
    # GT uses the last measured Gazebo pose at/before each query, age <=25 ms.
    idx,mask=causal(times,times,grid,25_000_000)
    robot=np.full((n,3),np.nan);humans=np.full((n,h,3),np.nan);human_mask=np.zeros((n,h),dtype=bool)
    source=np.full(n,-1,dtype=np.int64)
    for i in np.flatnonzero(mask):
        row=scenes[idx[i]];source[i]=row['stamp_ns'];robot[i]=[row['robot'][k] for k in ['x','y','yaw']]
        present={v['id']:v for v in row['humans']}
        for j,hid in enumerate(ids):
            if hid in present:humans[i,j]=[present[hid][k] for k in ['x','y','yaw']];human_mask[i,j]=True
    robot_velocity=np.full((n,3),np.nan);human_velocity=np.full((n,h,3),np.nan);velocity_mask=np.zeros(n,dtype=bool)
    for i in np.flatnonzero(mask):
        previous=int(np.searchsorted(times,source[i]-100_000_000,side='right'))-1
        if previous<0 or not 0<source[i]-times[previous]<=125_000_000:continue
        # Refuse differentiation across a missing raw-label segment.
        if np.max(np.diff(times[previous:idx[i]+1]),initial=0)>25_000_000:continue
        before=scenes[previous];elapsed=(source[i]-times[previous])/NS
        p=np.array([before['robot'][k] for k in ['x','y','yaw']]);delta=robot[i]-p;delta[2]=math.remainder(delta[2],2*math.pi)
        robot_velocity[i]=delta/elapsed;older={v['id']:v for v in before['humans']}
        for j,hid in enumerate(ids):
            if human_mask[i,j] and hid in older:
                delta=humans[i,j]-np.array([older[hid][k] for k in ['x','y','yaw']]);delta[2]=math.remainder(delta[2],2*math.pi);human_velocity[i,j]=delta/elapsed
        velocity_mask[i]=True
    m=raw['map']
    if m is None or m.header.frame_id!='map':raise ValueError('Verified map missing for occupancy GT')
    original=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width)
    from PIL import Image
    import yaml
    meta=yaml.safe_load((run/'assets/warehouse.yaml').read_text())
    if not np.allclose([m.info.origin.position.x,m.info.origin.position.y],meta['origin'][:2]) or abs(m.info.resolution-meta['resolution'])>1e-8 or abs(m.info.origin.orientation.w)<1-1e-8:
        raise ValueError('Recorded map coordinates differ from frozen Gazebo-validated coordinates')
    expected=np.flipud(np.asarray(Image.open(run/'assets/warehouse.pgm')))
    if original.shape!=expected.shape or not np.array_equal(original>=50,expected==0):raise ValueError('Recorded map differs from frozen Gazebo-validated map')
    factor=2;resolution=float(m.info.resolution)*factor;height,width=m.info.height//factor,m.info.width//factor
    if original.shape!=(height*factor,width*factor):raise ValueError('Occupancy dimensions not divisible by scale')
    static=(original>=50).reshape(height,factor,width,factor).any(axis=(1,3))
    known=(original>=0).reshape(height,factor,width,factor).all(axis=(1,3))
    origin=np.array([m.info.origin.position.x,m.info.origin.position.y],dtype=np.float64)
    packed=np.zeros((n,(height*width+7)//8),dtype=np.uint8);occ_mask=mask&human_mask.all(axis=1)
    for i in np.flatnonzero(occ_mask):packed[i]=np.packbits(disc_occupancy(humans[i,:,:2],[radius]*h,height,width,origin,resolution).ravel(),bitorder='little')
    return dict(query_ns=grid,source_stamp_ns=source,source_age_ns=np.where(mask,grid-source,-1),
        robot_world=robot,robot_available=mask,human_ids=np.asarray(ids,dtype=np.int64),human_world=humans,human_available=human_mask,
        robot_velocity_world=robot_velocity,human_velocity_world=human_velocity,
        robot_velocity_available=velocity_mask,human_velocity_available=np.isfinite(human_velocity).all(axis=2),
        human_occupancy_packed=packed,occupancy_available=occ_mask,static_occupancy_packed=np.packbits(static.ravel(),bitorder='little'),
        static_occupancy_known_packed=np.packbits(known.ravel(),bitorder='little'),occupancy_shape=np.array([height,width],dtype=np.int64),
        occupancy_origin_xy=origin,occupancy_resolution_m=np.array(resolution),human_radius_m=np.full(h,radius),
        contact_truth=np.full(n,-1,dtype=np.int8),contact_available=np.zeros(n,dtype=bool))


def alignment_refs(raw,grid):
    refs={};masks={}
    camera={t:i for i,t in enumerate(raw['stamp']['camera_info'])}
    for key in ['depth','lidar','odom','imu','tf_map_odom','tf_odom_base']:
        availability=np.asarray(raw['record'][key],dtype=np.int64).copy()
        if key=='depth':
            for i,t in enumerate(raw['stamp']['depth']):
                availability[i]=max(availability[i],raw['record']['camera_info'][camera[t]]) if t in camera else np.iinfo(np.int64).max
        refs[key],masks[key]=causal(raw['stamp'][key],availability,grid,MAX_AGE[key],raw['publish'][key])
    refs['camera_info']=np.array([camera.get(raw['stamp']['depth'][i],-1) if i>=0 else -1 for i in refs['depth']],dtype=np.int64)
    masks['camera_info']=masks['depth']&(refs['camera_info']>=0)
    c=raw['controls'];refs['published_control'],masks['published_control']=causal([r['source_stamp_ns'] for r in c],[r['record_stamp_ns'] for r in c],grid,MAX_AGE['control'],[r['publish_wall_ns'] for r in c])
    return refs,masks


def make_windows(grid,refs,masks,gt,static_available,required_gt=('robot_available','human_available','occupancy_available')):
    windows=[];rejections=Counter()
    for anchor in range(len(grid)):
        hist,future=time_window(anchor,len(grid));reasons=[]
        if (hist<0).any():reasons.append('insufficient_history_time')
        if (future<0).any():reasons.append('insufficient_future_time')
        obs_refs={};obs_masks={}
        for key in refs:
            ids=np.full(len(hist),-1,dtype=np.int64);good=hist>=0
            ids[good]=refs[key][hist[good]];valid=good.copy();valid[good]&=masks[key][hist[good]];ids[~valid]=-1
            obs_refs[key]=ids.tolist();obs_masks[key]=valid.tolist()
            if key!='published_control' and not valid.all():reasons.append('missing_or_stale_history_'+key)
        if np.any(hist<0) or np.any(grid[np.maximum(hist,0)]<static_available):reasons.append('history_static_tf_not_yet_available')
        valid_future=future>=0
        for key in required_gt:
            good=valid_future.copy()
            if good.any():good[good]&=gt[key][future[good]].reshape(np.count_nonzero(good),-1).all(axis=1)
            if not good.all():reasons.append('missing_future_gt_'+key)
        rejections.update(set(reasons))
        windows.append(dict(sample_id=anchor,anchor_ns=int(grid[anchor]),valid=not reasons,reasons=sorted(set(reasons)),
            history_query_ns=(int(grid[anchor])+np.arange(-HISTORY_STEPS,1)*STEP).tolist(),
            future_query_ns=(int(grid[anchor])+np.arange(1,FUTURE_STEPS+1)*STEP).tolist(),
            obs=dict(file='observations.npz',indices={k:v for k,v in obs_refs.items() if k!='published_control'},availability={k:v for k,v in obs_masks.items() if k!='published_control'},
                     observed_tracks_available=False,observed_occlusion_known=False),
            act=dict(file='controls.jsonl',past_published_control_ids=obs_refs['published_control'],past_published_control_available=obs_masks['published_control'],
                     past_requested_commands_available=False,action_condition_available=False,action_condition=None,
                     note='No distinct requested-command stream or candidate sequence was recorded. Future executed motion is GT only.'),
            gt=dict(file='human_states_gt.npz',future_indices=future.tolist(),contact_truth_available=False),
            complete_requested_obs_act=False))
    return windows,dict(rejections)


def kinematics(run):
    from dynamuhsim.simulation.robot_geometry import read_model
    model=read_model(run/'runtime/mecanum_camera.sdf');p=model.find("plugin[@name='gz::sim::systems::MecanumDrive']")
    return dict(model='Gazebo MecanumDrive / omnidirectional',columns=['vx','vy','wz'],units=['m/s','m/s','rad/s'],
        command_frame='base_footprint',wheel_radius_m=float(p.findtext('wheel_radius')),
        wheel_separation_m=float(p.findtext('wheel_separation')),wheelbase_m=float(p.findtext('wheelbase')),
        vy_retained=True,unicycle_projection=False)


def export_episode(audit_path,output):
    audit=check_export_gate(audit_path);run=Path(audit['run_dir']);output=Path(output)
    if output.exists():raise FileExistsError('Preserve existing export: '+str(output))
    output.mkdir(parents=True,exist_ok=False)
    raw=decode_sources(run)
    end=min(audit['topic_statistics']['/camera/depth/image_rect_raw']['clocks']['source_sim']['end_ns'],audit['topic_statistics']['/dataset/scene_poses']['clocks']['source_sim']['end_ns'])
    grid=np.arange(0,end//STEP*STEP+1,STEP,dtype=np.int64)
    obs=observation_banks(raw,grid);gt=gt_banks(raw,grid,run);refs,masks=alignment_refs(raw,grid)
    statics_needed=[('world','map'),('base_footprint','base_link'),('base_link','base_scan'),('base_link','imu_link'),('base_link','camera_link'),('camera_link','camera_optical_frame')]
    static_available=max(raw['statics'][e]['record_ns'] for e in statics_needed)
    windows,rejections=make_windows(grid,refs,masks,gt,static_available)
    validate_no_leakage(windows,obs,raw['controls'])
    np.savez_compressed(output/'observations.npz',**obs);np.savez_compressed(output/'human_states_gt.npz',**gt)
    with (output/'controls.jsonl').open('x') as f:
        metadata=dict(record_type='metadata',kinematics=kinematics(run),past_requested_commands='NOT_MEASURED',
                      action_condition='NOT_MEASURED',source='/cmd_vel (observed publications only)',future_executed_actions_exported_as_candidates=False)
        f.write(json.dumps(metadata)+'\n')
        for row in raw['controls']:f.write(json.dumps(row,allow_nan=False)+'\n')
        for row in raw['events']:
            if row['event']=='goal_accepted':f.write(json.dumps(dict(record_type='declared_goal_event',source='/dataset/events',event=row,raw_action_service_request_available=False))+'\n')
    with (output/'wm_index.jsonl').open('x') as f:
        for row in windows:f.write(json.dumps(row,allow_nan=False)+'\n')
    valid=[w for w in windows if w['valid']]
    report=dict(schema='task03_verified_episode_export/1',status='PARTIAL_EXPORT',input_raw_verdict=audit['verdict'],
        export_gate='All used modality/type/hash/TF/GT gates proven; missing embedded optional action-status schema is excluded, never relabelled as RAW_CAPTURE_PASS.',
        run_dir=str(run),audit_path=str(Path(audit_path).resolve()),audit_sha256=sha(audit_path),input_sha256=audit['input_sha256'],
        history_s=2.,future_s=3.,sampling_hz=10.,history_samples=21,future_samples=30,
        windows_total=len(windows),windows_valid=len(valid),windows_invalid=len(windows)-len(valid),rejection_counts_nonexclusive=rejections,
        validity_definition='Core sensor + causal TF history and measured future robot/human/occupancy GT. Unavailable observed tracks, requested commands, action-condition and contact truth remain explicitly masked optional fields.',
        complete_requested_obs_act_windows=0,training_ready=False,
        windows_with_full_past_published_control=sum(all(w['act']['past_published_control_available']) for w in valid),
        unavailable=['observed_track_detector','observed_occlusion_labels','distinct_requested_command_stream','candidate_action_condition','contact_truth'],
        no_leakage_verified=True,no_interpolation=True,gt_resampling='Last actual Gazebo pose at/before query, age <=25ms; masks reject missing/gaps. No synthetic snapshots.',
        max_age_ns=MAX_AGE,kinematics=kinematics(run),
        observation_shapes={k:list(v.shape) for k,v in obs.items()},gt_shapes={k:list(v.shape) for k,v in gt.items()},
        sensor_frames={k:len(obs[k+'_stamp_ns']) for k in ['depth','lidar','odom','imu','camera_info']},
        source_element_availability=dict(depth_valid_pixels=int(obs['depth_pixel_available'].sum()),
            depth_total_pixels=int(obs['depth_pixel_available'].size),lidar_valid_rays=int(obs['lidar_ray_available'].sum()),
            lidar_total_rays=int(obs['lidar_ray_available'].size),observed_tracks=0,
            imu_field_available_counts=obs['imu_field_available'].sum(axis=0).tolist(),
            imu_covariance_available_counts=obs['imu_covariance_available'].sum(axis=0).tolist()),
        gt_available_grid_rows=int(gt['robot_available'].sum()),human_tracks=len(gt['human_ids']),
        recorded_frames={k:sorted(v) for k,v in raw['frames'].items()},
        valid_anchor_start_ns=valid[0]['anchor_ns'] if valid else None,valid_anchor_end_ns=valid[-1]['anchor_ns'] if valid else None,
        example_sample_id=valid[0]['sample_id'] if valid else None,large_sensor_arrays_duplicated_per_window=False,
        input_unchanged=all(sha(p)==h for p,h in audit['input_sha256'].items()),
        source_sha256={str(Path(__file__).resolve()):sha(__file__)})
    if not report['input_unchanged']:raise ValueError('Input changed during export')
    write_schema(output,report)
    report['output_sha256']={p.name:sha(p) for p in output.iterdir() if p.is_file()}
    with (output/'export_report.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    return report


def validate_no_leakage(windows,obs,controls):
    if any(k in obs for k in ['human_world','robot_world','human_ids','human_occupancy_packed','contact_truth']):
        raise ValueError('GT fields must not enter OBS')
    for w in windows:
        anchor=w['anchor_ns']
        for key,indices in w['obs']['indices'].items():
            for i,ok,query in zip(indices,w['obs']['availability'][key],w['history_query_ns']):
                if not ok:
                    if i!=-1:raise ValueError('Unavailable reference must be -1')
                    continue
                if i<0 or obs[key+'_stamp_ns'][i]>query or obs[key+'_record_ns'][i]>query or query>anchor:
                    raise ValueError('OBS future leakage: '+key)
        for i,ok,query in zip(w['act']['past_published_control_ids'],w['act']['past_published_control_available'],w['history_query_ns']):
            if ok and (controls[i]['source_stamp_ns']>query or controls[i]['record_stamp_ns']>query or query>anchor):raise ValueError('ACT future leakage')
        if w['future_query_ns'][0]!=anchor+STEP or w['future_query_ns'][-1]!=anchor+3*NS:raise ValueError('Future off-by-one')
    if obs['observed_tracks'].shape[1]!=0 or obs['observed_tracks_available'].any():raise ValueError('No observed detector was recorded; GT tracks must not enter OBS')


def load_sample(directory,sample_id,allow_invalid=False,*,recovery_dir=None,task=None):
    directory=Path(directory)
    receipt=directory/'export_report.json'
    if receipt.exists() and json.loads(receipt.read_text()).get('schema')=='task03_corrected_export/1':
        return load_factual_sample(directory,sample_id,task or 'SENSOR_FACTUAL_WM',allow_invalid)
    if task is not None:raise ValueError('Named factual tasks require a corrected contract directory')
    if recovery_dir is not None:
        from .episode_recovery import load_recovered_sample
        return load_recovered_sample(recovery_dir,sample_id,allow_invalid,expected_base=directory)
    directory=Path(directory);index=next(json.loads(s) for s in (directory/'wm_index.jsonl').read_text().splitlines() if json.loads(s)['sample_id']==sample_id)
    if not index['valid'] and not allow_invalid:raise ValueError('Invalid window: '+str(index['reasons']))
    return _load_index_sample(directory,index)


def _load_index_sample(directory,index):
    controls=[json.loads(s) for s in (directory/'controls.jsonl').read_text().splitlines()];controls={r['control_id']:r for r in controls if r['record_type']=='observed_published_command'}
    act=np.full((21,3),np.nan);act_mask=np.asarray(index['act']['past_published_control_available'],dtype=bool)
    for j,(i,ok) in enumerate(zip(index['act']['past_published_control_ids'],act_mask)):
        if ok:act[j]=controls[i]['value']
    result=dict(index=index,obs={},act=dict(past_published_commands=act,availability=act_mask,
                past_requested_commands=None,action_condition=None,action_condition_available=False),gt={})
    with np.load(directory/'observations.npz',allow_pickle=False) as bank:
        for key,array in [('depth','depth'),('lidar','lidar'),('odom','odom_pose'),('imu','imu'),('tf_map_odom','tf_map_odom'),('tf_odom_base','tf_odom_base')]:
            result['obs'][key]=gather(bank[array],index['obs']['indices'][key],index['obs']['availability'][key])
            result['obs'][key+'_available']=np.asarray(index['obs']['availability'][key],dtype=bool)
        result['obs']['query_ns']=np.asarray(index['history_query_ns'],dtype=np.int64)
        for key in index['obs']['indices']:
            good=np.asarray(index['obs']['availability'][key],dtype=bool)
            for clock in ['stamp_ns','record_ns','publish_wall_ns']:
                result['obs'][key+'_'+clock]=np.where(good,gather(bank[key+'_'+clock],index['obs']['indices'][key],good),-1)
            result['obs'][key+'_age_ns']=np.where(good,result['obs']['query_ns']-result['obs'][key+'_stamp_ns'],-1)
        result['obs']['depth_pixel_available']=gather(bank['depth_pixel_available'],index['obs']['indices']['depth'],index['obs']['availability']['depth'])
        result['obs']['lidar_ray_available']=gather(bank['lidar_ray_available'],index['obs']['indices']['lidar'],index['obs']['availability']['lidar'])
        for key,source in [('camera_k','camera_info'),('camera_p','camera_info'),('camera_d','camera_info'),('lidar_metadata','lidar'),('odom_twist_body','odom'),('imu_covariance','imu'),('imu_field_available','imu'),('imu_covariance_available','imu')]:
            result['obs'][key]=gather(bank[key],index['obs']['indices'][source],index['obs']['availability'][source])
        static_mask=bank['static_record_ns']<=index['anchor_ns']
        result['obs']['static_transform']=gather(bank['static_transform'],np.arange(len(static_mask)),static_mask)
        result['obs']['static_available']=static_mask
        result['obs']['static_parent']=bank['static_parent'];result['obs']['static_child']=bank['static_child']
        result['obs']['observed_tracks']=np.empty((21,0,5),dtype=np.float32);result['obs']['observed_tracks_available']=np.zeros(21,dtype=bool)
    with np.load(directory/'human_states_gt.npz',allow_pickle=False) as bank:
        ids=index['gt']['future_indices']
        for key in ['human_world','robot_world','human_velocity_world','robot_velocity_world','human_available','robot_available','human_velocity_available','robot_velocity_available','human_occupancy_packed','occupancy_available','contact_truth','contact_available','source_stamp_ns']:
            result['gt'][key]=gather(bank[key],ids)
        height,width=bank['occupancy_shape'];dynamic=np.unpackbits(result['gt']['human_occupancy_packed'],axis=1,bitorder='little')[:,:height*width].reshape(30,height,width).astype(bool)
        static=np.unpackbits(bank['static_occupancy_packed'],bitorder='little')[:height*width].reshape(height,width).astype(bool)
        result['gt']['occupancy']=dynamic|static[None,:,:]
        result['gt']['occupancy_cell_available']=np.broadcast_to(np.unpackbits(bank['static_occupancy_known_packed'],bitorder='little')[:height*width].reshape(height,width).astype(bool),(30,height,width))&result['gt']['occupancy_available'][:,None,None]
        result['gt']['human_ids']=bank['human_ids'];result['gt']['occupancy_origin_xy']=bank['occupancy_origin_xy'];result['gt']['occupancy_resolution_m']=bank['occupancy_resolution_m']
        result['gt']['query_ns']=np.asarray(index['future_query_ns'],dtype=np.int64)
    return result


def recorded_action_refs(controls,queries,after_ns=None):
    """As-of recorded requests, max age 100ms; future rows must be post-anchor."""
    source=np.asarray([c['source_stamp_ns'] for c in controls],dtype=np.int64)
    record=np.asarray([c['record_stamp_ns'] for c in controls],dtype=np.int64)
    ids,mask=causal(source,record,queries,MAX_AGE['control'],[c['publish_wall_ns'] for c in controls])
    selected=np.maximum(ids,0)
    if len(controls):
        finite=np.isfinite(np.asarray([c['value'] for c in controls])).all(axis=1)
        mask&=finite[selected]
        if after_ns is not None:mask&=(source[selected]>after_ns)&(record[selected]>after_ns)
    ids[~mask]=-1
    return ids,mask


def qualify_factual_window(window,controls,provenance_valid):
    w=dict(window);w.pop('complete_requested_obs_act',None);w['act']=dict(window['act'])
    sensor=bool(w['valid']);reasons={'sensor_factual_valid':list(w['reasons'])}
    past,pm=recorded_action_refs(controls,w['history_query_ns'])
    future,fm=recorded_action_refs(controls,w['future_query_ns'],w['anchor_ns'])
    w['act'].update(past_published_control_ids=past.tolist(),past_published_control_available=pm.tolist(),
        future_recorded_requested_ids=future.tolist(),future_recorded_requested_available=fm.tolist(),
        past_requested_commands_available=bool(provenance_valid and pm.all()),
        note='Public Nav2 requested body twist; future rows are offline factual records, never anchor-time candidates.')
    for field,mask,label in [('past_action_factual_valid',pm,'past_requested'),('future_recorded_action_valid',fm,'future_recorded_requested')]:
        reasons[field]=list(w['reasons'])+([] if provenance_valid else ['requested_provenance_unverified'])
        if not mask.all():reasons[field].append('missing_or_stale_'+label)
        w[field]=sensor and provenance_valid and bool(mask.all())
    w.update(sensor_factual_valid=sensor,semantic_observed_tracks_valid=False,counterfactual_valid=False)
    reasons['semantic_observed_tracks_valid']=['no_verified_semantic_observed_tracks']
    reasons['counterfactual_valid']=['no_counterfactual_protocol_or_outcomes']
    w['eligibility_reasons']=reasons
    return w


def correct_export_contract(base,recovery_dir,output):
    """Recompute task eligibility from immutable banks; write only a small index."""
    from .episode_recovery import verify_base,trace_commands,read_jsonl,write_json
    base=Path(base).resolve();recovery_dir=Path(recovery_dir).resolve();output=Path(output).resolve()
    recovery=json.loads((recovery_dir/'missing_sources.json').read_text())
    audit,old=verify_base(recovery['audit_path'],base)
    proof_path=recovery_dir/'command_provenance.json';proof=json.loads(proof_path.read_text())
    if Path(recovery['base_export']).resolve()!=base or sha(proof_path)!=recovery['recovery_artifact_sha256']['command_provenance.json']:
        raise ValueError('Command proof does not belong to this export')
    if proof!=trace_commands(audit['run_dir'],Path(__file__).resolve().parents[3]):raise ValueError('Command routing proof changed')
    controls=[c for c in read_jsonl(base/'controls.jsonl') if c['record_type']=='observed_published_command']
    if [c['control_id'] for c in controls]!=list(range(len(controls))):raise ValueError('Unexpected command ID order')
    with np.load(base/'observations.npz',allow_pickle=False) as bank:
        grid=bank['grid_query_ns'];raw=dict(controls=controls)
        for field,suffix in [('stamp','stamp_ns'),('record','record_ns'),('publish','publish_wall_ns')]:
            raw[field]={k:bank[k+'_'+suffix] for k in ['depth','camera_info','lidar','odom','imu','tf_map_odom','tf_odom_base']}
        obs={k:bank[k] for k in bank.files if k.endswith(('_stamp_ns','_record_ns')) or k in ['observed_tracks','observed_tracks_available']}
        edges=list(zip(bank['static_parent'],bank['static_child']))
        needed=[('world','map'),('base_footprint','base_link'),('base_link','base_scan'),('base_link','imu_link'),('base_link','camera_link'),('camera_link','camera_optical_frame')]
        static_available=max(int(bank['static_record_ns'][edges.index(e)]) for e in needed)
    with np.load(base/'human_states_gt.npz',allow_pickle=False) as gt:
        if not np.array_equal(grid,gt['query_ns']):raise ValueError('OBS/GT query grids differ')
        human_ok=gt['human_available']&np.isfinite(gt['human_world']).all(axis=2)
        human_ok&=((gt['source_age_ns']>=0)&(gt['source_age_ns']<=25_000_000))[:,None]
    refs,masks=alignment_refs(raw,grid)
    windows,_=make_windows(grid,refs,masks,dict(human_available=human_ok),static_available,required_gt=('human_available',))
    windows=[qualify_factual_window(w,controls,True) for w in windows]
    validate_no_leakage(windows,obs,controls)
    counts={task:sum(w[field] for w in windows) for task,field in TASK_FIELDS.items()}
    reasons={field:dict(Counter(r for w in windows for r in w['eligibility_reasons'][field])) for field in TASK_FIELDS.values()}
    report=dict(schema='task03_corrected_export/1',base_export=str(base),command_provenance=str(proof_path),
        command_provenance_sha256=sha(proof_path),requested_stage=proof['stage'],raw_verdict=audit['verdict'],
        raw_input_sha256=audit['input_sha256'],base_sha256={**old['output_sha256'],'export_report.json':sha(base/'export_report.json')},
        windows_total=len(windows),eligible_windows=counts,missing_reasons_nonexclusive=reasons,
        usable_now={k:v>0 for k,v in counts.items()},max_age_ns=MAX_AGE,history_samples=21,future_samples=30,
        intersection_past_and_future=sum(w['past_action_factual_valid'] and w['future_recorded_action_valid'] for w in windows),
        examples={task:next((w['sample_id'] for w in windows if w[field]),None) for task,field in TASK_FIELDS.items()},
        source_sha256={str(Path(__file__).resolve()):sha(__file__)},raw_unchanged=all(sha(p)==h for p,h in audit['input_sha256'].items()))
    if not report['raw_unchanged']:raise ValueError('Audited input changed')
    output.mkdir(parents=True,exist_ok=False)
    with (output/'wm_index.jsonl').open('x') as f:
        for w in windows:f.write(json.dumps(w,allow_nan=False)+'\n')
    write_json(output/'export_report.json',report)
    return report


def load_factual_sample(directory,sample_id,task,allow_invalid=False):
    report=json.loads((directory/'export_report.json').read_text());base=Path(report['base_export'])
    if task not in TASK_FIELDS:raise ValueError('Unknown dataset task: '+task)
    for name,h in report['base_sha256'].items():
        if sha(base/name)!=h:raise ValueError('Base bank changed: '+name)
    if sha(report['command_provenance'])!=report['command_provenance_sha256']:raise ValueError('Command proof changed')
    index=next(json.loads(s) for s in (directory/'wm_index.jsonl').read_text().splitlines() if json.loads(s)['sample_id']==sample_id)
    field=TASK_FIELDS[task]
    if not index[field] and not allow_invalid:raise ValueError(task+': '+str(index['eligibility_reasons'][field]))
    sample=_load_index_sample(base,index);sample['task']=task;sample['act']={}
    sample['eligibility']={k:index[k] for k in TASK_FIELDS.values()}
    if task in ['PAST_ACTION_FACTUAL_WM','FUTURE_ACTION_CONDITIONED_WM']:
        controls=[json.loads(s) for s in (base/'controls.jsonl').read_text().splitlines() if json.loads(s)['record_type']=='observed_published_command']
        future=task=='FUTURE_ACTION_CONDITIONED_WM';prefix='future_recorded_requested' if future else 'past_published_control'
        ids=index['act'][prefix+'_ids'];mask=np.asarray(index['act'][prefix+'_available'],dtype=bool)
        query=np.asarray(index['future_query_ns'] if future else index['history_query_ns'],dtype=np.int64)
        act=dict(query_ns=query,available=mask,requested_stage=report['requested_stage'],frame='base_footprint',
            future_candidate_requested=None,counterfactual=False,observed_after_anchor=future)
        key='future_recorded_requested_actions' if future else 'past_requested_actions'
        act[key]=gather(np.asarray([c['value'] for c in controls],dtype=float),ids,mask)
        for clock in ['source_stamp_ns','record_stamp_ns','publish_wall_ns']:
            act[clock]=np.where(mask,gather(np.asarray([c[clock] for c in controls],dtype=np.int64),ids,mask),-1)
        act['age_ns']=np.where(mask,query-act['source_stamp_ns'],-1);sample['act']=act
    return sample


def write_schema(output,report):
    text='''# TASK03 OBS / ACT / GT schema v1

This is a **PARTIAL_EXPORT**, not training-ready complete observations/actions.
TASK02's only failing gate is the empty embedded schema of optional action-status;
that topic is excluded. Used sources must pass all type/hash/TF/GT gates and input
hashes must still match. The original RAW verdict remains PARTIAL.

## Window/time convention

Simulation timestamps are int64 nanoseconds. Grid is 10 Hz, inclusive 0 through the
last complete 0.1 s tick. History is **21 queries [anchor−2.0, ..., anchor]**;
future is **30 queries [anchor+0.1, ..., anchor+3.0]**. Anchor belongs only to history.
Every sensor reference requires source_stamp <= query AND record_stamp <= query <=
anchor, with per-modality freshness thresholds in export_report.json. DDS publish
wall time is stored separately; it is not compared directly with simulation time.
No interpolation, extrapolation, image filling, or synthetic simulator snapshot.
GT is the last actual Gazebo pose before its query, at most 25 ms old. Query and
actual source times are both retained. Invalid references are −1 plus false masks;
the loader never indexes a bank at −1. Invalid windows remain in wm_index.jsonl.

`valid` means complete core depth/LiDAR/odom/IMU/causal-TF history and measured future
robot/human/occupancy GT. Missing observed detector, requested-command instrumentation,
candidate action-condition and contact truth are explicitly unavailable optional
fields; `complete_requested_obs_act` is false for every window. Do not infer complete
ACT-conditioned training readiness from `valid`.

## observations.npz — OBS only

Large raw sensor banks occur once, not once per window. `depth` is (D,240,424) float32;
raw +/-Inf and NaN are preserved and depth_pixel_available marks finite positive
values. `lidar` is (L,360), with lidar_ray_available for finite in-range returns;
lidar_metadata columns are angle_min, angle_max, angle_increment, range_min,
range_max, scan_time, time_increment. `odom_pose` columns are xyz,qx,qy,qz,qw in odom;
`odom_twist_body` columns are vx,vy,vz,wx,wy,wz in child base_footprint.
`imu` columns are qx,qy,qz,qw,wx,wy,wz,ax,ay,az in imu_link. IMU field and covariance
availability differ: covariance all zero is unknown uncertainty, not perfect data.
Camera K/P/D and sensor source/record/publish timestamps are kept in their banks.
`observed_tracks` has shape (grid_count,0,5), observed_track_ids is empty and all
observed-track/occlusion-knowledge masks are false. No GT IDs or positions are copied
into observed tracks. No detector or visibility/occlusion classifier was recorded.

## Frames/transforms

ROS REP-style axes: +X forward, +Y left, +Z up for base_link/base_footprint; optical
+Z forward, +X right, +Y down. T_parent_child maps child coordinates into parent:
`p_parent = T_parent_child @ [p_child,1]`. Static TF includes world→map (identity),
base_footprint→base_link (identity), base_link→camera_link, base_scan, imu_link,
camera_link→camera_optical_frame. Dynamic observable TF banks are map→odom and
odom→base_footprint. Thus T_world_base_link = T_world_map @ T_map_odom @
T_odom_base_footprint @ T_base_footprint_base_link, using only causal referenced TF.
These are ROS localization transforms, never substituted with robot Gazebo GT.
World and odom are distinct; raw odometry is never relabelled as world coordinates.

## controls.jsonl — ACT provenance, not future execution

One metadata row, unique raw published command rows with control_id, and the recorded
goal event. `value=[vx,vy,wz]` is the observed /cmd_vel publication in base_footprint,
not measured execution and not a separately recorded internal requested command.
The actual MecanumDrive is omnidirectional: vy is retained; no unicycle reduction.
Native wheel radius/separation/wheelbase are recorded in report/metadata.
Past distinct requested commands and candidate action-condition are NOT_MEASURED.
No future executed command is renamed candidate requested action. wm_index ACT
references only past published commands, with missing/stale-command masks.

## human_states_gt.npz — GT only

Shared 10 Hz banks: human_world=(N,H,3), robot_world=(N,3), columns world x,y,yaw.
human_ids are real simulator IDs. Velocity_world columns are vx,vy,wz, derived from
measured pose displacement over 0.1–0.125 s, without bridging raw gaps >25 ms;
separate validity masks apply. Source timestamps and age are retained.
contact_truth is −1 and contact_available false: no valid contact instrumentation
was recorded. Proximity/overlap is NOT labelled contact.

Occupancy GT is world-fixed, excludes ego robot, and is NOT an observation. Grid
resolution is 0.1 m; origin_xy is its lower-left corner. Array indexing [row,col]
increases world Y,X. A dynamic cell is occupied when its square intersects a human
proxy disk of the recorded native radius. Static geometry comes from the recorded
/map checked against the frozen Gazebo-validated map; 2x2 occupied-cell OR is a
conservative reduction. Human occupancy uses only measured GT states, including
occluded humans, without visibility assumptions. `human_occupancy_packed` uses
np.packbits(bitorder='little'), one shared bank row per grid tick. Static occupancy
and known-cell masks occur once; compose union on load. occupancy_available false
means unknown, never free. Out-of-domain cells have no label.

## Index/loading

wm_index.jsonl stores only small time/index/mask metadata and GT references. It does
not duplicate sensor frames or packed occupancy. All NPZ arrays are non-object;
load with allow_pickle=False. Example:

```python
from dynamuhsim.datasets.single_episode_export import load_sample
sample = load_sample(EXPORT_DIR, SAMPLE_ID)  # IDs printed in export_report.json
print(sample['obs']['depth'].shape)          # (21,240,424)
print(sample['gt']['human_world'].shape)     # (30,6,3)
print(sample['gt']['occupancy'].shape)       # (30,420,620)
assert not sample['obs']['observed_tracks_available'].any()
assert sample['act']['action_condition'] is None
```

This export neither trains a World Model nor provides counterfactual data.
'''
    with (output/'schema.md').open('x') as f:f.write(text)
