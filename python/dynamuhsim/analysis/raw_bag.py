"""TASK02: independent raw-message validation and descriptive episode statistics."""
from collections import Counter, defaultdict
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

from .mcap_records import McapFile

REQUIRED = ['/clock','/scan','/camera/depth/image_rect_raw','/camera/camera_info',
            '/imu','/odom','/tf','/tf_static','/cmd_vel','/common/ground_truth',
            '/dataset/scene_poses','/dataset/events']
SENSORS = ['/camera/depth/image_rect_raw','/scan','/imu','/odom']
NM = 'NOT_MEASURED'
NS = 1_000_000_000


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarize(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return None
    return dict(count=len(a),min=float(a.min()),max=float(a.max()),mean=float(a.mean()),
                median=float(np.median(a)),p95=float(np.quantile(a,.95)),p99=float(np.quantile(a,.99)))


def timing(values, expected_period=None):
    a=np.asarray(values,dtype=np.int64)
    if not len(a):return dict(status=NM)
    delta=np.diff(a);positive=delta[delta>0]/NS
    period=expected_period or (float(np.median(positive)) if len(positive) else None)
    threshold=max(3*period,.05) if period else None
    gaps=[dict(before_ns=int(a[i]),after_ns=int(a[i+1]),gap_s=float(d/NS)) for i,d in enumerate(delta) if threshold and d/NS>threshold]
    inferred=int(sum(max(0,round(float(d/NS)/period)-1) for d in delta if d>0)) if expected_period else None
    span=(int(a[-1])-int(a[0]))/NS
    return dict(status='MEASURED',count=len(a),unique_count=len(set(map(int,a))),start_ns=int(a.min()),end_ns=int(a.max()),
                span_s=(int(a.max())-int(a.min()))/NS,regressions=int(np.count_nonzero(delta<0)),
                adjacent_duplicates=int(np.count_nonzero(delta==0)),duplicate_values=len(a)-len(set(map(int,a))),
                zero_stamps=int(np.count_nonzero(a==0)),period_s=summarize(positive),
                frequency_hz=summarize(1/positive) if len(positive) else None,
                throughput_hz=(len(a)-1)/span if span>0 else None,large_gap_threshold_s=threshold,
                large_gap_count=len(gaps),large_gaps=gaps,expected_period_s=expected_period,
                actual_missing_messages=NM,inferred_missing_from_period=inferred,
                inferred_missing_percent=100*inferred/(len(set(map(int,a)))+inferred) if inferred is not None else None,
                note='Frequency uses strictly positive adjacent intervals; duplicates/regressions are reported separately. Missing counts are inference, not sensor sequence evidence.')


def merge(intervals):
    out=[]
    for a,b in sorted(intervals):
        if b<a:continue
        if out and a<=out[-1][1]+1e-8:out[-1][1]=max(b,out[-1][1])
        else:out.append([float(a),float(b)])
    return out


def intersection(left,right):
    return merge([[max(a,c),min(b,d)] for a,b in left for c,d in right if max(a,c)<=min(b,d)])


def duration(intervals):
    return sum(b-a for a,b in intervals)


def inside(t, intervals):
    return any(a-1e-9<=t<=b+1e-9 for a,b in intervals)


def support(stamps, freshness):
    return merge([[t/NS,t/NS+freshness] for t in sorted(set(stamps))])


def velocities(points, window=.2):
    """Measured position displacement over >=window seconds, in the world XY frame."""
    unique={row['time']:row for row in points};rows=[unique[t] for t in sorted(unique)]
    if len(rows)<2:return [],rows
    times=np.array([r['time'] for r in rows]);out=[]
    for i,row in enumerate(rows):
        j=int(np.searchsorted(times,row['time']-window,side='right'))-1
        if j<0 or row['time']-rows[j]['time']>window+.08:continue
        elapsed=row['time']-rows[j]['time']
        out.append(dict(time=(row['time']+rows[j]['time'])/2,
                        speed=math.hypot(row['x']-rows[j]['x'],row['y']-rows[j]['y'])/elapsed))
    return out,rows


def choose_run(path):
    path=Path(path).resolve()
    if (path/'episode/raw_bag/metadata.yaml').is_file():return path
    candidates=[]
    for receipt in sorted(path.glob('*/manifest.json')):
        m=json.loads(receipt.read_text())
        if m.get('schema')=='dynamuhsim_capture_episode/1' and m.get('status')=='PASS' and m.get('result',{}).get('actual_goal_reached'):
            candidates.append(receipt.parent)
    if len(candidates)!=1:
        raise ValueError(f'Expected one completed TASK01 capture; found {len(candidates)}. Specify its exact run directory.')
    return candidates[0]


def expected_periods(run):
    from dynamuhsim.simulation.robot_geometry import read_model
    model=read_model(run/'runtime/mecanum_camera.sdf')
    rates={s.get('type'):float(s.findtext('update_rate')) for s in model.findall('.//sensor') if s.findtext('update_rate')}
    odom=float(model.findtext("plugin[@name='gz::sim::systems::MecanumDrive']/odom_publish_frequency"))
    step=float(ET.parse(run/'runtime/warehouse.sdf').getroot().findtext('world/physics/max_step_size'))
    nominal={'/camera/depth/image_rect_raw':rates['depth_camera'],'/camera/camera_info':rates['depth_camera'],
             '/scan':rates['gpu_lidar'],'/imu':rates['imu'],'/odom':odom,'/clock':1/step,'/common/ground_truth':50.}
    effective={t:math.ceil((1/hz-1e-10)/step)*step for t,hz in nominal.items()}
    return nominal,effective,step


def read_raw(run):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    streams=defaultdict(lambda:dict(record=[],publish=[],source=[],frames=Counter(),sequences=[],source_field=None,locations=[]))
    tf=defaultdict(list);statics=defaultdict(list);scenes=[];events=[];cmd=[];odom=[];truth=[];camera=[];depth_specs=[]
    depth_finite_fractions=[]
    schema_report={};integrity=[];failures=[];invalid_stamps=[];image=Counter();lidar=Counter();imu=Counter();classes={}
    for file in sorted((run/'episode/raw_bag').glob('*.mcap')):
        reader=McapFile(file)
        for row in reader.messages():
            topic=row['topic'];s=streams[topic];s['record'].append(row['record_ns']);s['publish'].append(row['publish_ns']);s['sequences'].append(row['sequence'])
            schema=row['schema'];name=schema['name'];key=(name,schema['encoding'])
            if key not in classes:
                try:classes[key]=get_message(name)
                except Exception as error:raise ValueError(f'Local ROS type {name} unavailable at {row["location"]}: {error}') from error
            schema_report[name]=dict(encoding=schema['encoding'],embedded_bytes=len(schema['definition'].encode()),
                embedded_sha256=hashlib.sha256(schema['definition'].encode()).hexdigest(),embedded_complete=bool(schema['definition'].strip()),local_type_available=True,
                schema_record_location=reader.schema_locations[row['schema_id']])
            try:msg=deserialize_message(row['payload'],classes[key])
            except Exception as error:
                failures.append(dict(topic=topic,error=str(error),location=row['location']));continue
            s['decoded']=s.get('decoded',0)+1;s['type']=name
            def stamp(value,label):
                if value.sec<0 or not 0<=value.nanosec<NS:invalid_stamps.append(dict(topic=topic,field=label,location=row['location']))
                return int(value.sec)*NS+int(value.nanosec)
            source=None
            if hasattr(msg,'header'):
                source=stamp(msg.header.stamp,'header.stamp');s['source_field']='header.stamp';s['frames'][msg.header.frame_id]+=1
            if topic=='/clock':source=stamp(msg.clock,'clock');s['source_field']='clock'
            elif topic in ['/tf','/tf_static']:
                for tr in msg.transforms:
                    edge=(tr.header.frame_id,tr.child_frame_id);ts=stamp(tr.header.stamp,'transform.header.stamp')
                    value=dict(time_ns=ts,record_ns=row['record_ns'],translation=[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z],
                               quaternion=[tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w])
                    (statics if topic=='/tf_static' else tf)[edge].append(value)
            elif topic in ['/dataset/scene_poses','/dataset/events','/common/ground_truth']:
                value=json.loads(msg.data);source=int(value['stamp_ns']) if 'stamp_ns' in value else round(value['time']*NS)
                s['source_field']='payload.stamp_ns' if 'stamp_ns' in value else 'payload.time'
                if topic=='/dataset/scene_poses':scenes.append(value)
                elif topic=='/dataset/events':events.append(value)
                else:truth.append(value)
            elif topic=='/cmd_vel':cmd.append(dict(time=source/NS,speed=math.hypot(msg.twist.linear.x,msg.twist.linear.y),vx=msg.twist.linear.x,vy=msg.twist.linear.y,wz=msg.twist.angular.z,frame=msg.header.frame_id))
            elif topic=='/odom':odom.append(dict(time=source/NS,speed=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y),frame=msg.header.frame_id,child=msg.child_frame_id))
            elif topic=='/camera/camera_info':
                camera.append(dict(time_ns=source,width=msg.width,height=msg.height,k=list(msg.k),p=list(msg.p),d=list(msg.d),distortion_model=msg.distortion_model,frame=msg.header.frame_id))
            elif topic=='/camera/depth/image_rect_raw':
                image['frames']+=1
                depth_specs.append(dict(time_ns=source,width=msg.width,height=msg.height,frame=msg.header.frame_id))
                valid=msg.encoding=='32FC1' and msg.step>=msg.width*4 and len(msg.data)==msg.step*msg.height
                image['valid_layout_frames']+=int(valid)
                if valid:
                    a=np.frombuffer(bytes(msg.data),dtype='>f4' if msg.is_bigendian else '<f4').reshape(msg.height,msg.step//4)[:,:msg.width]
                    positive=np.isfinite(a)&(a>0);depth_finite_fractions.append(float(positive.mean()))
                    image['pixels']+=a.size;image['nan_pixels']+=int(np.isnan(a).sum());image['positive_inf_pixels']+=int(np.isposinf(a).sum());image['negative_inf_pixels']+=int(np.isneginf(a).sum());image['nonpositive_finite_pixels']+=int((np.isfinite(a)&(a<=0)).sum());image['finite_positive_pixels']+=int(positive.sum());image['frames_without_finite_positive_depth']+=int(not positive.any())
            elif topic=='/scan':
                a=np.asarray(msg.ranges);lidar['frames']+=1;lidar['rays']+=len(a);lidar['nan_rays']+=int(np.isnan(a).sum());lidar['inf_no_return_rays']+=int(np.isinf(a).sum());lidar['finite_in_range_rays']+=int((np.isfinite(a)&(a>=msg.range_min)&(a<=msg.range_max)).sum())
            elif topic=='/imu':
                imu['frames']+=1;imu['orientation_unavailable_frames']+=int(msg.orientation_covariance[0]<0)
                imu['orientation_unknown_covariance_frames']+=int(not any(msg.orientation_covariance))
                imu['angular_velocity_unknown_covariance_frames']+=int(not any(msg.angular_velocity_covariance))
                imu['linear_acceleration_unknown_covariance_frames']+=int(not any(msg.linear_acceleration_covariance))
            if source is not None:s['source'].append(source)
        integrity.append(dict(file=str(file),sha256=digest(file),bytes=file.stat().st_size,**reader.integrity))
    image['finite_positive_pixel_fraction_percent']=summarize([100*f for f in depth_finite_fractions])
    return dict(streams=streams,tf=tf,statics=statics,scenes=scenes,events=events,cmd=cmd,odom=odom,truth=truth,camera=camera,depth_specs=depth_specs,
                schemas=schema_report,integrity=integrity,decode_failures=failures,invalid_stamps=invalid_stamps,
                image=dict(image),lidar=dict(lidar),imu=dict(imu))


def tf_calibration(raw,run):
    from scipy.spatial.transform import Rotation
    all_edges=set(raw['tf'])|set(raw['statics']);parents=defaultdict(set)
    for parent,child in all_edges:parents[child].add(parent)
    def path_to_root(child):
        seen=set();edges=[]
        while child in parents:
            if child in seen or len(parents[child])!=1:return None
            seen.add(child);parent=next(iter(parents[child]));edges.append((parent,child));child=parent
        return edges if child=='world' else None
    frames=sorted({f for t in SENSORS+['/camera/camera_info'] for f in raw['streams'].get(t,{}).get('frames',{})})
    paths={f:path_to_root(f) for f in frames};mounts=[]
    urdf=run/'runtime/mecanum_aligned.urdf'
    if urdf.exists():
        for joint in ET.parse(urdf).getroot().findall('joint'):
            if joint.get('type')!='fixed':continue
            edge=(joint.find('parent').get('link'),joint.find('child').get('link'));origin=joint.find('origin')
            xyz=np.array(list(map(float,origin.get('xyz').split())));q=Rotation.from_euler('xyz',list(map(float,origin.get('rpy').split()))).as_quat()
            rows=raw['statics'].get(edge,[])
            passed=bool(rows) and all(np.allclose(r['translation'],xyz,atol=1e-7) and abs(np.dot(r['quaternion'],q))>1-1e-7 for r in rows)
            mounts.append(dict(edge=' -> '.join(edge),passed=passed,xyz=xyz.tolist()))
    infos=raw['camera'];depth=raw['streams'].get('/camera/depth/image_rect_raw',{})
    valid_intrinsics=bool(infos) and all(c['width']==424 and c['height']==240 and c['k'][0]>0 and c['k'][4]>0 and c['k'][8]==1 for c in infos)
    exact_info=sum(t in {c['time_ns'] for c in infos} for t in depth.get('source',[]))
    indexed={c['time_ns']:c for c in infos}
    dimensions_match=bool(raw['depth_specs']) and all(p['time_ns'] in indexed and all(p[k]==indexed[p['time_ns']][k] for k in ['width','height','frame']) for p in raw['depth_specs'])
    conflicts=[c for c,p in parents.items() if len(p)>1]
    static_conflicts=[]
    for edge,rows in raw['statics'].items():
        if any(not np.allclose(r['translation'],rows[0]['translation'],atol=1e-8) or abs(np.dot(r['quaternion'],rows[0]['quaternion']))<1-1e-7 for r in rows):static_conflicts.append(' -> '.join(edge))
    finite_transforms=all(all(math.isfinite(v) for v in r['translation']+r['quaternion']) and abs(np.linalg.norm(r['quaternion'])-1)<1e-6 for rows in list(raw['tf'].values())+list(raw['statics'].values()) for r in rows)
    optical=raw['statics'].get(('camera_link','camera_optical_frame'),[])
    optical_axes=bool(optical) and all(np.allclose(Rotation.from_quat(r['quaternion']).apply([[1,0,0],[0,1,0],[0,0,1]]),[[0,-1,0],[0,0,-1],[1,0,0]],atol=1e-6) for r in optical)
    world_map=raw['statics'].get(('world','map'),[])
    world_map_identity=bool(world_map) and all(np.allclose(r['translation'],[0,0,0],atol=1e-8) and abs(r['quaternion'][3])>1-1e-8 for r in world_map)
    dynamic_needed=set(e for p in paths.values() if p for e in p if e in raw['tf'])
    ranges={ ' -> '.join(e):[min(r['time_ns'] for r in raw['tf'][e])/NS,max(r['time_ns'] for r in raw['tf'][e])/NS] for e in dynamic_needed}
    continuous={}
    for e in dynamic_needed:
        ts=sorted(set(r['time_ns']/NS for r in raw['tf'][e]));delta=np.diff(ts)
        limit=max(3*float(np.median(delta)),.05) if len(delta) else 0.
        continuous[' -> '.join(e)]=merge([[a,b] for a,b in zip(ts,ts[1:]) if b-a<=limit+1e-9])
    valid=bool(paths) and all(p is not None for p in paths.values()) and not conflicts and not static_conflicts and finite_transforms and optical_axes and world_map_identity and valid_intrinsics and dimensions_match and bool(mounts) and all(m['passed'] for m in mounts) and exact_info==len(depth.get('source',[]))
    return dict(passed=valid,frames=frames,frame_paths={f:[' -> '.join(e) for e in p] if p is not None else NM for f,p in paths.items()},
                multiple_parent_children=conflicts,conflicting_static_edges=static_conflicts,mounts=mounts,
                finite_normalized_transforms=finite_transforms,optical_axis_convention_valid=optical_axes,world_map_identity=world_map_identity,
                dynamic_coverage_s=ranges,dynamic_continuous_intervals_s=continuous,intrinsics=infos[0] if infos else NM,intrinsics_valid=valid_intrinsics,
                depth_frames_with_exact_camera_info=exact_info,depth_frames=len(depth.get('source',[])),depth_camera_info_layout_compatible=dimensions_match)


def compare_sources(raw,speed):
    scene=sorted(raw['scenes'],key=lambda s:s['stamp_ns']);times=[s['stamp_ns']/NS for s in scene]
    errors=defaultdict(list)
    for truth in raw['truth']:
        if not times:break
        i=int(np.searchsorted(times,truth['time']));candidates=[j for j in [i-1,i] if 0<=j<len(times)]
        j=min(candidates,key=lambda j:abs(times[j]-truth['time']))
        if abs(times[j]-truth['time'])>.00250001:continue
        row=scene[j];errors['robot'].append(math.hypot(row['robot']['x']-truth['robot']['x'],row['robot']['y']-truth['robot']['y']))
        humans={h['id']:h for h in row['humans']}
        for h in truth['humans']:
            if h['id'] in humans:errors['human_'+str(h['id'])].append(math.hypot(humans[h['id']]['x']-h['x'],humans[h['id']]['y']-h['y']))
    comparisons={}
    for label,rows in [('observed_cmd_vel',raw['cmd']),('odom_twist',raw['odom'])]:
        indexed={r['time']:r for r in rows};ts=sorted(indexed);delta=[]
        for point in speed.get('robot',[]):
            i=int(np.searchsorted(ts,point['time']));js=[j for j in [i-1,i] if 0<=j<len(ts)]
            if not js:continue
            j=min(js,key=lambda j:abs(ts[j]-point['time']))
            if abs(ts[j]-point['time'])<=.05:delta.append(indexed[ts[j]]['speed']-point['speed'])
        comparisons[label]=dict(pairing_tolerance_sim_s=.05,paired_samples=len(delta),
            norm_speed_residual_mps=summarize(delta),rmse_mps=float(np.sqrt(np.mean(np.square(delta)))) if delta else None,
            interpretation='Descriptive time-paired norms only; frame components and command/measurement are not equated.')
    return dict(pose_label_crosscheck=dict(source_a='/dataset/scene_poses',source_b='/common/ground_truth',
                 maximum_timestamp_difference_s=.0025,position_error_m={k:summarize(v) for k,v in errors.items()},
                 passed=bool(errors) and all(max(v)<.02 for v in errors.values())),velocity_norm_comparisons=comparisons)


def check_local_type_hashes(metadata):
    from ament_index_python.packages import get_package_share_directory
    result={}
    for row in metadata['topics_with_message_count']:
        topic=row['topic_metadata'];name=topic['type'];package,folder,base=name.split('/')
        path=Path(get_package_share_directory(package))/folder/(base+'.json')
        local=None
        if path.exists():
            data=json.loads(path.read_text());local=next((v['hash_string'] for v in data.get('type_hashes',[]) if v['type_name']==name),None)
        recorded=topic.get('type_description_hash')
        result[name]=dict(recorded=recorded or NM,local=local or NM,match=local==recorded if local and recorded else None,
                          local_description_path=str(path),local_msg_definition_exists=(path.with_suffix('.msg')).exists())
    return result


def wall_bounds(run, raw):
    text=(run/'episode/recorder.log').read_text();bounds={}
    for label,phrase in [('start','Starting recording to'),('end','Recording stopped')]:
        lines=[line for line in text.splitlines() if phrase in line]
        if lines:
            match=re.search(r'\[(\d+)\.(\d{9})\]',lines[0])
            if match:bounds[label+'_ns']=int(match[1])*NS+int(match[2])
    if len(bounds)==2:
        bounds['duration_s']=(bounds['end_ns']-bounds['start_ns'])/NS
        bounds['start_local']=dt.datetime.fromtimestamp(bounds['start_ns']/NS,dt.timezone(dt.timedelta(hours=7))).isoformat()
        bounds['end_local']=dt.datetime.fromtimestamp(bounds['end_ns']/NS,dt.timezone(dt.timedelta(hours=7))).isoformat()
        bounds['source']='recorder.log wall-clock start/stop; not MCAP log_time'
    epoch=[p for s in raw['streams'].values() for p in s['publish'] if p>10**17]
    return dict(recorder_process_wall=bounds or NM,raw_publish_wall=dict(start_ns=min(epoch),end_ns=max(epoch),span_s=(max(epoch)-min(epoch))/NS) if epoch else NM,
                per_message_wall_receive_time=NM,warning='MCAP publish_time is DDS source publication time, not recorder arrival time. Sim log_time minus epoch publish_time is not a latency.')


def analyze(run, raw):
    import yaml
    manifest=json.loads((run/'manifest.json').read_text());result=manifest.get('result',manifest)
    metadata=yaml.safe_load((run/'episode/raw_bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
    nominal,periods,physics=expected_periods(run);stats={};time_rows=[]
    clock=raw['streams'].get('/clock',dict(record=[],source=[]));clock_order=np.argsort(clock['record']);clock_log=np.array(clock['record'],dtype=np.int64)[clock_order];clock_val=np.array(clock['source'],dtype=np.int64)[clock_order]
    for topic in sorted(set(raw['streams'])|set(REQUIRED)):
        s=raw['streams'].get(topic)
        if not s:stats[topic]=dict(status=NM,count=None);continue
        source=s['source'];entry=dict(status='MEASURED',count=len(s['record']),decoded=s.get('decoded',0),type=s['type'],
            frames=dict(s['frames']),source_field=s['source_field'] or ('per-transform header.stamp' if topic.startswith('/tf') else NM),
            embedded_schema=raw['schemas'][s['type']]['embedded_complete'],nominal_hz=nominal.get(topic),
            effective_simulation_period_s=periods.get(topic),mcap_sequence='ALL_ZERO_UNAVAILABLE' if not any(s['sequences']) else 'PRESENT')
        entry['clocks']={}
        for field,values in [('record_sim',s['record']),('publish_wall',s['publish']),('source_sim',source)]:
            measured=timing(values,periods.get(topic) if field=='source_sim' else None);entry['clocks'][field]=measured
            if measured['status']=='MEASURED':time_rows.append(dict(topic=topic,field=field,**measured))
        if source:
            a=np.array(source,dtype=np.int64);record=np.array(s['record'],dtype=np.int64)
            idx=np.searchsorted(clock_log,record,side='right')-1;valid=idx>=0
            ages=(clock_val[idx[valid]]-a[valid])/NS
            threshold=max(1.5*periods.get(topic,.02),.05)
            entry.update(record_minus_source_s=summarize((record-a)/NS),source_age_at_record_clock_s=summarize(ages),
                         freshness_threshold_s=threshold,too_old_frames=int(np.count_nonzero(ages>threshold)) if len(ages) else None,
                         too_old_percent=float(100*np.count_nonzero(ages>threshold)/len(a)) if len(ages) else None,
                         future_source_over_5ms_frames=int(np.count_nonzero(ages<-.005)),clock_association_missing=int(np.count_nonzero(~valid)))
        else:entry.update(record_minus_source_s=None,source_age_at_record_clock_s=None,too_old_frames=None,too_old_percent=None,freshness_threshold_s=None)
        stats[topic]=entry
    for edge,rows in raw['tf'].items():
        measure=timing([r['time_ns'] for r in rows]);time_rows.append(dict(topic='/tf:'+ '->'.join(edge),field='transform_source_sim',**measure))
    calibration=tf_calibration(raw,run)
    tf_clock_checks={}
    for edge,rows in raw['tf'].items():
        tf_clock_checks[' -> '.join(edge)]=dict(source=timing([r['time_ns'] for r in rows]),
            record_minus_transform_s=summarize([(r['record_ns']-r['time_ns'])/NS for r in rows]))
    calibration['dynamic_timestamp_checks']=tf_clock_checks
    calibration['future_transform_note']='AMCL map->odom is intentionally future-dated by transform_tolerance; a negative record-minus-transform offset is not a sensor clock reset.'
    scene=raw['scenes'];tracks=defaultdict(list);frame_counts=Counter();duplicates=[]
    for row in scene:
        t=row['stamp_ns']/NS;tracks['robot'].append(dict(time=t,**row['robot']))
        ids=[h['id'] for h in row['humans']]
        if len(ids)!=len(set(ids)):duplicates.append(t)
        for h in row['humans']:tracks['human_'+str(h['id'])].append(dict(time=t,**h));frame_counts[h['id']]+=1
    trajectories={};speed={};motion={}
    for name,points in tracks.items():
        speeds,points=velocities(points);speed[name]=speeds
        path=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(points,points[1:]));times=[p['time'] for p in points]
        trajectories[name]=dict(source='/dataset/scene_poses (Gazebo pose/info)',frame='world',frames=len(points),start=points[0],end=points[-1],path_length_m=path,
            mean_speed_time_weighted_mps=path/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None,
            max_gap_s=max(np.diff(times)) if len(times)>1 else None,position_finite=all(math.isfinite(p[k]) for p in points for k in ['x','y','yaw']),
            speed_020s_window_mps=summarize([s['speed'] for s in speeds]))
        motion[name]=merge([[a['time'],b['time']] for a,b in zip(speeds,speeds[1:]) if a['speed']>.01 and b['speed']>.01 and b['time']-a['time']<=.05])
    all_record=[t for s in raw['streams'].values() for t in s['record']];clock_range=[min(clock['source'])/NS,max(clock['source'])/NS] if clock['source'] else [None,None]
    core=[[max(clock_range[0],min(all_record)/NS),min(clock_range[1],max(all_record)/NS)]] if clock['source'] else []
    source_coverage={}
    for topic in SENSORS+['/dataset/scene_poses']:
        stamps=raw['streams'].get(topic,{}).get('source',[])
        p=periods.get(topic) or (np.median(np.diff(stamps))/NS if len(stamps)>1 else .02)
        fresh=max(1.5*p,.05);source_coverage[topic]=support(stamps,fresh)
        core=intersection(core,source_coverage[topic])
        if stamps:core=intersection(core,[[min(stamps)/NS,max(stamps)/NS]])
    for intervals in calibration['dynamic_continuous_intervals_s'].values():core=intersection(core,intervals)
    if not calibration['passed']:core=[]
    centers=[[a+2,b-3] for a,b in core if b-a>=5]
    active_events={e['event']:e for e in raw['events']};start=active_events.get('goal_accepted',{}).get('stamp_ns');end=active_events.get('action_end',{}).get('stamp_ns')
    active=[[start/NS,end/NS]] if start is not None and end is not None else []
    for name,points in tracks.items():
        distance=0.
        for a,b in zip(points,points[1:]):
            elapsed=b['time']-a['time']
            if elapsed<=0:continue
            overlap=duration(intersection([[a['time'],b['time']]],active))
            distance+=math.hypot(b['x']-a['x'],b['y']-a['y'])*overlap/elapsed
        trajectories[name]['navigation_path_length_m']=distance
        trajectories[name]['navigation_mean_speed_time_weighted_mps']=distance/duration(active) if active else None
    human_motion=merge([interval for k,v in motion.items() if k!='robot' for interval in v]);together=intersection(motion.get('robot',[]),human_motion)
    anchors=sorted(set(raw['streams'].get('/camera/depth/image_rect_raw',{}).get('source',[])));anchor_checks={}
    for topic in ['/scan','/imu','/odom','/dataset/scene_poses']:
        samples=sorted(set(raw['streams'].get(topic,{}).get('source',[])))
        if not samples:anchor_checks[topic]=dict(status=NM);continue
        threshold=stats[topic].get('freshness_threshold_s') or .05;missing=stale=0;ages=[]
        for anchor in anchors:
            idx=int(np.searchsorted(samples,anchor,side='right'))-1
            if idx<0:missing+=1;continue
            age=(anchor-samples[idx])/NS;ages.append(age);stale+=int(age>threshold)
        anchor_checks[topic]=dict(status='MEASURED',anchors=len(anchors),missing_previous_sample=missing,too_old=stale,
            unavailable_percent=100*(missing+stale)/len(anchors),age_s=summarize(ages),threshold_s=threshold)
    metadata_counts={r['topic_metadata']['name']:r['message_count'] for r in metadata['topics_with_message_count']}
    observed_counts={t:s['count'] for t,s in stats.items() if s['status']=='MEASURED'}
    schema_missing=[name for name,s in raw['schemas'].items() if not s['embedded_complete']]
    type_hashes=check_local_type_hashes(metadata);source_comparison=compare_sources(raw,speed)
    freeze_path=run/'capture_freeze.json';runtime_params=run/'runtime/nav2_runtime.yaml';runtime_world=run/'runtime/warehouse.sdf'
    ready_event=next((e for e in raw['events'] if e['event']=='ready'),{})
    linkage=dict(freeze_identity=freeze_path.exists() and digest(freeze_path)==result.get('capture_freeze_sha256'),
        runtime_nav_parameters=runtime_params.exists() and digest(runtime_params)==result.get('runtime_nav_params_sha256'),
        runtime_world=runtime_world.exists() and digest(runtime_world)==result.get('runtime_world_sha256'),
        event_scenario_seed=ready_event.get('scenario')==manifest['plan'].get('scenario') and ready_event.get('seed')==manifest['plan'].get('seed'),
        manifest_message_counts=observed_counts==result.get('recording',{}).get('topic_message_counts'))
    gates=dict(full_decode=not raw['decode_failures'] and not raw['invalid_stamps'],metadata_counts=all(metadata_counts.get(t,0)==observed_counts.get(t,0) for t in set(metadata_counts)|set(observed_counts)),
        required_topics=all(stats[t]['status']=='MEASURED' for t in REQUIRED),embedded_schemas=not schema_missing,
        local_type_hashes=all(v['match'] is True for v in type_hashes.values()),
        manifest_linkage=all(linkage.values()),
        source_sensor_monotonic=all(stats[t]['clocks']['source_sim'].get('regressions',1)==0 for t in SENSORS if stats[t]['status']=='MEASURED'),
        clock_monotonic=timing(clock['source']).get('regressions',0)==0 if clock['source'] else None,tf_calibration=calibration['passed'],
        human_ids=bool(scene) and not duplicates and len(frame_counts)==manifest['plan']['human_count'] and all(n==len(scene) for n in frame_counts.values()),
        tf_source_monotonic=all(v['source']['regressions']==0 for v in tf_clock_checks.values()),
        no_large_sensor_gaps=all(stats[t].get('clocks',{}).get('source_sim',{}).get('large_gap_count')==0 for t in SENSORS),
        physical_pose_sources_agree=source_comparison['pose_label_crosscheck']['passed'],
        history_future_available=bool(centers))
    severe=not gates['full_decode'] or not gates['metadata_counts'] or gates['clock_monotonic'] is False or not gates['source_sensor_monotonic'] or any(v['match'] is False for v in type_hashes.values())
    verdict='FAIL' if severe else ('RAW_CAPTURE_PASS' if all(gates.values()) else 'PARTIAL')
    messages=sum(observed_counts.values())
    report=dict(schema='dynamuhsim_raw_audit/1',verdict=verdict,gates=gates,run_dir=str(run),bag_dir=str(run/'episode/raw_bag'),
        manifest_sha256=digest(run/'manifest.json'),metadata_sha256=digest(run/'episode/raw_bag/metadata.yaml'),mcap=raw['integrity'],
        message_count=messages,manifest_linkage=linkage,schemas=raw['schemas'],local_type_hashes=type_hashes,missing_embedded_schemas=schema_missing,decode_failures=raw['decode_failures'],invalid_stamps=raw['invalid_stamps'],
        topic_statistics=stats,time=dict(record_time_domain='simulation (--use-sim-time), NOT Unix wall-clock despite metadata key nanoseconds_since_epoch',
            record_sim_start_s=min(all_record)/NS,record_sim_end_s=max(all_record)/NS,record_span_s=(max(all_record)-min(all_record))/NS,
            clock_sim_start_s=clock_range[0],clock_sim_end_s=clock_range[1],clock_span_s=clock_range[1]-clock_range[0] if clock['source'] else None,
            wall=wall_bounds(run,raw),navigation_active_sim_s=duration(active),navigation_active_wall_s=result.get('wall_seconds',NM),
            events=raw['events'],physics_step_s=physics),calibration=calibration,
        quality=dict(depth=raw['image'] if raw['image'].get('frames') else dict(status=NM),lidar=raw['lidar'] or dict(status=NM),imu=raw['imu'] or dict(status=NM),depth_anchor_alignment=anchor_checks,
                     missing_messages_note='No sensor sequence counters exist. Inferred missing percentages use SDF rates quantized to physics ticks; NOT a measured loss rate.'),
        tracks=dict(human_ids=sorted(frame_counts),scene_frames=len(scene),human_frames={str(k):v for k,v in frame_counts.items()},duplicate_id_frames=duplicates,
                    identity_scope='Six ground-truth simulator IDs, not image detections or re-identification tracks.',trajectories=trajectories),
        usable=dict(definition='Intersection of actual sensor/scene timestamp coverage, causal latest-sample freshness and dynamic TF coverage; valid camera calibration required.',
            intervals_s=core,duration_s=duration(core),active_intervals_s=intersection(core,active),active_duration_s=duration(intersection(core,active)),
            robot_and_any_human_moving_intervals_s=together,robot_and_any_human_moving_s=duration(together),
            robot_and_any_human_moving_during_navigation_s=duration(intersection(together,active)),
            motion_threshold_mps=.01,motion_velocity_window_s=.2,history_s=2.,future_s=3.,center_intervals_s=centers,
            eligible_center_duration_s=duration(centers),usable_depth_anchors=sum(inside(t/NS,core) for t in anchors),
            excluded_depth_anchors=sum(not inside(t/NS,core) for t in anchors),
            excluded_depth_anchor_percent=100*sum(not inside(t/NS,core) for t in anchors)/len(anchors) if anchors else None,
            eligible_depth_anchors=sum(inside(t/NS,centers) for t in anchors),
            eligible_active_depth_anchors=sum(inside(t/NS,intersection(centers,active)) for t in anchors)),
        commands=dict(requested_navigation_goal=active_events.get('goal_accepted',{}).get('goal_world',NM),
            actual_action_request_service_payload=NM,requested_internal_mppi_velocity=NM,
            observed_cmd_vel=dict(source='/cmd_vel',description='Published robot-frame command; not measured physical velocity',speed_mps=summarize([r['speed'] for r in raw['cmd']]),
                                  first=raw['cmd'][0] if raw['cmd'] else NM,last=raw['cmd'][-1] if raw['cmd'] else NM),
            odometry_velocity=dict(source='/odom.twist.twist.linear',description='Reported odometry twist; not Gazebo truth',frames=sorted({(r['frame'],r['child']) for r in raw['odom']}),speed_mps=summarize([r['speed'] for r in raw['odom']])),
            actual_velocity=dict(source='/dataset/scene_poses',method='World-XY displacement over >=0.2 s and <=0.28 s; timestamp at interval midpoint',robot_mps=trajectories.get('robot',{}).get('speed_020s_window_mps')),
            equivalence_asserted=False,source_cross_validation=source_comparison),
        task01_fix_locations=[dict(issue='Missing self-contained ROS message definition',type=t,schema_location=raw['schemas'][t]['schema_record_location'],
             location='python/dynamuhsim/runtime/capture_recording.py: Recording.__init__/ready; inspect rosbag2 MCAP message-definition resolution and add a schema preflight for future captures. Local .msg/.idl files exist, so missing installation is not established. Do not modify this bag.') for t in schema_missing],
        limitations=['Single episode: descriptive statistics only, no population inference or simulator ranking.',
                     'Absolute per-message wall receipt time and internal requested MPPI command are NOT_MEASURED.',
                     'Motion overlap is thresholded from 0.2 s window-average measured speed; transition timing is approximate at that window resolution.',
                     'No simulation, policy, Nav2, Gazebo or original data was modified.'])
    return report,time_rows,speed,tracks,raw


def write_csv(path,rows,columns):
    with path.open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');writer.writeheader()
        for row in rows:writer.writerow({k:NM if row.get(k) is None else row.get(k) for k in columns})


def plots(out,report,raw,speed,tracks):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    topics=sorted(raw['streams']);fig,ax=plt.subplots(figsize=(13,7))
    for i,topic in enumerate(topics):
        ts=np.asarray(raw['streams'][topic]['record'])/NS;ax.scatter(ts,np.full(len(ts),i),s=2)
    events={e['event']:e['stamp_ns']/NS for e in raw['events']}
    for label,color in [('goal_accepted','green'),('action_end','red'),('observation_end','gray')]:
        if label in events:ax.axvline(events[label],color=color,ls='--',label=label)
    ax.set(yticks=range(len(topics)),yticklabels=topics,xlabel='Recorded simulation time (s)',title='Actual recorded messages by topic (one episode)')
    ax.legend(loc='upper left',bbox_to_anchor=(1.01,1));fig.tight_layout();fig.savefig(out/'topic_timeline.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for values,label,style in [(raw['cmd'],'Published /cmd_vel','--'),(raw['odom'],'Reported /odom twist',':'),(speed.get('robot',[]),'Gazebo position-derived robot speed','-')]:
        if values:axes[0].plot([p['time'] for p in values],[p['speed'] for p in values],style,lw=1,label=label)
    for name,values in sorted(speed.items()):
        if name!='robot' and values:axes[1].plot([p['time'] for p in values],[p['speed'] for p in values],lw=1,label=name)
    for ax in axes:
        ax.set_ylabel('XY speed (m/s)');ax.grid(alpha=.2);ax.legend(ncol=2)
        if 'goal_accepted' in events:ax.axvline(events['goal_accepted'],color='gray',ls='--')
    axes[0].set_title('Commands, odometry and measured motion are distinct sources')
    axes[1].set_title('Six Gazebo human tracks: displacement / elapsed simulation time (0.2 s window)')
    axes[1].set_xlabel('Source simulation time (s)');fig.tight_layout();fig.savefig(out/'robot_human_speeds.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,7))
    for name,points in sorted(tracks.items()):ax.plot([p['x'] for p in points],[p['y'] for p in points],lw=1.3,label=name)
    ax.set(xlabel='Gazebo world X (m)',ylabel='Gazebo world Y (m)',title='Recorded physical trajectories, not planned paths');ax.axis('equal');ax.grid(alpha=.2);ax.legend();fig.tight_layout();fig.savefig(out/'recorded_trajectories.png',dpi=160);plt.close(fig)


def write_outputs(out,report,time_rows,speed,tracks,raw):
    topic_rows=[]
    for topic,s in report['topic_statistics'].items():
        t=s.get('clocks',{}).get('source_sim',{});f=t.get('frequency_hz') or {}
        topic_rows.append(dict(topic=topic,status=s['status'],type=s.get('type'),messages=s.get('count'),decoded=s.get('decoded'),embedded_schema=s.get('embedded_schema'),source_field=s.get('source_field'),
            start_sim_s=t['start_ns']/NS if 'start_ns' in t else None,end_sim_s=t['end_ns']/NS if 'end_ns' in t else None,
            median_hz=f.get('median'),mean_instantaneous_hz=f.get('mean'),min_hz=f.get('min'),max_hz=f.get('max'),throughput_hz=t.get('throughput_hz'),
            source_regressions=t.get('regressions'),source_duplicates=t.get('duplicate_values'),inferred_missing=t.get('inferred_missing_from_period'),inferred_missing_percent=t.get('inferred_missing_percent'),
            actual_missing=NM,too_old_frames=s.get('too_old_frames'),too_old_percent=s.get('too_old_percent'),freshness_threshold_s=s.get('freshness_threshold_s')))
    write_csv(out/'topic_statistics.csv',topic_rows,list(topic_rows[0]))
    flat=[]
    for row in time_rows:
        p=row.get('period_s') or {};f=row.get('frequency_hz') or {}
        offset=report['topic_statistics'].get(row['topic'],{}).get('record_minus_source_s') or {}
        if row['field']=='transform_source_sim':
            edge=row['topic'].removeprefix('/tf:').replace('->',' -> ')
            offset=report['calibration']['dynamic_timestamp_checks'].get(edge,{}).get('record_minus_transform_s') or {}
        if row['field'] not in ['source_sim','transform_source_sim']:offset={}
        flat.append(dict(topic=row['topic'],field=row['field'],count=row['count'],start_ns=row['start_ns'],end_ns=row['end_ns'],span_s=row['span_s'],regressions=row['regressions'],duplicates=row['duplicate_values'],zero_stamps=row['zero_stamps'],
            time_domain='DDS source epoch wall time' if row['field']=='publish_wall' else 'simulation',
            period_mean_s=p.get('mean'),period_median_s=p.get('median'),median_hz=f.get('median'),mean_hz=f.get('mean'),min_hz=f.get('min'),max_hz=f.get('max'),large_gaps=row['large_gap_count'],
            record_minus_source_min_s=offset.get('min'),record_minus_source_median_s=offset.get('median'),record_minus_source_p95_s=offset.get('p95'),record_minus_source_max_s=offset.get('max')))
    write_csv(out/'time_statistics.csv',flat,list(flat[0]))
    plots(out,report,raw,speed,tracks)
    write_report(out,report)
    with (out/'raw_audit.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False)


def write_report(out,r):
    wall=r['time']['wall']['recorder_process_wall']
    wall_text=(f"{wall['start_local']} → {wall['end_local']}; **{wall['duration_s']:.6f} wall s** (log recorder start/stop)." if isinstance(wall,dict) else NM)
    lines=['# TASK02 — Kiểm định raw ROS bag và thống kê episode đầu tiên','',f"**Kết luận: `{r['verdict']}`.**",'',f"Input duy nhất: `{r['run_dir']}`.",
        'Không chạy thêm simulation. Không thay đổi human policy, Nav2, Gazebo hoặc raw bag.','',
        '## Khả năng đọc và schema','',f"Đọc và deserialize {r['message_count']:,} ROS messages. Metadata counts khớp: {r['gates']['metadata_counts']}. Chunk CRC và summary CRC được kiểm độc lập trong thứ tự ghi vật lý.",
        f"Schema nhúng bị thiếu: {', '.join(r['missing_embedded_schemas']) or 'không có'}. Có thể dùng interface ROS đã cài để decode, nhưng không coi đó là bag tự chứa đầy đủ schema.",'',
        f"RIHS01 type hashes của các interface local đối chiếu metadata: **{r['gates']['local_type_hashes']}** ({len(r['local_type_hashes'])} types). Không suy luận thiếu package: file interface local có tồn tại; lỗi được xác nhận là schema rỗng trong MCAP.",
        '## Thời gian','',f"Record time là **simulation time**: {r['time']['record_sim_start_s']:.6f} → {r['time']['record_sim_end_s']:.6f} s; span {r['time']['record_span_s']:.6f} s.",
        f"/clock: {r['time']['clock_sim_start_s']} → {r['time']['clock_sim_end_s']} s (None = NOT_MEASURED).",
        f"Wall-time recorder: {wall_text}",
        f"Navigation: {r['time']['navigation_active_sim_s']:.4f} sim s / {r['time']['navigation_active_wall_s']:.4f} wall s. MCAP publish_time là thời gian DDS publish wall-clock; không trừ trực tiếp với record time để gọi là latency.",'',
        '## Topic và tần số theo timestamp nguồn','',
        '| Topic | Messages | Median Hz | Mean instantaneous Hz | Min–max Hz | Duplicate source stamp | Gap lớn |',
        '|---|---:|---:|---:|---:|---:|---:|']
    def fmt(v):return NM if v is None else f'{v:.4f}'
    for topic,s in r['topic_statistics'].items():
        t=s.get('clocks',{}).get('source_sim',{});f=t.get('frequency_hz') or {}
        lines.append(f"| {topic} | {s.get('count',NM)} | {fmt(f.get('median'))} | {fmt(f.get('mean'))} | {fmt(f.get('min'))}–{fmt(f.get('max'))} | {t.get('duplicate_values',NM)} | {t.get('large_gap_count',NM)} |")
    lines+=['','Mean instantaneous Hz là mean(1/Δt) với Δt>0; throughput=(N−1)/span được xuất riêng. Tần số wall-publish và record nằm trong time_statistics.csv. Không dùng duplicate record timestamps để kết luận mất frame.',
        'Số mất thực tế là NOT_MEASURED vì MCAP sequence đều không cung cấp sensor sequence; số suy luận dùng chu kỳ SDF đã lượng tử hóa theo physics tick.',
        'Odometry cấu hình 30 Hz nhưng 1/30 s lượng tử hóa lên 14 physics ticks × 2,5 ms = 35 ms, tức 28,5714 Hz; không gán chênh lệch này thành packet loss. AMCL publish thích nghi theo chuyển động, nên các gap của /amcl_pose không tự động là mất dữ liệu.',
        f"/cmd_vel có {r['topic_statistics'].get('/cmd_vel',{}).get('clocks',{}).get('source_sim',{}).get('duplicate_values',NM)} stamp mô phỏng lặp; tần số wall publication xem time_statistics.csv. Các lệnh vẫn là messages riêng, không xóa bỏ khỏi audit.",
        f"/tf có {r['topic_statistics'].get('/tf',{}).get('clocks',{}).get('publish_wall',{}).get('regressions',NM)} lần DDS publish-time lùi trong thứ tự ghi tổng hợp nhiều publisher. Kiểm định timestamp nguồn theo từng cạnh TF: {r['gates']['tf_source_monotonic']}; không đánh đồng hai phép kiểm này.",'',
        '### Record time − source stamp và độ cũ theo /clock','',
        '| Topic | Median offset (ms) | P95 (ms) | Min–max (ms) | Frame quá cũ |','|---|---:|---:|---:|---:|']
    for topic in SENSORS:
        s=r['topic_statistics'][topic];o=s.get('record_minus_source_s') or {}
        lines.append(f"| {topic} | {fmt(o.get('median')*1000 if o else None)} | {fmt(o.get('p95')*1000 if o else None)} | {fmt(o.get('min')*1000 if o else None)}–{fmt(o.get('max')*1000 if o else None)} | {s.get('too_old_frames',NM)} |")
    lines+=['','Các offset trên cùng miền simulation time, không phải độ trễ truyền tải wall-time. Offset −2,5 ms có thể xuất hiện do tick clock/callback; số lần và cực trị đều được giữ trong JSON.','',
        '## Dữ liệu dùng được và history/future','',
        f"- Khoảng dùng được có đủ sensor/scene/calibration/TF: **{r['usable']['duration_s']:.4f} sim s**, `{r['usable']['intervals_s']}`.",
        f"- Trong navigation active: **{r['usable']['active_duration_s']:.4f} s**.",
        f"- {r['usable']['usable_depth_anchors']} depth anchors đủ toàn bộ coverage; {r['usable']['excluded_depth_anchors']} bị loại ({fmt(r['usable']['excluded_depth_anchor_percent'])}%), chủ yếu warmup trước TF map→odom. Không gọi đây là frame depth bị mất.",
        f"- Robot và ít nhất một người cùng di chuyển (>0,01 m/s): **{r['usable']['robot_and_any_human_moving_s']:.4f} s**; trong navigation: {r['usable']['robot_and_any_human_moving_during_navigation_s']:.4f} s.",
        f"- Tâm có đủ history 2 s + future 3 s: `{r['usable']['center_intervals_s']}`, tổng {r['usable']['eligible_center_duration_s']:.4f} s; **{r['usable']['eligible_depth_anchors']} depth anchors**, trong navigation {r['usable']['eligible_active_depth_anchors']}.",
        '- Đây là khả năng coverage raw, chưa tạo hay xác nhận tensor, nhãn học máy hoặc tính độc lập thống kê của các cửa sổ. Activity timing dùng cửa sổ vận tốc 0,2 s nên là phép đo xấp xỉ tại độ phân giải đó.','',
        '### Frame thiếu/cũ tại các mốc depth','',
        '| Nguồn | Mốc depth | Thiếu mẫu trước đó | Quá cũ | % không dùng được |', '|---|---:|---:|---:|---:|']
    for t,s in r['quality']['depth_anchor_alignment'].items():lines.append(f"| {t} | {s.get('anchors',NM)} | {s.get('missing_previous_sample',NM)} | {s.get('too_old',NM)} | {s.get('unavailable_percent',NM)} |")
    lines+=['','Quá cũ = tuổi mẫu > max(1,5 × chu kỳ hiệu dụng; 0,05 s). Đối chiếu causal offline bằng header/payload timestamp; không giả định mọi packet đến cùng lúc.',
        f"Depth pixel QC: `{r['quality']['depth']}`. +Inf/−Inf được báo riêng, không gộp −Inf thành finite depth ≤0 và không coi các pixel không-return là frame bị mất.",
        f"LiDAR QC: `{r['quality']['lidar']}`. IMU: `{r['quality']['imu']}`; covariance −1 nghĩa là trường không có, còn covariance toàn zero là uncertainty NOT_MEASURED, không phải sai số bằng zero.",'',
        '## TF và calibration','',f"TF/calibration đủ theo cổng kiểm: **{r['calibration']['passed']}**. Dynamic TF coverage: `{r['calibration']['dynamic_coverage_s']}`.",
        f"Depth có CameraInfo cùng stamp: {r['calibration']['depth_frames_with_exact_camera_info']}/{r['calibration']['depth_frames']}. Các mount fixed được đối chiếu URDF SDF-derived; TF được xét theo từng parent→child.",'',
        '## Quỹ đạo, human ID và vận tốc','',f"Có **{len(r['tracks']['human_ids'])} ground-truth human tracks**, ID `{r['tracks']['human_ids']}`, {r['tracks']['scene_frames']} scene frames. Frames/ID: `{r['tracks']['human_frames']}`. Duplicate-ID frames: {len(r['tracks']['duplicate_id_frames'])}.",
        'Đây là ID simulator; không phải track detector/re-identification trong ảnh.','',
        '| Track | Số frame | Quãng đường (m) | Time-weighted mean speed toàn bag (m/s) | Max gap (s) |','|---|---:|---:|---:|---:|']
    for name,t in r['tracks']['trajectories'].items():lines.append(f"| {name} | {t['frames']} | {t['path_length_m']:.4f} | {fmt(t['mean_speed_time_weighted_mps'])} | {fmt(t['max_gap_s'])} |")
    lines+=['',f"Goal yêu cầu được ghi trong event: `{r['commands']['requested_navigation_goal']}`. Raw action service payload và requested internal MPPI velocity: **NOT_MEASURED**.",
        '`/cmd_vel` là lệnh đã publish; `/odom.twist` là vận tốc odometry; vận tốc thực tính độc lập từ dịch chuyển pose Gazebo trong cửa sổ 0,2–0,28 s. Không đồng nhất ba nguồn hoặc so sánh trực tiếp các thành phần ở frame khác nhau.',
        f"Đối chiếu hai luồng pose cùng Gazebo (scene labels và adapter ground truth), cùng timestamp sai khác tối đa 2,5 ms: **{r['commands']['source_cross_validation']['pose_label_crosscheck']['passed']}**. Đây không phải hai cảm biến độc lập. Sai lệch từng ID và so sánh norm vận tốc theo thời gian nằm trong JSON; không dùng kết quả đó để tuyên bố lệnh bằng vận tốc thực.",
        'Toàn bộ clock offsets, duplicate, gap, checksum, frames và các giới hạn nằm trong raw_audit.json; không ẩn giá trị NOT_MEASURED.','',
        '## Biểu đồ','', '![Timeline](topic_timeline.png)','', '![Vận tốc](robot_human_speeds.png)','', '![Quỹ đạo](recorded_trajectories.png)','',
        '## Chỗ cần sửa tại TASK01','']
    for fix in r['task01_fix_locations']:lines.append(f"- {fix['type']}: {fix['issue']}. Vị trí record: `{fix['schema_location']}`. {fix['location']}.")
    lines+=['','Không sửa bag hiện hữu; không chạy lại simulation trong TASK02. Một episode chỉ cho thống kê mô tả; không suy rộng success rate hoặc phân bố dataset.']
    with (out/'first_episode_report.md').open('x') as f:f.write('\n'.join(lines)+'\n')
