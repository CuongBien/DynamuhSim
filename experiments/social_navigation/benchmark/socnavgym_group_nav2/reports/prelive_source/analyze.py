#!/usr/bin/env python3
"""Independent actual-pose, sensor and controller audit; goal failures stay outcomes."""
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
import argparse,base64,bisect,gzip,hashlib,json,math,subprocess,sys,xml.etree.ElementTree as ET,zlib
from pathlib import Path
import numpy as np,shapely,yaml
from PIL import Image
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent;REPO=SOCIAL.parent.parent
sys.path.insert(0,str(BENCH/'socnavgym_response_fix'));from audit_geometry import gaps
p=argparse.ArgumentParser();p.add_argument('--live',type=Path,required=True);a=p.parse_args();out=a.live.resolve();load=lambda p:json.loads(p.read_text());lines=lambda p:[json.loads(line) for line in p.open()];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
result=load(out/'result.json');assert result['status']=='MEASURED',result.get('error');people=result['condition']=='people';native_dt=result['native_dt'];runtime=REPO/result['runtime'];plan=load(HERE/'protocol.json');boxes=load(SOCIAL/'warehouse/layout.json')['obstacles'];assert len(boxes)==42
for name,digest in result['source_sha256'].items():assert sha(REPO/name)==digest,name
assert result['rendering']['hardware_verified'] and 'NVIDIA' in result['rendering']['renderer']
assert result['cleanup']['status']=='PASS' and not result['cleanup']['remaining'] and result['gazebo_clean_exit'] and result['server_shutdown']['status']=='STOPPED'
assert any('NVIDIA' in s for s in result['cleanup']['ogre_renderer_lines']) and not result['faults']
assert result['cmd_vel_publishers_before_goal'] and all(e['node'] in ['controller_server','behavior_server'] for e in result['cmd_vel_publishers_before_goal']+result['cmd_vel_publishers_after_goal'])
base=yaml.safe_load((SOCIAL/'warehouse/nav2_warehouse.yaml').read_text());copy=yaml.safe_load((runtime/'nav2_runtime.yaml').read_text());assert copy['amcl']['ros__parameters']['initial_pose']['x']==-28.;copy['amcl']['ros__parameters']['initial_pose']=base['amcl']['ros__parameters']['initial_pose'];assert copy==base
assert sha(runtime/'nav2_runtime.yaml')==result['runtime_nav_params_sha256'] and sha(runtime/'warehouse.sdf')==result['runtime_world_sha256']
world=ET.parse(runtime/'warehouse.sdf').getroot().find('world');assert abs(float(world.find('physics/max_step_size').text)-.0025)<1e-12
for box in boxes:
    model=world.find("model[@name='"+box['name']+"']");assert model is not None;xy=list(map(float,model.find('pose').text.split()));size=list(map(float,model.find('.//collision/geometry/box/size').text.split()));assert abs(xy[0]-box['x'])<1e-9 and abs(xy[1]-box['y'])<1e-9 and abs(size[0]-box['sx'])<1e-9 and abs(size[1]-box['sy'])<1e-9
assert len(world.findall('actor'))==(6 if people else 0)
poses=lines(out/'gazebo_poses.jsonl');required=['mecanum_robot']+([f'common_{i}_proxy' for i in range(1,7)] if people else []);complete=[row for row in poses if all(name in row['poses'] for name in required)];assert len(complete)>100
all_times=[p['time'] for p in complete];active=[p for p in complete if result['goal_start_sim']<=p['time']<=result['action_end_sim']+.5];assert len(active)>100
xy=np.array([[row['poses']['mecanum_robot'][k] for k in ['x','y']] for row in active]);times=np.array([row['time'] for row in active]);mask=np.diff(times)>1e-10;speeds=np.linalg.norm(np.diff(xy,axis=0),axis=1)[mask]/np.diff(times)[mask];path=float(np.sum(np.linalg.norm(np.diff(xy,axis=0),axis=1)))
matches=[row for row in complete if row.get('phase')=='active' and row['poses']['mecanum_robot']==result['final_world_pose']];assert matches,'Recorded goal pose not found in actual Gazebo stream';final_sample=matches[-1];robot=final_sample['poses']['mecanum_robot'];goal=plan['goal'];xy_error=math.hypot(robot['x']-goal[0],robot['y']-goal[1]);yaw_error=abs(math.atan2(math.sin(robot['yaw']-goal[2]),math.cos(robot['yaw']-goal[2])));assert abs(xy_error-result['goal_xy_error_world_m'])<1e-10 and abs(yaw_error-result['goal_yaw_error_world_rad'])<1e-10
actual_reached=result['action_status']==4 and xy_error<=.20 and yaw_error<=.25;assert actual_reached==result['actual_goal_reached'];assert path>1.
commands=lines(out/'commands.jsonl');commands=[row for row in commands if row['phase']=='active'];assert len(commands)>20 and max(math.hypot(row['vx'],row['vy']) for row in commands)>.1
plans=lines(out/'plans.jsonl');assert any(row['phase']=='active' and len(row['poses'])>10 for row in plans)
costmaps=lines(out/'local_costmaps.jsonl');assert costmaps
for row in costmaps:assert len(zlib.decompress(base64.b64decode(row['data'])))==row['width']*row['height']
amcl=lines(out/'amcl.jsonl');amcl_errors=[];amcl_yaws=[]
for row in amcl:
    i=bisect.bisect_left(all_times,row['time']);indices=list(range(max(0,i-1),min(len(complete),i+2)))
    if not indices:continue
    ref=min(indices,key=lambda j:abs(all_times[j]-row['time']))
    if abs(all_times[ref]-row['time'])>.03:continue
    actual=complete[ref]['poses']['mecanum_robot'];amcl_errors.append(math.hypot(row['x']-actual['x'],row['y']-actual['y']));amcl_yaws.append(abs(math.atan2(math.sin(row['yaw']-actual['yaw']),math.cos(row['yaw']-actual['yaw']))))
assert len(amcl_errors)>10
odom=lines(out/'odom.jsonl');odom_errors=[]
for row in odom:
    i=min(bisect.bisect_left(all_times,row['time']),len(complete)-1)
    if abs(all_times[i]-row['time'])<.02:
        actual=complete[i]['poses']['mecanum_robot'];odom_errors.append(math.hypot(actual['x']-(-28+row['x']),actual['y']-row['y']))
assert odom_errors and max(odom_errors)<.05
polygons=np.array([shapely.box(b['x']-b['sx']/2,b['y']-b['sy']/2,b['x']+b['sx']/2,b['y']+b['sy']/2) for b in boxes],dtype=object)
robot_static=float(np.min(shapely.distance(shapely.linestrings(np.stack((xy[:-1],xy[1:]),axis=1))[:,None],polygons[None,:]))-.25)
measured=None;interpolation_error=[];worker_stats=None;lidar_errors=[];lidar_ids=set();footprint_gap=None;costmap_hits=[];slow_near=0.;near_seconds=0.;worker=[]
if people:
    for i in range(1,7):
        model=world.find(f"model[@name='common_{i}_proxy']")
        for tag in ['collision','visual']:assert abs(float(model.find(f'.//{tag}/geometry/cylinder/radius').text)-.36)<1e-12
    worker_raw=lines(runtime/'worker_trajectory.jsonl');worker=[row for row in worker_raw if 'state_time' in row];assert len(worker)==len(worker_raw)
    inactive=[row for row in worker if not row['active_phase']];moving=[row for row in worker if row['active_phase']];assert inactive and moving and all(row['native_step']==0 and row['before']==row['humans'] for row in inactive)
    assert [row['native_step'] for row in moving]==list(range(1,len(moving)+1))
    wt=[row['time'] for row in worker]
    for row in complete:
        j=bisect.bisect_right(wt,row['time'])-1
        if j<0:continue
        w=worker[j];alpha=(row['time']-w['time'])/native_dt
        if not -1e-6<=alpha<=1.00001:continue
        for old,new in zip(w['before'],w['humans']):
            real=row['poses'][f'common_{old["id"]}_proxy'];interpolation_error.append(math.hypot(real['x']-(old['x']+alpha*(new['x']-old['x'])),real['y']-(old['y']+alpha*(new['y']-old['y']))))
    assert interpolation_error and max(interpolation_error)<.025
    before=np.array([[[row['poses'][f'common_{i}_proxy'][k] for k in ['x','y']] for i in range(1,7)] for row in active[:-1]]);after=np.array([[[row['poses'][f'common_{i}_proxy'][k] for k in ['x','y']] for i in range(1,7)] for row in active[1:]])
    measured=gaps(before,after,xy[:-1],xy[1:],boxes)
    corners=np.array([[.12,.1],[.12,-.1],[-.12,-.1],[-.12,.1]]);fg=[]
    for row in active:
        robot=row['poses']['mecanum_robot'];angle=robot['yaw'];rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);foot=shapely.Polygon(corners@rotation.T+np.array([robot['x'],robot['y']]))
        fg.extend(foot.distance(shapely.Point(row['poses'][f'common_{i}_proxy']['x'],row['poses'][f'common_{i}_proxy']['y']))-.36 for i in range(1,7))
    footprint_gap=min(fg)
    proximity=[min(math.hypot(row['poses'][f'common_{i}_proxy']['x']-row['poses']['mecanum_robot']['x'],row['poses'][f'common_{i}_proxy']['y']-row['poses']['mecanum_robot']['y'])-.61 for i in range(1,7)) for row in active]
    for i,(a,b) in enumerate(zip(active,active[1:])):
        dt=b['time']-a['time']
        if dt<=1e-10:continue
        if proximity[i]<1.:
            near_seconds+=dt
            if math.hypot(b['poses']['mecanum_robot']['x']-a['poses']['mecanum_robot']['x'],b['poses']['mecanum_robot']['y']-a['poses']['mecanum_robot']['y'])/dt<.05:slow_near+=dt
    worker_stats=dict(native_dt=native_dt,inactive_transport_steps=len(inactive),active_native_steps=len(moving),first_active_sim=moving[0]['time'],start_gate_delay_s=moving[0]['time']-result['goal_start_sim'],guard_present_steps=sum(row.get('motion_guard') is not None for row in moving))
    for scan in lines(out/'scans.jsonl'):
        i=bisect.bisect_left(all_times,scan['time']);indices=list(range(max(0,i-1),min(len(complete),i+2)))
        if not indices:continue
        i=min(indices,key=lambda j:abs(all_times[j]-scan['time']))
        if abs(all_times[i]-scan['time'])>.025:continue
        row=complete[i];r=row['poses']['mecanum_robot'];angles=scan['angle_min']+np.arange(len(scan['ranges']))*scan['angle_increment']+r['yaw'];dx=np.cos(angles);dy=np.sin(angles);static=np.full(len(angles),np.inf)
        for b in boxes:
            near=np.full(len(angles),-np.inf);far=np.full(len(angles),np.inf)
            for origin,direction,center,size in [(r['x'],dx,b['x'],b['sx']),(r['y'],dy,b['y'],b['sy'])]:
                parallel=np.abs(direction)<1e-12;low=center-size/2;high=center+size/2;t1=np.divide(low-origin,direction,out=np.zeros_like(direction),where=~parallel);t2=np.divide(high-origin,direction,out=np.zeros_like(direction),where=~parallel);near=np.maximum(near,np.where(parallel,-np.inf if low<=origin<=high else np.inf,np.minimum(t1,t2)));far=np.minimum(far,np.where(parallel,np.inf if low<=origin<=high else -np.inf,np.maximum(t1,t2)))
            static=np.minimum(static,np.where((far>=np.maximum(near,0))&(near>=0),near,np.inf))
        dynamic=np.full(len(angles),np.inf);labels=np.zeros(len(angles),dtype=int)
        for hid in range(1,7):
            h=row['poses'][f'common_{hid}_proxy'];ox,oy=r['x']-h['x'],r['y']-h['y'];dot=ox*dx+oy*dy;disc=dot*dot-(ox*ox+oy*oy-.36**2);hit=-dot-np.sqrt(np.maximum(disc,0));hit=np.where((disc>=0)&(hit>0),hit,np.inf);labels=np.where(hit<dynamic,hid,labels);dynamic=np.minimum(dynamic,hit)
        selected=(dynamic>max(.14,scan['range_min']))&(dynamic<min(3.4,scan['range_max']))&(static>dynamic+.03);observed=np.array([np.nan if v is None else v for v in scan['ranges']]);errors=np.abs(observed-dynamic);errors=np.where(np.isfinite(errors),errors,10.);lidar_errors.extend(errors[selected].tolist());lidar_ids.update(int(v) for v in labels[selected&(errors<.15)])
    assert len(lidar_errors)>50 and sum(e<.15 for e in lidar_errors)/len(lidar_errors)>.7 and len(lidar_ids)>=2
costmap_evidence=dict(eligible_human_observations=0,observations_with_occupied_ring=0,matched_ids=[],scope='Descriptive occupied cells near measured human surfaces in odom; not source attribution or recall')
if people:
    matched=set();odom_times=[v['time'] for v in odom]
    for cm in costmaps:
        if cm['phase']!='active' or cm['frame']!='odom':continue
        j=bisect.bisect_left(all_times,cm['time']);indices=list(range(max(0,j-1),min(len(complete),j+2)))
        if not indices:continue
        j=min(indices,key=lambda k:abs(all_times[k]-cm['time']))
        if abs(all_times[j]-cm['time'])>.03:continue
        k=min(bisect.bisect_left(odom_times,cm['time']),len(odom)-1)
        if abs(odom_times[k]-cm['time'])>.04:continue
        row=complete[j];world_robot=row['poses']['mecanum_robot'];o=odom[k];angle=o['yaw']-world_robot['yaw'];R=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);offset=np.array([o['x'],o['y']])-R@np.array([world_robot['x'],world_robot['y']]);grid=np.frombuffer(zlib.decompress(base64.b64decode(cm['data'])),dtype=np.int8).reshape(cm['height'],cm['width']);res=cm['resolution']
        for hid in range(1,7):
            h=row['poses'][f'common_{hid}_proxy'];dist=math.hypot(h['x']-world_robot['x'],h['y']-world_robot['y'])
            if not .7<dist<3.1:continue
            center=R@np.array([h['x'],h['y']])+offset;ix=int(math.floor((center[0]-cm['origin']['x'])/res));iy=int(math.floor((center[1]-cm['origin']['y'])/res));span=math.ceil(.46/res);xs=np.arange(max(0,ix-span),min(cm['width'],ix+span+1));ys=np.arange(max(0,iy-span),min(cm['height'],iy+span+1))
            if not len(xs) or not len(ys):continue
            X,Y=np.meshgrid(cm['origin']['x']+(xs+.5)*res,cm['origin']['y']+(ys+.5)*res);distance=np.hypot(X-center[0],Y-center[1]);ring=(distance>=.26)&(distance<=.46)
            if not np.any(ring):continue
            costmap_evidence['eligible_human_observations']+=1
            if np.any((grid[np.ix_(ys,xs)]>=65)&ring):costmap_evidence['observations_with_occupied_ring']+=1;matched.add(hid)
    costmap_evidence['matched_ids']=sorted(matched)
cp=HERE/'generated/checkpoint';preserved=load(cp/'files.json');allowed={'experiments/social_navigation/'+name for name in ['README.md','NEXT_STEPS.md','COMMIT_HISTORY.md','FRAMEWORK_COMPARISON.md']}
for name,digest in preserved.items():
    if name not in allowed:assert sha(REPO/name)==digest,name
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary']) and (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'])
# Archive every raw stream exactly. Original raw files stay ignored locally.
for q in [*out.glob('*.jsonl'),out/'launch.log',runtime/'warehouse.sdf',runtime/'nav2_runtime.yaml',runtime/'worker_trajectory.jsonl',runtime/'worker.log',runtime/'ogre2_session.log']:
    if q.exists():target=out/(q.name+'.gz');target.write_bytes(gzip.compress(q.read_bytes(),mtime=0));assert gzip.decompress(target.read_bytes())==q.read_bytes()
if (out/'camera_preview_4x.gif').exists():
    with Image.open(out/'camera_preview_4x.gif') as im:
        duration=0
        for i in range(im.n_frames):im.seek(i);im.load();duration+=im.info.get('duration',0)
        assert duration==result['media']['gif_duration_ms']
validation=dict(status='AUDITED_NAV2_PILOT',model=result['model'],native_dt=native_dt,condition=result['condition'],navigation_status=result['navigation_status'],action_status=result['action_status'],actual_goal_reached=actual_reached,goal_xy_world_m=xy_error,goal_yaw_world_rad=yaw_error,active_seconds=result['active_sim_seconds'],wall_active_seconds=result['wall_seconds'],observed_rtf=result['active_sim_seconds']/result['wall_seconds'],robot_path_m=path,robot_y_range_m=[float(np.min(xy[:,1])),float(np.max(xy[:,1]))],robot_peak_speed_mps=float(np.max(speeds)),robot_static_disc_gap_m=robot_static,actual_swept_human_gaps_m=measured,actual_sampled_rotated_footprint_human_gap_m=footprint_gap,geometry_outcome='OVERLAP_GT_5MM' if measured and min(measured.values())<-.005 else 'NO_OVERLAP_GT_5MM',slow_while_near_human_s=slow_near,near_human_seconds=near_seconds,amcl_xy_error_m=dict(samples=len(amcl_errors),median=float(np.median(amcl_errors)),p95=float(np.percentile(amcl_errors,95)),maximum=max(amcl_errors)),amcl_yaw_max_rad=max(amcl_yaws),odom_world_feedback_max_error_m=max(odom_errors),actual_pose_samples=len(complete),proxy_interpolation_error_max_m=max(interpolation_error,default=None),worker=worker_stats,lidar_human_beams=len(lidar_errors),lidar_fraction_within15cm=sum(e<.15 for e in lidar_errors)/len(lidar_errors) if lidar_errors else None,lidar_matched_ids=sorted(lidar_ids),local_costmap_frames=len(costmaps),local_costmap_human_surface_evidence=costmap_evidence,active_command_count=len(commands),nvidia=True,shutdown_clean=True,goal_sample_sim=final_sample['time'],post_goal_recording_tail_xy_drift_m=math.hypot(complete[-1]['poses']['mecanum_robot']['x']-final_sample['poses']['mecanum_robot']['x'],complete[-1]['poses']['mecanum_robot']['y']-final_sample['poses']['mecanum_robot']['y']),preserved_old_files=len(preserved)-len(allowed),original_user_staging_and_outside_preserved=True,scope='One-seed small Nav2 integration; goal outcome independent from geometry. Slow/near is descriptive, not proof of social yielding.')
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n');print(json.dumps(validation,indent=2))
