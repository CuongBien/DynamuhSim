#!/usr/bin/env python3
"""Audit actual SceneBroadcaster geometry, worker clock, RGB and LiDAR."""
import argparse,bisect,hashlib,json,math,os,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
import numpy as np,shapely
from PIL import Image
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent;REPO=SOCIAL.parent.parent
p=argparse.ArgumentParser();p.add_argument('--live',type=Path,required=True);a=p.parse_args();out=a.live.resolve()
load=lambda p:json.loads(p.read_text());lines=lambda p:[json.loads(l) for l in p.open()];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
boxes=load(SOCIAL/'warehouse/layout.json')['obstacles'];assert len(boxes)==42
polygons=np.array([shapely.box(b['x']-b['sx']/2,b['y']-b['sy']/2,b['x']+b['sx']/2,b['y']+b['sy']/2) for b in boxes],dtype=object)
def measured_gaps(before,after,robot_before,robot_after):
    # Arrays are time x human x XY. Independent closest-approach vector algebra
    # and GEOS segment-to-polygon distance; no guard geometry imports.
    a=np.asarray(before);b=np.asarray(after);ra=np.asarray(robot_before);rb=np.asarray(robot_after)
    delta=b-a;pair=1e9
    for i in range(a.shape[1]):
        for j in range(i):
            rel=a[:,i]-a[:,j];vel=delta[:,i]-delta[:,j];den=np.sum(vel*vel,axis=1)
            t=np.clip(np.divide(-np.sum(rel*vel,axis=1),den,out=np.zeros_like(den),where=den>1e-25),0,1)
            pair=min(pair,float(np.min(np.linalg.norm(rel+t[:,None]*vel,axis=1)-.72)))
    rel=a-ra[:,None,:];vel=delta-(rb-ra)[:,None,:];den=np.sum(vel*vel,axis=2)
    t=np.clip(np.divide(-np.sum(rel*vel,axis=2),den,out=np.zeros_like(den),where=den>1e-25),0,1)
    robot=float(np.min(np.linalg.norm(rel+t[:,:,None]*vel,axis=2)-.61))
    segments=shapely.linestrings(np.stack((a,b),axis=2).reshape(-1,2,2));static=float(np.min(shapely.distance(segments[:,None],polygons[None,:]))-.36)
    return dict(pair=pair,robot=robot,static=static)
result=load(out/'result.json');assert result['status'] in ['RECORDED','ERROR','EXPECTED_FAULT_RECORDED'],result.get('error')
expected_fault=result['status']=='EXPECTED_FAULT_RECORDED';error_prefix=result['status']!='RECORDED';assert not error_prefix or result['faults'],'Infrastructure errors are not behavior prefixes'
fault_mode=False;dt=result['native_dt']
for name,digest in result['source_sha256'].items():assert sha(REPO/name)==digest,name
runtime=REPO/result['runtime'];worker_raw=lines(runtime/'worker_trajectory.jsonl');worker=[s for s in worker_raw if 'state_time' in s];gt=lines(out/'ground_truth.jsonl');poses=lines(out/'gazebo_poses.jsonl');scans=lines(out/'scans.jsonl');odom=lines(out/'odom.jsonl')
assert worker and len(gt)>100 and len(poses)>100 and len(scans)>10
assert result['ready']['obstacles']==42 and result['ready']['native_dt']==result['native_dt'] and len(result['ready']['humans'])==6
assert result['rendering']['hardware_verified'] and 'NVIDIA' in result['rendering']['renderer']
assert result['cleanup']['status']=='PASS' and not result['cleanup']['remaining'] and result['gazebo_clean_exit'] and result['server_shutdown']['status']=='STOPPED'
assert any('NVIDIA' in s for s in result['cleanup']['ogre_renderer_lines'])
assert not any(s in (runtime/'worker.log').read_text() for s in ['Traceback','ConnectionResetError','BrokenPipeError'])
assert error_prefix or not result['faults']
assert not result['container_remaining']
assert all([h['id'] for h in s['humans']]==list(range(1,7)) for s in gt)
for x,y in zip(worker,worker[1:]):assert abs(y['time']-x['time']-dt)<1e-6
# Geometry mapping verifies existing world models; actor/proxy additions are separate.
world=ET.parse(runtime/'warehouse.sdf').getroot().find('world')
for box in boxes:
    model=world.find("model[@name='"+box['name']+"']");assert model is not None
    pos=[float(v) for v in model.find('pose').text.split()];size=[float(v) for v in model.find('.//collision/geometry/box/size').text.split()]
    assert max(abs(pos[i]-box[k]) for i,k in enumerate(['x','y']))<1e-9
    assert abs(size[0]-box['sx'])<1e-9 and abs(size[1]-box['sy'])<1e-9
required=['mecanum_robot']+[f'common_{i}_proxy' for i in range(1,7)]
complete=[s for s in poses if all(k in s['poses'] for k in required)];assert len(complete)>100
assert all(b['time']>=a['time'] for a,b in zip(complete,complete[1:]))
worker_times=[s['time'] for s in worker];pose_times=[s['time'] for s in complete]
errors=[];robot_prediction=[];feedback=[]
for row in complete:
    index=bisect.bisect_right(worker_times,row['time'])-1
    if index<0:continue
    w=worker[index];alpha=(row['time']-w['time'])/dt
    if not -.00001<=alpha<=1.00001:continue
    for old,new in zip(w['before'],w['humans']):
        actual=row['poses'][f'common_{old["id"]}_proxy'];expected=[old[k]+alpha*(new[k]-old[k]) for k in ['x','y']]
        errors.append(math.hypot(actual['x']-expected[0],actual['y']-expected[1]))
for w in worker:
    i=min(range(max(0,bisect.bisect_left(pose_times,w['time']+dt)-1),min(len(complete),bisect.bisect_left(pose_times,w['time']+dt)+2)),key=lambda i:abs(pose_times[i]-w['time']-dt))
    if abs(pose_times[i]-w['time']-dt)<.015:
        r=w['robot_input'];actual=complete[i]['poses']['mecanum_robot'];robot_prediction.append(math.hypot(actual['x']-r['x']-dt*r['vx'],actual['y']-r['y']-dt*r['vy']))
assert len(errors)>100 and max(errors)<.025
before=[[[s['poses'][f'common_{i}_proxy'][k] for k in ['x','y']] for i in range(1,7)] for s in complete[:-1]]
after=[[[s['poses'][f'common_{i}_proxy'][k] for k in ['x','y']] for i in range(1,7)] for s in complete[1:]]
rb=[[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete[:-1]];re=[[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete[1:]]
full_recording_gaps=measured_gaps(before,after,rb,re)
assessment_end=min(worker[0]['time']+10.,result['faults'][0]['time'] if error_prefix else worker[0]['time']+10.)
mask=np.array([row['time']<=assessment_end+1e-8 for row in complete[1:]])
gaps=measured_gaps(np.asarray(before)[mask],np.asarray(after)[mask],np.asarray(rb)[mask],np.asarray(re)[mask])
# Finite observed pose segments, not a claim of continuous physics proof.

assert abs(float(world.find('physics/max_step_size').text)-.0025)<1e-12
for hid in range(1,7):
    model=world.find("model[@name='common_"+str(hid)+"_proxy']")
    assert abs(float(model.find('.//collision/geometry/cylinder/radius').text)-.36)<1e-12
    assert abs(float(model.find('.//visual/geometry/cylinder/radius').text)-.36)<1e-12
assert len(world.findall('actor'))==6
for a,b in zip(worker,worker[1:]):assert a['humans']==b['before']
max_speed=max_yaw_rate=0.;human_path=0.
for before,after in zip(complete,complete[1:]):
    sample_dt=after['time']-before['time']
    if sample_dt<=1e-10:continue
    for hid in range(1,7):
        a=before['poses'][f'common_{hid}_proxy'];b=after['poses'][f'common_{hid}_proxy'];dist=math.hypot(b['x']-a['x'],b['y']-a['y']);human_path+=dist
        max_speed=max(max_speed,dist/sample_dt);max_yaw_rate=max(max_yaw_rate,abs(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])))/sample_dt)
assert max_speed<=1.00001 and max_yaw_rate<=1.57001
xy=np.array([[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete]);displacement=xy[-1]-xy[0]
assert error_prefix or result['sim_seconds_observed']>=10.
initial=load(HERE.parent/'common_response/inputs'/(result['case']+'.json'))['robot_trace'][0]
for o in odom:
    i=min(max(bisect.bisect_left(pose_times,o['time']),0),len(complete)-1)
    if abs(pose_times[i]-o['time'])<.02:
        r=complete[i]['poses']['mecanum_robot'];x=initial['x']+math.cos(initial['yaw'])*o['x']-math.sin(initial['yaw'])*o['y'];y=initial['y']+math.sin(initial['yaw'])*o['x']+math.cos(initial['yaw'])*o['y'];feedback.append(math.hypot(r['x']-x,r['y']-y))
assert feedback and max(feedback)<.04
assert robot_prediction and max(robot_prediction)<.2
freeze=None;arrivals=[]
assert len(worker_raw)==len(worker)+(1 if error_prefix else 0)
trace=load(HERE.parent/'common_response/inputs'/(result['case']+'.json'))['robot_trace'];trace_error=[]
for row in complete:
    elapsed=row['time']-worker[0]['time']
    if 0<=elapsed<=10:
        reference=trace[min(round(elapsed/.05),len(trace)-1)];r=row['poses']['mecanum_robot'];trace_error.append(math.hypot(r['x']-reference['x'],r['y']-reference['y']))
# Exact ray intersections from measured Gazebo proxies, all factory boxes, and
# actual robot pose; only beams whose nearest predicted target is a human count.
lidar_errors=[];lidar_ids=set()
for scan in scans:
    i=bisect.bisect_left(pose_times,scan['time']);indices=range(max(0,i-1),min(len(complete),i+2));i=min(indices,key=lambda i:abs(pose_times[i]-scan['time']))
    if abs(pose_times[i]-scan['time'])>.025:continue
    s=complete[i];r=s['poses']['mecanum_robot'];angles=scan['angle_min']+np.arange(len(scan['ranges']))*scan['angle_increment']+r['yaw'];dx=np.cos(angles);dy=np.sin(angles);static=np.full(len(angles),np.inf)
    for b in boxes:
        near=np.full(len(angles),-np.inf);far=np.full(len(angles),np.inf)
        for origin,direction,center,size in [(r['x'],dx,b['x'],b['sx']),(r['y'],dy,b['y'],b['sy'])]:
            parallel=np.abs(direction)<1e-12;low=center-size/2;high=center+size/2
            t1=np.divide(low-origin,direction,out=np.zeros_like(direction),where=~parallel);t2=np.divide(high-origin,direction,out=np.zeros_like(direction),where=~parallel)
            near=np.maximum(near,np.where(parallel,-np.inf if low<=origin<=high else np.inf,np.minimum(t1,t2)))
            far=np.minimum(far,np.where(parallel,np.inf if low<=origin<=high else -np.inf,np.maximum(t1,t2)))
        static=np.minimum(static,np.where((far>=np.maximum(near,0))&(near>=0),near,np.inf))
    dynamic=np.full(len(angles),np.inf);labels=np.zeros(len(angles),dtype=int)
    for hid in range(1,7):
        h=s['poses'][f'common_{hid}_proxy'];ox,oy=r['x']-h['x'],r['y']-h['y'];dot=ox*dx+oy*dy;disc=dot*dot-(ox*ox+oy*oy-.36**2);hit=-dot-np.sqrt(np.maximum(disc,0));hit=np.where((disc>=0)&(hit>0),hit,np.inf)
        closer=hit<dynamic;labels=np.where(closer,hid,labels);dynamic=np.minimum(dynamic,hit)
    selected=(dynamic>max(.14,scan['range_min']))&(dynamic<min(3.4,scan['range_max']))&(static>dynamic+.03)
    measured=np.array([np.nan if v is None else v for v in scan['ranges']]);error=np.abs(measured-dynamic);error=np.where(np.isfinite(error),error,10.)
    lidar_errors.extend(error[selected].tolist());lidar_ids.update(int(v) for v in labels[selected&(error<.15)])
assert len(lidar_errors)>50 and sum(e<.15 for e in lidar_errors)/len(lidar_errors)>.7 and len(lidar_ids)>=2
interlock=None
if expected_fault:
    assert result['interlock_check']['clock_before']==result['interlock_check']['clock_after']
    assert result['interlock_check']['manual_resume_attempted'] and abs(result['interlock_check']['repaused_clock_delta_s'])<1e-9
    assert 0<=result['interlock_check']['resume_clock_delta_s']<=.00500001
    assert len(result['faults'])==1 and result['faults'][0]['world_pause_requested'] and result['faults'][0]['restart_required']
    fault=result['faults'][0];reference=fault['robot_input'];frozen=fault['last_applied_humans'];post=[row for row in complete if row['time']>=fault['time']-1e-8]
    assert post,'No actual pose at/after fault'
    robot_drift=max(math.hypot(row['poses']['mecanum_robot']['x']-reference['x'],row['poses']['mecanum_robot']['y']-reference['y']) for row in post)
    human_drift=max(math.hypot(row['poses'][f'common_{h["id"]}_proxy']['x']-h['x'],row['poses'][f'common_{h["id"]}_proxy']['y']-h['y']) for row in post for h in frozen)
    assert robot_drift<=.00500001 and human_drift<1e-8
    assert len(worker)==result['fault_after_updates']==55
    interlock=dict(**result['interlock_check'],post_fault_actual_samples=len(post),robot_drift_from_fault_input_max_m=robot_drift,human_frozen_error_max_m=human_drift)
assert min(full_recording_gaps.values())>=-.005,'Actual recording has penetration beyond5mm'
with Image.open(out/'camera_preview_4x.gif') as im:
    duration=0
    for frame in range(im.n_frames):im.seek(frame);im.load();duration+=im.info.get('duration',0)
    assert duration==result['media']['gif_duration_ms']
for p in out.glob('*.png'):
    with Image.open(p) as im:im.verify()

cp=HERE/'generated/checkpoint';preserved=load(cp/'files.json');allowed={'experiments/social_navigation/'+name for name in ['README.md','NEXT_STEPS.md','COMMIT_HISTORY.md','FRAMEWORK_COMPARISON.md']}
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)
for name,digest in preserved.items():
    if name not in allowed:assert sha(REPO/name)==digest,name
assert not subprocess.check_output(['docker','ps','-aq','--filter','name=dynamuhsim-response-gz-'],text=True).strip()
projections=[w['motion_guard']['projection'] for w in worker];projection_stats=dict(margins_used_m=sorted(set(v['margin_m'] for v in projections)),cycles_max=max(v['cycles'] for v in projections),compute_ms_max=max(v['compute_ms'] for v in projections),altered_proposals=sum(v['velocity_change_norm']>1e-6 for v in projections),native_plus_guard_compute_ms_max=max(w['compute_ms'] for w in worker))
validation=dict(status='PASS_TERMINAL_PAUSE_INTERLOCK' if expected_fault else 'AUDITED_ERROR_PREFIX' if error_prefix else 'PASS_RESPONSE_LIVE',error=result.get('error'),observed_seconds=gt[-1]['time']-gt[0]['time'],worker_updates=len(worker),projection=projection_stats,population=6,native_dt=dt,physics_step_s=.0025,
    actual_pose_samples=len(complete),proxy_comparison_samples=len(errors),proxy_position_error_max_m=max(errors),actual_swept_gap_m=gaps,full_recording_including_teardown_gap_m=full_recording_gaps,assessment_end_sim=assessment_end,
    robot_displacement_xy_m=displacement.tolist(),robot_prediction_error_max_m=max(robot_prediction),robot_odom_feedback_error_max_m=max(feedback),
    human_speed_max_mps=max_speed,human_yaw_rate_max_radps=max_yaw_rate,human_path_total_m=human_path,
    actual_goal_arrivals=arrivals,freeze_check=freeze,lidar_human_beams=len(lidar_errors),lidar_within_15cm_fraction=sum(e<.15 for e in lidar_errors)/len(lidar_errors),lidar_matched_ids=sorted(lidar_ids),
    pause_check=result.get('pause_check'),interlock=interlock,nvidia=True,fresh_ogre=True,shutdown_clean=True,owned_process_container_cleanup=True,
    gui_user_confirmation=load(out/'gui_confirmation.json') if (out/'gui_confirmation.json').exists() else None,
    preserved_old_source_evidence_files=len(preserved)-len(allowed),intentional_status_documents=sorted(allowed),original_user_index_outside_preserved=True,
    model=result['model'],case=result['case'],robot_script_position_error_max_m=max(trace_error),geometry_outcome='OVERLAP_GT_5MM' if min(gaps.values())<-.005 else 'NO_OVERLAP_GT_5MM',scope='SocNavGym response projection v2;6 individuals, bounded actual robot command, optional terminal simulation-pause test; no Nav2/group/general safety claim')
import gzip
for path in [runtime/'worker_trajectory.jsonl',runtime/'warehouse.sdf',runtime/'worker.log',runtime/'ogre2_session.log',out/'launch.log',*out.glob('*.jsonl')]:
    if path.exists():
        target=out/(path.name+'.gz');target.write_bytes(gzip.compress(path.read_bytes(),mtime=0));assert gzip.decompress(target.read_bytes())==path.read_bytes()
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
print(json.dumps(validation,indent=2))
