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
            pair=min(pair,float(np.min(np.linalg.norm(rel+t[:,None]*vel,axis=1)-.6)))
    rel=a-ra[:,None,:];vel=delta-(rb-ra)[:,None,:];den=np.sum(vel*vel,axis=2)
    t=np.clip(np.divide(-np.sum(rel*vel,axis=2),den,out=np.zeros_like(den),where=den>1e-25),0,1)
    robot=float(np.min(np.linalg.norm(rel+t[:,:,None]*vel,axis=2)-.55))
    segments=shapely.linestrings(np.stack((a,b),axis=2).reshape(-1,2,2));static=float(np.min(shapely.distance(segments[:,None],polygons[None,:]))-.3)
    return dict(pair=pair,robot=robot,static=static)
result=load(out/'result.json');assert result['status'] in ['RECORDED','EXPECTED_FAULT_RECORDED'],result.get('error')
fault_mode=result['status']=='EXPECTED_FAULT_RECORDED'
for name,digest in result['source_sha256'].items():assert sha(REPO/name)==digest,name
runtime=REPO/result['runtime'];worker_raw=lines(runtime/'worker_trajectory.jsonl');worker=[s for s in worker_raw if 'error' not in s];gt=lines(out/'ground_truth.jsonl');poses=lines(out/'gazebo_poses.jsonl');scans=lines(out/'scans.jsonl');odom=lines(out/'odom.jsonl')
assert worker and len(gt)>100 and len(poses)>100 and len(scans)>10
assert result['ready']['obstacles']==42 and result['ready']['native_dt']==.25 and len(result['ready']['humans'])==4
assert result['rendering']['hardware_verified'] and 'NVIDIA' in result['rendering']['renderer']
assert result['cleanup']['status']=='PASS' and not result['cleanup']['remaining'] and result['gazebo_clean_exit'] and result['server_shutdown']['status']=='STOPPED'
assert any('NVIDIA' in s for s in result['cleanup']['ogre_renderer_lines'])
assert not any(s in (runtime/'worker.log').read_text() for s in ['Traceback','ConnectionResetError','BrokenPipeError'])
if not fault_mode:assert result['pause_check']==dict(time_delta=0.,step_delta=0)
assert not result['container_remaining']
assert all([h['id'] for h in s['humans']]==list(range(1,5)) for s in gt)
for x,y in zip(worker,worker[1:]):assert abs(y['time']-x['time']-.25)<1e-6
# Geometry mapping verifies existing world models; actor/proxy additions are separate.
world=ET.parse(runtime/'warehouse.sdf').getroot().find('world')
for box in boxes:
    model=world.find("model[@name='"+box['name']+"']");assert model is not None
    pos=[float(v) for v in model.find('pose').text.split()];size=[float(v) for v in model.find('.//collision/geometry/box/size').text.split()]
    assert max(abs(pos[i]-box[k]) for i,k in enumerate(['x','y']))<1e-9
    assert abs(size[0]-box['sx'])<1e-9 and abs(size[1]-box['sy'])<1e-9
required=['mecanum_robot']+[f'complexitynav_{i}_proxy' for i in range(1,5)]
complete=[s for s in poses if all(k in s['poses'] for k in required)];assert len(complete)>100
assert all(b['time']>=a['time'] for a,b in zip(complete,complete[1:]))
worker_times=[s['time'] for s in worker];pose_times=[s['time'] for s in complete]
errors=[];robot_prediction=[];feedback=[]
for row in complete:
    index=bisect.bisect_right(worker_times,row['time'])-1
    if index<0:continue
    w=worker[index];alpha=(row['time']-w['time'])/.25
    if not -.00001<=alpha<=1.00001:continue
    for old,new in zip(w['before'],w['humans']):
        actual=row['poses'][f'complexitynav_{old["id"]}_proxy'];expected=[old[k]+alpha*(new[k]-old[k]) for k in ['x','y']]
        errors.append(math.hypot(actual['x']-expected[0],actual['y']-expected[1]))
for w in worker:
    i=min(range(max(0,bisect.bisect_left(pose_times,w['time']+.25)-1),min(len(complete),bisect.bisect_left(pose_times,w['time']+.25)+2)),key=lambda i:abs(pose_times[i]-w['time']-.25))
    if abs(pose_times[i]-w['time']-.25)<.015:
        r=w['robot_input'];actual=complete[i]['poses']['mecanum_robot'];robot_prediction.append(math.hypot(actual['x']-r['x']-.25*r['vx'],actual['y']-r['y']-.25*r['vy']))
assert len(errors)>100 and max(errors)<.025
before=[[[s['poses'][f'complexitynav_{i}_proxy'][k] for k in ['x','y']] for i in range(1,5)] for s in complete[:-1]]
after=[[[s['poses'][f'complexitynav_{i}_proxy'][k] for k in ['x','y']] for i in range(1,5)] for s in complete[1:]]
rb=[[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete[:-1]];re=[[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete[1:]]
gaps=measured_gaps(before,after,rb,re);assert min(gaps.values())>=-.005,'Actual proxy penetration beyond5mm'
# Finite observed pose segments, not a claim of continuous physics proof.

assert abs(float(world.find('physics/max_step_size').text)-.0025)<1e-12
for hid in range(1,5):
    model=world.find("model[@name='complexitynav_"+str(hid)+"_proxy']")
    assert abs(float(model.find('.//collision/geometry/cylinder/radius').text)-.3)<1e-12
    assert abs(float(model.find('.//visual/geometry/cylinder/radius').text)-.3)<1e-12
assert len(world.findall('actor'))==4
for a,b in zip(worker,worker[1:]):assert a['humans']==b['before']
max_speed=max_yaw_rate=0.;human_path=0.
for before,after in zip(complete,complete[1:]):
    dt=after['time']-before['time']
    if dt<=1e-10:continue
    for hid in range(1,5):
        a=before['poses'][f'complexitynav_{hid}_proxy'];b=after['poses'][f'complexitynav_{hid}_proxy'];dist=math.hypot(b['x']-a['x'],b['y']-a['y']);human_path+=dist
        max_speed=max(max_speed,dist/dt);max_yaw_rate=max(max_yaw_rate,abs(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])))/dt)
assert max_speed<=1.00001 and max_yaw_rate<=1.57001
xy=np.array([[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete]);displacement=xy[-1]-xy[0]
if not fault_mode:
    assert displacement[0]>.4 and displacement[1]>.2
    assert max(s['robot_input']['vx'] for s in worker)>.08 and max(s['robot_input']['vy'] for s in worker)>.08
    assert result['sim_seconds_observed']>=60.
else:assert np.linalg.norm(displacement)<.01
for o in odom:
    i=min(max(bisect.bisect_left(pose_times,o['time']),0),len(complete)-1)
    if abs(pose_times[i]-o['time'])<.02:
        r=complete[i]['poses']['mecanum_robot'];feedback.append(math.hypot(r['x']-(-27+o['x']),r['y']-o['y']))
assert feedback and max(feedback)<.03
assert robot_prediction and max(robot_prediction)<.05
freeze=None;arrivals=[]
if fault_mode:
    assert len(result['faults'])==1 and 'INJECTED' in result['faults'][0]['error']
    assert len(worker)==result['fault_after_updates']==20 and len(worker_raw)==21 and worker_raw[-1]['fatal']
    begin=result['freeze_window']['start_sim']+.05;end=result['freeze_window']['end_sim']
    stopped=[s for s in complete if begin<=s['time']<=end];assert len(stopped)>20
    error=max(math.hypot(s['poses'][name]['x']-stopped[0]['poses'][name]['x'],s['poses'][name]['y']-stopped[0]['poses'][name]['y']) for s in stopped for name in [f'complexitynav_{i}_proxy' for i in range(1,5)])
    assert error<1e-8
    freeze=dict(samples=len(stopped),span_s=stopped[-1]['time']-stopped[0]['time'],position_error_max_m=error)
else:
    assert not result['faults'] and len(worker_raw)==len(worker)
    for w in worker:
        if w['state_time']>pose_times[-1]:continue
        i=min(range(max(0,bisect.bisect_left(pose_times,w['state_time'])-1),min(len(complete),bisect.bisect_left(pose_times,w['state_time'])+2)),key=lambda i:abs(pose_times[i]-w['state_time']))
        if abs(pose_times[i]-w['state_time'])>.02:continue
        for a,b in zip(w['before'],w['humans']):
            if b['completed_legs']>a['completed_legs']:
                h=complete[i]['poses'][f'complexitynav_{a["id"]}_proxy'];distance=math.hypot(h['x']-a['gx'],h['y']-a['gy']);assert distance<.325
                arrivals.append(dict(time=w['state_time'],id=a['id'],goal=[a['gx'],a['gy']],actual_goal_error_m=distance))
    assert {r['id'] for r in arrivals}==set(range(1,5))
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
    for hid in range(1,5):
        h=s['poses'][f'complexitynav_{hid}_proxy'];ox,oy=r['x']-h['x'],r['y']-h['y'];dot=ox*dx+oy*dy;disc=dot*dot-(ox*ox+oy*oy-.3**2);hit=-dot-np.sqrt(np.maximum(disc,0));hit=np.where((disc>=0)&(hit>0),hit,np.inf)
        closer=hit<dynamic;labels=np.where(closer,hid,labels);dynamic=np.minimum(dynamic,hit)
    selected=(dynamic>max(.14,scan['range_min']))&(dynamic<min(3.4,scan['range_max']))&(static>dynamic+.03)
    measured=np.array([np.nan if v is None else v for v in scan['ranges']]);error=np.abs(measured-dynamic);error=np.where(np.isfinite(error),error,10.)
    lidar_errors.extend(error[selected].tolist());lidar_ids.update(int(v) for v in labels[selected&(error<.15)])
assert len(lidar_errors)>50 and sum(e<.15 for e in lidar_errors)/len(lidar_errors)>.7 and len(lidar_ids)>=2
with Image.open(out/'camera_preview_4x.gif') as im:
    duration=0
    for frame in range(im.n_frames):im.seek(frame);im.load();duration+=im.info.get('duration',0)
    assert duration==result['media']['gif_duration_ms']
for p in out.glob('*.png'):
    with Image.open(p) as im:im.verify()

cp=HERE/'generated/checkpoint';preserved=load(cp/'files.json');allowed={'experiments/social_navigation/'+n for n in ['README.md','NEXT_STEPS.md','COMMIT_HISTORY.md']}
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)
for name,digest in preserved.items():
    if name not in allowed:assert sha(REPO/name)==digest,name
assert not subprocess.check_output(['docker','ps','-aq','--filter','name=dynamuhsim-cnav-live-'],text=True).strip()
validation=dict(status='PASS_EXPECTED_FAULT' if fault_mode else 'PASS_LIVE_PILOT',worker_updates=len(worker),population=4,native_dt=.25,physics_step_s=.0025,
    actual_pose_samples=len(complete),proxy_comparison_samples=len(errors),proxy_position_error_max_m=max(errors),actual_swept_gap_m=gaps,
    robot_displacement_xy_m=displacement.tolist(),robot_prediction_error_max_m=max(robot_prediction),robot_odom_feedback_error_max_m=max(feedback),
    human_speed_max_mps=max_speed,human_yaw_rate_max_radps=max_yaw_rate,human_path_total_m=human_path,
    actual_goal_arrivals=arrivals,freeze_check=freeze,lidar_human_beams=len(lidar_errors),lidar_within_15cm_fraction=sum(e<.15 for e in lidar_errors)/len(lidar_errors),lidar_matched_ids=sorted(lidar_ids),
    pause_check=result.get('pause_check'),nvidia=True,fresh_ogre=True,shutdown_clean=True,owned_process_container_cleanup=True,
    gui_user_confirmation=load(out/'gui_confirmation.json') if (out/'gui_confirmation.json').exists() else None,
    preserved_old_source_evidence_files=len(preserved)-3,original_user_index_outside_preserved=True,
    scope='One4-human mixed-policy local-aisle pilot, modified route/ORCA-projection, no Nav2 or dense-crowd/ranking/general-safety claim')
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
print(json.dumps(validation,indent=2))
