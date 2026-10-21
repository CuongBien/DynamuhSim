#!/usr/bin/env python3
"""Independent Shapely/sensor validation of offline and live lifecycle runs."""
import argparse,bisect,hashlib,json,math,os,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
import numpy as np,shapely
from PIL import Image
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent;REPO=SOCIAL.parent.parent
p=argparse.ArgumentParser();p.add_argument('--live',type=Path);p.add_argument('--offline-dir',type=Path,default=HERE/'reports/offline02');p.add_argument('--offline-only',action='store_true');a=p.parse_args();out=a.live.resolve() if a.live else None;load=lambda p:json.loads(p.read_text());lines=lambda p:[json.loads(l) for l in p.open()];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
layout=load(SOCIAL/'warehouse/layout.json');boxes=layout['obstacles'];assert len(boxes)==42
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
# Independent preflight check retains failed offline01 and validates all3 offline02.
offline=a.offline_dir.resolve();op=load(offline/'protocol.json')
for name,digest in op['source_sha256'].items():assert sha(Path(name))==digest,name
checks=[]
for case in op['cases']:
    result=load(offline/(case+'.json'));rows=load(offline/(case+'_trajectory.json'));assert result['status']=='PASS' and len(rows)==1200
    before=[[[h['x'],h['y']] for h in r['before']] for r in rows];after=[[[h['x'],h['y']] for h in r['humans']] for r in rows]
    rb=[[r['robot']['x'],r['robot']['y']] for r in rows];re=[[r['robot']['x']+.1*r['robot']['vx'],r['robot']['y']+.1*r['robot']['vy']] for r in rows]
    assert np.max(np.abs(np.asarray(re[:-1])-np.asarray(rb[1:])))<1e-8
    gaps=measured_gaps(before,after,rb,re);assert min(gaps.values())>=-1e-7
    assert result['formations']>0
    checks.append(dict(case=case,steps=1200,formation_calls=result['formations'],independent_swept_gap_m=gaps))
(offline/'independent_validation.json').write_text(json.dumps(dict(status='PASS',cases=checks,method='GEOS/Shapely for all42AABBs; vector relative closest approach for pair and prescribed robot motion'),indent=2)+'\n')
if a.offline_only:
    print(json.dumps(dict(status='PASS',offline_cases=checks)));raise SystemExit(0)
if out is None:p.error('--live is required unless --offline-only is used')
result=load(out/'result.json');assert result['status']=='RECORDED',result.get('error')
for name,digest in result['source_sha256'].items():assert sha(REPO/name)==digest,name
runtime=REPO/result['runtime'];worker=lines(runtime/'worker_trajectory.jsonl');gt=lines(out/'ground_truth.jsonl');poses=lines(out/'gazebo_poses.jsonl');scans=lines(out/'scans.jsonl');odom=lines(out/'odom.jsonl')
assert worker and len(gt)>500 and len(poses)>500 and len(scans)>50
assert result['ready']['obstacles']==42 and result['ready']['wall_observation_shape']==[336]
assert result['rendering']['hardware_verified'] and 'NVIDIA' in result['rendering']['renderer']
assert result['cleanup']['status']=='PASS' and not result['cleanup']['remaining'] and result['gazebo_clean_exit'] and result['server_shutdown']['status']=='STOPPED'
assert any('NVIDIA' in s for s in result['cleanup']['ogre_renderer_lines'])
assert not any(s in (runtime/'worker.log').read_text() for s in ['Traceback','ConnectionResetError','BrokenPipeError'])
assert result['pause_check']==dict(time_delta=0.,step_delta=0)
assert all([h['id'] for h in s['humans']]==list(range(1,7)) for s in gt)
for x,y in zip(worker,worker[1:]):assert abs(y['time']-x['time']-.1)<1e-6
# Geometry mapping verifies existing world models; actor/proxy additions are separate.
world=ET.parse(runtime/'warehouse.sdf').getroot().find('world')
for box in boxes:
    model=world.find("model[@name='"+box['name']+"']");assert model is not None
    pos=[float(v) for v in model.find('pose').text.split()];size=[float(v) for v in model.find('.//collision/geometry/box/size').text.split()]
    assert max(abs(pos[i]-box[k]) for i,k in enumerate(['x','y']))<1e-9
    assert abs(size[0]-box['sx'])<1e-9 and abs(size[1]-box['sy'])<1e-9
required=['mecanum_robot']+[f'lifecycle_{i}_proxy' for i in range(1,7)]
complete=[s for s in poses if all(k in s['poses'] for k in required)];assert len(complete)>500
assert all(b['time']>=a['time'] for a,b in zip(complete,complete[1:]))
worker_times=[s['time'] for s in worker];pose_times=[s['time'] for s in complete]
errors=[];robot_prediction=[];feedback=[]
for row in complete:
    index=bisect.bisect_right(worker_times,row['time'])-1
    if index<0:continue
    w=worker[index];alpha=(row['time']-w['time'])/.1
    if not -.00001<=alpha<=1.00001:continue
    for old,new in zip(w['before'],w['humans']):
        actual=row['poses'][f'lifecycle_{old["id"]}_proxy'];expected=[old[k]+alpha*(new[k]-old[k]) for k in ['x','y']]
        errors.append(math.hypot(actual['x']-expected[0],actual['y']-expected[1]))
for w in worker:
    i=min(range(max(0,bisect.bisect_left(pose_times,w['time']+.1)-1),min(len(complete),bisect.bisect_left(pose_times,w['time']+.1)+2)),key=lambda i:abs(pose_times[i]-w['time']-.1))
    if abs(pose_times[i]-w['time']-.1)<.015:
        r=w['robot_input'];actual=complete[i]['poses']['mecanum_robot'];robot_prediction.append(math.hypot(actual['x']-r['x']-.1*r['vx'],actual['y']-r['y']-.1*r['vy']))
assert len(errors)>1000 and max(errors)<.025
before=[[[s['poses'][f'lifecycle_{i}_proxy'][k] for k in ['x','y']] for i in range(1,7)] for s in complete[:-1]]
after=[[[s['poses'][f'lifecycle_{i}_proxy'][k] for k in ['x','y']] for i in range(1,7)] for s in complete[1:]]
rb=[[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete[:-1]];re=[[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete[1:]]
gaps=measured_gaps(before,after,rb,re);assert min(gaps.values())>=-.005,'Actual proxy penetration beyond5mm'
# Finite observed pose segments, not a claim of continuous physics proof.
signature=lambda s:[(g['id'],g['type'],g['members']) for g in s['groups']]
events=[];previous=[]
for s in gt:
    ids=[i for g in s['groups'] for i in g['members']];assert len(ids)==len(set(ids)) and set(ids)<=set(range(1,7))
    current=signature(s)
    if current!=previous:events.append(dict(time=s['time'],step=s['step'],before=previous,after=current));previous=current
assert events and any(not e['before'] and e['after'] for e in events),'No visible formation transition'
assert len(events)>=2,'No second lifecycle transition observed'
assert len(set(i for e in events for g in e['after'] for i in g[2]))>=3
release=[]
for event in events:
    old={g[0]:set(g[2]) for g in event['before']};new={g[0]:set(g[2]) for g in event['after']}
    released=sorted(set().union(*(members-new.get(gid,set()) for gid,members in old.items()))) if old else []
    if not released:continue
    i=min(range(len(complete)),key=lambda i:abs(pose_times[i]-event['time']));j=min(range(len(complete)),key=lambda i:abs(pose_times[i]-event['time']-5.))
    moves={hid:math.hypot(complete[j]['poses'][f'lifecycle_{hid}_proxy']['x']-complete[i]['poses'][f'lifecycle_{hid}_proxy']['x'],complete[j]['poses'][f'lifecycle_{hid}_proxy']['y']-complete[i]['poses'][f'lifecycle_{hid}_proxy']['y']) for hid in released}
    release.append(dict(time=event['time'],released_ids=released,displacement_within_5s_m=moves))
    assert max(moves.values())>.3,'Membership release did not produce measured human departure'
# Also measure trajectory continuity/speed on actual Gazebo proxy poses.
max_speed=max_yaw_rate=0.;human_path=0.
for previous,current in zip(complete,complete[1:]):
    dt=current['time']-previous['time']
    if dt<=1e-10:continue
    for hid in range(1,7):
        before=previous['poses'][f'lifecycle_{hid}_proxy'];after=current['poses'][f'lifecycle_{hid}_proxy'];distance=math.hypot(after['x']-before['x'],after['y']-before['y']);human_path+=distance
        max_speed=max(max_speed,distance/dt);max_yaw_rate=max(max_yaw_rate,abs(math.atan2(math.sin(after['yaw']-before['yaw']),math.cos(after['yaw']-before['yaw'])))/dt)
assert max_speed<=.80001 and max_yaw_rate<=1.57001

inputs=[w['robot_input'] for w in worker];assert max(r['vx'] for r in inputs)>.08 and max(r['vy'] for r in inputs)>.08
xy=np.array([[s['poses']['mecanum_robot']['x'],s['poses']['mecanum_robot']['y']] for s in complete]);displacement=xy[-1]-xy[0]
assert displacement[0]>.20 and displacement[1]>.12
for o in odom:
    i=bisect.bisect_left(pose_times,o['time']);i=min(max(i,0),len(complete)-1)
    if abs(pose_times[i]-o['time'])<.02:
        r=complete[i]['poses']['mecanum_robot'];feedback.append(math.hypot(r['x']-(-27+o['x']),r['y']-o['y']))
assert feedback and max(feedback)<.03
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
        h=s['poses'][f'lifecycle_{hid}_proxy'];ox,oy=r['x']-h['x'],r['y']-h['y'];dot=ox*dx+oy*dy;disc=dot*dot-(ox*ox+oy*oy-.36**2);hit=-dot-np.sqrt(np.maximum(disc,0));hit=np.where((disc>=0)&(hit>0),hit,np.inf)
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
cp=HERE/'generated/checkpoint'
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)
for name,digest in load(cp/'hashes.json').items():assert sha(REPO/name)==digest,name
validation=dict(status='PASS' if release else 'PARTIAL',engineering_checks='PASS',formation_observed=True,membership_release_observed=bool(release),release_events=release,actual_human_speed_max_mps=max_speed,actual_human_yaw_rate_max_radps=max_yaw_rate,actual_human_path_total_m=human_path,offline_cases=checks,live_pose_samples=len(complete),proxy_comparison_samples=len(errors),proxy_position_error_max_m=max(errors),actual_measured_swept_gap_m=gaps,
    robot_displacement_xy_m=displacement.tolist(),robot_prediction_error_max_m=max(robot_prediction),robot_odom_feedback_error_max_m=max(feedback),membership_events=events,
    lidar_human_beams=len(lidar_errors),lidar_within_15cm_fraction=sum(e<.15 for e in lidar_errors)/len(lidar_errors),lidar_matched_ids=sorted(lidar_ids),pause_resume=result['pause_check'],
    gpu_nvidia=True,shutdown_clean=True,owned_cleanup=True,gui_user_confirmation=load(out/'gui_confirmation.json') if (out/'gui_confirmation.json').exists() else None,
    preserved_files=len(load(cp/'hashes.json')),media_decoded=True,scope='One seed,6 humans and local aisle goal sampling. All42 factory AABBs mapped. Prescribed velocity commands, no Nav2 comparison or general safety claim.')
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
files=[*list(out.glob('*.gz')),*list(out.glob('*.jsonl')),*list(out.glob('*.png')),out/'camera_preview_4x.gif',out/'result.json',out/'protocol.json',out/'archives.json',out/'moving_groups.json',out/'world.sdf',runtime/'warehouse.sdf',runtime/'worker_trajectory.jsonl',runtime/'worker.log',runtime/'ogre2_session.log']
(out/'manifest.json').write_text(json.dumps(dict(analyzer_sha256=sha(Path(__file__)),shapely_version=shapely.__version__,files={str(p.relative_to(REPO)):sha(p) for p in files if p.exists()}),indent=2)+'\n')
print(json.dumps(validation,indent=2))
