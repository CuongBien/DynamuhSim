#!/usr/bin/env python3
"""Audit actual SceneBroadcaster geometry, worker clock, RGB and LiDAR."""
import argparse,bisect,hashlib,json,math,os,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
import numpy as np,shapely
from PIL import Image
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent;REPO=SOCIAL.parent.parent
import sys
from functools import partial
sys.path.insert(0,str(REPO/'python'))
from dynamuhsim.analysis.warehouse import (
    measured_gaps as swept_gaps, trajectory_errors, pose_intervals, human_motion, lidar_measurements,
)
PROXIES={i:f'common_{i}_proxy' for i in range(1,7)}
p=argparse.ArgumentParser();p.add_argument('--live',type=Path,required=True);a=p.parse_args();out=a.live.resolve()
load=lambda p:json.loads(p.read_text());lines=lambda p:[json.loads(l) for l in p.open()];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
analysis_sources={str(p.relative_to(REPO)):sha(p) for p in [Path(__file__),REPO/'python/dynamuhsim/analysis/warehouse.py']}
boxes=load(SOCIAL/'warehouse/layout.json')['obstacles'];assert len(boxes)==42
polygons=np.array([shapely.box(b['x']-b['sx']/2,b['y']-b['sy']/2,b['x']+b['sx']/2,b['y']+b['sy']/2) for b in boxes],dtype=object)
measured_gaps=partial(swept_gaps,polygons=polygons)
result=load(out/'result.json');assert result['status'] in ['RECORDED','ERROR'],result.get('error')
error_prefix=result['status']=='ERROR';assert not error_prefix or result['faults'],'Infrastructure errors are not behavior prefixes'
fault_mode=False;dt=result['native_dt']
for name,digest in result['source_sha256'].items():assert sha(REPO/name)==digest,name
runtime=REPO/result['runtime'];worker_raw=lines(runtime/'worker_trajectory.jsonl');worker=[s for s in worker_raw if 'error' not in s];gt=lines(out/'ground_truth.jsonl');poses=lines(out/'gazebo_poses.jsonl');scans=lines(out/'scans.jsonl');odom=lines(out/'odom.jsonl')
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
pose_times=[s['time'] for s in complete]
errors,robot_prediction=trajectory_errors(complete,worker,dt,PROXIES,pose_times)
feedback=[]
assert len(errors)>100 and max(errors)<.025
before,after,rb,re=pose_intervals(complete,PROXIES)
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
max_speed,max_yaw_rate,human_path=human_motion(complete,PROXIES)
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
lidar_errors,lidar_ids=lidar_measurements(scans,complete,pose_times,boxes,PROXIES)
assert len(lidar_errors)>50 and sum(e<.15 for e in lidar_errors)/len(lidar_errors)>.7 and len(lidar_ids)>=2
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
assert not subprocess.check_output(['docker','ps','-aq','--filter','name=dynamuhsim-common-gz-'],text=True).strip()
validation=dict(status='AUDITED_ERROR_PREFIX' if error_prefix else 'PASS_LIVE_PILOT',error=result.get('error'),observed_seconds=gt[-1]['time']-gt[0]['time'],worker_updates=len(worker),population=6,native_dt=dt,physics_step_s=.0025,
    actual_pose_samples=len(complete),proxy_comparison_samples=len(errors),proxy_position_error_max_m=max(errors),actual_swept_gap_m=gaps,full_recording_including_teardown_gap_m=full_recording_gaps,assessment_end_sim=assessment_end,
    robot_displacement_xy_m=displacement.tolist(),robot_prediction_error_max_m=max(robot_prediction),robot_odom_feedback_error_max_m=max(feedback),
    human_speed_max_mps=max_speed,human_yaw_rate_max_radps=max_yaw_rate,human_path_total_m=human_path,
    actual_goal_arrivals=arrivals,freeze_check=freeze,lidar_human_beams=len(lidar_errors),lidar_within_15cm_fraction=sum(e<.15 for e in lidar_errors)/len(lidar_errors),lidar_matched_ids=sorted(lidar_ids),
    pause_check=result.get('pause_check'),nvidia=True,fresh_ogre=True,shutdown_clean=True,owned_process_container_cleanup=True,
    gui_user_confirmation=load(out/'gui_confirmation.json') if (out/'gui_confirmation.json').exists() else None,
    preserved_old_source_evidence_files=len(preserved)-len(allowed),intentional_status_documents=sorted(allowed),original_user_index_outside_preserved=True,
    model=result['model'],case=result['case'],robot_script_position_error_max_m=max(trace_error),geometry_outcome='OVERLAP_GT_5MM' if min(gaps.values())<-.005 else 'NO_OVERLAP_GT_5MM',scope='Common6-human Gazebo near-condition pilot; same initial fixtures and commanded trace, measured physical robot feedback; no Nav2/group/dense/general-safety claim')
validation['analysis_source_sha256']=analysis_sources
import gzip
for path in [runtime/'worker_trajectory.jsonl',runtime/'warehouse.sdf',runtime/'worker.log',runtime/'ogre2_session.log',out/'launch.log',*out.glob('*.jsonl')]:
    if path.exists():
        target=out/(path.name+'.gz');target.write_bytes(gzip.compress(path.read_bytes(),mtime=0));assert gzip.decompress(target.read_bytes())==path.read_bytes()
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
print(json.dumps(validation,indent=2))
