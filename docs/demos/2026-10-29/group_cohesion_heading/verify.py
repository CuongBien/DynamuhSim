"""Only the declared upper group proposal changes; per-person safety sources match."""
import argparse,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];reference=ROOT/plan['reference_root'];checks={};evidence={}
def world(path):
 tree=ET.parse(path).getroot();items=tree.findall("world/plugin[@name='response_experiment::Adapter']/socket")
 if len(items)!=1:raise ValueError('Expected one worker socket')
 items[0].text='<RUN_LOCAL_SOCKET>';return ET.tostring(tree)
for case in plan['cases']:
 name=case['id'];run=raw/name;old=reference/case['reference_id'];before=json.loads((old/'episode/result.json').read_text());after=json.loads((run/'episode/result.json').read_text());audit=json.loads((raw/(name+'_audit.json')).read_text())
 for item in ['assets/mecanum_camera.sdf','assets/mecanum_aligned.urdf','assets/encounter.json','assets/human_goal_clock.json','runtime/nav2_runtime.yaml']:checks[name+':same:'+item]=(run/item).read_bytes()==(old/item).read_bytes()
 checks[name+':same_initial_humans']=before['ready']['humans']==after['ready']['humans'];checks[name+':same_initial_groups']=before['ready']['groups']==after['ready']['groups'];checks[name+':same_world_except_socket']=world(run/'runtime/warehouse.sdf')==world(old/'runtime/warehouse.sdf')
 protected=[k for k in before['source_sha256'] if '/vendor/SocNavGym/' in k or k.endswith('/responsive_motion.py') or k.endswith('/projection.py') or k in ['python/dynamuhsim/simulation/goal_clock.py','python/dynamuhsim/simulation/human_groups.py']];checks[name+':same_person_guard_native_group_and_clock_sources']=bool(protected) and all(before['source_sha256'][k]==after['source_sha256'][k] for k in protected)
 checks[name+':declared_motion_profile']=after['human_motion_profile']=='group_cohesion_v2';checks[name+':clock_profile']=after['human_goal_clock']=='sim_v1';checks[name+':clock_trace']=audit['checks']['goal_clock_trace'];checks[name+':direct_goal_trace']=audit['checks']['shared_group_goal_proposals'];checks[name+':cohesion_heading_trace']=audit['checks']['group_cohesion_heading_proposals'];checks[name+':capture_integrity']=audit['status']=='PASS';checks[name+':native_gpu']=after['rendering']['hardware_verified'] and 'NVIDIA' in after['rendering']['renderer'];checks[name+':no_injected_cmd_vel']=after['no_scripted_robot_velocity_publisher'];checks[name+':clean_exit']=after['gazebo_clean_exit'] and after['cleanup']['status']=='PASS'
 phase1=ROOT/plan['component_reference_root']/case['component_reference_id'];phase1_result=json.loads((phase1/'episode/result.json').read_text());key='python/dynamuhsim/simulation/group_intent.py';checks[name+':same_shared_goal_code_as_phase1']=phase1_result['source_sha256'][key]==after['source_sha256'][key]
 files=['capture_freeze.json','plan.json','assets/human_motion.json','runtime/nav2_runtime.yaml','runtime/warehouse.sdf','runtime/worker_trajectory.jsonl','episode/result.json','episode/gazebo_poses.jsonl','episode/native_costmaps.jsonl','episode/events.jsonl','episode/camera_preview_4x.gif'];evidence[name]=dict(runtime_commit=after['head'],sha256={f:hashlib.sha256((run/f).read_bytes()).hexdigest() for f in files})
receipt=dict(schema='group_cohesion_heading_verification/1',status='PASS' if all(checks.values()) else 'FAIL',checks=checks,runs=evidence)
with a.output.open('x') as f:f.write(json.dumps(receipt,indent=2)+'\n')
print(receipt['status'],[k for k,v in checks.items() if not v])
