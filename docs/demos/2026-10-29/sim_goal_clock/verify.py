"""Read-only equivalence of robot/navigation/human motion, with one declared clock change."""
import argparse,hashlib,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];old=ROOT/plan['reference_root'];checks={};evidence={}
def world(path):
 tree=ET.parse(path).getroot();socket=tree.findall("world/plugin[@name='response_experiment::Adapter']/socket")
 if len(socket)!=1:raise ValueError('Expected one worker socket')
 socket[0].text='<RUN_LOCAL_SOCKET>';return ET.tostring(tree)
for case in plan['cases']:
 name=case['id'];run=raw/name;reference=old/case['reference_id'];before=json.loads((reference/'episode/result.json').read_text());after=json.loads((run/'episode/result.json').read_text());audit=json.loads((raw/(name+'_audit.json')).read_text())
 for item in ['assets/mecanum_camera.sdf','assets/mecanum_aligned.urdf','assets/encounter.json','runtime/nav2_runtime.yaml']:
  checks[name+':same:'+item]=(run/item).read_bytes()==(reference/item).read_bytes()
 if case['human_motion_profile']!='legacy':checks[name+':same_group_orca_asset']=(run/'assets/human_motion.json').read_bytes()==(reference/'assets/human_motion.json').read_bytes()
 checks[name+':same_initial_humans']=before['ready']['humans']==after['ready']['humans'];checks[name+':same_initial_groups']=before['ready']['groups']==after['ready']['groups'];checks[name+':same_world_except_socket']=world(run/'runtime/warehouse.sdf')==world(reference/'runtime/warehouse.sdf');source='python/dynamuhsim/simulation/human_groups.py';checks[name+':same_group_orca_code']=before['source_sha256'][source]==after['source_sha256'][source]
 vendor=[k for k in before['source_sha256'] if '/vendor/SocNavGym/' in k];checks[name+':same_native_vendor']=bool(vendor) and all(before['source_sha256'][k]==after['source_sha256'][k] for k in vendor);checks[name+':clock_profile']=after.get('human_goal_clock')=='sim_v1';checks[name+':clock_audit']=audit['checks']['goal_clock_trace'];checks[name+':capture_integrity']=audit['status']=='PASS';checks[name+':native_gpu']=after['rendering']['hardware_verified'] and 'NVIDIA' in after['rendering']['renderer'];checks[name+':no_injected_cmd_vel']=after['no_scripted_robot_velocity_publisher'];checks[name+':clean_exit']=after['gazebo_clean_exit'] and after['cleanup']['status']=='PASS'
 files=['capture_freeze.json','plan.json','assets/human_goal_clock.json','runtime/nav2_runtime.yaml','runtime/warehouse.sdf','runtime/worker_trajectory.jsonl','episode/result.json','episode/gazebo_poses.jsonl','episode/native_costmaps.jsonl','episode/events.jsonl','episode/camera_preview_4x.gif'];evidence[name]=dict(runtime_commit=after['head'],file_sha256={f:hashlib.sha256((run/f).read_bytes()).hexdigest() for f in files})
receipt=dict(schema='sim_goal_clock_verification/1',status='PASS' if all(checks.values()) else 'FAIL',checks=checks,runs=evidence,scope='Historical references retain their original source hashes. Only run-local worker socket is normalized for world comparison; raw files stay untouched.')
with a.output.open('x') as f:f.write(json.dumps(receipt,indent=2)+'\n')
print(receipt['status'],[k for k,v in checks.items() if not v])
