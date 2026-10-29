#!/usr/bin/env python3
"""Exact frozen-input, matched initialization and treatment-difference receipts."""
from pathlib import Path
import argparse,hashlib,json,sys,xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.capture import config_changes
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=HERE/'verification.json');args=parser.parse_args();args.output.parent.mkdir(parents=True,exist_ok=True)
plan=json.loads((HERE/'plan.json').read_text());base=ROOT/plan['runtime'];checks={};evidence={}

def normalized_world(path):
    root=ET.parse(path).getroot();sockets=root.findall("world/plugin[@name='response_experiment::Adapter']/socket")
    if len(sockets)!=1:raise ValueError('Expected one run-local human-worker socket')
    sockets[0].text='<RUN_LOCAL_SOCKET>'
    return ET.tostring(root,encoding='utf-8')

for scenario in ['standing','walking']:
    runs=[base/(scenario+suffix) for suffix in ['_base','_h5']];results=[json.loads((run/'episode/result.json').read_text()) for run in runs]
    for name in ['assets/mecanum_camera.sdf','assets/mecanum_aligned.urdf']:
        checks[scenario+':identical:'+name]=(runs[0]/name).read_bytes()==(runs[1]/name).read_bytes()
    checks[scenario+':world_same_except_socket']=normalized_world(runs[0]/'runtime/warehouse.sdf')==normalized_world(runs[1]/'runtime/warehouse.sdf')
    checks[scenario+':initial_native_humans_same']=results[0]['ready']['humans']==results[1]['ready']['humans']
    configs=[yaml.safe_load((run/'runtime/nav2_runtime.yaml').read_text()) for run in runs];changes=config_changes(*configs);expected={'/controller_server/ros__parameters/FollowPath/CostCritic/'+k for k in ['cost_weight','near_collision_cost']}|{'/controller_server/ros__parameters/FollowPath/time_steps'}
    checks[scenario+':only_three_declared_control_changes']={c['path'] for c in changes}==expected
    keys=[k for k in results[0]['source_sha256'] if k.startswith('experiments/social_navigation/navigation/') or k.startswith('experiments/social_navigation/vendor/') or k.startswith('experiments/social_navigation/warehouse/')]
    checks[scenario+':same_backend_map_sources']=bool(keys) and all(results[0]['source_sha256'][k]==results[1]['source_sha256'].get(k) for k in keys)
    for run,result in zip(runs,results):
        audit=json.loads((base/(run.name+'_audit.json')).read_text());checks[run.name+':capture_integrity']=audit['status']=='PASS';checks[run.name+':native_nvidia']=result['rendering']['hardware_verified'] and 'NVIDIA' in result['rendering']['renderer'];checks[run.name+':no_injected_velocity']=result['no_scripted_robot_velocity_publisher'];checks[run.name+':cleanup']=result['cleanup']['status']=='PASS' and not result['cleanup']['remaining']
        files=['capture_freeze.json','plan.json','preflight.json','runtime/nav2_runtime.yaml','runtime/warehouse.sdf','episode/result.json','episode/gazebo_poses.jsonl','episode/native_costmaps.jsonl','episode/ground_truth.jsonl','episode/commands.jsonl','episode/plans.jsonl','episode/events.jsonl','episode/raw_bag/metadata.yaml','episode/camera_preview_4x.gif']
        text=(run/'episode/launch.log').read_text();warnings=[line for line in text.splitlines() if 'missed its desired rate' in line or 'missed the desired rate' in line]
        evidence[run.name]=dict(runtime_commit=result['head'],initial_native_humans=result['ready']['humans'],live_parameters=result['live_parameters'],source_sha256={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in files},controller_loop_rate_warning_count=len(warnings),controller_loop_rate_warning_examples=warnings[:3])
receipt=dict(schema='factory_regression_verification/1',status='PASS' if all(checks.values()) else 'FAIL',checks=checks,runs=evidence,world_comparison='Read-only XML comparison normalizes only the expected run-local worker socket; raw worlds stay untouched.',scope='Integrity and exact input/control differences, independent from goal/geometry/social outcomes. Backend seed45; MPPI RNG not fixed.')
with args.output.open('x') as f:f.write(json.dumps(receipt,indent=2)+'\n')
print(receipt['status']);print([k for k,v in checks.items() if not v])
