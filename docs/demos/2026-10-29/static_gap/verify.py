#!/usr/bin/env python3
"""Preserve provenance and validate the exact treatment differences across all recorded runs."""
from pathlib import Path
import hashlib,json,sys
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.capture import config_changes
import yaml
plan=json.loads((HERE/'plan.json').read_text());base=ROOT/plan['runtime'];checks={};details={}
for original,cost,horizon in [('wide80','wide80_cost','wide80_horizon'),('narrow52','narrow52_cost','narrow52_horizon')]:
    runs=[base/n for n in [original,cost,horizon]]
    for name in ['assets/mecanum_camera.sdf','assets/mecanum_aligned.urdf','assets/gap.pgm','assets/gap.yaml','assets/gap_fixture.json','runtime/warehouse.sdf']:
        checks[original+':unchanged:'+name]=len({(r/name).read_bytes() for r in runs})==1
    configs=[yaml.safe_load((r/'runtime/nav2_runtime.yaml').read_text()) for r in runs]
    expected_cost={'/controller_server/ros__parameters/FollowPath/CostCritic/'+k for k in ['cost_weight','near_collision_cost']}
    checks[original+':only_two_cost_changes']={c['path'] for c in config_changes(configs[0],configs[1])}==expected_cost
    checks[original+':only_time_steps_change']={c['path'] for c in config_changes(configs[1],configs[2])}=={'/controller_server/ros__parameters/FollowPath/time_steps'}
    for run in runs:
        result=json.loads((run/'episode/result.json').read_text());audit=json.loads((base/(run.name+'_audit.json')).read_text());checks[run.name+':audit']=audit['status']=='PASS';checks[run.name+':native_gpu']=result['rendering']['hardware_verified'];checks[run.name+':no_scripted_robot_velocity_publisher']=result['no_scripted_robot_velocity_publisher'];checks[run.name+':scene_humans_zero']=result['fixture']['humans']==0
        files=['capture_freeze.json','plan.json','preflight.json','episode/result.json','episode/gazebo_poses.jsonl','episode/native_costmaps.jsonl','episode/plans.jsonl','episode/events.jsonl','episode/raw_bag/metadata.yaml','episode/camera_preview_4x.gif']
        details[run.name]=dict(runtime_commit=result['head'],live_controller=result['live_parameters']['/controller_server'],raw_sha256={n:hashlib.sha256((run/n).read_bytes()).hexdigest() for n in files})
report=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,runs=details,scope='Exact control/config/geometry differences and capture integrity. Physical passage acceptance is separately reported. MPPI RNG not fixed, no single-parameter causal claim vs original joint treatment.')
with (HERE/'verification.json').open('x') as f:f.write(json.dumps(report,indent=2)+'\n')
print(report['status']);print([k for k,v in checks.items() if not v])
