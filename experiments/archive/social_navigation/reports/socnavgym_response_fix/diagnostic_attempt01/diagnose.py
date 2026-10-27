#!/usr/bin/env python3
"""Exact recorded successful-prefix replay, then reconstruct last unlogged request."""
import copy,gzip,json,sys,time
from pathlib import Path
from backend import create,COMMON,HERE
out=HERE/'reports/diagnostic01';out.mkdir(exist_ok=False)
old=HERE.parent/'common_gazebo/reports/soc_crossing01';case=json.loads((COMMON/'inputs/crossing_near.json').read_text())
with gzip.open(old/'worker_trajectory.jsonl.gz','rt') as f:recorded=[json.loads(line) for line in f]
legacy=create(case,legacy=True);successful=[r for r in recorded if 'error' not in r]
for row in successful:
    actual=legacy.advance(row['robot_input'])
    for human,expected in zip(actual['humans'],row['humans']):
        for key in ['id','x','y','radius','vx','vy','native_goal']:assert human[key]==expected[key],(row['step'],key)
# The baseline did not log the failing request. Reconstruct its pose from the
# nearest SceneBroadcaster sample, velocity from the last two planning poses.
fault_time=json.loads((old/'result.json').read_text())['faults'][0]['time']
with gzip.open(old/'gazebo_poses.jsonl.gz','rt') as f:poses=[json.loads(line) for line in f]
pose=min(poses,key=lambda p:abs(p['time']-fault_time));assert abs(pose['time']-fault_time)<.005
robot=dict(successful[-1]['robot_input']);robot.update(pose['poses']['mecanum_robot'])
for key,velocity in [('x','vx'),('y','vy')]:robot[velocity]=(robot[key]-successful[-1]['robot_input'][key])/.1
import motion
original=motion.admissible_fraction;captured={}
def inspect(*args):
    captured['args']=copy.deepcopy(args)
    return original(*args)
motion.admissible_fraction=inspect
try:legacy.advance(robot);raise AssertionError('Reconstructed request did not reproduce baseline failure')
except RuntimeError as error:
    assert 'No admissible bounded' in str(error);baseline_error=str(error)
finally:motion.admissible_fraction=original;legacy.close()
from projection import project
starts,deltas,radii,rpos,rdelta,rradius,boxes=captured['args'];fixed=[]
projected,audit=project(starts,deltas,radii,rpos,rdelta,rradius,boxes,.1,.8,fixed)
result=dict(status='REPRODUCED_AND_ALTERNATIVE_FOUND',exact_recorded_prefix_steps=len(successful),failed_robot_request_reconstructed=True,nearest_pose_time=pose['time'],fault_time=fault_time,baseline_error=baseline_error,robot=robot,starts=starts,original_orca_deltas=deltas,projected_deltas=projected,audit=audit)
(out/'diagnostic.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
