#!/usr/bin/env python3
"""Read-only provenance and snapshot/live-path comparison; descriptive distances are not a new gate."""
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
plan=json.loads((HERE/'plan.json').read_text());run=ROOT/plan['run'];previous=ROOT/'experiments/social_navigation/benchmark/generated/geometry_ab_20261029/C_aligned_045'
result=json.loads((run/'episode/result.json').read_text());audit=json.loads((run.parent/'D_audit.json').read_text());analysis=json.loads((HERE/'results.json').read_text())
checks={name:(run/'runtime'/name).read_bytes()==(previous/'runtime'/name).read_bytes() for name in ['nav2_runtime.yaml','mecanum_camera.sdf']}
checks['capture_integrity']=audit['status']=='PASS';checks['native_bytes_equal_mcap']=audit['checks']['native_costmaps_match_bag'];checks['no_scripted_robot_velocity_publisher']=result['no_scripted_robot_velocity_publisher']
comparisons=[]
for probe in analysis['probes']:
    free=np.array(probe['paths']['native_free']);live=np.array(probe['nearest_recorded_plan_map']);distance=cKDTree(free).query(live)[0]
    comparisons.append(dict(snapshot_sim_s=probe['time_sim_s'],live_plan_stamp_s=probe['nearest_recorded_plan_stamp_s'],median_nearest_path_point_distance_m=float(np.median(distance)),max_nearest_path_point_distance_m=float(distance.max()),scope='Descriptive point-to-point distances, async clocks; no exact replay claim.'))
files=['capture_freeze.json','plan.json','preflight.json','runtime/warehouse.sdf','runtime/nav2_runtime.yaml','episode/result.json','episode/native_costmaps.jsonl','episode/plans.jsonl','episode/tf.jsonl','episode/gazebo_poses.jsonl','episode/events.jsonl','episode/raw_bag/metadata.yaml']
receipt=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,snapshot_vs_live_path=comparisons,source_hashes={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in files},runtime_commit=result['head'])
with (HERE/'verification.json').open('x') as f:f.write(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt,indent=2))
