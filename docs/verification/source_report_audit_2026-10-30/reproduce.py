from pathlib import Path
import sys,json,os
import numpy as np
snapshot=Path('/tmp/depth-detector-review-1sfeuh4f')
sys.path.insert(0,str(snapshot/'experiments/social_navigation/benchmark/depth_identity_guard'))
from integrated_pipeline import EndToEndPerceptionPipeline
from snapshots.sensor import odom_at
root=Path('/home/cuongbien/DynamuhSim')
normalized=root/'experiments/social_navigation/datasets/handoff_pilot/generated/pilot01/normalized'
reference=root/'experiments/social_navigation/perception/depth_identity_guard/generated/integration_20261030_01/comparison'
output=Path(sys.argv[1]);summary={};all_same=True
for ep in ['pilot_conversation','pilot_walking']:
 o=dict(np.load(normalized/ep/'observations.npz',allow_pickle=False));depth=np.load(normalized/ep/'depth.npy',mmap_mode='r',allow_pickle=False)
 c=json.loads((normalized/ep/'calibration.json').read_text());expected=[json.loads(l) for l in (reference/ep/'generalized_3d_classifier.jsonl').read_text().splitlines()]
 pipeline=EndToEndPerceptionPipeline();mismatches=[];count=0;max_xy=0.;rows=[]
 for i,t in enumerate(o['depth__stamp_ns']):
  od=odom_at(int(t),o['odom__stamp_ns'],o['odom__available_ns'],o['odom__values'])
  tracks=pipeline.process_depth_frame(int(t),depth[i],c['k'],np.array(c['optical_to_base_link']),od['pose'],stride=4) if od else []
  reference_tracks=expected[i]['tracks'];count+=len(tracks)
  same=len(tracks)==len(reference_tracks)
  if same:
   for a,b in zip(tracks,reference_tracks):
    distance=float(np.max(np.abs(np.asarray(a['xy_base'])-b['xy_base'])))
    max_xy=max(max_xy,distance)
    same=same and a['track_id']==b['track_id'] and a['point_count']==b['point_count'] and a['confirmed']==b['confirmed'] and distance<=1e-9
  if not same:mismatches.append(i)
  rows.append(dict(depth_index=i,stamp_ns=int(t),tracks=tracks))
  if i%150==0:print(ep,i,'/',len(depth),flush=True)
 with (output/(ep+'_original_SENSOR_ONLY.jsonl')).open('x') as f:
  for row in rows:f.write(json.dumps(row)+'\n')
 summary[ep]=dict(frames=len(depth),original_detection_rows=count,merged_detection_rows=sum(len(row['tracks']) for row in expected),mismatched_frames=mismatches,max_xy_difference_m=max_xy)
 all_same=all_same and not mismatches
result=dict(status='MATCH' if all_same else 'DIFFERS',original_commit='70bcda0e26ff86b3389a49295feda3ef72958b1d',episodes=summary,
 calibration_policy='Same recorded CameraInfo/TF and causal odom on both pipelines; compares detector API, not original hardcoded ROS wrapper',
 scope='Full native-depth replay, no classifier retraining/test reopening or simulator launch',
 claim='No detection/track difference for these 910 frames under identical inputs' if all_same else 'Differences require investigation')
(output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
