#!/usr/bin/env python3
"""Check that renamed CrowdSimPlus proxies are visible to the real ROS LiDAR."""
import argparse,json,math
from bisect import bisect_right
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args()
trial=json.loads(a.trial.read_text());raw=Path(trial['diagnostics']);gate=float((raw.parent/'start_gate').read_text());gz=[];scans=[];costmaps=[]
for line in raw.open():
    r=json.loads(line);v=r['data']
    if r['kind']=='gazebo':gz.append(v)
    elif r['kind']=='scan':scans.append(v)
    elif r['kind']=='costmap':costmaps.append(v)
times=[x['time'] for x in gz];metrics=[]
for s in scans:
    t=s['header']['stamp']['sec']+s['header']['stamp']['nanosec']/1e9
    if not gate<=t<=gate+trial['sim_seconds']:continue
    poses=gz[max(0,bisect_right(times,t)-1)]['poses']
    if len(poses)!=4:continue
    ro=poses['mecanum_robot'];c,si=math.cos(ro['yaw']),math.sin(ro['yaw'])
    angles=s['angle_min']+np.arange(len(s['ranges']))*s['angle_increment'];rays=np.stack([np.cos(angles),np.sin(angles)],axis=1)
    expected=np.full(len(rays),np.inf)
    for name,h in poses.items():
        if name=='mecanum_robot':continue
        dx,dy=h['x']-ro['x'],h['y']-ro['y'];xy=np.array([c*dx+si*dy,-si*dx+c*dy])
        projection=rays@xy;disc=projection**2-(xy@xy-.36**2)
        dist=projection-np.sqrt(np.maximum(disc,0));valid=(disc>=0)&(dist>=s['range_min'])&(dist<=s['range_max'])&(projection>0)
        expected=np.where(valid,np.minimum(expected,dist),expected)
    actual=np.array(s['ranges']);valid=np.isfinite(expected)&np.isfinite(actual);matches=np.zeros(len(rays),bool)
    matches[valid]=np.abs(actual[valid]-expected[valid])<.05
    metrics.append(dict(time=t,elapsed=t-gate,expected_human_beams=int(np.isfinite(expected).sum()),matched_human_beams=int(matches.sum()),frame=s['header']['frame_id']))
assert metrics and max(x['matched_human_beams'] for x in metrics)>=5
peak=max(metrics,key=lambda x:x['matched_human_beams'])
summary=dict(status='PASS',trial=trial['name'],scans_checked=len(metrics),scan_frames=sorted(set(x['frame'] for x in metrics)),
    peak_surface_match=peak,nonempty_costmaps=sum(any(c>0 for c in m['data']) for m in costmaps),costmap_frames=sorted(set(m['header']['frame_id'] for m in costmaps)),
    tolerance_m=.05,method='Expected circle ray intersection from previous Gazebo sample; observed ROS scan range within 5cm. Occlusion/noise/discrete timing may reduce matches.',
    limitation='Supporting visibility evidence, not a proof of every obstacle layer/controller update or force contact.')
assert summary['nonempty_costmaps']>0
(a.trial.parent/'sensor_evidence.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
