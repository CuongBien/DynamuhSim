"""Offline TF-vs-Gazebo diagnostics, never modifies any recorded pose."""
import bisect,json,math
import numpy as np

def assess(root):
    rows=lambda f:[json.loads(s) for s in (root/f).read_text().splitlines()]
    poses=rows('gazebo_poses.jsonl');times=[x['time'] for x in poses];tf=rows('tf.jsonl');values=[]
    for row in tf:
        i=bisect.bisect_left(times,row['time']);choices=[n for n in [i-1,i] if 0<=n<len(times)];j=min(choices,key=lambda n:abs(times[n]-row['time']))
        actual=poses[j]['poses']['mecanum_robot'];estimate=row['map_base']
        values.append(dict(time=row['time'],pose_alignment_s=abs(times[j]-row['time']),xy_error_m=math.hypot(actual['x']-estimate['x'],actual['y']-estimate['y']),yaw_error_rad=abs(math.atan2(math.sin(actual['yaw']-estimate['yaw']),math.cos(actual['yaw']-estimate['yaw'])))))
    return dict(tf_samples=len(values),xy_error_median_m=float(np.median([v['xy_error_m'] for v in values])),xy_error_max_m=max(v['xy_error_m'] for v in values),yaw_error_median_rad=float(np.median([v['yaw_error_rad'] for v in values])),yaw_error_max_rad=max(v['yaw_error_rad'] for v in values),last=values[-1],samples=values)
