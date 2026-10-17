#!/usr/bin/env python3
"""Summarize the preserved longer patrol from the worker's complete trajectory."""
import json
import math
from pathlib import Path

HERE=Path(__file__).resolve().parent
report=HERE/'reports'
first=(report/'walking_long_headless.log').read_text().splitlines()[0]
runtime=Path(first.removeprefix('Runtime artifacts: '))
rows=[json.loads(line) for line in (runtime/'trajectory.jsonl').read_text().splitlines()]
layout=json.loads((HERE.parent/'warehouse/layout.json').read_text())
centers=[(sum(h['x'] for h in row['humans'])/3,sum(h['y'] for h in row['humans'])/3) for row in rows]
turns=[]
previous=None
for i in range(10,len(rows)):
    dx=centers[i][0]-centers[i-10][0]
    direction=1 if dx>.1 else -1 if dx<-.1 else None
    if direction is not None:
        if previous is not None and direction!=previous:
            turns.append(rows[i]['time'])
        previous=direction

def clearance(h,box):
    qx=abs(h['x']-box['x'])-box['sx']/2
    qy=abs(h['y']-box['y'])-box['sy']/2
    signed=math.hypot(max(0,qx),max(0,qy))+min(max(qx,qy),0)
    return signed-h['radius']

result=dict(status='PASS',runtime=str(runtime.relative_to(HERE)),steps=len(rows)-1,
    time_range_s=[rows[0]['time'],rows[-1]['time']],
    max_step_clock_error_s=max(abs(b['time']-a['time']-.1) for a,b in zip(rows,rows[1:])),
    ids_stable=all([h['id'] for h in row['humans']]==[1,2,3] for row in rows),
    center_path_m=sum(math.dist(a,b) for a,b in zip(centers,centers[1:])),
    x_direction_reversal_times_s=turns,
    center_x_range_m=[min(c[0] for c in centers),max(c[0] for c in centers)],
    min_human_factory_clearance_m=min(clearance(h,box) for row in rows for h in row['humans'] for box in layout['obstacles']),
    min_human_pair_clearance_m=min(math.hypot(a['x']-b['x'],a['y']-b['y'])-a['radius']-b['radius']
        for row in rows for i,a in enumerate(row['humans']) for b in row['humans'][i+1:]))
overlap_times=[row['time'] for row in rows if any(math.hypot(a['x']-b['x'],a['y']-b['y'])<a['radius']+b['radius'] for i,a in enumerate(row['humans']) for b in row['humans'][i+1:])]
result['steps_with_human_pair_overlap']=len(overlap_times)
result['first_human_pair_overlap_time_s']=overlap_times[0] if overlap_times else None
result['behavior_quality_pass']=not bool(overlap_times)
result['status']='TRANSPORT_PASS_BEHAVIOR_LIMITATION' if overlap_times else 'PASS'
assert result['ids_stable'] and result['max_step_clock_error_s']<1e-7 and len(turns)>=1
(report/'patrol_analysis.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
