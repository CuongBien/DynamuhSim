#!/usr/bin/env python3
"""Diagnose upstream observation schema and summarize recorded group motion."""
import json
import math
from pathlib import Path
import numpy as np
import gymnasium as gym
import socnavgym

HERE=Path(__file__).resolve().parent
REPORT=HERE/'reports'
env=gym.make('SocNavGym-v2',config=str(HERE/'vendor/SocNavGym/tests/test_env.yaml'))
obs,_=env.reset(seed=42)
def describe(value):
    if isinstance(value,dict):
        return {str(k):describe(v) for k,v in value.items()}
    v=np.asarray(value)
    return dict(shape=list(v.shape),dtype=str(v.dtype))
schema=dict(observation=describe(obs),declared_space=str(env.observation_space),
            contains=bool(env.observation_space.contains(obs)),
            per_key={k:dict(actual_shape=list(v.shape),declared_shape=list(env.observation_space[k].shape),contains=bool(env.observation_space[k].contains(v))) for k,v in obs.items()})
(REPORT/'observation_schema.json').write_text(json.dumps(schema,indent=2))
env.close()
all_stats={}
for path in sorted(REPORT.glob('socnavgym_*_trajectory.json')):
    rows=json.loads(path.read_text())
    scenario=path.stem.removeprefix('socnavgym_').removesuffix('_trajectory')
    group_stats=[]
    if scenario.startswith('groups') and rows:
        for group in rows[0]['groups']:
            ids=set(group['members'])
            centers=[]
            radii=[]
            for row in rows:
                matching=next((g for g in row['groups'] if set(g['members'])==ids and g['type']==group['type']),None)
                if matching is None or row['episode']!=rows[0]['episode']:
                    break
                people=[h for h in row['humans'] if h['id'] in ids]
                center=[sum(h[k] for h in people)/len(people) for k in ['x','y']]
                centers.append(center)
                radii.extend(math.dist([h['x'],h['y']],center) for h in people)
            if centers:
                group_stats.append(dict(type=group['type'],members=sorted(ids),
                    center_displacement_m=math.dist(centers[0],centers[-1]),
                    center_path_length_m=sum(math.dist(a,b) for a,b in zip(centers,centers[1:])),
                    mean_member_radius_m=float(np.mean(radii)),max_member_radius_m=max(radii)))
    all_stats[scenario]=dict(recorded_steps=len(rows),group_motion=group_stats)
(REPORT/'group_motion_summary.json').write_text(json.dumps(all_stats,indent=2))
print(json.dumps(all_stats,indent=2))

