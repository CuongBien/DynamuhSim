#!/usr/bin/env python3
import gzip,json,math,sys
from pathlib import Path
from fixture import make_kernel
from projection import project as fixed_project
from standing_projection import project_standing
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'socnavgym_response_fix'));from audit_geometry import gaps
HERE=Path(__file__).resolve().parent;rows=[json.loads(s) for s in gzip.decompress((HERE/'reports/standing01_v2/worker_trajectory.jsonl.gz').read_bytes()).decode().splitlines()];last=rows[-2];fault=rows[-1];robot=fault['request']['robot'];hs=fault['last_safe_humans'];starts=[[h['x'],h['y']] for h in hs];zero=[[0.,0.]]*6;radii=[.36]*6;fixed={i for i,h in enumerate(hs) if h['id'] in last['motion_guard']['stationary_ids']};standing={hid for g in last['groups'] if g['type']=='standing' for hid in g['members']};yielding={i for i,h in enumerate(hs) if h['id'] in standing and math.hypot(h['x']-robot['x'],h['y']-robot['y'])<1.4};boxes=json.loads((HERE.parent.parent/'warehouse/layout.json').read_text())['obstacles'];rp=[robot['x'],robot['y']];rd=[robot['vx']*.1,robot['vy']*.1]
try:fixed_project(starts,zero,radii,rp,rd,.25,boxes,.1,.8,fixed);raise AssertionError('Expected fixed standing infeasibility')
except RuntimeError:pass
delta,audit=project_standing(starts,zero,radii,rp,rd,.25,boxes,.1,.8,fixed,yielding);assert max(math.hypot(*delta[i])/.1 for i in yielding)<=.2000001
for i in fixed-yielding:assert math.hypot(*delta[i])<1e-8
actual=gaps(np.array([starts]),np.array([np.array(starts)+np.array(delta)]),np.array([rp]),np.array([np.array(rp)+np.array(rd)]),boxes);assert min(actual.values())>=.002-1e-8
far=[-28.,0.];old,_=fixed_project(starts,zero,radii,far,[0.,0.],.25,boxes,.1,.8,fixed);new,_=project_standing(starts,zero,radii,far,[0.,0.],.25,boxes,.1,.8,fixed,set());assert old==new
result=dict(status='PASS',replayed_captured_robot_interval=True,fixed_reference='INFEASIBLE',yielding_human_ids=[hs[i]['id'] for i in sorted(yielding)],speed_cap=.2,independent_swept_gaps=actual,far_control_exact=True,projection=audit)
(HERE/'reports/reaction_gate.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
