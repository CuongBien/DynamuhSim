#!/usr/bin/env python3
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
os.environ['SDL_VIDEODRIVER']='dummy';os.environ['SDL_AUDIODRIVER']='dummy';os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
import copy,json,math,sys
from pathlib import Path
from fixture import make_kernel
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent/'socnavgym_response_fix'));from audit_geometry import gaps
out=HERE/'reports/offline01';out.mkdir();robot=dict(x=-28.,y=0.,yaw=0.,vx=0.,vy=0.,omega=0.);results=[]
for scenario in ['walking','standing']:
 for seed in [42,43]:
    k=make_kernel(scenario,seed);rows=[dict(time=0.,humans=k.humans(),groups=k.groups())]
    try:
        for i in range(250):
            k.advance(robot);rows.append(dict(time=(i+1)*.1,humans=k.humans(),groups=k.groups()))
        xy=np.array([[[h['x'],h['y']] for h in row['humans']] for row in rows]);r=np.repeat([[-28.,0.]],len(xy)-1,axis=0);layout=json.loads((HERE.parent.parent/'warehouse/layout.json').read_text());measured=gaps(xy[:-1],xy[1:],r,r,layout['obstacles']);assert min(measured.values())>=-.001
        saved=copy.deepcopy(rows[:21]);k.close();k=make_kernel(scenario,seed);repeat=[dict(time=0.,humans=k.humans(),groups=k.groups())]
        for i in range(20):k.advance(robot);repeat.append(dict(time=(i+1)*.1,humans=k.humans(),groups=k.groups()))
        assert repeat==saved
        result=dict(scenario=scenario,seed=seed,steps=250,exact_repeat_prefix=20,swept_gaps=measured,initial_groups=rows[0]['groups'],group_states=sum(len(row['groups']) for row in rows))
        (out/(scenario+str(seed)+'_trajectory.json')).write_text(json.dumps(rows)+'\n');results.append(result);print(json.dumps(result),flush=True)
    finally:k.close()
(out/'validation.json').write_text(json.dumps(dict(status='PASS',cases=results),indent=2)+'\n')
