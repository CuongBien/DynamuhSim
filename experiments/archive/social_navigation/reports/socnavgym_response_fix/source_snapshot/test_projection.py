#!/usr/bin/env python3
"""Independent swept-disc/GEOS checks for approach, walls and infeasible states."""
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
import json,math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent/'socnavgym_warehouse_lifecycle'))
import numpy as np,shapely
from projection import project
out=HERE/'reports/projection_tests01';out.mkdir(exist_ok=False)
boxes=[dict(x=20.,y=20.,sx=1.,sy=1.)]
def gap(p,d,r):
    p=np.array(p);d=np.array(d);t=np.clip(-p@d/(d@d),0,1) if d@d>1e-25 else 0.;return float(np.linalg.norm(p+t*d)-r)
def check(starts,proposal,robot,robot_delta,boxes,fixed=()):
    result,audit=project(starts,proposal,[.36]*len(starts),robot,robot_delta,.25,boxes,.1,.8,set(fixed))
    for i,(p,d) in enumerate(zip(starts,result)):
        assert math.hypot(*d)<=.08000001
        assert gap(np.array(p)-robot,np.array(d)-robot_delta,.61)>=.002-1e-8
        segment=shapely.LineString([p,np.array(p)+d])
        for box in boxes:assert segment.distance(shapely.box(box['x']-box['sx']/2,box['y']-box['sy']/2,box['x']+box['sx']/2,box['y']+box['sy']/2))-.36>=.002-1e-8
        for j in range(i):assert gap(np.array(p)-starts[j],np.array(d)-result[j],.72)>=.002-1e-8
    return result,audit
results=[]
x,a=check([[0.,0.]],[[0.,0.]],[-.65,0.],[.075,0.],boxes);assert x[0][0]>.07;results.append(dict(name='approaching_robot_requires_new_direction',status='PASS',displacements=x,audit=a))
x,a=check([[-.38,0.],[.38,0.]],[[.08,0.],[-.08,0.]],[10.,10.],[0.,0.],boxes);results.append(dict(name='opposed_humans',status='PASS',displacements=x,audit=a))
x,a=check([[-.9,0.]],[[.08,0.]],[10.,10.],[0.,0.],[dict(x=0.,y=0.,sx=1.,sy=2.)]);results.append(dict(name='solid_wall',status='PASS',displacements=x,audit=a))
for name,starts,fixed in [('stationary_human_cannot_escape',[[0.,0.]],{0}),('initial_overlap',[[0.,0.]],set())]:
    try:project(starts,[[0.,0.]],[.36],[-.65,0.] if fixed else [-.5,0.],[.075,0.],.25,boxes,.1,.8,fixed);raise AssertionError('Expected terminal rejection')
    except (RuntimeError,ValueError) as error:results.append(dict(name=name,status='PASS_TERMINAL_REJECTION',error=str(error)))
(out/'validation.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2))
