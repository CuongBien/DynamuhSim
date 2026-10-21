#!/usr/bin/env python3
"""Small independent-runtime preflight, preserving native actions and failures."""
import argparse,copy,hashlib,inspect,json,math,platform,random,subprocess,time
from pathlib import Path
import numpy as np
from shapely.geometry import LineString, Point, box
import torch,gym,shapely
from kernel import ContractKernel,make_native,fixture,DT,ORIGIN
from crowd_sim.envs.utils.action import ActionXY
from crowd_sim.envs.policy.multi_policy import CentralizedMultiPolicy

HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args()
out=a.report_dir;out.mkdir(parents=True,exist_ok=False)
layout=json.loads((HERE/'generated/layout.json').read_text())
boxes=[(b,box(b['x']-b['sx']/2,b['y']-b['sy']/2,b['x']+b['sx']/2,b['y']+b['sy']/2)) for b in layout['obstacles']]
assert len(boxes)==42
polygons=[dict(name=b['name'],vertices=list(poly.exterior.coords)[:-1],area=poly.area) for b,poly in boxes]
(out/'geometry_ledger.json').write_text(json.dumps(dict(frame='world',origin=ORIGIN,boxes=polygons,native_internal_obstacles_consumed=0,diagnostic_only=True),indent=2)+'\n')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
sources={str(p):sha(p) for p in [HERE/'kernel.py',Path(__file__),HERE/'generated/layout.json']}
for mod in ['crowd_sim.envs.crowd_sim','crowd_sim.envs.policy.multi_policy','crowd_sim.envs.policy.orca','crowd_sim.envs.policy.socialforce']:
 m=__import__(mod,fromlist=['_']);sources[inspect.getfile(m)]=sha(inspect.getfile(m))
meta=dict(source_commit=subprocess.check_output(['git','-C','/opt/ComplexityNav','rev-parse','HEAD']).decode().strip(),
 native_dirty=subprocess.check_output(['git','-C','/opt/ComplexityNav','status','--porcelain']).decode(),
 python=platform.python_version(),torch=torch.__version__,gym=gym.__version__,shapely=shapely.__version__,
 native_timestep_s=DT,robot_radius_m=.25,human_radius_m=.3,source_sha256=sources,
 scope='Offline contract only; native centralized multipolicy ORCA/SFM, not RGL/CV-MPC robot and not Gazebo')
(out/'protocol.json').write_text(json.dumps(meta,indent=2)+'\n')

def trace(name,t):
    x,y=-27.,0.;vx=vy=0.
    if name=='forward':x+=.1*min(t,6.);vx=.1 if t<6. else 0.
    elif name=='lateral_then_forward':
        y=.1*min(t,3.);x+=.1*max(0.,min(t-3.,3.))
        vy=.1 if t<3. else 0.;vx=.1 if 3.<=t<6. else 0.
    return dict(x=x,y=y,vx=vx,vy=vy,yaw=0.)

def req(k,robot,advance=True):return dict(epoch=k.epoch,time=k.time,frame='world',advance=advance,robot=robot)

def native_state(env,ordered):
    return [dict(id=f['id'],x=float(h.px),y=float(h.py),vx=float(h.vx),vy=float(h.vy),gx=float(h.gx),gy=float(h.gy)) for f,h in zip(ordered,env.humans)]

def swept_gaps(rows):
    minima=dict(static=float('inf'),pair=float('inf'),robot=float('inf'));hits=[]
    for prev,row in zip(rows,rows[1:]):
        robot0=row['robot_input'];robot1=dict(x=robot0['x']+DT*robot0['vx'],y=robot0['y']+DT*robot0['vy'])
        for h0,h1 in zip(prev['humans'],row['humans']):
            seg=LineString([(h0['x'],h0['y']),(h1['x'],h1['y'])])
            for b,poly in boxes:
                gap=seg.distance(poly)-h1['radius'];minima['static']=min(minima['static'],gap)
                if gap< -1e-8:hits.append(dict(time=row['time'],human=h1['id'],box=b['name'],gap_m=gap))
            def relative(a0,a1,b0,b1,radius):
                u=np.array([a0['x']-b0['x'],a0['y']-b0['y']]);v=np.array([a1['x']-b1['x'],a1['y']-b1['y']])-u
                lam=float(np.clip(-u.dot(v)/v.dot(v),0,1)) if v.dot(v)>1e-24 else 0.
                return float(np.linalg.norm(u+lam*v)-radius)
            minima['robot']=min(minima['robot'],relative(h0,h1,robot0,robot1,h1['radius']+.25))
        for i,(h0,h1) in enumerate(zip(prev['humans'],row['humans'])):
            for j in range(i+1,len(row['humans'])):
                minima['pair']=min(minima['pair'],relative(h0,h1,prev['humans'][j],row['humans'][j],h1['radius']+row['humans'][j]['radius']))
    return {k:None if not math.isfinite(v) else v for k,v in minima.items()},hits

summaries=[];traces={}
for name in ['stationary','forward','lateral_then_forward']:
    k=ContractKernel();ref,ordered=make_native(fixture(),origin=(0.,0.))
    rows=[k.state()];maxerr=0.;initial_objects=k.identity
    for n in range(80):
        robot=trace(name,n*DT)
        old=k.state();paused=k.request(req(k,robot,False));assert paused==old
        row=k.request(req(k,robot));rows.append(row)
        # Independent native world-coordinate reference; never uses the adapter frame transform.
        r=ref.robot;r.px=robot['x'];r.py=robot['y'];r.vx=robot['vx'];r.vy=robot['vy'];r.theta=robot['yaw'];r.gx=26.;r.gy=0.
        ref.step(ActionXY(robot['vx'],robot['vy']))
        for h,truth in zip(row['humans'],native_state(ref,ordered)):
            assert h['id']==truth['id']
            for key in ['x','y','vx','vy','gx','gy']:maxerr=max(maxerr,abs(h[key]-truth[key]))
        assert row['time']==(n+1)*DT and k.identity==initial_objects
    assert maxerr<1e-5,maxerr
    gaps,hits=swept_gaps(rows)
    path=sum(math.hypot(h1['x']-h0['x'],h1['y']-h0['y']) for a,b in zip(rows,rows[1:]) for h0,h1 in zip(a['humans'],b['humans']))
    assert path>1.
    summary=dict(case=name,steps=80,sim_seconds=20.,native_world_reference_max_error=maxerr,stable_ids=True,pause_identity=True,human_path_total_m=path,swept_gap_m=gaps,static_overlap_segments=len(hits))
    summaries.append(summary);traces[name]=rows
    (out/(name+'_trajectory.json')).write_text(json.dumps(rows,allow_nan=False)+'\n')
    (out/(name+'_hits.json')).write_text(json.dumps(hits,indent=2)+'\n')
# Reject malformed requests without changing state, then reset epoch and exact20s repeat.
k=ContractKernel();good=req(k,trace('stationary',0));rejected=[]
for key,value in [('time',.1),('time',float('nan')),('frame','odom'),('epoch',999),('advance',1)]:
 bad=copy.deepcopy(good);bad[key]=value;before=k.state()
 try:k.request(bad)
 except ValueError:rejected.append(dict(field=key,value=str(value)))
 else:raise AssertionError('Malformed input accepted')
 assert k.state()==before
bad=copy.deepcopy(good);bad['robot']['vx']=float('nan');before=k.state()
try:k.request(bad)
except ValueError:rejected.append(dict(field='robot.vx',value='nan'))
else:raise AssertionError('Nonfinite robot accepted')
assert k.state()==before
k.request(good);before=k.state()
try:k.request(good)
except ValueError:rejected.append(dict(field='time',value='duplicate interval'))
else:raise AssertionError('Duplicate update accepted')
assert k.state()==before
old_epoch=k.epoch;k.reset();assert k.epoch==old_epoch+1
try:k.request(good)
except ValueError:rejected.append(dict(field='epoch',value='previous epoch'))
else:raise AssertionError('Stale epoch accepted')
repeat=[k.state()]
for n in range(80):repeat.append(k.request(req(k,trace('stationary',n*DT))))
for a,b in zip(repeat,traces['stationary']):
 for field in ['step','time','humans']:assert a[field]==b[field],field
# Native geometry acceptance is intentionally measured, not repaired.
obstacle=[]
for policy in ['orca','socialforce']:
    person=[dict(id=1,policy=policy,x=-23.,y=8.5,gx=-17.,gy=8.5)]
    k=ContractKernel(person);rows=[k.state()]
    for n in range(40):rows.append(k.request(req(k,trace('stationary',n*DT))))
    gaps,hits=swept_gaps(rows)
    target=[h for h in hits if h['box']=='rack_north_0']
    assert target,'Expected native geometry limitation was not reproduced; review source claim'
    obstacle.append(dict(policy=policy,steps=40,initial_circle_box_clear=True if all(Point(person[0]['x'],person[0]['y']).distance(poly)>=.3 for b,poly in boxes) else False,
        target='rack_north_0',first_overlap=target[0],overlap_segments=len(target),swept_gap_m=gaps))
    (out/('obstacle_'+policy+'_trajectory.json')).write_text(json.dumps(rows,allow_nan=False)+'\n')
# Native centralized planners receive a border parameter but their stock implementations ignore it.
k=ContractKernel();states=[h.get_full_state() for h in k.env.humans]+[k.env.robot.get_full_state()];policies=[h.policy for h in k.env.humans]
# Put valid robot feedback in before querying planner (reset's native default is unrelated).
r=k.env.robot;r.px=50.;r.py=15.;r.gx=49.;r.gy=15.
states=[h.get_full_state() for h in k.env.humans]+[r.get_full_state()]
planner=CentralizedMultiPolicy()
action_a=planner.predict(states,policies,[(-6,6),(6,6),(6,-6),(-6,-6)],[(-6,6),(6,6),(6,-6),(-6,-6)])
action_b=planner.predict(states,policies,[(-60,60),(60,60),(60,-60),(-60,-60)],[(-60,60),(60,60),(60,-60),(-60,-60)])
flat=lambda value:[tuple(x) for xs in value for x in xs]
border_delta=max(abs(a-b) for x,y in zip(flat(action_a),flat(action_b)) for a,b in zip(x,y));assert border_delta==0.
result=dict(status='NOT_READY_FOR_GAZEBO',contract_checks='PASS',warehouse_geometry_acceptance='FAIL_NATIVE_INTERNAL_OBSTACLES',cases=summaries,
    valid_human_updates=400,native_reference_updates=240,negative_requests_rejected=rejected,reset_repeat_steps=80,epoch_increased=True,
    obstacle_probes=obstacle,border_argument_action_difference=border_delta,native_source_unmodified=not meta['native_dirty'],
    limits=['No Gazebo or LiDAR test in this cohort','No RGL checkpoint or learned robot policy','Grouping is not a social formation model','Native internal obstacle ignorance retained, no clipping/repair'])
(out/'validation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps(result,indent=2))
