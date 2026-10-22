#!/usr/bin/env python3
"""Paired native/variant scenarios. Preserve every failure and its source."""
import argparse,copy,hashlib,inspect,json,math,platform,subprocess,time,traceback
from pathlib import Path
import numpy as np
from kernel import GeometryKernel
from native_contract import ContractKernel,fixture,DT
from swept import admissible_fraction,clearance
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args()
out=a.report_dir;out.mkdir(parents=True,exist_ok=False)
layout=json.loads((HERE/'generated/layout.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files=[HERE/n for n in ['kernel.py','roadmap.py','probe.py','generated/swept.py','generated/native_contract.py','generated/layout.json']]
snapshot=out/'source_snapshot';snapshot.mkdir()
for f in files:(snapshot/f.name).write_bytes(f.read_bytes())
config=dict(timestep_s=.25,human_radius_m=.3,robot_radius_m=.25,speed_ceiling_mps=1.,orca_extra_radius_m=.02,
 roadmap_corner_padding_m=.04,swept_margin_m=.002,static_boxes=42,seed=42,
 motion='Native policy intention at roadmap waypoint, then42-obstacle global disc ORCA projection and exact swept fraction. Explicit modified behavior.',
 gate='All adapter cases: finite, continuous, no measured negative swept gaps, speed<=1.00001m/s. Rack/narrow: each human completes at least one native goal leg within60s. Mixed20s: total travel>5m.')
(out/'protocol.json').write_text(json.dumps(dict(config=config,source_sha256={str(f.relative_to(HERE)):sha(f) for f in files},native_commit=subprocess.check_output(['git','-C','/opt/ComplexityNav','rev-parse','HEAD']).decode().strip(),native_dirty=subprocess.check_output(['git','-C','/opt/ComplexityNav','status','--porcelain']).decode(),python=platform.python_version()),indent=2)+'\n')

def robot_input(name,t):
    x,y=-27.,0.;vx=vy=0.
    if name=='forward':x+=.1*min(t,6.);vx=.1 if t<6. else 0.
    if name=='lateral_then_forward':
        y=.1*min(t,3.);x+=.1*max(0.,min(t-3.,3.));vy=.1 if t<3. else 0.;vx=.1 if 3.<=t<6. else 0.
    return dict(x=x,y=y,vx=vx,vy=vy,yaw=0.)

def request(k,robot,advance=True):return dict(epoch=k.epoch,time=k.time,frame='world',robot=robot,advance=advance)

def normalized_state(row):
    return [{key:h[key] for key in ['id','x','y','vx','vy','gx','gy','radius','policy']} for h in row['humans']]

def measure(rows):
    minima=dict(static=1e9,pair=1e9,robot=1e9);path=0.;max_speed=0.;min_fraction=1.;stops=0
    for prev,row in zip(rows,rows[1:]):
        starts=[(h['x'],h['y']) for h in prev['humans']];deltas=[(b['x']-a['x'],b['y']-a['y']) for a,b in zip(prev['humans'],row['humans'])]
        robot=row['robot_input'];gaps=clearance(starts,deltas,[h['radius'] for h in row['humans']],(robot['x'],robot['y']),(DT*robot['vx'],DT*robot['vy']),.25,layout['obstacles'])
        for key,v in gaps.items():minima[key]=min(minima[key],v)
        lengths=[math.hypot(*d) for d in deltas];path+=sum(lengths);max_speed=max(max_speed,max(lengths)/DT)
        for d,h in zip(deltas,row['humans']):assert math.hypot(d[0]-DT*h['vx'],d[1]-DT*h['vy'])<1e-9
        if 'geometry' in row:
            fraction=row['geometry']['safe_fraction'];min_fraction=min(min_fraction,fraction);stops+=int(fraction==0.)
    legs={str(h['id']):int(h.get('completed_legs',0)) for h in rows[-1]['humans']}
    return dict(swept_gap_m=minima,human_path_total_m=path,max_speed_mps=max_speed,min_safe_fraction=min_fraction,full_stop_ticks=stops,completed_legs=legs)

cases=[dict(name=n,humans=fixture(),steps=80) for n in ['stationary','forward','lateral_then_forward']]
for policy in ['orca','socialforce']:
 cases.append(dict(name='rack_'+policy,humans=[dict(id=1,policy=policy,x=-23.,y=8.5,gx=-17.,gy=8.5)],steps=240))
cases.append(dict(name='narrow_two_way',humans=[dict(id=1,policy='orca',x=9.,y=-.4,gx=23.,gy=-.4),dict(id=2,policy='socialforce',x=23.,y=.4,gx=9.,gy=.4)],steps=240))
(out/'fixtures.json').write_text(json.dumps(cases,indent=2)+'\n')
# Analytic regression: fixed full robot displacement, tunneling, infeasible input.
unit={}
f,g=admissible_fraction([(0.,0.)],[(.5,0.)],[.2],(-.8,0.),(.6,0.),.2,[dict(x=100.,y=100.,sx=1.,sy=1.)]);assert f==1.;unit['escape_incoming_robot']=True
try:admissible_fraction([(0.,0.)],[(0.,0.)],[.2],(-.5,0.),(.5,0.),.2,[dict(x=100.,y=100.,sx=1.,sy=1.)])
except RuntimeError:unit['infeasible_robot_rejected']=True
else:raise AssertionError('Infeasible robot input accepted')
f,g=admissible_fraction([(-2.,0.)],[(4.,0.)],[.2],(100.,100.),(0.,0.),.2,[dict(x=0.,y=0.,sx=.2,sy=2.)]);assert f<1. and min(g.values())>=.002-1e-8;unit['between_endpoint_tunneling_rejected']=True
results=[];all_rows={};primary=None
start=time.monotonic()
for case in cases:
    pair={}
    for variant in ['native','geometry']:
        rows=[];error=None;k=None;begin=time.monotonic()
        try:
            k=GeometryKernel(layout,case['humans']) if variant=='geometry' else ContractKernel(case['humans'])
            rows=[k.state()]
            for n in range(case['steps']):
                r=robot_input(case['name'],n*DT);before=k.state();loglen=len(k.projector.log) if variant=='geometry' else 0
                assert k.request(request(k,r,False))==before
                if variant=='geometry':assert len(k.projector.log)==loglen
                row=k.request(request(k,r));rows.append(row)
                assert [h['id'] for h in row['humans']]==[h['id'] for h in rows[0]['humans']]
                if variant=='native':
                    for rowh,h in zip(row['humans'],k.env.humans):rowh['completed_legs']=int(h.current_goal)
            metrics=measure(rows)
            if variant=='geometry':
                assert min(metrics['swept_gap_m'].values())>=-1e-6,metrics
                assert metrics['max_speed_mps']<=1.00001
                if case['steps']==240:assert min(metrics['completed_legs'].values())>=1,metrics
                else:assert metrics['human_path_total_m']>5.,metrics
            status='PASS'
        except Exception as exc:
            error=repr(exc);status='ERROR';metrics=measure(rows) if len(rows)>1 else None
            (out/(case['name']+'_'+variant+'_error.txt')).write_text(traceback.format_exc())
        path=out/(case['name']+'_'+variant+'_trajectory.json');path.write_text(json.dumps(rows,allow_nan=False)+'\n')
        result=dict(case=case['name'],variant=variant,status=status,steps=len(rows)-1,required_steps=case['steps'],metrics=metrics,error=error,wall_seconds=time.monotonic()-begin)
        results.append(result);pair[variant]=rows;all_rows[(case['name'],variant)]=rows
        (out/'progress.json').write_text(json.dumps(results,indent=2)+'\n')
        if case['name']=='stationary' and variant=='geometry' and status=='PASS':primary=k
        print(json.dumps({k:v for k,v in result.items() if k!='metrics'}),flush=True)
    if pair['native'] and pair['geometry']:assert normalized_state(pair['native'][0])==normalized_state(pair['geometry'][0])
# Reset while active; no solver history/clock/IDs should leak across epochs.
reset=dict(status='NOT_RUN')
if primary:
    try:
        old=primary.epoch;primary.reset();assert primary.epoch==old+1 and primary.time==0 and not primary.projector.log
        replay=[primary.state()]
        for n in range(80):replay.append(primary.request(request(primary,robot_input('stationary',n*DT))))
        for a,b in zip(replay,all_rows[('stationary','geometry')]):assert normalized_state(a)==normalized_state(b) and a['time']==b['time']
        (out/'reset_trajectory.json').write_text(json.dumps(replay,allow_nan=False)+'\n')
        reset=dict(status='PASS',steps=80,epoch=primary.epoch)
        before=primary.state();bad=request(primary,robot_input('stationary',0));bad['epoch']=old
        try:primary.request(bad)
        except ValueError:unit['stale_epoch_rejected']=True
        else:raise AssertionError('Stale reset epoch accepted')
        assert primary.state()==before
        for point,goal in [((-20.,8.5),(-17.,8.5)),((-23.,8.5),(-20.,8.5))]:
            try:primary.roadmap.waypoint(point,goal)
            except ValueError:pass
            else:raise AssertionError('Roadmap accepted invalid occupied disc')
        unit['occupied_start_goal_rejected']=True
    except Exception as exc:reset=dict(status='ERROR',error=repr(exc))
accepted=all(row['status']=='PASS' and row['steps']==row['required_steps'] for row in results if row['variant']=='geometry') and reset['status']=='PASS'
summary=dict(status='PASS' if accepted else 'FAIL',cases=results,reset=reset,unit_checks=unit,wall_seconds=time.monotonic()-start,scope='Offline only; native baseline retained; modified route plus safety projection, not native parity or general safety proof')
(out/'validation.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2));raise SystemExit(0 if accepted else 2)
