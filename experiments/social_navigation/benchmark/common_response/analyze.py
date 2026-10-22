#!/usr/bin/env python3
"""Shared-input audit and paired response metrics; never treats failures as safe runs."""
import argparse,csv,hashlib,json,math,os,subprocess
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
import numpy as np,shapely
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,default=HERE/'reports/run01');a=p.parse_args();out=a.report_dir.resolve();load=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protocol=load(out/'protocol.json');assert protocol['population']==6
for name,digest in protocol['input_sha256'].items():assert sha(HERE/name)==digest,name
for name,digest in protocol['source_sha256'].items():assert sha(REPO/name)==digest,name
layout=load(HERE/'inputs/layout.json');assert len(layout['obstacles'])==42
polygons=np.array([shapely.box(b['x']-b['sx']/2,b['y']-b['sy']/2,b['x']+b['sx']/2,b['y']+b['sy']/2) for b in layout['obstacles']],dtype=object)

rows_out=[];data={};paired=[];repeats=[]
for model,settings in protocol['models'].items():
    summary=load(out/model/'summary.json');repeats.append(dict(model=model,**summary['repeat']))
    for name in protocol['fixtures']:
        case=load(HERE/'inputs'/(name+'.json'));rows=load(out/model/(name+'_trajectory.json'));result=load(out/model/(name+'_result.json'));dt=settings['dt'];data[(model,name)]=rows
        if not rows:
            rows_out.append(dict(model=model,case=name,status=result['status'],steps=0,error=result['error'],geometry_outcome='NO_DATA'));continue
        assert len(rows[0]['humans'])==6
        for h,f in zip(rows[0]['humans'],case['humans']):
            assert h['id']==f['id'] and abs(h['radius']-.36)<1e-12
            assert abs(h['x']-f['x'])<1e-8 and abs(h['y']-f['y'])<1e-8
        for i,row in enumerate(rows):
            assert abs(row['time']-i*dt)<1e-9 and [h['id'] for h in row['humans']]==list(range(1,7))
            assert all(math.isfinite(h[k]) for h in row['humans'] for k in ['x','y','vx','vy'])
            if i:
                expected=case['robot_trace'][round((i-1)*dt/.05)].copy();expected.pop('time');assert row['robot_input']==expected
        n=len(rows)-1
        if n==0:
            rows_out.append(dict(model=model,case=name,status=result['status'],steps=0,error=result['error'],geometry_outcome='NO_UPDATES'));continue
        before=np.array([[[h['x'],h['y']] for h in row['humans']] for row in rows[:-1]]);after=np.array([[[h['x'],h['y']] for h in row['humans']] for row in rows[1:]]);delta=after-before
        robots=np.array([[row['robot_input'][k] for k in ['x','y']] for row in rows[1:]]);robot_delta=np.array([[row['robot_input'][k]*dt for k in ['vx','vy']] for row in rows[1:]])
        def gap(a,d,r):
            den=np.sum(d*d,axis=-1);t=np.clip(np.divide(-np.sum(a*d,axis=-1),den,out=np.zeros_like(den),where=den>1e-25),0,1)
            return np.linalg.norm(a+t[...,None]*d,axis=-1)-r
        pair_series=np.full(n,np.inf)
        for i in range(6):
            for j in range(i):pair_series=np.minimum(pair_series,gap(before[:,i]-before[:,j],delta[:,i]-delta[:,j],.72))
        robot_series=np.min(gap(before-robots[:,None,:],delta-robot_delta[:,None,:],.61),axis=1)
        segments=shapely.linestrings(np.stack((before,after),axis=2).reshape(-1,2,2));static_matrix=shapely.distance(segments[:,None],polygons[None,:])-.36
        static_series=np.min(static_matrix.reshape(n,6,42),axis=(1,2))
        path=float(np.sum(np.linalg.norm(delta,axis=2)));speeds=np.linalg.norm(delta,axis=2)/dt
        goals=np.array([[h['gx'],h['gy']] for h in case['humans']]);initial=before[0];final=after[-1]
        progress=float(np.sum(np.linalg.norm(goals-initial,axis=1)-np.linalg.norm(goals-final,axis=1)))
        mins=dict(pair=float(np.min(pair_series)),robot=float(np.min(robot_series)),static=float(np.min(static_series)))
        overlap=sum(int(np.sum(series<-.001)) for series in [pair_series,robot_series,static_series])
        outcome='ERROR_PREFIX' if result['status']!='COMPLETED' else 'OVERLAP_GT_1MM' if overlap else 'NO_OVERLAP_GT_1MM'
        blocked_ticks=int(np.sum(np.all(speeds<.01,axis=1)))
        row=dict(model=model,case=name,status=result['status'],steps=n,seconds_observed=n*dt,error=result['error'],geometry_outcome=outcome,
          min_pair_gap_m=mins['pair'],min_robot_gap_m=mins['robot'],min_static_gap_m=mins['static'],pair_flagged_tick_s=float(np.sum(pair_series<-.001)*dt),robot_flagged_tick_s=float(np.sum(robot_series<-.001)*dt),static_flagged_tick_s=float(np.sum(static_series<-.001)*dt),
          human_path_m=path,goal_progress_m=progress,speed_max_mps=float(np.max(speeds)),all_humans_slow_tick_s=blocked_ticks*dt,wall_s=result['wall_seconds'])
        rows_out.append(row)
        if result['status']=='COMPLETED':assert n==result['required_steps']
    repeat=load(out/model/'repeat_prefix.json') if (out/model/'repeat_prefix.json').exists() else None
    if repeat:assert repeat==data[(model,protocol['fixtures'][0])][:21]
    for scene in ['opposed','crossing','blocked_narrow']:
        far=data[(model,scene+'_far')];near=data[(model,scene+'_near')]
        end=min(far[-1]['time'] if far else 0.,near[-1]['time'] if near else 0.)
        samples=[]
        for t in np.arange(0.,end+1e-8,.5):
            f=far[round(t/settings['dt'])];n=near[round(t/settings['dt'])]
            distances=[math.hypot(a['x']-b['x'],a['y']-b['y']) for a,b in zip(f['humans'],n['humans'])];samples.append(dict(time=float(t),mean_position_difference_m=float(np.mean(distances)),max_position_difference_m=max(distances)))
        first=next((samples[i]['time'] for i in range(len(samples)-1) if samples[i]['max_position_difference_m']>.05 and samples[i+1]['max_position_difference_m']>.05),None)
        fmetric=next(r for r in rows_out if r['model']==model and r['case']==scene+'_far');nmetric=next(r for r in rows_out if r['model']==model and r['case']==scene+'_near')
        paired.append(dict(model=model,scene=scene,common_prefix_seconds=float(end),full_pair=fmetric['status']==nmetric['status']=='COMPLETED',response_first_5cm_two_samples_s=first,
          max_position_difference_m=max((s['max_position_difference_m'] for s in samples),default=None),near_goal_progress_m=nmetric.get('goal_progress_m'),far_goal_progress_m=fmetric.get('goal_progress_m'),samples=samples))
(out/'metrics.json').write_text(json.dumps(rows_out,indent=2)+'\n');(out/'paired_response.json').write_text(json.dumps(paired,indent=2)+'\n')
fields=sorted({k for row in rows_out for k in row})
with (out/'metrics.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows_out)
cp=HERE/'generated/checkpoint';allowed={'experiments/social_navigation/'+n for n in ['README.md','NEXT_STEPS.md','COMMIT_HISTORY.md','FRAMEWORK_COMPARISON.md']};saved=load(cp/'files.json')
for name,digest in saved.items():
    if name not in allowed:assert sha(REPO/name)==digest,name
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)
assert not subprocess.check_output(['docker','ps','-aq','--filter','name=dynamuhsim-common-'],text=True).strip()
audit=load(out/'observation_api_audit.json');assert sha(REPO/audit['source_file'])==audit['source_sha256']
validation=dict(internal_prediction_assumptions_equal=False,status='AUDITED_WITH_ALL_OUTCOMES_RETAINED',cases=len(rows_out),models=6,completed=sum(r['status']=='COMPLETED' for r in rows_out),errors=[dict(model=r['model'],case=r['case'],error=r['error']) for r in rows_out if r['status']!='COMPLETED'],
 repeat_checks=repeats,full_paired_comparisons=sum(p['full_pair'] for p in paired),initial_state_and_common_inputs_verified=True,geometry_boxes=42,human_radius=.36,robot_radius=.25,comparison_grid_s=.5,
 preserved_old_files=len(saved)-len(allowed),original_staging_outside_preserved=True,owned_containers_remaining=0,
 notes=['Offline scripted disc inputs, not Gazebo/physical-robot replay','Different native dt/intended speed; no ranking from raw distance or timing','Flagged tick seconds are sums of flagged intervals, not exact penetration duration','Native CN missing interior geometry is a negative control','Groups/conversation capabilities use separate previously recorded evidence','Shared trace dispatch does not make internal observation APIs equal: Soc native infers robot intent from dummy goal; see observation_api_audit.json'])
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n');print(json.dumps(validation,indent=2))
