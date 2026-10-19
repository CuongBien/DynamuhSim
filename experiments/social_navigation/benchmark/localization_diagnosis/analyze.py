#!/usr/bin/env python3
"""Analyze saved throat trials without starting ROS, Gazebo or changing parameters."""
import argparse,bisect,csv,hashlib,json,math,statistics,subprocess
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from geometry import MapField,rays,stamp,yaw,grid_footprint_cost
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,default=BENCH/'bottleneck/reports/run01');p.add_argument('--output',type=Path,default=HERE/'reports/run01');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
field=MapField(SOCIAL/'warehouse/warehouse.yaml');boxes=json.loads((SOCIAL/'warehouse/layout.json').read_text())['obstacles']
names=['empty_bottleneck_s42']+['crowdsimplus'+('_sfm' if policy=='sfm' else '')+'_individual_goals_bottleneck_s'+str(seed) for policy in ['orca_plus','sfm'] for seed in [42,43]]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
original_manifest=json.loads((a.input/'run_manifest.json').read_text()) if (a.input/'run_manifest.json').exists() else None
def pose(d):p=d['position'];return dict(x=p['x'],y=p['y'],yaw=yaw(d['orientation']))
def nearest(rows,t,max_dt=.06):
    times=rows[0];values=rows[1];i=bisect.bisect_left(times,t);indices=range(max(0,i-1),min(len(times),i+1));index=min(indices,key=lambda i:abs(times[i]-t))
    return (times[index],values[index]) if abs(times[index]-t)<=max_dt else None
def series(rows):
    rows=sorted(rows,key=lambda r:r[0]);return [r[0] for r in rows],[r[1] for r in rows]
def pairs_distance(a,b):return math.hypot(a['x']-b['x'],a['y']-b['y'])
def compose(a,b):c,s=math.cos(a['yaw']),math.sin(a['yaw']);return dict(x=a['x']+c*b['x']-s*b['y'],y=a['y']+s*b['x']+c*b['y'],yaw=a['yaw']+b['yaw'])
def safe(v):
    if isinstance(v,float) and not math.isfinite(v):return None
    if isinstance(v,dict):return {k:safe(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [safe(x) for x in v]
    if isinstance(v,np.generic):return safe(v.item())
    return v
def plot_samples(ax,rows,values,**kwargs):
    # Do not visually interpolate periods without AMCL publications.
    xs=[];ys=[]
    for i,(row,value) in enumerate(zip(rows,values)):
        if i and row['elapsed']-rows[i-1]['elapsed']>.45:xs.append(np.nan);ys.append(np.nan)
        xs.append(row['elapsed']);ys.append(value)
    return ax.plot(xs,ys,marker='.',markersize=3,**kwargs)
summaries=[];traces={};inputs=[];events=[];snapshots={};tf_static_details={}
for name in names:
    trial=json.loads((a.input/(name+'.json')).read_text());assert trial['protocol_valid'] and trial['condition']=='bottleneck' and trial['robot_start']==[8.,0.,0.]
    runtime=BENCH/trial['runtime'];rawpath=runtime/'diagnostics.jsonl';gate=float((runtime/'start_gate').read_text());end=gate+trial['sim_seconds']
    if original_manifest is not None:
        original=next(t for t in original_manifest['trials'] if t['name']==name);assert sha(rawpath)==original['raw_sha256']['diagnostics.jsonl']
    groups={k:[] for k in ['gazebo','odom','amcl','scan','costmap','cmd_vel','map_odom']};static=[];counts={}
    for line in rawpath.open():
        raw=json.loads(line);kind=raw['kind'];d=raw['data'];counts[kind]=counts.get(kind,0)+1
        if kind=='gazebo':groups[kind].append((d['time'],d['poses']))
        elif kind in ('odom','amcl','scan','costmap','cmd_vel'):groups[kind].append((stamp(d),dict(data=d,receipt_sim=raw['receipt_sim'])))
        elif kind=='tf_static':static+=d['transforms']
        elif kind=='tf':
            for t in d['transforms']:
                if (t['header']['frame_id'],t['child_frame_id'])==('map','odom'):
                    tr=t['transform'];groups['map_odom'].append((stamp(t)-.5,dict(x=tr['translation']['x'],y=tr['translation']['y'],yaw=yaw(tr['rotation']),receipt_sim=raw['receipt_sim'],header_time=stamp(t))))
    for k in groups:groups[k]=series(groups[k])
    cfg=yaml.safe_load((runtime/'nav2_scenario.yaml').read_text())['amcl']['ros__parameters']
    assert cfg['laser_model_type']=='likelihood_field' and cfg['max_beams']==60 and cfg.get('sigma_hit',.2)==.2 and cfg.get('z_hit',.5)==.5 and cfg.get('z_rand',.5)==.5
    laser=[t for t in static if t['child_frame_id']=='base_scan'];assert len(laser)==1 and laser[0]['header']['frame_id']=='base_link'
    assert all(abs(laser[0]['transform']['translation'][k])<1e-12 for k in ['x','y']) and abs(yaw(laser[0]['transform']['rotation']))<1e-12
    tf_static_details[name]=laser[0]
    odom_errors=[];lag=[];tf_offsets=[]
    for t,raw in zip(*groups['odom']):
        match=nearest(groups['gazebo'],t)
        if not gate<=t<=end or match is None:continue
        actual=match[1]['mecanum_robot'];od=pose(raw['data']['pose']['pose']);odw=compose(dict(x=8.,y=0.,yaw=0.),od);odom_errors.append(dict(elapsed=t-gate,position_error_m=pairs_distance(actual,odw),time_error_s=abs(match[0]-t)))
    for t,raw in zip(*groups['scan']):
        if gate<=t<=end and raw['receipt_sim'] is not None:lag.append(raw['receipt_sim']-t)
    for t,tf in zip(*groups['map_odom']):
        if gate<=t<=end and tf['receipt_sim'] is not None:tf_offsets.append(tf['header_time']-tf['receipt_sim'])
    rows=[];scan_cache={}
    for t,raw in zip(*groups['amcl']):
        if not gate<=t<=end:continue
        actual=nearest(groups['gazebo'],t);od=nearest(groups['odom'],t);scan=nearest(groups['scan'],t,.02);tf=nearest(groups['map_odom'],t,.025)
        if actual is None or od is None:continue
        ro=actual[1]['mecanum_robot'];estimate=pose(raw['data']['pose']['pose']);odpose=pose(od[1]['data']['pose']['pose']);odworld=compose(dict(x=8.,y=0.,yaw=0.),odpose)
        row=dict(elapsed=t-gate,time=t,actual=ro,amcl=estimate,odom_world=odworld,position_error_m=pairs_distance(ro,estimate),odom_error_m=pairs_distance(ro,odworld),actual_time_error_s=abs(actual[0]-t),amcl_covariance_xx=raw['data']['pose']['covariance'][0],amcl_covariance_yy=raw['data']['pose']['covariance'][7],amcl_covariance_yawyaw=raw['data']['pose']['covariance'][35],static_occupied_at_amcl=bool(field.occupied_at(estimate['x'],estimate['y'])),static_occupied_at_actual=bool(field.occupied_at(ro['x'],ro['y'])))
        if tf is not None:row['tf_composed_amcl_error_m']=pairs_distance(compose(tf[1],odpose),estimate)
        grid=nearest(groups['costmap'],t,.3)
        if grid is not None:
            gt=grid[0];god=nearest(groups['odom'],gt)
            if god is not None:
                gd=grid[1]['data'];assert gd['header']['frame_id']=='odom';row.update(grid_footprint_cost(gd,pose(god[1]['data']['pose']['pose'])));row['local_grid_time_offset_s']=gt-t
        if scan is not None:
            st,sd=scan[0],scan[1]['data'];gzs=nearest(groups['gazebo'],st);sr=gzs[1]['mecanum_robot'];humans=[dict(id=int(k.split('_')[1]),**v) for k,v in gzs[1].items() if k!='mecanum_robot']
            angles=sd['angle_min']+np.arange(len(sd['ranges']))*sd['angle_increment'];rs,rh=rays(sr,angles,boxes,humans);observed=np.array(sd['ranges']);valid=np.isfinite(observed)&(observed>sd['range_min'])&(observed<sd['range_max'])
            difference=np.full(len(observed),np.inf);np.subtract(observed,rh,out=difference,where=np.isfinite(observed)&np.isfinite(rh));human=valid&(rh<rs-.05)&(np.abs(difference)<.08);predicted=np.minimum(rs,rh);residual=np.abs(observed[valid]-predicted[valid]);static_valid=valid&~human&np.isfinite(rs)&(rs<sd['range_max'])
            candidate=[[ro['x'],ro['y'],ro['yaw']],[estimate['x'],estimate['y'],estimate['yaw']]]
            score,nused=field.score(candidate,sd);masked,nmasked=field.score(candidate,sd,~human)
            row.update(scan_time_offset_s=st-t,valid_scan_beams=int(valid.sum()),confirmed_human_beams=int(human.sum()),human_beam_fraction=float(human.sum()/max(1,valid.sum())),
                ray_model_abs_error_median_m=float(np.median(residual)) if len(residual) else None,ray_model_match_fraction=float(np.mean(residual<.08)) if len(residual) else None,static_return_abs_error_median_m=float(np.median(np.abs(observed[static_valid]-rs[static_valid]))) if np.any(static_valid) else None,
                score_truth=float(score[0]),score_amcl=float(score[1]),score_truth_human_masked=float(masked[0]),score_amcl_human_masked=float(masked[1]),selected_beams=nused,selected_human_beams=nused-nmasked,selected_beams_after_human_mask=nmasked)
            scan_cache[round(t,6)]=dict(scan=sd,human_mask=human.tolist(),actual=sr,amcl=estimate,humans=humans)
        rows.append(row)
    assert len(rows)>20
    jump=max([(i,pairs_distance(after['amcl'],before['amcl'])) for i,(before,after) in enumerate(zip(rows,rows[1:]),1)],key=lambda v:v[1]);i,jump_distance=jump;before,after=rows[i-1],rows[i]
    ev=dict(name=name,elapsed=after['elapsed'],amcl_position_step_m=jump_distance,actual_position_step_m=pairs_distance(before['actual'],after['actual']),odom_position_step_m=pairs_distance(before['odom_world'],after['odom_world']),before=before,after=after)
    events.append(ev)
    summary=dict(name=name,status=trial['status'],recorded_kinds=counts,amcl_samples=len(rows),amcl_error_max_m=max(row['position_error_m'] for row in rows),
        odom_error_max_m=max(row['position_error_m'] for row in odom_errors),odom_error_rms_m=math.sqrt(statistics.mean(row['position_error_m']**2 for row in odom_errors)),odom_match_time_error_max_s=max(row['time_error_s'] for row in odom_errors),
        scan_receipt_delay_min_s=min(lag),scan_receipt_delay_max_s=max(lag),tf_future_offset_median_s=statistics.median(tf_offsets),
        tf_composed_amcl_error_max_m=max((row['tf_composed_amcl_error_m'] for row in rows if 'tf_composed_amcl_error_m' in row),default=None),
        largest_amcl_step_m=jump_distance,largest_step_elapsed_s=after['elapsed'],actual_step_during_largest_amcl_step_m=ev['actual_position_step_m'],odom_step_during_largest_amcl_step_m=ev['odom_position_step_m'],
        first_static_occupied_amcl_elapsed_s=next((row['elapsed'] for row in rows if row['static_occupied_at_amcl']),None),
        median_ray_model_error_m=statistics.median(row['ray_model_abs_error_median_m'] for row in rows if row.get('ray_model_abs_error_median_m') is not None),
        first_persistent_stop_elapsed_s=None,global_costmap_recorded=False,particle_cloud_recorded=False)
    # Stationary translation is measured from odometry, including spin recovery.
    stop_start=None
    for t,raw in zip(*groups['odom']):
        if not gate<=t<=end:continue
        vel=raw['data']['twist']['twist']['linear'];slow=math.hypot(vel['x'],vel['y'])<.02
        if slow and stop_start is None:stop_start=t
        elif not slow:stop_start=None
        if stop_start is not None and t-stop_start>=1. and stop_start-gate>2.:
            summary['first_persistent_stop_elapsed_s']=stop_start-gate;break
    summary['ray_model_match_fraction_median']=statistics.median(row['ray_model_match_fraction'] for row in rows if row.get('ray_model_match_fraction') is not None)
    summaries.append(summary);traces[name]=dict(amcl=rows,odom=odom_errors)
    inputs.append(dict(name=name,report=str(a.input/(name+'.json')),runtime=str(runtime),diagnostics_sha256=sha(rawpath),scene_sha256=sha(runtime/'scene.json'),nav_params_sha256=sha(runtime/'nav2_scenario.yaml')))
    if name=='crowdsimplus_individual_goals_bottleneck_s42':
        target_times=[before['time'],after['time']]
        for label,target in zip(['before_jump','after_jump'],target_times):snapshots[label]=dict(time=target,elapsed=target-gate,**scan_cache[round(target,6)])
    print(name,summary['status'],'AMCLmax',round(summary['amcl_error_max_m'],3),'odommax',round(summary['odom_error_max_m'],4),'jump',round(jump_distance,3),flush=True)
failure_windows=[]
for summary in summaries:
    if summary['status']!='ABORTED':continue
    rows=traces[summary['name']]['amcl'];anchors={}
    for label,t in [('persistent_translation_stop',summary['first_persistent_stop_elapsed_s']),('estimated_center_enters_occupied',summary['first_static_occupied_amcl_elapsed_s'])]:
        if t is not None:
            matched=min(rows,key=lambda row:abs(row['elapsed']-t));anchors[label]=dict(requested_elapsed_s=t,nearest_amcl_elapsed_s=matched['elapsed'],time_offset_s=matched['elapsed']-t,sample=matched)
    anchors['last_matched_amcl']=rows[-1]
    failure_windows.append(dict(name=summary['name'],anchors=anchors))
(a.output/'failure_windows.json').write_text(json.dumps(safe(failure_windows),indent=2,allow_nan=False))
manifest=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),inputs=inputs,map_sha256=sha(SOCIAL/'warehouse/warehouse.pgm'),
    installed_amcl_package=subprocess.check_output(['dpkg-query','-W','-f=${Version}','ros-jazzy-nav2-amcl'],text=True),
    source_references=json.loads((HERE/'generated/reference/manifest.json').read_text()),
    tf_stamp_backshift_used_s=.5,episode_launch_sha256=sha(SOCIAL.parents[1]/'install/custom_corridor/share/custom_corridor/launch/nav2_corridor.launch.py'),
    method='Offline telemetry alignment plus EDT reconstruction of stock 1.3.13 likelihood_field sensor score. No particle posterior, native PF replay, ROS writes or Gazebo trial.')
(a.output/'summary.json').write_text(json.dumps(safe(dict(trials=summaries,scope=manifest['method'])),indent=2,allow_nan=False));(a.output/'events.json').write_text(json.dumps(safe(events),indent=2,allow_nan=False));(a.output/'timeline.json').write_text(json.dumps(safe(traces),indent=2,allow_nan=False));(a.output/'scan_snapshots.json').write_text(json.dumps(safe(snapshots),indent=2,allow_nan=False));(a.output/'manifest.json').write_text(json.dumps(manifest,indent=2));(a.output/'lidar_tf.json').write_text(json.dumps(tf_static_details,indent=2))
with (a.output/'results.csv').open('w',newline='') as f:
    keys=[k for k in summaries[0] if k!='recorded_kinds'];writer=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore',lineterminator='\n');writer.writeheader();writer.writerows(summaries)
# Per-trial localization error; odometry plotted separately from AMCL to show scale.
fig,axes=plt.subplots(3,2,figsize=(12,9));axes=axes.flat
for ax,summary in zip(axes,summaries):
    name=summary['name'];data=traces[name];rows=data['amcl'];od=data['odom'];plot_samples(ax,rows,[r['position_error_m'] for r in rows],label='AMCL vs Gazebo');ax.plot([r['elapsed'] for r in od],[r['position_error_m'] for r in od],label='spawn + odom vs Gazebo');ax.set(title=name.replace('crowdsimplus','CSP').replace('individual_goals_',''),xlabel='simulation elapsed [s]',ylabel='position error [m]');ax.grid(alpha=.2)
axes[0].legend(fontsize=8);axes[-1].axis('off');fig.tight_layout();fig.savefig(a.output/'localization_vs_odom.png',dpi=160,bbox_inches='tight');plt.close(fig)
name='crowdsimplus_individual_goals_bottleneck_s42';rows=traces[name]['amcl'];event=next(e for e in events if e['name']==name);jump=event['elapsed']
fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True)
plot_samples(axes[0],rows,[r['actual']['y'] for r in rows],label='actual y');plot_samples(axes[0],rows,[r['amcl']['y'] for r in rows],label='AMCL y');axes[0].axhspan(.9,2.4,color='#ddd');axes[0].set(ylabel='world y [m]');axes[0].legend(loc='upper left');covax=axes[0].twinx();plot_samples(covax,rows,[math.sqrt(max(0,r['amcl_covariance_yy'])) for r in rows],c='#9467bd',ls=':',label='published sigma y');covax.set_ylabel('published sigma y [m]',color='#9467bd')
plot_samples(axes[1],rows,[r.get('human_beam_fraction',np.nan) for r in rows]);axes[1].set(ylabel='confirmed human / finite beams')
plot_samples(axes[2],rows,[r.get('footprint_occupancy_max',np.nan) for r in rows]);axes[2].set(ylabel='local footprint grid max [0..100]')
for key,label in [('score_truth','truth'),('score_amcl','AMCL'),('score_truth_human_masked','truth, human rays masked'),('score_amcl_human_masked','AMCL, human rays masked')]:plot_samples(axes[3],rows,[r.get(key,np.nan) for r in rows],label=label,ls='--' if 'masked' in key else '-')
axes[3].set(ylabel='reconstructed sensor score',xlabel='simulation elapsed [s]');axes[3].legend(fontsize=8,ncol=2)
for ax in axes:ax.axvline(jump,color='red',ls=':');ax.grid(alpha=.2)
fig.suptitle('ORCA+ seed42 | sensor score only, not particle posterior');fig.tight_layout();fig.savefig(a.output/'jump_timeline.png',dpi=160,bbox_inches='tight');plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(12,5))
for ax,(label,snap) in zip(axes,snapshots.items()):
    for b in boxes:ax.add_patch(Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],color='#ddd'))
    sd=snap['scan'];ranges=np.array([v if v is not None else np.nan for v in sd['ranges']]);valid=np.isfinite(ranges)&(ranges<sd['range_max'])&(ranges>sd['range_min']);angles=sd['angle_min']+np.arange(len(ranges))*sd['angle_increment'];human=np.array(snap['human_mask'])
    for key,color,marker in [('actual','#2774b5','.'),('amcl','#c54d3b','x')]:
        ro=snap[key];x=ro['x']+ranges*np.cos(ro['yaw']+angles);y=ro['y']+ranges*np.sin(ro['yaw']+angles);ax.scatter(x[valid],y[valid],s=9,c=color,marker=marker,label=key+' endpoints');ax.scatter([ro['x']],[ro['y']],s=70,c=color,marker='s')
    ro=snap['actual'];x=ro['x']+ranges*np.cos(ro['yaw']+angles);y=ro['y']+ranges*np.sin(ro['yaw']+angles);ax.scatter(x[human],y[human],s=18,c='#d19a26',label='confirmed human hits')
    for h in snap['humans']:ax.add_patch(Circle((h['x'],h['y']),.36,fill=False,color='#3c966c'))
    ax.set(xlim=(11,20),ylim=(-3,4),title=f"{label}: {snap['elapsed']:.3f} s",xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal');ax.grid(alpha=.2)
axes[0].legend(fontsize=8);fig.tight_layout();fig.savefig(a.output/'scan_before_after.png',dpi=160,bbox_inches='tight');plt.close(fig)
# Candidate-pose likelihood landscape at the after-jump AMCL heading.
snap=snapshots['after_jump'];xs=np.arange(13.,19.01,.05);ys=np.arange(-2.5,3.51,.05);xx,yy=np.meshgrid(xs,ys);theta=snap['amcl']['yaw'];poses=np.column_stack([xx.ravel(),yy.ravel(),np.full(xx.size,theta)]);sd=snap['scan'];sd=dict(sd,ranges=[v if v is not None else float('inf') for v in sd['ranges']]);scores,_=field.score(poses,sd);masked,_=field.score(poses,sd,~np.array(snap['human_mask']));occupied=field.occupied_at(xx,yy)
fig,axes=plt.subplots(1,2,figsize=(11,5))
for ax,values,title in zip(axes,[scores,masked],['all selected rays','oracle human-ray mask']):
    im=ax.imshow(values.reshape(xx.shape),extent=[xs[0],xs[-1],ys[0],ys[-1]],origin='lower',aspect='equal');ax.contour(xx,yy,occupied.astype(float),levels=[.5],colors='white',linewidths=1)
    for key,color in [('actual','cyan'),('amcl','red')]:ax.scatter([snap[key]['x']],[snap[key]['y']],c=color,s=45,edgecolors='black',label=key+' XY')
    ax.set(title=title,xlabel='candidate x [m]',ylabel='candidate y [m]');fig.colorbar(im,ax=ax,shrink=.75,label='sensor score')
axes[0].legend(fontsize=8);fig.suptitle('Fixed after-jump heading; white contour bounds occupied map cells\nXY markers have different recorded headings; not posterior probability');fig.tight_layout();fig.savefig(a.output/'sensor_landscape.png',dpi=160,bbox_inches='tight');plt.close(fig)
print('Saved',a.output)
