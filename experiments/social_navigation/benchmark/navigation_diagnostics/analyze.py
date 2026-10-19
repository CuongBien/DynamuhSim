#!/usr/bin/env python3
"""Offline analysis of opt-in navigation captures; retains baseline trial identities."""
import argparse,base64,bisect,csv,hashlib,json,math,os,statistics,subprocess,sys,zlib
from collections import Counter
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent
sys.path.insert(0,str(BENCH/'localization_diagnosis'))
from geometry import MapField,stamp,yaw,grid_footprint_cost
sys.path.insert(0,str(SOCIAL/'crowdsimplus_adapter'))
from passage_metrics import PassageCounter
sys.path.insert(0,str(BENCH))
from footprint_geometry import footprint_box
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args()
field=MapField(SOCIAL/'warehouse/warehouse.yaml');boxes=json.loads((SOCIAL/'warehouse/layout.json').read_text())['obstacles']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
reference=BENCH/'localization_diagnosis/generated/reference'
reference_manifest=json.loads((reference/'manifest.json').read_text())
for item in reference_manifest:assert sha(Path(item['file']))==item['sha256'],item['file']
generated=HERE/'generated';generated.mkdir(exist_ok=True)
binary=generated/'native_clusters'
subprocess.run(['g++','-O2','-std=c++17',str(HERE/'native_clusters.cpp'),'-I'+str(reference/'include'),'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib','-lpf_lib','-o',str(binary)],check=True)
env=os.environ.copy();env['LD_LIBRARY_PATH']='/opt/ros/jazzy/lib'+(':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
def pose(d):return dict(x=d['position']['x'],y=d['position']['y'],yaw=yaw(d['orientation']))
def distance(a,b):return math.hypot(a['x']-b['x'],a['y']-b['y'])
def rowsort(rows):return sorted(rows,key=lambda r:r['_time'])
def nearest(rows,t,key='_time',limit=.08):
    # Lists are small except Gazebo/odom. The prebuilt index is attached separately.
    if not rows:return None
    times=[r[key] for r in rows];i=bisect.bisect_left(times,t)
    choices=rows[max(0,i-1):i+1];v=min(choices,key=lambda r:abs(r[key]-t))
    return v if abs(v[key]-t)<=limit else None
def unpack(d):
    assert d['data_encoding']=='zlib_base64_int8'
    raw=zlib.decompress(base64.b64decode(d['data_b64']));assert len(raw)==d['info']['width']*d['info']['height']
    return dict(header=d['header'],info=d['info'],data=np.frombuffer(raw,dtype=np.int8))
def map_background(ax):
    for b in boxes:ax.add_patch(Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],color='#ddd',zorder=0))
    ax.set(xlim=(7,26),ylim=(-4,4),xlabel='world / map x [m]',ylabel='world / map y [m]');ax.set_aspect('equal');ax.grid(alpha=.2)
def plot_publications(ax, rows, values, **kwargs):
    x=[];y=[]
    for i,(r,v) in enumerate(zip(rows,values)):
        if i and r['elapsed']-rows[i-1]['elapsed']>.45:x.append(np.nan);y.append(np.nan)
        x.append(r['elapsed']);y.append(v)
    return ax.plot(x,y,marker='.',markersize=2,**kwargs)
def safe(v):
    if isinstance(v,np.generic):return safe(v.item())
    if isinstance(v,float) and not math.isfinite(v):return None
    if isinstance(v,dict):return {k:safe(x) for k,x in v.items()}
    if isinstance(v,(tuple,list)):return [safe(x) for x in v]
    return v
summaries=[];timelines={};manifests=[];samples={};native_checks=[]
for policy in ['sfm','orca_plus']:
    name='crowdsimplus'+('_sfm' if policy=='sfm' else '')+'_individual_goals_bottleneck_s42'
    result=json.loads((a.report_dir/(name+'.json')).read_text());assert result['protocol_valid']
    runtime=BENCH/result['runtime'];gate=float((runtime/'start_gate').read_text());end=gate+result['sim_seconds']
    groups={k:[] for k in ['gazebo','odom','amcl','scan','particle_cloud','global_plan','global_costmap','costmap','navigation_rosout','navigation_graph']};counts=Counter()
    for line in (runtime/'diagnostics.jsonl').open():
        r=json.loads(line);k=r['kind'];counts[k]+=1
        if k not in groups:continue
        d=r['data']
        if k=='gazebo':t=d['time']
        elif k in ['navigation_rosout','navigation_graph']:t=r['receipt_sim']
        else:t=stamp(d)
        r['_time']=t if t is not None else -math.inf;groups[k].append(r)
    for k in groups:groups[k]=rowsort(groups[k])
    assert all(groups[k] for k in ['particle_cloud','global_plan','global_costmap'])
    active=lambda r:gate<=r['_time']<=end
    clouds=[r for r in groups['particle_cloud'] if active(r)]
    text=[]
    for r in clouds:
        particles=r['data']['particles'];text.append(str(len(particles)))
        for item in particles:
            po=pose(item['pose']);text.append(' '.join(map(str,[po['x'],po['y'],po['yaw'],item['weight']])))
    native=subprocess.run([str(binary)],input='\n'.join(text)+'\n',capture_output=True,text=True,check=True,env=env)
    output=native.stdout.splitlines();assert len(output)==len(clouds)
    cluster_rows=[];cloud_match_errors=[]
    for r,line in zip(clouds,output):
        values=list(map(float,line.split()));n=int(values[0]);assert len(values)==1+4*n
        clusters=[dict(weight=values[1+4*i],x=values[2+4*i],y=values[3+4*i],yaw=values[4+4*i]) for i in range(n)];clusters.sort(key=lambda c:-c['weight'])
        parts=r['data']['particles'];xs=np.array([v['pose']['position']['x'] for v in parts]);ys=np.array([v['pose']['position']['y'] for v in parts]);weights=np.array([v['weight'] for v in parts]);total=float(weights.sum());assert abs(total-1)<1e-6
        occupied=field.occupied_at(xs,ys)
        matching=nearest(groups['amcl'],r['receipt_monotonic'],key='receipt_monotonic',limit=.15)
        row=dict(elapsed=r['_time']-gate,time=r['_time'],receipt_sim=r['receipt_sim'],particles=len(parts),weight_sum=total,clusters=clusters,
            occupied_weight=float(weights[occupied].sum()/total),north_machine_weight=float(weights[(xs>=12)&(xs<=20)&(ys>.9)&(ys<2.4)].sum()/total),
            cloud_header_minus_receipt_s=r['_time']-r['receipt_sim'] if r['receipt_sim'] is not None else None)
        if matching:
            ap=pose(matching['data']['pose']['pose']);err=distance(clusters[0],ap)
            row.update(amcl=ap,amcl_header_offset_s=matching['_time']-r['_time'],amcl_receipt_wall_offset_s=matching['receipt_monotonic']-r['receipt_monotonic'],max_cluster_amcl_error_m=err)
            cloud_match_errors.append(err)
        cluster_rows.append(row)
    dense=[]
    for r in groups['gazebo']:
        if not active(r):continue
        poses=r['data']['poses'];ro=poses.get('mecanum_robot')
        if ro is None or len(poses)!=4:continue
        humans=[dict(id=int(k.split('_')[1]),**v) for k,v in poses.items() if k!='mecanum_robot']
        def clearance(h,hx=.12,hy=.10):
            dx,dy=h['x']-ro['x'],h['y']-ro['y'];c,si=math.cos(ro['yaw']),math.sin(ro['yaw'])
            qx,qy=abs(c*dx+si*dy)-hx,abs(-si*dx+c*dy)-hy
            return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
        dense.append(dict(time=r['_time'],robot=ro,humans=humans,robot_human=min(clearance(h) for h in humans),padded=min(clearance(h,.13,.11) for h in humans),
            human_pair=min(math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(humans) for j in humans[i+1:]),
            robot_machine=min(footprint_box(ro,box) for box in boxes)))
    assert dense
    passages=PassageCounter([dict(id=0,**dense[0]['robot']),*dense[0]['humans']],result['scene']['passage'])
    for r in dense:passages.update(r['time']-gate,[dict(id=0,**r['robot']),*r['humans']])
    amcl=[]
    for r in groups['amcl']:
        if not active(r):continue
        truth=nearest(groups['gazebo'],r['_time']);od=nearest(groups['odom'],r['_time'])
        if not truth or not od:continue
        ap=pose(r['data']['pose']['pose']);ro=truth['data']['poses']['mecanum_robot'];op=pose(od['data']['pose']['pose']);op['x']+=8
        row=dict(elapsed=r['_time']-gate,time=r['_time'],actual=ro,amcl=ap,odom_world=op,error_m=distance(ap,ro),odom_error_m=distance(op,ro),occupied=bool(field.occupied_at(ap['x'],ap['y'])),covariance_yy=r['data']['pose']['covariance'][7])
        grid=nearest(groups['global_costmap'],r['_time'],limit=1.2)
        if grid:
            gd=unpack(grid['data']);assert gd['header']['frame_id']=='map'
            row['global_costmap_time_offset_s']=grid['_time']-r['_time'];row['global_footprint']=grid_footprint_cost(gd,ap)
        local=nearest(groups['costmap'],r['_time'],limit=.35)
        if local:
            lo=nearest(groups['odom'],local['_time'])
            if lo:row['local_footprint']=grid_footprint_cost(local['data'],pose(lo['data']['pose']['pose']))
        amcl.append(row)
    plans=[]
    for r in groups['global_plan']:
        if not active(r):continue
        assert r['data']['header']['frame_id']=='map'
        points=[v['pose']['position'] for v in r['data']['poses']];core=[v for v in points if 12<=v['x']<=20]
        plans.append(dict(elapsed=r['_time']-gate,time=r['_time'],points=[[v['x'],v['y']] for v in points],
            length_m=sum(distance(u,v) for u,v in zip(points,points[1:])),leaves_channel_core=any(abs(v['y'])>.9 for v in core),core_points=len(core)))
    jump_i=max(range(1,len(amcl)),key=lambda i:distance(amcl[i]['amcl'],amcl[i-1]['amcl']))
    jump_before,jump_after=amcl[jump_i-1:jump_i+1]
    odom_errors=[];stop=None;firststop=None;reverse=[]
    for r in groups['odom']:
        if not active(r):continue
        v=r['data']['twist']['twist'];speed=math.hypot(v['linear']['x'],v['linear']['y']);ro=nearest(groups['gazebo'],r['_time'])
        po=pose(r['data']['pose']['pose']);po['x']+=8
        if ro:odom_errors.append(distance(po,ro['data']['poses']['mecanum_robot']))
        if speed<.02:
            if stop is None:stop=r['_time']
            if r['_time']-stop>=1 and stop-gate>2 and firststop is None:firststop=stop-gate
        else:stop=None
    rosout=[dict(elapsed=r['_time']-gate,level=r['data']['level'],node=r['data']['name'],message=r['data']['msg']) for r in groups['navigation_rosout'] if active(r)]
    graphs=[r['data'] for r in groups['navigation_graph']]
    global_pubs=[len(g['publishers'].get('/plan',[])) for g in graphs];amcl_pubs=[len(g['publishers'].get('/amcl_pose',[])) for g in graphs]
    baseline=json.loads((BENCH/'bottleneck/reports/run01'/(name+'.json')).read_text())
    samekeys=['map_sha256','nav_params_sha256','runtime_nav_params_sha256','scene_sha256','crowdsim_image_id']
    assert all(result[k]==baseline[k] for k in samekeys)
    scanlags=[r['receipt_sim']-r['_time'] for r in groups['scan'] if active(r) and r['receipt_sim'] is not None]
    summary=dict(name=name,policy=policy,status=result['status'],sim_seconds=result['sim_seconds'],goal_error_world_m=result['goal_error_world_m'],amcl_error_max_m=max(r['error_m'] for r in amcl),
        odom_error_max_m=max(odom_errors),largest_amcl_step_m=distance(jump_before['amcl'],jump_after['amcl']),largest_jump_elapsed_s=jump_after['elapsed'],actual_step_during_largest_jump_m=distance(jump_before['actual'],jump_after['actual']),
        first_persistent_translation_stop_s=firststop,first_estimated_center_occupied_s=next((r['elapsed'] for r in amcl if r['occupied']),None),
        max_particle_occupied_weight=max(r['occupied_weight'] for r in cluster_rows),max_native_cluster_count=max(len(r['clusters']) for r in cluster_rows),
        particle_cloud_samples=len(clouds),native_cluster_matches=len(cloud_match_errors),native_cluster_match_error_max_m=max(cloud_match_errors),native_cluster_match_error_median_m=statistics.median(cloud_match_errors),
        global_plan_samples=len(plans),global_plan_leaves_channel_count=sum(r['leaves_channel_core'] for r in plans),global_costmap_samples=sum(active(r) for r in groups['global_costmap']),
        scan_receipt_lag_max_s=max(scanlags),plan_publishers_at_snapshots=global_pubs,amcl_publishers_at_snapshots=amcl_pubs,
        channel_traversals=passages.counts(),bypass_events=sum(not e['through_channel'] for e in passages.events),dense_samples=len(dense),dense_gap_max_s=max(v['time']-u['time'] for u,v in zip(dense,dense[1:])),
        min_robot_human_footprint_m=min(r['robot_human'] for r in dense),min_robot_human_padded_m=min(r['padded'] for r in dense),min_human_pair_m=min(r['human_pair'] for r in dense),min_robot_machine_footprint_m=min(r['robot_machine'] for r in dense),
        capture_callback_wall_seconds=result['navigation_capture']['callbacks_wall_seconds'],capture_callback_wall_max_seconds=result['navigation_capture']['callback_wall_max_seconds'],observed_rtf=result['observed_rtf'],
        config_scene_map_image_identical_to_baseline=True,counts=dict(counts))
    summaries.append(summary);timelines[name]=dict(amcl=amcl,particle_cloud=cluster_rows,global_plans=plans,rosout=rosout,jump_before=jump_before,jump_after=jump_after,graphs=graphs)
    selected_cloud=max(cluster_rows,key=lambda r:r['occupied_weight']);cr=nearest(clouds,selected_cloud['time']);samples[name]=dict(cloud=cr,summary=selected_cloud)
    manifests.append(dict(name=name,runtime=str(runtime.relative_to(BENCH)),start_gate=gate,end=end,raw_sha256={f:sha(runtime/f) for f in ['diagnostics.jsonl','start_gate','scene.json','nav2_scenario.yaml','crowdsim_metadata.json']},baseline_comparison={k:result[k] for k in samekeys}))
    native_checks.append(dict(name=name,clouds=len(clouds),matched=len(cloud_match_errors),error_max_m=max(cloud_match_errors)))
    print(json.dumps(summary),flush=True)
# Actual trajectory with all independently published global plans.
fig,axes=plt.subplots(2,1,figsize=(13,8))
for ax,su in zip(axes,summaries):
    trace=timelines[su['name']];map_background(ax)
    for pl in trace['global_plans']:
        po=np.array(pl['points']);ax.plot(po[:,0],po[:,1],c='#d99025' if pl['leaves_channel_core'] else '#7aad90',alpha=.15,lw=1)
    ar=trace['amcl'];ax.plot([r['actual']['x'] for r in ar],[r['actual']['y'] for r in ar],label='actual Gazebo',c='#2674b5');ax.plot([r['amcl']['x'] for r in ar],[r['amcl']['y'] for r in ar],label='AMCL publications',c='#c74c3b',marker='.',ms=3)
    ax.set_title(f"{su['policy']} seed42 | {su['status']} | faint lines: published global plans");ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(a.report_dir/'plans_and_trajectory.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(4,2,figsize=(13,11),sharex='col')
for col,su in enumerate(summaries):
    tr=timelines[su['name']];ar=tr['amcl'];pc=tr['particle_cloud'];x=[r['elapsed'] for r in ar]
    plot_publications(axes[0,col],ar,[r['error_m'] for r in ar],label='AMCL error');plot_publications(axes[0,col],ar,[r['odom_error_m'] for r in ar],label='spawn+odom error');axes[0,col].set_title(su['policy']+' | '+su['status']);axes[0,col].set_ylabel('position error [m]');axes[0,col].legend(fontsize=7)
    axes[1,col].plot([r['elapsed'] for r in pc],[r['occupied_weight'] for r in pc],label='occupied static cells');axes[1,col].plot([r['elapsed'] for r in pc],[r['north_machine_weight'] for r in pc],label='north machine interior');axes[1,col].set_ylabel('recorded particle weight');axes[1,col].set_ylim(-.03,1.03);axes[1,col].legend(fontsize=7)
    axes[2,col].plot([r['elapsed'] for r in pc],[len(r['clusters']) for r in pc]);axes[2,col].set_ylabel('native cluster count')
    for key,label in [('global_footprint','global grid at AMCL'),('local_footprint','local grid at odom')]:
        plot_publications(axes[3,col],ar,[r.get(key,{}).get('footprint_occupancy_max',np.nan) for r in ar],label=label)
    axes[3,col].set(xlabel='simulation elapsed [s]',ylabel='footprint grid max [0..100]');axes[3,col].legend(fontsize=7)
    for ax in axes[:,col]:ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(a.report_dir/'navigation_timeline.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(15,8),layout='constrained')
for row,su in enumerate(summaries):
    tr=timelines[su['name']];sample=samples[su['name']];r=sample['cloud'];cr=sample['summary'];parts=r['data']['particles'];xs=[v['pose']['position']['x'] for v in parts];ys=[v['pose']['position']['y'] for v in parts]
    for ax in axes[row,:]:map_background(ax)
    axes[row,0].scatter(xs,ys,s=7,c='#9868c5',alpha=.4,label='recorded particles');axes[row,0].set_title(f"{su['policy']} | max occupied weight at {cr['elapsed']:.3f}s: {cr['occupied_weight']:.3f}")
    if 'amcl' in cr:axes[row,0].scatter([cr['amcl']['x']],[cr['amcl']['y']],marker='x',c='red',s=60,label='nearest AMCL')
    axes[row,0].legend(fontsize=7)
    # A real published global grid nearest the largest AMCL publication step.
    result=json.loads((a.report_dir/(su['name']+'.json')).read_text());raw=BENCH/result['runtime']/'diagnostics.jsonl';target=tr['jump_after']['time'];best=None
    for line in raw.open():
        gr=json.loads(line)
        if gr['kind']=='global_costmap' and (best is None or abs(stamp(gr['data'])-target)<abs(stamp(best['data'])-target)):best=gr
    gd=unpack(best['data']);info=gd['info'];v=gd['data'].reshape(info['height'],info['width']);o=info['origin']['position'];extent=[o['x'],o['x']+info['width']*info['resolution'],o['y'],o['y']+info['height']*info['resolution']]
    grid_image=axes[row,1].imshow(np.ma.masked_where(v<=0,v),origin='lower',extent=extent,vmin=1,vmax=100,cmap='magma_r',alpha=.6,zorder=1)
    for po,label,c in [(tr['jump_after']['actual'],'actual','#2674b5'),(tr['jump_after']['amcl'],'AMCL','#c74c3b')]:axes[row,1].scatter([po['x']],[po['y']],c=c,s=35,label=label)
    axes[row,1].set_title(f"{su['policy']} | global grid nearest largest AMCL step | dt {stamp(gd)-target:+.3f}s");axes[row,1].legend(fontsize=7)
fig.colorbar(grid_image,ax=axes[:,1].tolist(),shrink=.7,label='global OccupancyGrid [1..100]');fig.savefig(a.report_dir/'particles_and_global_grid.png',dpi=160);plt.close(fig)
manifest=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),trials=manifests,native_library='/opt/ros/jazzy/lib/libpf_lib.so',native_library_sha256=sha(Path('/opt/ros/jazzy/lib/libpf_lib.so')),native_harness_sha256=sha(HERE/'native_clusters.cpp'),native_binary_sha256=sha(binary),reference_manifest=reference_manifest,
    particle_method='Recorded post-resample ParticleCloud weights; re-cluster with installed pf_kdtree and pf_cluster_stats, no original hidden filter replay. Cloud header is AMCL now(), AMCL pose header is scan stamp; match receipts within 0.15 wall s and retain offsets.',
    grid_method='Exact int8 lossless decompression; nearest publication within 1.2 sim s for global grid and 0.35 for local. Padded footprint samples are diagnostic, not contact force or planner service results.',
    limitations='Two diagnostic reruns only. Old failing trials retained; stochastic process not fully seeded. Passive capture adds CPU/IO load. No tuning or claimed fix.')
for filename,value in [('summary.json',summaries),('timeline.json',timelines),('manifest.json',manifest),('native_cluster_check.json',native_checks)]: (a.report_dir/filename).write_text(json.dumps(safe(value),indent=2,allow_nan=False))
with (a.report_dir/'results.csv').open('w',newline='') as f:
    keys=[k for k in summaries[0] if not isinstance(summaries[0][k],(dict,list))];writer=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore',lineterminator='\n');writer.writeheader();writer.writerows(summaries)
