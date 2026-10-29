#!/usr/bin/env python3
"""Read-only snapshot probes using installed native NavFn, with explicit counterfactual copies."""
from pathlib import Path
import argparse,base64,hashlib,json,math,subprocess,zlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
PROBE=ROOT/'experiments/social_navigation/navigation/planner_diagnostics/generated/build/navfn_probe'
def rows(p):return [json.loads(s) for s in p.read_text().splitlines() if s.strip()]
def xy(p):return np.array([p['x'],p['y']])
def rotation(a):return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
def nearest(v,t):return min(v,key=lambda x:abs(x['time']-t))
def metrics(path,grid,origin,res):
    if not len(path):return None
    points=[];weights=[]
    for a,b in zip(path,path[1:]):
        length=float(np.linalg.norm(b-a));n=max(1,math.ceil(length/(res*.25)))
        points.extend(a+(np.arange(n)[:,None]+.5)/n*(b-a));weights.extend([length/n/res]*n)
    ij=np.floor((np.array(points)-origin)/res+.5).astype(int);cost=grid[ij[:,1],ij[:,0]];weights=np.array(weights)
    translated=np.where(cost<253,np.floor(50+.8*cost),np.where(cost==255,253,254))
    return dict(length_m=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),diagnostic_weighted_line_integral=float(weights@translated),mean_native_cost=float(weights@cost/weights.sum()),max_native_cost=int(cost.max()),hard_samples=int(np.isin(cost,[253,254]).sum()),unknown_samples=int((cost==255).sum()))
def snapshot_human_gap(path,humans):
    if len(path)<2:return None
    best=math.inf;who=None
    for hid,h in humans.items():
        a=path[:-1]-h;b=path[1:]-path[:-1];den=np.sum(b*b,axis=1);fraction=np.divide(-np.sum(a*b,axis=1),den,out=np.zeros_like(den),where=den>0);fraction=np.clip(fraction,0,1)
        gap=float(np.linalg.norm(a+fraction[:,None]*b,axis=1).min())-.36-.168062484748657
        if gap<best:best=gap;who=hid
    return dict(min_signed_gap_m=best,closest_human=who,scope='Snapshot geometry only; people frozen in this offline check, not future collision prediction.')

def main():
    plan=json.loads((HERE/'plan.json').read_text());run=ROOT/plan['run']
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-root',type=Path,default=run.parent/'offline_native');parser.add_argument('--report-dir',type=Path,default=HERE);args=parser.parse_args()
    for name in ['results.json','planner_cost_comparison.png']:
        if (args.report_dir/name).exists():raise FileExistsError('Preserve previous analysis: '+str(args.report_dir/name))
    args.report_dir.mkdir(parents=True,exist_ok=True);out=args.output_root;out.mkdir(parents=True,exist_ok=False)
    result=json.loads((run/'episode/result.json').read_text());events={r['event']:r['stamp_ns']/1e9 for r in rows(run/'episode/events.jsonl')};start=events['goal_accepted'];end=events['action_end']
    snapshots=[r for r in rows(run/'episode/native_costmaps.jsonl') if r['topic']=='/global_costmap/costmap_raw'];poses=rows(run/'episode/gazebo_poses.jsonl');tfs=rows(run/'episode/tf.jsonl');plans=rows(run/'episode/plans.jsonl');answers=[];plot_data=[]
    for offset in plan['snapshot_offsets_after_goal_s']:
        cm=min(snapshots,key=lambda x:abs(x['stamp_ns']/1e9-(start+offset)));t=cm['stamp_ns']/1e9
        if not start<=t<=end:raise ValueError('Probe outside active interval')
        if cm['frame']!='map' or abs(cm['origin']['qz'])>1e-8:raise ValueError('Expected map-frame upright costmap')
        raw=zlib.decompress(base64.b64decode(cm['data']));assert hashlib.sha256(raw).hexdigest()==cm['sha256'];grid=np.frombuffer(raw,dtype='u1').reshape(cm['height'],cm['width']);origin=xy(cm['origin']);res=cm['resolution']
        tf=nearest(tfs,t);actual=nearest(poses,t)['poses'];robot=actual['mecanum_robot'];base=tf['map_base'];R=rotation(base['yaw']-robot['yaw']);tr=xy(base)-R@xy(robot);to_map=lambda p:R@xy(p)+tr
        humans={i:to_map(actual[f'common_{i}_proxy']) for i in range(1,7)};gate=(humans[5]+humans[6])/2;source=xy(base);goal=np.array([-20.,0.]);to_cell=lambda p:np.floor((p-origin)/res+.5).astype(int).tolist()
        native=out/f'{offset}s_native.u8';native.write_bytes(raw);flatgrid=np.where(grid<253,0,grid).astype('u1');flat=out/f'{offset}s_flat_soft_only.u8';flat.write_bytes(flatgrid.tobytes())
        def probe(file,a,b):
            v=json.loads(subprocess.check_output([str(PROBE),str(file),str(cm['width']),str(cm['height']),*map(str,to_cell(a)),*map(str,to_cell(b))],text=True))
            v['path_map_m']=(np.array(v.get('path_cells',[]))*res+origin).tolist() if v.get('path_cells') else []
            return v
        free=probe(native,source,goal);leg1=probe(native,source,gate);leg2=probe(native,gate,goal);flatfree=probe(flat,source,goal)
        via_path=np.array(leg1['path_map_m']+leg2['path_map_m']) if leg1['found'] and leg2['found'] else np.empty((0,2))
        paths={'native_free':np.array(free['path_map_m']),'native_via_A':via_path,'flattened_soft_free':np.array(flatfree['path_map_m'])}
        item=dict(offset_requested_s=offset,time_sim_s=t,time_after_goal_s=t-start,costmap_sha256=cm['sha256'],robot_map=source.tolist(),gate_A_map=gate.tolist(),gate_A_native_cost=int(grid[to_cell(gate)[1],to_cell(gate)[0]]),grid_shape=list(grid.shape),metadata=dict(origin=cm['origin'],resolution=res,frame=cm['frame']),native_free=dict(found=free['found'],potential=free.get('potential'),metrics=metrics(paths['native_free'],grid,origin,res)),native_via_A=dict(found=bool(len(via_path)),potential_sum=leg1.get('potential',0)+leg2.get('potential',0) if len(via_path) else None,metrics=metrics(via_path,grid,origin,res)),flattened_soft_free=dict(found=flatfree['found'],metrics_on_original_costmap=metrics(paths['flattened_soft_free'],grid,origin,res)),paths={k:v.tolist() for k,v in paths.items()})
        item['snapshot_human_geometry']={key:snapshot_human_gap(path,humans) for key,path in paths.items()}
        h4=humans[4];rad4=h4-source;distance=float(np.linalg.norm(rad4));occluders=[]
        if distance>.36:
            for hid,h in humans.items():
                v=h-source;d=float(np.linalg.norm(v))
                if hid==4 or d>=distance or d<=.36:continue
                angle=abs(math.atan2(math.sin(math.atan2(v[1],v[0])-math.atan2(rad4[1],rad4[0])),math.cos(math.atan2(v[1],v[0])-math.atan2(rad4[1],rad4[0]))))
                if angle+math.asin(.36/distance)<=math.asin(.36/d):occluders.append(hid)
        coords=np.floor((h4-origin)/res+.5).astype(int);item['H4_observability']=dict(distance_from_robot_m=distance,outside_obstacle_marking_range=distance>3.,full_angular_proxy_occlusion_by=occluders,native_cost_at_gt_projected_center=int(grid[coords[1],coords[0]]),scope='Geometric line-of-sight diagnosis using snapshot proxies; no detector inference or labels fed to controller.')
        item['via_A_potential_ratio']=item['native_via_A']['potential_sum']/free['potential'] if len(via_path) and free.get('potential') else None
        live_plan=nearest(plans,t);item['nearest_recorded_plan_stamp_s']=live_plan['time'];item['nearest_recorded_plan_map']=[[p['x'],p['y']] for p in live_plan['poses']]
        answers.append(item);plot_data.append((item,grid,origin,res,humans,paths))
    pkg=subprocess.check_output(['dpkg-query','-W','-f=${Version}','ros-jazzy-nav2-navfn-planner'],text=True)
    report=dict(schema='native_navfn_review/1',capture_status=result['navigation_status'],actual_goal_reached=result['actual_goal_reached'],active_sim_s=result['active_sim_seconds'],runtime_commit=result['head'],costmap_counts=result['planner_diagnostics']['counts'],installed_navfn_package_version=pkg,navfn_library_sha256=hashlib.sha256(Path('/opt/ros/jazzy/lib/libnav2_navfn_planner.so').read_bytes()).hexdigest(),probe_sha256=hashlib.sha256(PROBE.read_bytes()).hexdigest(),probe_source_sha256=hashlib.sha256((PROBE.parents[2]/'navfn_probe.cc').read_bytes()).hexdigest(),probes=answers,scope='Offline native NavFn on actual uint8 snapshots, not physical execution of constrained or flattened paths. No MPPI rollouts/critic breakdown captured. Exact endpoints, no tolerance fallback; each leg clears its own start cell as NavfnPlanner does. Line integral is diagnostic quadrature on common original costs, not the native eikonal potential.',controller_parameters_changed=False)
    (args.report_dir/'results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    fig,axes=plt.subplots(2,3,figsize=(18,10),layout='constrained');colors={'native_free':'#2673d9','native_via_A':'#bb2679','flattened_soft_free':'#13a56b'}
    for col,(item,grid,origin,res,humans,paths) in enumerate(plot_data):
        ax=axes[0,col];im=ax.imshow(np.ma.masked_equal(grid,255),origin='lower',extent=[origin[0],origin[0]+grid.shape[1]*res,origin[1],origin[1]+grid.shape[0]*res],cmap='YlOrRd',vmin=0,vmax=254,interpolation='nearest')
        for i,p in humans.items():ax.add_patch(Circle(p,.36,fill=False,ec='black'));ax.text(*p,f'H{i}',ha='center',va='center',fontsize=8,clip_on=True)
        for key,path in paths.items():
            if len(path):ax.plot(path[:,0],path[:,1],color=colors[key],lw=1.8,label=key)
        live=np.array(item['nearest_recorded_plan_map']);ax.plot(live[:,0],live[:,1],color='#323a49',ls=':',lw=1.5,label='Recorded live /plan (async)');ax.plot(*item['gate_A_map'],'mx',ms=9,mew=2);ax.plot(*item['robot_map'],'co',ms=5);ax.set_xlim(-28.5,-19.5);ax.set_ylim(-2.2,2.8);ax.set_aspect('equal');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.set_title(f"Actual snapshot +{item['time_after_goal_s']:.2f}s after goal\nGate A native cost={item['gate_A_native_cost']}");ax.legend(fontsize=6,loc='upper left');ax.text(.99,.02,'H4: '+('outside marking range' if item['H4_observability']['outside_obstacle_marking_range'] else 'occluded by '+str(item['H4_observability']['full_angular_proxy_occlusion_by'])),transform=ax.transAxes,ha='right',fontsize=8,bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
        ax=axes[1,col];names=['native_free','native_via_A','flattened_soft_free'];values=[];lengths=[]
        for key in names:
            metric=item[key].get('metrics',item[key].get('metrics_on_original_costmap'));values.append(metric['diagnostic_weighted_line_integral'] if metric else 0);lengths.append(metric['length_m'] if metric else 0)
        ax.bar(range(3),values,color=[colors[k] for k in names]);ax.set_xticks(range(3),['Native free','Via gate A','Soft costs zero']);ax.set_ylabel('Diagnostic integral on ORIGINAL native costs');ax.grid(axis='y',alpha=.2)
        for j,(v,length) in enumerate(zip(values,lengths)):ax.text(j,v,f'{length:.2f}m',ha='center',va='bottom')
        ratio=item['via_A_potential_ratio'];gap=item['snapshot_human_geometry']['native_via_A'];warning=(f"\nVia-A overlaps H{gap['closest_human']} in snapshot" if gap and gap['min_signed_gap_m']<0 else '')
        ax.set_title('Native via-A / free potential: '+(f'{ratio:.3f}' if ratio is not None else 'unreachable')+warning,fontsize=11)
    fig.colorbar(im,ax=list(axes[0]),shrink=.6,label='Recorded native cost 0..254 (255 unknown masked)');fig.suptitle('Native NavFn: an open gap is not the cheapest remaining route\nMagenta/green are OFFLINE queries only; they were never sent to the robot',fontsize=16);fig.savefig(args.report_dir/'planner_cost_comparison.png',dpi=145);plt.close(fig)
    print(json.dumps([{k:v for k,v in x.items() if k not in ['paths','nearest_recorded_plan_map','metadata']} for x in answers],indent=2))
if __name__=='__main__':main()
