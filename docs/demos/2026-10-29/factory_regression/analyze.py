#!/usr/bin/env python3
"""Read-only matched warehouse regression, reusing the established physical-clearance analysis."""
import argparse,base64,importlib.util,json,math,zlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle
from matplotlib.transforms import Affine2D
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('recorded_geometry_ab',HERE.parent/'geometry_ab/compare.py');geo=importlib.util.module_from_spec(spec);spec.loader.exec_module(geo)

def closest_index(times,t):
    i=int(np.searchsorted(times,t));return min([j for j in [i-1,i] if 0<=j<len(times)],key=lambda j:abs(times[j]-t))

def social_metrics(run,data,result):
    gt=sorted(geo.rows(run/'episode/ground_truth.jsonl'),key=lambda x:x['time']);times=np.array([d['time'] for d in data]);entries=[];qualified=0;alignment_skipped=0;considered=0;max_alignment=0.;max_shift=0.;standing_samples=0
    start=times[0];end=times[-1];initial={int(h['id']):geo.xy(h) for h in result['ready']['humans']}
    previous=None;duration=0.;qualified_duration=0.
    for g in gt:
        if not start<=g['time']<=end:continue
        index=closest_index(times,g['time']);dt=abs(times[index]-g['time'])
        if dt>.03:alignment_skipped+=1;continue
        max_alignment=max(max_alignment,dt);poses=data[index]['poses'];robot=geo.xy(poses['mecanum_robot']);humans={int(h['id']):h for h in g['humans']};entered=False;entered_qualified=False
        for group in g['groups']:
            if group['type']!='standing' or len(group['members'])!=3:continue
            standing_samples+=1;members=list(map(int,group['members']));triangle=np.array([geo.xy(poses[f'common_{hid}_proxy']) for hid in members]);matrix=np.column_stack([triangle[1]-triangle[0],triangle[2]-triangle[0]])
            for hid,xy in zip(members,triangle):max_shift=max(max_shift,float(np.linalg.norm(xy-initial[hid])))
            if abs(np.linalg.det(matrix))<1e-9:continue
            considered+=1;bary=np.linalg.solve(matrix,robot-triangle[0]);inside=bool(np.all(bary>=-1e-9) and bary.sum()<=1+1e-9)
            if inside:
                stationary=all(abs(humans[hid]['speed'])<=.05 for hid in members);entered=True;entered_qualified|=stationary;qualified+=stationary
                entries.append(dict(time_sim_s=g['time'],time_after_first_active_s=g['time']-start,group_id=group['id'],members=members,all_speeds_at_most_005=stationary,robot_world=robot.tolist(),triangle_world=triangle.tolist()))
        elapsed=min(.1,max(0.,g['time']-previous)) if previous is not None else 0.;previous=g['time']
        if entered:duration+=elapsed
        if entered_qualified:qualified_duration+=elapsed
    return dict(standing_trio_samples=standing_samples,nondegenerate_triangle_tests=considered,robot_center_entry_samples=len(entries),speed_qualified_entry_samples=int(qualified),any_entry_duration_approx_s=duration,speed_qualified_entry_duration_approx_s=qualified_duration,max_standing_member_shift_from_initial_m=max_shift,max_pose_to_group_metadata_alignment_s=max_alignment,skipped_metadata_alignment=alignment_skipped,entries=entries,scope='Actual proxy triangles plus nearest native group membership/type. Robot center only, not full footprint or validated O-space/social label. Duration approximated at group-message rate; no contact-force evidence.')

def clearance_series(data):
    points=np.array([geo.xy(r['poses']['mecanum_robot']) for r in data]);human=np.array([[geo.xy(r['poses'][f'common_{i}_proxy']) for i in range(1,7)] for r in data]);gap=np.linalg.norm(points[:,None,:]-human,axis=2).min(axis=1)-geo.RADIUS-geo.HUMAN
    return points,human,gap

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--report-dir',type=Path,default=HERE);args=parser.parse_args();out=args.report_dir
    for name in ['results.json','factory_paths.png','factory_costmaps.png']:
        if (out/name).exists():raise FileExistsError(out/name)
    out.mkdir(parents=True,exist_ok=True);plan=json.loads((HERE/'plan.json').read_text());all_runs={};reports=[]
    for case in plan['cases']:
        run=ROOT/plan['runtime']/case['id'];metric,data,points=geo.compute(run);result=json.loads((run/'episode/result.json').read_text());audit=json.loads((run.parent/(case['id']+'_audit.json')).read_text());social=social_metrics(run,data,result);points,humans,gaps=clearance_series(data)
        metric.update(id=case['id'],scenario=case['scenario'],treatment=case['treatment'],capture_integrity=audit['status'],social_diagnostic=social,physical_screen_pass=audit['status']=='PASS' and metric['actual_goal_reached'] and metric['minimum_swept_robot_human_gap_m']>0 and metric['minimum_static_gap_lower_bound_m']>0)
        reports.append(metric);all_runs[case['id']]=(metric,data,points,humans,gaps,run)
    flags=[]
    for scenario in ['standing','walking']:
        baseline=all_runs[scenario+'_base'][0];candidate=all_runs[scenario+'_h5'][0];drop=baseline['minimum_swept_robot_human_gap_m']-candidate['minimum_swept_robot_human_gap_m'];candidate['human_gap_change_vs_baseline_m']=-drop
        if drop>.10:flags.append(dict(scenario=scenario,kind='human_gap_reduced_over_010m',drop_m=drop))
        if candidate['social_diagnostic']['robot_center_entry_samples']>0:flags.append(dict(scenario=scenario,kind='candidate_inside_standing_trio_triangle',samples=candidate['social_diagnostic']['robot_center_entry_samples'],speed_qualified=candidate['social_diagnostic']['speed_qualified_entry_samples']))
        if not candidate['physical_screen_pass']:flags.append(dict(scenario=scenario,kind='candidate_physical_screen_failed'))
    candidates=[m for m in reports if m['treatment']=='horizon5'];report=dict(schema='factory_regression_result/1',candidate_physical_screen='PASS' if all(m['physical_screen_pass'] for m in candidates) else 'FAIL',capture_integrity='PASS' if all(m['capture_integrity']=='PASS' for m in reports) else 'FAIL',review_flags=flags,cases=reports,defaults_promoted=False,training_ready=False,scope='Two scenarios, one attempt per treatment at backend seed45. MPPI RNG not fixed; interacting people react to robot. Descriptive geometry, not native contact truth or social-navigation acceptance.')
    (out/'results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    fig,axes=plt.subplots(2,2,figsize=(16,11),layout='constrained');colors={'baseline':'#316bd3','horizon5':'#11946f'}
    for col,scenario in enumerate(['standing','walking']):
        ax=axes[0,col];base=all_runs[scenario+'_base'];candidate=all_runs[scenario+'_h5']
        for name,lo,hi in geo.static_boxes(base[-1]):ax.add_patch(Rectangle(lo,*(hi-lo),color='#959ca5',alpha=.6))
        for family in [base,candidate]:
            metric,data,points,humans,gaps,run=family;ax.plot(points[:,0],points[:,1],color=colors[metric['treatment']],lw=2,label=f"{metric['treatment']}: {metric['status']}, {metric['active_sim_s']:.1f}s")
            if scenario=='walking':
                for i in range(6):ax.plot(humans[:,i,0],humans[:,i,1],color=colors[metric['treatment']],alpha=.18,lw=.8)
        for i,h in enumerate(candidate[3][0]):ax.add_patch(Circle(h,.36,color='#e1b450',alpha=.7));ax.text(*h,f'P{i+1}',ha='center',va='center',fontsize=8)
        entries=candidate[0]['social_diagnostic']['entries']
        if entries:xy=np.array([e['robot_world'] for e in entries]);ax.plot(xy[:,0],xy[:,1],'rx',ms=4,label='Center inside standing trio triangle')
        ax.plot(-28,0,'bo',ms=5);ax.plot(-20,0,'r*',ms=12);ax.set_xlim(-29,-19);ax.set_ylim(-3.2,3.4);ax.set_aspect('equal');ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.set_title(scenario+' — actual paths; gold circles = INITIAL people');ax.legend(fontsize=8);ax.grid(alpha=.2)
        ax=axes[1,col]
        for family in [base,candidate]:
            metric,data,points,humans,gaps,run=family;times=np.array([p['time'] for p in data]);ax.plot(times-times[0],gaps,color=colors[metric['treatment']],lw=1.4,label=f"{metric['treatment']}, swept min {metric['minimum_swept_robot_human_gap_m']:.3f}m")
        ax.axhline(0,color='#b22243',ls='--',label='Envelope overlap boundary');ax.set_xlabel('Seconds after first active pose');ax.set_ylabel('Sampled minimum robot-human envelope gap (m)');ax.set_title('Instantaneous gap series; exact swept minimum reported separately');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Fresh factory A/B: same six-person initialization, native NavFn + MPPI\nOnly three controller parameters differ; no forced waypoint, defaults unchanged',fontsize=16);fig.savefig(out/'factory_paths.png',dpi=145);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(15,12),layout='constrained')
    for ax,case in zip(axes.flat,plan['cases']):
        metric,data,points,humans,gaps,run=all_runs[case['id']];target=data[int(np.argmin(gaps))]['time'];maps=[r for r in geo.rows(run/'episode/native_costmaps.jsonl') if r['topic']=='/global_costmap/costmap_raw'];cm=min(maps,key=lambda x:abs(x['stamp_ns']/1e9-target));t=cm['stamp_ns']/1e9;tf=geo.nearest(geo.rows(run/'episode/tf.jsonl'),t);actual=geo.nearest(data,t)['poses'];robot=actual['mecanum_robot'];estimated=tf['map_base'];angle=robot['yaw']-estimated['yaw'];R=geo.rotation(angle);tr=geo.xy(robot)-R@geo.xy(estimated);affine=Affine2D().rotate(angle).translate(*tr);grid=np.frombuffer(zlib.decompress(base64.b64decode(cm['data'])),dtype='u1').reshape(cm['height'],cm['width']);o=cm['origin'];res=cm['resolution'];im=ax.imshow(np.ma.masked_equal(grid,255),origin='lower',extent=[o['x'],o['x']+cm['width']*res,o['y'],o['y']+cm['height']*res],transform=affine+ax.transData,cmap='YlOrRd',vmin=0,vmax=254,interpolation='nearest')
        for i in range(1,7):h=geo.xy(actual[f'common_{i}_proxy']);ax.add_patch(Circle(h,.36,fill=False,ec='black'));ax.text(*h,f'P{i}',ha='center',va='center',fontsize=8,clip_on=True)
        past=points[np.array([p['time'] for p in data])<=t];ax.plot(past[:,0],past[:,1],color='#11946f',label='Actual past path');p=geo.nearest(geo.rows(run/'episode/plans.jsonl'),t);path=np.array([[v['x'],v['y']] for v in p['poses']])@R.T+tr;ax.plot(path[:,0],path[:,1],color='#316bd3',lw=1.5,label='Recorded /plan (async)');ax.add_patch(Circle(geo.xy(robot),geo.RADIUS,fill=False,ec='#00adca',lw=2));ax.set_xlim(robot['x']-2.8,robot['x']+2.8);ax.set_ylim(robot['y']-2.8,robot['y']+2.8);ax.set_aspect('equal');ax.set_title(case['id']+f" | native map {t:.2f}s\nSnapshot near minimum human gap (async)");ax.set_xlabel('world x (m), diagnostic alignment');ax.set_ylabel('world y (m)');ax.legend(fontsize=7)
    fig.colorbar(im,ax=list(axes.flat),shrink=.55,label='Recorded native cost0..254; unknown255 masked');fig.suptitle('Recorded global costmaps, estimated TF and actual proxy poses\nGT geometry is an analysis overlay, never a new input to Nav2',fontsize=16);fig.savefig(out/'factory_costmaps.png',dpi=145);plt.close(fig)
    print(json.dumps(dict(candidate_physical_screen=report['candidate_physical_screen'],capture_integrity=report['capture_integrity'],review_flags=flags,cases=[{k:v for k,v in m.items() if k not in ['social_diagnostic','gate_A_crossings']} for m in reports]),indent=2))
if __name__=='__main__':main()
