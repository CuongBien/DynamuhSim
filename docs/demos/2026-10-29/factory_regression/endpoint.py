#!/usr/bin/env python3
"""Describe final-position/orientation convergence; does not attribute individual MPPI critic scores."""
from pathlib import Path
import argparse,json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
rows=lambda p:[json.loads(s) for s in p.read_text().splitlines()]
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--report-dir',type=Path,default=HERE);args=parser.parse_args();OUT=args.report_dir;OUT.mkdir(parents=True,exist_ok=True)
plan=json.loads((HERE/'plan.json').read_text());result=[];streams={}
for case in plan['cases']:
    run=ROOT/plan['runtime']/case['id'];events={e['event']:e['stamp_ns']/1e9 for e in rows(run/'episode/events.jsonl')};start,end=events['goal_accepted'],events['action_end'];world=[r for r in rows(run/'episode/gazebo_poses.jsonl') if start<=r['time']<=end];estimated=[r for r in rows(run/'episode/tf.jsonl') if start<=r['time']<=end]
    def errors(data,key):
        t=np.array([r['time']-start for r in data]);p=[r[key] if key=='map_base' else r['poses']['mecanum_robot'] for r in data];xy=np.array([math.hypot(r['x']+20,r['y']) for r in p]);yaw=np.array([abs(math.atan2(math.sin(r['yaw']),math.cos(r['yaw']))) for r in p]);return t,xy,yaw
    t,xy,yaw=errors(world,'world');te,xye,yawe=errors(estimated,'map_base');duration=end-start
    def entry(times,mask):
        i=np.flatnonzero(mask);return float(times[i[0]]) if len(i) else None
    first=entry(t,xy<=.2);both=entry(t,(xy<=.2)&(yaw<=.25));ee=entry(te,xye<=.05);eb=entry(te,(xye<=.05)&(yawe<=.15));outside=np.flatnonzero(xy>.2);last_outside=float(t[outside[-1]]) if len(outside) else None
    result.append(dict(id=case['id'],active_sim_s=duration,first_world_xy_within_020_s=first,first_world_xy_yaw_within_policy_s=both,time_after_first_world_xy_entry_s=None if first is None else duration-first,last_world_xy_outside_020_s=last_outside,first_estimated_xy_within_005_s=ee,first_estimated_xy_yaw_within_live_tolerances_s=eb,scope='Descriptive phase timing from recorded scene/TF, may include re-entries and joint position/yaw refinement; not time spent solely rotating or a captured critic-score explanation.'))
    streams[case['id']]=(t,xy,yaw,te,xye,yawe)
for name in ['endpoint_analysis.json','endpoint_convergence.png']:
    if (OUT/name).exists():raise FileExistsError(OUT/name)
(OUT/'endpoint_analysis.json').write_text(json.dumps(dict(schema='factory_endpoint_analysis/1',cases=result),indent=2)+'\n')
fig,axes=plt.subplots(2,2,figsize=(15,8),layout='constrained');colors={'base':'#316bd3','h5':'#11946f'}
for col,scenario in enumerate(['standing','walking']):
    for label in ['base','h5']:
        t,xy,yaw,te,xye,yawe=streams[scenario+'_'+label];color=colors[label];axes[0,col].plot(t,xy,color=color,label=label+' actual world');axes[0,col].plot(te,xye,color=color,ls='--',alpha=.7,label=label+' estimated map TF');axes[1,col].plot(t,yaw,color=color,label=label+' actual world');axes[1,col].plot(te,yawe,color=color,ls='--',alpha=.7,label=label+' estimated map TF')
    for ax in axes[:,col]:ax.set_xlim(20,None);ax.set_xlabel('Seconds after accepted goal');ax.grid(alpha=.2);ax.legend(fontsize=7)
    axes[0,col].set_ylim(0,.6);axes[0,col].axhline(.2,color='black',ls=':',lw=.8);axes[0,col].axhline(.05,color='#a33',ls=':',lw=.8);axes[0,col].set_title(scenario+' — near-goal XY (view clipped to last0.6m)');axes[0,col].set_ylabel('XY error to goal (m)');axes[1,col].set_ylim(0,1.1);axes[1,col].axhline(.25,color='black',ls=':',lw=.8);axes[1,col].axhline(.15,color='#a33',ls=':',lw=.8);axes[1,col].set_title('Near-goal yaw convergence');axes[1,col].set_ylabel('Absolute wrapped yaw error (rad)')
fig.suptitle('Endpoint refinement contributes to the longer candidate completion time\nWorld-policy thresholds0.20m/0.25rad; live estimated thresholds0.05m/0.15rad unchanged',fontsize=15);fig.savefig(OUT/'endpoint_convergence.png',dpi=145);plt.close(fig);print(json.dumps(result,indent=2))
