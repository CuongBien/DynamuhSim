#!/usr/bin/env python3
"""Audit recorded progress-checker recovery events separately from the frozen physical screen."""
from pathlib import Path
import argparse,json,math
import yaml
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
rows=lambda p:[json.loads(s) for s in p.read_text().splitlines()]
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--report-dir',type=Path,default=HERE);args=parser.parse_args();OUT=args.report_dir;OUT.mkdir(parents=True,exist_ok=True)
plan=json.loads((HERE/'plan.json').read_text());cases=[];curves={}
for case in plan['cases']:
    run=ROOT/plan['runtime']/case['id'];ev={v['event']:v['stamp_ns']/1e9 for v in rows(run/'episode/events.jsonl')};start,end=ev['goal_accepted'],ev['action_end'];logs=rows(run/'episode/roslogs.jsonl');fail=[dict(time_sim_s=v['time'],time_after_goal_s=v['time']-start,node=v['node'],text=v['text']) for v in logs if v['node']=='controller_server' and v['text']=='Failed to make progress'];feedback=rows(run/'episode/feedback.jsonl');config=yaml.safe_load((run/'runtime/nav2_runtime.yaml').read_text());text=(run/'episode/launch.log').read_text().splitlines();contexts=[]
    for i,line in enumerate(text):
        if 'Failed to make progress' in line:contexts.append(text[max(0,i-1):i+8])
    cases.append(dict(id=case['id'],progress_failure_count=len(fail),max_action_recoveries=max((v.get('recoveries',0) for v in feedback),default=0),events=fail,seconds_from_last_progress_failure_to_action_end=None if not fail else end-fail[-1]['time_sim_s'],progress_checker=config['controller_server']['ros__parameters']['progress_checker'],native_log_context=contexts))
    poses=[v for v in rows(run/'episode/gazebo_poses.jsonl') if start<=v['time']<=end];tf=[v for v in rows(run/'episode/tf.jsonl') if start<=v['time']<=end]
    curves[case['id']]=(np.array([v['time']-start for v in poses]),np.array([abs(math.atan2(math.sin(v['poses']['mecanum_robot']['yaw']),math.cos(v['poses']['mecanum_robot']['yaw']))) for v in poses]))
receipt=dict(schema='factory_recovery_diagnostic/1',candidate_recovery_free=False if any(v['progress_failure_count'] for v in cases if v['id'].endswith('_h5')) else True,cases=cases,scope='Post-run diagnostic additional to the predeclared physical screen. Both navigation status and physical screen may pass while navigation depends on a recovery. Logs show reset/clear/retry; no individual critic scoring or sole inner-optimizer cause is inferred.',defaults_promoted=False)
for name in ['recovery_diagnostic.json','progress_recovery.png']:
    if (OUT/name).exists():raise FileExistsError(OUT/name)
(OUT/'recovery_diagnostic.json').write_text(json.dumps(receipt,indent=2)+'\n')
fig,axes=plt.subplots(1,2,figsize=(15,5),layout='constrained')
for ax,scenario in zip(axes,['standing','walking']):
    for label,color in [('base','#316bd3'),('h5','#11946f')]:
        t,yaw=curves[scenario+'_'+label];ax.plot(t,yaw,color=color,label=label+' actual yaw error')
    event=next(v for v in cases if v['id']==scenario+'_h5')['events'][0]['time_after_goal_s'];ax.axvline(event,color='#c12c47',ls='--',label='Recorded progress failure/reset');ax.axhline(.25,color='black',ls=':',label='Independent world yaw bound .25rad');ax.set_xlim(20,None);ax.set_ylim(0,1.3);ax.set_title(scenario+f' — reset at {event:.2f}s after goal');ax.set_xlabel('Seconds after accepted goal');ax.set_ylabel('Absolute actual yaw error (rad)');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('Candidate completes after one progress-checker recovery in each factory scenario\nPhysical arrival/clearance PASS is separate from recovery-free navigation',fontsize=15);fig.savefig(OUT/'progress_recovery.png',dpi=145);plt.close(fig);print(json.dumps(dict(candidate_recovery_free=receipt['candidate_recovery_free'],counts={v['id']:v['progress_failure_count'] for v in cases}),indent=2))
