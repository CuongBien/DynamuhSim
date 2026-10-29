"""Read-only analysis of all predeclared cells and bounded human counterfactuals."""
import argparse,importlib.util,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('recorded_geometry',HERE.parent/'geometry_ab/compare.py');geo=importlib.util.module_from_spec(s);s.loader.exec_module(geo)
s=importlib.util.spec_from_file_location('factory_social',HERE.parent/'factory_regression/analyze.py');social=importlib.util.module_from_spec(s);s.loader.exec_module(social)
rows=lambda p:[json.loads(x) for x in p.read_text().splitlines()]

def human_metrics(data):
 humans=np.array([[[h[k] for k in ['x','y','yaw','vx','vy']] for h in v['humans']] for v in data]);t=np.array([v['time'] for v in data]);yaw=np.unwrap(humans[:,:,2],axis=0);turn=np.abs(np.diff(yaw,axis=0)).sum(axis=0);length=np.linalg.norm(np.diff(humans[:,:,:2],axis=0),axis=2).sum(axis=0)
 speed=np.linalg.norm(humans[:,:,3:5],axis=2);heading=np.arctan2(humans[:,:,4],humans[:,:,3]);error=np.abs(np.arctan2(np.sin(humans[:,:,2]-heading),np.cos(humans[:,:,2]-heading)));moving=speed>.05
 radii=[];max_width=0.;min_gap=math.inf
 for i,row in enumerate(data):
  for group in row['groups']:
   if group['type']!='moving':continue
   ids=[int(h)-1 for h in group['members']];p=humans[i,ids,:2];radius=np.linalg.norm(p-p.mean(axis=0),axis=1).max();radii.append(radius)
  for a in range(6):
   for b in range(a+1,6):min_gap=min(min_gap,float(np.linalg.norm(humans[i,a,:2]-humans[i,b,:2]))-.72)
 return dict(samples=len(data),time_span_s=float(t[-1]-t[0]),yaw_total_rad=turn.tolist(),yaw_total_turns=(turn/(2*np.pi)).tolist(),path_m=length.tolist(),yaw_rad_per_m=np.divide(turn,length,out=np.zeros_like(turn),where=length>1e-9).tolist(),moving_heading_error_median_deg=float(np.degrees(np.median(error[moving]))) if moving.any() else None,moving_backwards_fraction=float(np.mean(error[moving]>np.pi/2)) if moving.any() else None,moving_group_member_radius_max_m=max(radii,default=None),moving_group_member_radius_median_m=float(np.median(radii)) if radii else None,minimum_sampled_human_human_gap_m=min_gap)

def native_turns(data):
 yaw=np.array([[v['poses'][f'common_{i}_proxy']['yaw'] for i in range(1,7)] for v in data]);xy=np.array([[[v['poses'][f'common_{i}_proxy'][k] for k in ['x','y']] for i in range(1,7)] for v in data]);return dict(duration_s=data[-1]['time']-data[0]['time'],yaw_total_turns=(np.abs(np.diff(np.unwrap(yaw,axis=0),axis=0)).sum(axis=0)/(2*np.pi)).tolist(),path_m=np.linalg.norm(np.diff(xy,axis=0),axis=2).sum(axis=0).tolist())

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--offline-only',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];offline={name:rows(raw/'offline_v1'/f'{name}.jsonl') for name in ['legacy','group_orca_v1']};summary=dict(offline={name:human_metrics(v) for name,v in offline.items()},offline_reproduction=json.loads((raw/'offline_v1/metrics.json').read_text()),cases=[])
 fig,axes=plt.subplots(2,2,figsize=(13,9),layout='constrained');colors=plt.get_cmap('tab10').colors
 for col,(name,data) in enumerate(offline.items()):
  t=np.array([v['time'] for v in data]);points=np.array([[[h['x'],h['y'],h['yaw']] for h in v['humans']] for v in data]);yaw=np.unwrap(points[:,:,2],axis=0)
  for i in range(6):
   axes[0,col].plot(points[:,i,0],points[:,i,1],color=colors[i],label=f'P{i+1}');axes[0,col].scatter(*points[0,i,:2],c=[colors[i]],marker='o',s=15);axes[0,col].scatter(*points[-1,i,:2],c=[colors[i]],marker='x',s=35)
   sel=np.arange(0,len(data),20);axes[0,col].quiver(points[sel,i,0],points[sel,i,1],np.cos(points[sel,i,2]),np.sin(points[sel,i,2]),color=colors[i],scale_units='xy',scale=3,width=.003)
   axes[1,col].plot(t,np.cumsum(np.r_[0,np.abs(np.diff(yaw[:,i]))])/(2*np.pi),color=colors[i])
  axes[0,col].set(xlim=(-29,-14),ylim=(-4,4),aspect='equal',title=name+' — first12s actual simulated XY/headings',xlabel='World X (m)',ylabel='World Y (m)');axes[0,col].legend(ncol=3,fontsize=8);axes[1,col].set(xlabel='Trace seconds',ylabel='Cumulative absolute yaw / full turns',ylim=(0,2.3));axes[1,col].grid(alpha=.2)
 fig.suptitle('Same120 measured robot inputs; positions/headings from SocNavGym execution\nCircle=start, cross=end, arrows=heading every2s. This is a 2D trace, not a Gazebo camera image.');fig.savefig(a.output/'human_offline.png',dpi=145);plt.close(fig)
 if not a.offline_only:
  curves={}
  for case in plan['robot_cases']+plan['human_cases']:
   run=raw/case['id'];result=json.loads((run/'episode/result.json').read_text());metric,data,pts=geo.compute(run);events={e['event']:e['stamp_ns']/1e9 for e in rows(run/'episode/events.jsonl')};start,end=events['goal_accepted'],events['action_end'];logs=rows(run/'episode/roslogs.jsonl');feedback=rows(run/'episode/feedback.jsonl');worker=[v for v in rows(run/'runtime/worker_trajectory.jsonl') if v.get('active_phase') and v['time']<=end];metric.update(id=case['id'],human_motion_profile=case['human_motion_profile'],progress_failure_count=sum(v['node']=='controller_server' and v['text']=='Failed to make progress' for v in logs),max_action_recoveries=max((v.get('recoveries',0) for v in feedback),default=0),human_first12=human_metrics(worker[:120]),human_full=human_metrics(worker));audit=json.loads((raw/(case['id']+'_audit.json')).read_text());human_xy=np.array([[[v['poses'][f'common_{i}_proxy'][axis] for axis in ['x','y']] for i in range(1,7)] for v in data]);human_static=math.inf
   for box,lo,hi in geo.static_boxes(run):
    distances=np.linalg.norm(np.maximum(np.maximum(lo-human_xy,human_xy-hi),0),axis=2);lower=np.minimum(distances[:-1],distances[1:])-.5*np.linalg.norm(np.diff(human_xy,axis=0),axis=2)-.36;human_static=min(human_static,float(lower.min()))
   metric['minimum_human_static_gap_lower_bound_m']=human_static;metric['native_human_full']=native_turns(data);metric['native_human_first12']=native_turns([v for v in data if v['time']<=start+12]);metric['social_diagnostic']=social.social_metrics(run,data,result);metric['capture_integrity']=audit['status'];metric['physical_pass']=audit['status']=='PASS' and metric['actual_goal_reached'] and metric['minimum_swept_robot_human_gap_m']>0 and metric['minimum_static_gap_lower_bound_m']>0;metric['recovery_free_pass']=metric['physical_pass'] and metric['progress_failure_count']==0 and metric['max_action_recoveries']==0;summary['cases'].append(metric);t=np.array([v['time']-start for v in data]);xy=np.linalg.norm(pts-np.array([-20.,0]),axis=1);yaw=np.abs(np.arctan2(np.sin([v['poses']['mecanum_robot']['yaw'] for v in data]),np.cos([v['poses']['mecanum_robot']['yaw'] for v in data])));curves[case['id']]=(t,xy,yaw)
  fig,ax=plt.subplots(2,2,figsize=(13,8),layout='constrained')
  for col,scenario in enumerate(['standing','walking']):
   for suffix in ['resample','group']:
    t,xy,yaw=curves[scenario+'_'+suffix];ax[0,col].plot(t,xy,label=suffix);ax[1,col].plot(t,yaw,label=suffix)
   for v in ax[:,col]:v.set_xlim(20,None);v.set_xlabel('Seconds after accepted goal');v.grid(alpha=.2);v.legend()
   ax[0,col].set(ylim=(0,.6),title=scenario+' endpoint actual XY',ylabel='Error (m)');ax[0,col].axhline(.2,color='black',ls=':');ax[1,col].set(ylim=(0,1.5),title='Actual yaw error',ylabel='Error (rad)');ax[1,col].axhline(.25,color='black',ls=':')
  fig.savefig(a.output/'endpoint.png',dpi=145);plt.close(fig)
  fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
  for ax,name in zip(axes,['walking_resample','walking_group']):
   run=raw/name;events={e['event']:e['stamp_ns']/1e9 for e in rows(run/'episode/events.jsonl')};data=[v for v in rows(run/'episode/gazebo_poses.jsonl') if events['goal_accepted']<=v['time']<=events['action_end']];t=np.array([v['time']-events['goal_accepted'] for v in data]);yaw=np.unwrap(np.array([[v['poses'][f'common_{i}_proxy']['yaw'] for i in range(1,7)] for v in data]),axis=0);turn=np.cumsum(np.vstack([np.zeros((1,6)),np.abs(np.diff(yaw,axis=0))]),axis=0)/(2*np.pi)
   for i in range(6):ax.plot(t,turn[:,i],label=f'P{i+1}')
   ax.set(title=name+' — native Gazebo pose yaw',xlabel='Seconds after accepted goal',ylabel='Cumulative absolute yaw / full turns',ylim=(0,6));ax.grid(alpha=.2);ax.legend(ncol=3,fontsize=8)
  fig.savefig(a.output/'native_turns.png',dpi=145);plt.close(fig)
 (a.output/'metrics.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n');print(json.dumps({c['id']:{k:c[k] for k in ['status','active_sim_s','capture_integrity','progress_failure_count','max_action_recoveries','physical_pass','minimum_swept_robot_human_gap_m']} for c in summary['cases']},indent=2))
if __name__=='__main__':main()
