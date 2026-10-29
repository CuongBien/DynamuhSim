"""Native yaw/safety and group-cohesion screens; preserve all outcomes."""
import argparse,importlib.util,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('ablation_enc',HERE.parent/'encounter_regression/analysis.py');enc=importlib.util.module_from_spec(s);s.loader.exec_module(enc)
rows=lambda p:[json.loads(s) for s in p.read_text().splitlines() if s.strip()]

def native_cohesion(run,data):
 stamps=np.array([v['time'] for v in data]);radii=[];series=[];skipped=0
 for row in rows(run/'episode/ground_truth.jsonl'):
  t=row['time']
  if not stamps[0]<=t<=stamps[-1]:continue
  ix=int(np.searchsorted(stamps,t));ix=min([i for i in [ix-1,ix] if 0<=i<len(stamps)],key=lambda i:abs(stamps[i]-t))
  if abs(stamps[ix]-t)>.03:skipped+=1;continue
  local=[]
  for g in row['groups']:
   if g['type']!='moving' or len(g['members'])<2:continue
   p=np.array([[data[ix]['poses'][f'common_{hid}_proxy'][key] for key in ['x','y']] for hid in g['members']]);radius=float(np.linalg.norm(p-p.mean(axis=0),axis=1).max());radii.append(radius);local.append(radius)
  if local:series.append((t-stamps[0],max(local)))
 return dict(max_member_centroid_radius_m=max(radii,default=None),median_member_centroid_radius_m=float(np.median(radii)) if radii else None,group_samples=len(radii),metadata_alignment_skips=skipped,scope='Actual native world poses; nearest native group membership/type metadata within.03s; radius excludes body radius.'),np.array(series)

def rate(m):
 v=m['native_human_full'];return float(np.mean(v['yaw_total_turns']))/v['duration_s']

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];reference=ROOT/plan['reference_root'];report=dict(schema='group_goal_ablation_result/1',cases=[],comparisons=[],failed_attempts=[],defaults_promoted=False,naturalness_accepted=False,training_ready=False);curves={}
 for case in plan['cases']:
  try:
   run=raw/case['id'];metric,data,points=enc.analyze_social(run,case);cohesion,cs=native_cohesion(run,data);metric['native_cohesion']=cohesion;audit=json.loads((raw/(case['id']+'_audit.json')).read_text());metric['goal_proposal_audit']=audit['checks']['shared_group_goal_proposals'];metric['goal_proposal_counts']=audit['metrics']['shared_group_goal_proposals'];report['cases'].append(metric);old_case=dict(case,id=case['reference_id'],human_motion_profile='group_orca_v1');before,old_data,_=enc.analyze_social(reference/case['reference_id'],old_case);old_cohesion,old_cs=native_cohesion(reference/case['reference_id'],old_data);change=dict(id=case['id'],reference_id=case['reference_id'],mean_yaw_rate_before_turns_per_s=rate(before),mean_yaw_rate_after_turns_per_s=rate(metric),yaw_rate_reduction_fraction=1-rate(metric)/rate(before),native_cohesion_before=old_cohesion,native_cohesion_after=cohesion,cohesion_radius_change_m=cohesion['max_member_centroid_radius_m']-old_cohesion['max_member_centroid_radius_m'],backward_fraction_change=metric['worker']['worker_full']['moving_backwards_fraction']-before['worker']['worker_full']['moving_backwards_fraction'],near_duration_change_s=metric['proximity']['cumulative_duration_s']-before['proximity']['cumulative_duration_s'],active_sim_change_s=metric['active_sim_s']-before['active_sim_s']);change['yaw_screen_pass']=change['yaw_rate_reduction_fraction']>=.5;change['cohesion_screen_pass']=change['cohesion_radius_change_m']<=.25;change['backward_screen_pass']=change['backward_fraction_change']<=.05;change['motion_screen_pass']=change['yaw_screen_pass'] and change['cohesion_screen_pass'] and change['backward_screen_pass'];change['candidate_screen_pass']=metric['encounter_screen_pass'] and change['motion_screen_pass'];report['comparisons'].append(change);curves[case['id']]=(case,metric,data,cs,before,old_data,old_cs)
  except Exception as e:report['failed_attempts'].append(dict(id=case['id'],error=type(e).__name__+': '+str(e)))
 fig,axes=plt.subplots(2,2,figsize=(14,10),layout='constrained')
 for col,case in enumerate(plan['cases']):
  if case['id'] not in curves:continue
  _,m,data,cs,old,old_data,old_cs=curves[case['id']]
  for label,values,cseries,color in [('group_orca_v1',old_data,old_cs,'#b5792d'),('group_goal_v1',data,cs,'#128466')]:
   t=np.array([v['time']-values[0]['time'] for v in values]);yaw=np.unwrap(np.array([[v['poses'][f'common_{i}_proxy']['yaw'] for i in range(1,7)] for v in values]),axis=0);turn=np.cumsum(np.vstack([np.zeros((1,6)),np.abs(np.diff(yaw,axis=0))]),axis=0).mean(axis=1)/(2*np.pi);axes[0,col].plot(t,turn,label=label,color=color)
   if len(cseries):axes[1,col].plot(cseries[:,0],cseries[:,1],label=label,color=color)
  bound=next(v for v in report['comparisons'] if v['id']==case['id'])['native_cohesion_before']['max_member_centroid_radius_m']+.25;axes[0,col].set(title=case['encounter_profile']+' — native absolute yaw',ylabel='Mean cumulative full turns/person');axes[1,col].axhline(bound,color='black',ls=':',label='Declared max radius bound (reference+.25m)');axes[1,col].set(title='Moving-group spread from actual native poses',ylabel='Max member distance from centroid (m)',xlabel='Seconds after first active native pose')
  for ax in axes[:,col]:ax.grid(alpha=.2);ax.legend(fontsize=8)
 fig.suptitle('Upper group ORCA removed; per-person ORCA and collision projection retained\nSame sim_v1 clock/robot/controller/initial fixture; yaw reduction and cohesion are separate screens.');fig.savefig(a.output/'yaw_and_cohesion.png',dpi=145);plt.close(fig)
 (a.output/'results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(report['comparisons'],indent=2));print('FAILED',report['failed_attempts'])
if __name__=='__main__':main()
