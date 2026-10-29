"""Read-only clock-domain and matched encounter outcome comparison."""
import argparse,importlib.util,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('clock_encounter',HERE.parent/'encounter_regression/analysis.py');enc=importlib.util.module_from_spec(s);s.loader.exec_module(enc)
rows=lambda p:[json.loads(v) for v in p.read_text().splitlines() if v.strip()]

def clock_metrics(run,start,end):
 data=[v for v in rows(run/'runtime/worker_trajectory.jsonl') if v.get('active_phase') and v['time']<=end];snapshots=[v['goal_clock'] for v in data];checks=[e for v in snapshots for e in v['events'] if e['event']=='checked'];assignments=[e for v in snapshots for e in v['events'] if e['event']=='assigned'];wall_exceed=0
 for row in data:
  ages=row.get('encounter_goal_ages_wall_s',{})
  wall_exceed+=sum(age>50 and row['goal_clock']['humans'][hid]['age_sim_s']<=50 for hid,age in ages.items())
 return dict(profile='sim_v1',gate_start_sim_s=snapshots[0]['gate_start_sim_s'],first_active_tick_after_gate_s=data[0]['time']-start,max_goal_age_sim_s=max(h['age_sim_s'] for v in snapshots for h in v['humans'].values()),checked_goal_count=len(checks),expired_evaluations=sum(v['expired'] for v in checks),timeout_only_evaluations=sum(v['expired'] and not v['geometric'] for v in checks),assignment_count=len(assignments),native_wall_age_over50_while_sim_age_not_over50=wall_exceed,assignments=assignments,scope='Counts native human-level checks/assignments, not unique group goals. Native wall ages are retained as diagnostic telemetry and are not used by sim_v1 behavior.')

def mean_rate(metric):
 v=metric['native_human_full'];return sum(v['yaw_total_turns'])/len(v['yaw_total_turns'])/v['duration_s']

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];reference=ROOT/plan['reference_root'];old={m['id']:m for m in json.loads((HERE.parent/'encounter_regression/results.json').read_text())['social']};report=dict(schema='sim_goal_clock_result/1',cases=[],comparisons=[],failed_attempts=[],defaults_promoted=False,naturalness_accepted=False,training_ready=False);curves={}
 for case in plan['cases']:
  run=raw/case['id']
  try:
   metric,data,points=enc.analyze_social(run,case);events={v['event']:v['stamp_ns']/1e9 for v in rows(run/'episode/events.jsonl')};metric['clock']=clock_metrics(run,events['goal_accepted'],events['action_end']);audit=json.loads((raw/(case['id']+'_audit.json')).read_text());metric['clock_integrity']=audit['checks']['goal_clock_trace'];metric['clock_audit']=audit['metrics']['goal_clock_trace'];report['cases'].append(metric);before=old[case['reference_id']];change=dict(id=case['id'],reference_id=case['reference_id'],active_sim_change_s=metric['active_sim_s']-before['active_sim_s'],near_duration_change_s=metric['proximity']['cumulative_duration_s']-before['proximity']['cumulative_duration_s'],first12_mean_turns_before=float(np.mean(before['native_human_first12']['yaw_total_turns'])),first12_mean_turns_after=float(np.mean(metric['native_human_first12']['yaw_total_turns'])),whole_turn_rate_change_fraction=mean_rate(metric)/mean_rate(before)-1,progress_failure_count=metric['progress_failure_count'],physical_pass=metric['physical_pass'],encounter_screen_pass=metric['encounter_screen_pass']);report['comparisons'].append(change);curves[case['id']]=(case,metric,data,events['goal_accepted'])
  except Exception as e:report['failed_attempts'].append(dict(id=case['id'],error=type(e).__name__+': '+str(e),raw=str(run.relative_to(ROOT))))
 fig,axes=plt.subplots(2,2,figsize=(15,10),layout='constrained')
 for ax,case in zip(axes.flat,plan['cases']):
  for kind,run,color in [('native_wall',reference/case['reference_id'],'#b57928'),('sim_v1',raw/case['id'],'#16856c')]:
   path=run/'runtime/worker_trajectory.jsonl'
   if not path.exists():continue
   events={v['event']:v['stamp_ns']/1e9 for v in rows(run/'episode/events.jsonl')};start,end=events['goal_accepted'],events['action_end'];data=[v for v in rows(path) if v.get('active_phase') and v['time']<=end];t=[v['state_time']-start for v in data]
   ages=[max(v['encounter_goal_ages_wall_s'].values(),default=0.) if kind=='native_wall' else max(h['age_sim_s'] for h in v['goal_clock']['humans'].values()) for v in data];ax.plot(t,ages,color=color,label=kind+' declared goal age')
  ax.axhline(50,color='black',ls=':',label='Timeout threshold50s');ax.set(title=case['reference_id'],xlabel='Seconds of Gazebo time after accepted goal',ylabel='Goal age in the declared clock (s)');ax.grid(alpha=.2);ax.legend(fontsize=8)
 fig.suptitle('Same timeout value, different clock domains\nHost waiting/simulation speed ages native goals; sim_v1 starts at accepted goal and follows Gazebo time.',fontsize=14);fig.savefig(a.output/'goal_ages.png',dpi=145);plt.close(fig)
 fig,axes=plt.subplots(2,2,figsize=(15,10),layout='constrained')
 for ax,case in zip(axes.flat,plan['cases']):
  for kind,run,color in [('native_wall',reference/case['reference_id'],'#b57928'),('sim_v1',raw/case['id'],'#16856c')]:
   if not (run/'episode/events.jsonl').exists():continue
   events={v['event']:v['stamp_ns']/1e9 for v in rows(run/'episode/events.jsonl')};start,end=events['goal_accepted'],events['action_end'];data=[v for v in rows(run/'episode/gazebo_poses.jsonl') if start<=v['time']<=end];t=np.array([v['time']-start for v in data]);yaw=np.unwrap(np.array([[v['poses'][f'common_{i}_proxy']['yaw'] for i in range(1,7)] for v in data]),axis=0);turn=np.cumsum(np.vstack([np.zeros((1,6)),np.abs(np.diff(yaw,axis=0))]),axis=0).mean(axis=1)/(2*np.pi);ax.plot(t,turn,label=kind,color=color)
  ax.set(title=case['reference_id'],xlabel='Seconds of Gazebo time after accepted goal',ylabel='Mean cumulative absolute yaw / full turns');ax.grid(alpha=.2);ax.legend()
 fig.suptitle('Recorded native human yaw: clock repair does not itself guarantee natural behavior');fig.savefig(a.output/'yaw_comparison.png',dpi=145);plt.close(fig)
 (a.output/'results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(report['comparisons'],indent=2));print('FAILED',report['failed_attempts'])
if __name__=='__main__':main()
