"""Descriptive lifecycle diagnosis; never changes frozen regression gates."""
import argparse,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];result=dict(schema='native_lifecycle_observations/1',scope='Descriptive diagnosis only; does not alter any frozen PASS/FAIL, initial continuous-standing contract or raw audit. Standing cap is conditional on actual stationary status at integration; dynamic individuals may use existing .8m/s bound.',cases=[]);fig,axes=plt.subplots(3,3,figsize=(15,10),layout='constrained')
 for case in plan['cases']:
  run=raw/case['id']
  if not (run/'runtime/worker_trajectory.jsonl').exists():
   result['cases'].append(dict(id=case['id'],status='NO_NATIVE_DATA',reason='Preserved startup error before Gazebo/worker trajectory.'));continue
  data=[json.loads(x) for x in (run/'runtime/worker_trajectory.jsonl').read_text().splitlines() if x.strip()];data=[v for v in data if v.get('active_phase')];events=[];counts=[];times=[];last=None;cap_bad=[];max_stationary_speed=max_dynamic_speed=0.
  for v in data:
   signature=sorted((g['type'],tuple(g['members'])) for g in v['groups']);counts.append(len({i for g in v['groups'] for i in g['members']}));times.append(v['state_time']-data[0]['time'])
   if last is not None and signature!=last:events.append(dict(time_sim_s=v['state_time'],after_start_s=v['state_time']-data[0]['time'],before=last,after=signature,finish_events=v['finish_events']))
   last=signature
   if case['scenario']=='standing':
    before={h['id']:h for h in v['before']};dt=v['state_time']-v['time'];stationary=set(v['motion_guard']['native_stationary_ids']);yielding=set(v['motion_guard']['reactive_standing_ids'])
    for h in v['humans']:
     old=before[h['id']];distance=math.hypot(h['x']-old['x'],h['y']-old['y']);speed=distance/dt
     if h['id'] in stationary:
      max_stationary_speed=max(max_stationary_speed,speed)
      if speed>.2+1e-7 or h['id'] not in yielding and distance>1e-8:cap_bad.append(dict(time=v['time'],id=h['id'],kind='actual_stationary_cap'))
     else:
      max_dynamic_speed=max(max_dynamic_speed,speed)
      if speed>.8+1e-7:cap_bad.append(dict(time=v['time'],id=h['id'],kind='actual_dynamic_cap'))
  item=dict(id=case['id'],grouped_member_min=min(counts),grouped_member_max=max(counts),fraction_steps_all_six_grouped=float(np.mean(np.array(counts)==6)),membership_type_transitions=events)
  if case['scenario']=='standing':item['conditional_guard_diagnostic']=dict(status='PASS' if not cap_bad else 'FAIL',actual_stationary_speed_max_mps=max_stationary_speed,actual_dynamic_speed_max_mps=max_dynamic_speed,bad_steps=cap_bad,scope='Categories from original integration guard trace; positions/speeds independently recomputed. Does NOT cure continuous-standing membership failure.')
  result['cases'].append(item);row=case['seed']-46;col=['head_on','crossing','standing'].index(case.get('encounter_profile','standing'));ax=axes[row,col];ax.step(times,counts,where='post');ax.set(title=case['id'],ylim=(-.3,6.5),yticks=[0,3,6],ylabel='People currently in native groups',xlabel='Simulation seconds after first active step');ax.grid(alpha=.2)
 fig.savefig(a.output/'native_group_membership.png',dpi=140);plt.close(fig);(a.output/'lifecycle_observations.json').write_text(json.dumps(result,indent=2)+'\n');print([(v['id'],v.get('grouped_member_min'),len(v.get('membership_type_transitions',[]))) for v in result['cases']])

if __name__=='__main__':main()
