"""Read-only encounter coverage and native physical-clearance analysis."""
import argparse,importlib.util,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent

def module(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
geo=module('encounter_geo',HERE.parent/'geometry_ab/compare.py')
gaps=module('encounter_gap',HERE.parent/'static_gap/analyze.py')
human=module('encounter_human',HERE.parent/'endpoint_humans/analyze.py')
rows=lambda p:[json.loads(s) for s in p.read_text().splitlines() if s.strip()]

def threshold_interval(a,b,radius):
 a=np.asarray(a,dtype=float);delta=np.asarray(b,dtype=float)-a;aa=float(delta@delta);bb=2*float(a@delta);cc=float(a@a)-radius**2
 if aa<1e-20:return (0.,1.) if cc<=0 else None
 disc=bb*bb-4*aa*cc
 if disc<0:return None
 root=math.sqrt(max(0.,disc));lo=max(0.,(-bb-root)/(2*aa));hi=min(1.,(-bb+root)/(2*aa));return (lo,hi) if lo<=hi else None

def union_length(intervals):
 intervals=sorted(v for v in intervals if v is not None)
 if not intervals:return 0.
 lo,hi=intervals[0];length=0.
 for a,b in intervals[1:]:
  if a<=hi:hi=max(hi,b)
  else:length+=hi-lo;lo,hi=a,b
 return length+hi-lo

def proximity(data,threshold=1.5):
 total=0.;minimum=math.inf;min_time=None
 for a,b in zip(data,data[1:]):
  p0,p1=a['poses'],b['poses'];r0,r1=geo.xy(p0['mecanum_robot']),geo.xy(p1['mecanum_robot']);intervals=[]
  for hid in range(1,7):
   key=f'common_{hid}_proxy';v0=geo.xy(p0[key])-r0;v1=geo.xy(p1[key])-r1;d=v1-v0;dd=float(d@d);fraction=float(np.clip(-v0@d/dd,0,1)) if dd else 0.;distance=float(np.linalg.norm(v0+fraction*d))
   if distance<minimum:minimum=distance;min_time=a['time']+fraction*(b['time']-a['time'])
   intervals.append(threshold_interval(v0,v1,threshold))
  total+=union_length(intervals)*(b['time']-a['time'])
 return dict(center_threshold_m=threshold,cumulative_duration_s=total,minimum_center_distance_m=minimum,minimum_time_sim_s=min_time,coverage='EXERCISED' if total>=.5 and minimum<=threshold else 'NOT_EXERCISED',method='Exact union of linear relative-motion circle intervals between consecutive native pose samples.')

def human_safety(run,data):
 xy=np.array([[[v['poses'][f'common_{i}_proxy'][axis] for axis in ['x','y']] for i in range(1,7)] for v in data]);static=pair=math.inf
 for _,lo,hi in geo.static_boxes(run):
  distances=np.linalg.norm(np.maximum(np.maximum(lo-xy,xy-hi),0),axis=2);static=min(static,float((np.minimum(distances[:-1],distances[1:])-.5*np.linalg.norm(np.diff(xy,axis=0),axis=2)-.36).min()))
 for i in range(6):
  for j in range(i):
   a=xy[:-1,i]-xy[:-1,j];delta=np.diff(xy[:,i]-xy[:,j],axis=0);dd=(delta*delta).sum(axis=1);f=np.divide(-(a*delta).sum(axis=1),dd,out=np.zeros_like(dd),where=dd>1e-20).clip(0,1);pair=min(pair,float(np.linalg.norm(a+f[:,None]*delta,axis=1).min())-.72)
 return dict(minimum_swept_human_human_gap_m=pair,minimum_human_static_lower_bound_m=static)

def worker_diagnostics(run,end,start,spec):
 data=[v for v in rows(run/'runtime/worker_trajectory.jsonl') if v.get('active_phase') and v['time']<=end];first=data[:120];intent=0.;initial_intent={tuple(g['members']):0. for g in spec['groups']};events=[];previous={};ages=[]
 for v in data:
  mapping={int(h['id']):h for h in v['humans']};targets={tuple(g['members']):g['goal'] for g in v.get('encounter_goals',[])}
  if v in first:
   directional=any((h['vx']<-.05 if spec['profile']=='head_on' else abs(h['vy'])>.05) for h in v['humans']);intent+=.1*directional
   for g in spec['groups']:
    ids=tuple(g['members']);direction=np.array(g['goal'])-g['center'];direction/=np.linalg.norm(direction);velocity=np.mean([[mapping[hid]['vx'],mapping[hid]['vy']] for hid in ids],axis=0)
    if targets.get(ids)==g['goal'] and velocity@direction>.05:initial_intent[ids]+=.1
  for members,target in targets.items():
   if members in previous and previous[members]!=target:events.append(dict(time_after_goal_s=v['time']-start,members=list(members),previous=previous[members],goal=target,reported_goal_ages_wall_s=v.get('encounter_goal_ages_wall_s',{})))
  previous=targets;ages.extend(v.get('encounter_goal_ages_wall_s',{}).values())
 return dict(worker_first12=human.human_metrics(first),worker_full=human.human_metrics(data),initial_direction_union_duration_first12_s=intent,initial_intent_executed=intent>=1.,per_group_initial_goal_progress_duration_first12_s={','.join(map(str,k)):v for k,v in initial_intent.items()},native_group_goal_changes=events,max_reported_native_goal_age_wall_s=max(ages,default=None),scope='Native goals may complete or expire by host wall time; target-change logs alone do not identify which condition fired.')

def progress(run):
 logs=rows(run/'episode/roslogs.jsonl');feedback=rows(run/'episode/feedback.jsonl');fail=[v for v in logs if v['node']=='controller_server' and v['text']=='Failed to make progress'];return dict(progress_failure_count=len(fail),max_action_recoveries=max((v.get('recoveries',0) for v in feedback),default=0))

def analyze_social(run,case):
 metric,data,points=geo.compute(run);result=json.loads((run/'episode/result.json').read_text());events={v['event']:v['stamp_ns']/1e9 for v in rows(run/'episode/events.jsonl')};start,end=events['goal_accepted'],events['action_end'];spec=json.loads((run/'assets/encounter.json').read_text());audit=json.loads((run.parent/(run.name+'_audit.json')).read_text());metric.update(id=case['id'],kind='social',encounter_profile=case['encounter_profile'],human_motion_profile=case['human_motion_profile'],audit=audit['status'],proximity=proximity(data),native_human_first12=human.native_turns([v for v in data if v['time']<=start+12]),native_human_full=human.native_turns(data),human_safety=human_safety(run,data),worker=worker_diagnostics(run,end,start,spec),**progress(run));metric['physical_pass']=audit['status']=='PASS' and metric['actual_goal_reached'] and metric['minimum_swept_robot_human_gap_m']>0 and metric['minimum_static_gap_lower_bound_m']>0 and min(metric['human_safety'].values())>0;metric['recovery_free']=metric['progress_failure_count']==0 and metric['max_action_recoveries']==0;metric['encounter_screen_pass']=metric['physical_pass'] and metric['recovery_free'] and metric['proximity']['coverage']=='EXERCISED' and metric['worker']['initial_intent_executed'];return metric,data,points

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];report=dict(schema='encounter_regression_result/1',social=[],gaps=[],failed_attempts=[],defaults_promoted=False,training_ready=False);streams={}
 for case in plan['social_cases']:
  run=raw/case['id']
  try:
   metric,data,points=analyze_social(run,case);report['social'].append(metric);streams[case['id']]=(metric,data,points)
  except Exception as e:report['failed_attempts'].append(dict(id=case['id'],error=type(e).__name__+': '+str(e),raw=str(run.relative_to(ROOT))))
 for case in plan['gap_cases']:
  run=raw/case['id']
  try:
   metric,spec,data,points,result=gaps.compute(run);metric.update(id=case['id'],kind='gap',**progress(run));metric['recovery_free']=metric['progress_failure_count']==0 and metric['max_action_recoveries']==0;metric['gap_screen_pass']=metric['accepted'] and metric['recovery_free'];report['gaps'].append(metric)
  except Exception as e:report['failed_attempts'].append(dict(id=case['id'],error=type(e).__name__+': '+str(e),raw=str(run.relative_to(ROOT))))
 fig,axes=plt.subplots(2,2,figsize=(14,10),layout='constrained')
 for col,profile in enumerate(['head_on','crossing']):
  for suffix,color in [('legacy','#386ec7'),('group','#0e8e65')]:
   name=profile+'_'+suffix
   if name not in streams:continue
   m,data,points=streams[name];start=data[0]['time'];t=np.array([v['time']-start for v in data]);minimum=np.array([min(np.linalg.norm(geo.xy(v['poses']['mecanum_robot'])-geo.xy(v['poses'][f'common_{i}_proxy'])) for i in range(1,7)) for v in data]);axes[0,col].plot(t,minimum,label=suffix,color=color);yaw=np.unwrap(np.array([[v['poses'][f'common_{i}_proxy']['yaw'] for i in range(1,7)] for v in data]),axis=0);turn=np.cumsum(np.vstack([np.zeros((1,6)),np.abs(np.diff(yaw,axis=0))]),axis=0).mean(axis=1)/(2*np.pi);axes[1,col].plot(t,turn,label=suffix,color=color)
  axes[0,col].set(title=profile+' — nearest native human center',ylabel='Robot/human center distance (m)');axes[0,col].axhline(1.5,color='black',ls=':',label='Coverage threshold1.5m');axes[1,col].set(title='Mean cumulative absolute yaw per human',ylabel='Full turns',xlabel='Seconds after first active native pose')
  for ax in axes[:,col]:ax.grid(alpha=.2);ax.legend()
 fig.savefig(a.output/'encounter_comparison.png',dpi=145);plt.close(fig)
 (a.output/'results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps({m['id']:{k:m[k] for k in ['status','audit','physical_pass','encounter_screen_pass','proximity','progress_failure_count']} for m in report['social']},indent=2));print('GAPS',json.dumps(report['gaps'],indent=2));print('FAILED',report['failed_attempts'])
if __name__=='__main__':main()
