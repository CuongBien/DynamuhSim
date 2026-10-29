"""Read-only frozen-cohort analysis: native geometry, motion and bridge receipts."""
import argparse,hashlib,importlib.util,json,math,html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent

def module(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

human=module('cohort_human',HERE.parent/'group_cohesion_heading/analysis.py');enc=human.enc;geo=enc.geo
media=module('cohort_media',HERE.parent/'bridge_shutdown/build_report.py')
rows=lambda p:[json.loads(x) for x in p.read_text().splitlines() if x.strip()]

def standing_trace(run):
 data=[v for v in rows(run/'runtime/worker_trajectory.jsonl') if v.get('active_phase')];bad=[];maximum_speed=maximum_yaw=maximum_shift=0.;expected_members={tuple([1,2,3]),tuple([4,5,6])};initial=None
 for v in data:
  before={int(h['id']):h for h in v['before']};after={int(h['id']):h for h in v['humans']};dt=v['state_time']-v['time'];robot=v['robot_input'];guard=v['motion_guard'];yielding=set(guard['reactive_standing_ids'])
  if initial is None:initial=before
  inferred={i for i,h in before.items() if math.hypot(h['x']-robot['x'],h['y']-robot['y'])<1.4}
  if set(before)!=set(range(1,7)) or set(after)!=set(before) or dt<=0:raise ValueError('Invalid standing trace population/time')
  reasons=[]
  if inferred!=yielding:reasons.append('yielding_set_not_reproducible')
  for field in ['groups_before','groups']:
   if len(v[field])!=2 or {tuple(g['members']) for g in v[field]}!=expected_members or any(g['type']!='standing' for g in v[field]):reasons.append('standing_membership_changed')
  for i,h in after.items():
   a=before[i];dist=math.hypot(h['x']-a['x'],h['y']-a['y']);speed=dist/dt;turn=abs(math.atan2(math.sin(h['yaw']-a['yaw']),math.cos(h['yaw']-a['yaw'])))/dt
   maximum_speed=max(maximum_speed,speed);maximum_yaw=max(maximum_yaw,turn);maximum_shift=max(maximum_shift,math.hypot(h['x']-initial[i]['x'],h['y']-initial[i]['y']))
   if speed>.2+1e-7:reasons.append('speed_cap')
   if i not in yielding and dist>1e-8:reasons.append('movement_outside_yielding_set')
   if turn>1.57+1e-7:reasons.append('yaw_cap')
  if reasons:bad.append(dict(time=v['time'],reasons=sorted(set(reasons))))
 return dict(status='PASS' if data and not bad else 'FAIL',active_steps=len(data),bad_steps=bad,max_step_speed_mps=maximum_speed,max_step_yaw_rate_rad_s=maximum_yaw,max_displacement_from_initial_m=maximum_shift,scope='Independent before/after displacement and yaw plus near-robot set reconstruction. Total displacement is descriptive, not a gated limit.')

def native_spread(run,data):
 stamps=np.array([v['time'] for v in data]);max_radius=0.;samples=0;skipped=0
 for row in rows(run/'episode/ground_truth.jsonl'):
  if not stamps[0]<=row['time']<=stamps[-1]:continue
  ix=int(np.argmin(abs(stamps-row['time'])))
  if abs(stamps[ix]-row['time'])>.03:skipped+=1;continue
  for group in row['groups']:
   points=np.array([geo.xy(data[ix]['poses'][f'common_{i}_proxy']) for i in group['members']]);max_radius=max(max_radius,float(np.linalg.norm(points-points.mean(axis=0),axis=1).max()));samples+=1
 return dict(max_member_centroid_radius_m=max_radius,group_samples=samples,alignment_skips=skipped)

def analyze_case(run,case,plan):
 result=json.loads((run/'episode/result.json').read_text());audit=json.loads((run.parent/(run.name+'_audit.json')).read_text())
 if case['scenario']=='walking':metric,data,_=enc.analyze_social(run,case)
 else:
  metric,data,_=geo.compute(run);metric.update(audit=audit['status'],proximity=enc.proximity(data),human_safety=enc.human_safety(run,data),native_human_full=enc.human.native_turns(data),**enc.progress(run))
 checks=dict(full_capture_audit=audit['status']=='PASS',ordered_bridge=audit['checks'].get('ordered_ground_truth_bridge_exit') is True,actual_goal=metric['actual_goal_reached'] and metric['status']=='SUCCEEDED',zero_recoveries=metric['max_action_recoveries']==0 and metric['progress_failure_count']==0,robot_person_clearance=metric['minimum_swept_robot_human_gap_m']>0,robot_static_clearance=metric['minimum_static_gap_lower_bound_m']>0,human_clearances=min(metric['human_safety'].values())>0,near_coverage=metric['proximity']['coverage']=='EXERCISED')
 reference_result=json.loads((ROOT/'experiments/social_navigation/benchmark/generated/bridge_shutdown_20261029/head_on_bridge_order/episode/result.json').read_text());protected=plan['protected_sha256'];checks['same_frozen_protected_sources']=all(result['source_sha256'].get(k)==v for k,v in protected.items() if k in reference_result['source_sha256'])
 for name in ['assets/mecanum_camera.sdf','assets/mecanum_aligned.urdf','runtime/nav2_runtime.yaml']:
  checks['same_robot_nav:'+name]=(run/name).read_bytes()==(ROOT/'experiments/social_navigation/benchmark/generated/bridge_shutdown_20261029/head_on_bridge_order'/name).read_bytes()
 bridge=result['server_shutdown'].get('ground_truth_bridge_shutdown',{});lines=(run/'episode/launch.log').read_text().splitlines();bi=lines.index(bridge['record']) if bridge.get('record') in lines else -1;gi=next((i for i,v in enumerate(lines) if v.startswith('[INFO] [gz-1]: process has finished cleanly')), -1);checks['bridge_clean_before_gazebo']=bridge.get('clean') is True and 0<=bi<gi
 extra=dict(native_spread=native_spread(run,data))
 if case['scenario']=='walking':
  limit=plan['walking_thresholds'][case['encounter_profile']];rate=float(np.mean(metric['native_human_full']['yaw_total_turns']))/metric['native_human_full']['duration_s'];backward=metric['worker']['worker_full']['moving_backwards_fraction'];radius=extra['native_spread']['max_member_centroid_radius_m'];checks.update(initial_intent=metric['worker']['initial_intent_executed'],yaw_rate_limit=rate<=limit['max_mean_yaw_rate_turns_s'],cohesion_limit=radius<=limit['max_member_centroid_radius_m'],backwards_limit=backward is not None and backward<=limit['max_moving_backwards_fraction']);extra['walking_motion']=dict(mean_yaw_rate_turns_s=rate,moving_backwards_fraction=backward,limits=limit,scope='Fixed historical seed45 absolute limits, not matched-seed improvement.')
 else:
  extra['standing_trace']=standing_trace(run);checks['standing_trace']=extra['standing_trace']['status']=='PASS'
 return dict(id=case['id'],scenario=case['scenario'],seed=case['seed'],status='PASS' if all(checks.values()) else 'FAIL',checks=checks,metric=metric,bridge=bridge,runtime_commit=result['head'],**extra),audit,data

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];report=dict(schema='seed_regression_result/1',cases=[],errors=[],defaults_promoted=False,training_ready=False);audits={};provenance={};curves={}
 for case in plan['cases']:
  try:
   run=raw/case['id'];audits[case['id']]=json.loads((raw/(case['id']+'_audit.json')).read_text());item,audit,data=analyze_case(run,case,plan);report['cases'].append(item);audits[case['id']]=audit;provenance[case['id']]=media.export(run,out);curves[case['id']]=data
  except Exception as error:
   result_path=raw/case['id']/'episode/result.json';original=json.loads(result_path.read_text()) if result_path.exists() else {}
   report['errors'].append(dict(id=case['id'],analysis_error=type(error).__name__+': '+str(error),runtime_status=original.get('status'),runtime_error=original.get('error'),runtime_commit=original.get('head'),cleanup=original.get('cleanup'),audit=audits.get(case['id']),raw_result_sha256=hashlib.sha256(result_path.read_bytes()).hexdigest() if result_path.exists() else None))
 report['status']='PASS' if len(report['cases'])==len(plan['cases']) and not report['errors'] and all(v['status']=='PASS' for v in report['cases']) else 'FAIL';report['bounded_pilot_profile_eligible']=report['status']=='PASS';report['execution']=json.loads((raw/'execution.json').read_text())
 for name,value in [('results.json',report),('audits.json',audits),('media_evidence.json',provenance)]: (out/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
 fig,axes=plt.subplots(2,3,figsize=(15,8),layout='constrained')
 for col,scene in enumerate(['head_on','crossing','standing']):
  for case in plan['cases']:
   if not case['id'].startswith(scene) or case['id'] not in curves:continue
   item=next(v for v in report['cases'] if v['id']==case['id']);data=curves[case['id']];t=np.array([v['time']-data[0]['time'] for v in data]);yaw=np.unwrap(np.array([[v['poses'][f'common_{i}_proxy']['yaw'] for i in range(1,7)] for v in data]),axis=0);turn=np.cumsum(np.vstack([np.zeros((1,6)),np.abs(np.diff(yaw,axis=0))]),axis=0).mean(axis=1)/(2*np.pi);axes[0,col].plot(t,turn,label=f"seed{case['seed']} ({item['status']})");nearest=[min(np.linalg.norm(geo.xy(v['poses']['mecanum_robot'])-geo.xy(v['poses'][f'common_{i}_proxy'])) for i in range(1,7)) for v in data];axes[1,col].plot(t,nearest,label=f"seed{case['seed']}")
  axes[0,col].set(title=scene,ylabel='Mean cumulative absolute yaw / turns');axes[1,col].axhline(1.5,color='gray',ls=':');axes[1,col].set(ylabel='Nearest robot/person center (m)',xlabel='Active simulation seconds')
  for ax in axes[:,col]:ax.grid(alpha=.2);ax.legend(fontsize=8)
 fig.savefig(out/'native_motion.png',dpi=140);plt.close(fig)
 report_counts=dict(planned=len(plan['cases']),analyzed_native=len(report['cases']),capture_audit_pass=sum(v.get('status')=='PASS' for v in audits.values()),integrated_pass=sum(v['status']=='PASS' for v in report['cases']),integrated_fail=sum(v['status']=='FAIL' for v in report['cases']),runtime_or_analysis_errors=len(report['errors']))
 cards='<p>'+html.escape(str(report_counts))+'</p>'
 for item in report['cases']:
  m=item['metric'];failed=[k for k,v in item['checks'].items() if not v];cards+=f"<article><h2>{item['id']}: {item['status']}</h2><p>Goal {m['status']}; {m['active_sim_s']:.2f}s; recoveries {m['max_action_recoveries']}; min robot/person surface gap {m['minimum_swept_robot_human_gap_m']*100:.1f}cm; group radius {item['native_spread']['max_member_centroid_radius_m']:.2f}m.</p><p>Failed gates: {', '.join(failed) or 'none'}</p><video controls preload='none' src='{item['id']}.webm'></video></article>"
 for error in report['errors']:cards+="<article><h2>"+html.escape(error['id'])+": ERROR / no valid native episode</h2><p>"+html.escape(str(error.get('runtime_error') or error['analysis_error']))+"</p><p>Original failure preserved; no retry. See results.json for raw receipt.</p></article>"
 (out/'index.html').write_text("<!doctype html><meta charset='utf-8'><title>Seed46–48 regression</title><style>body{background:#101923;color:#eaf0f5;font:17px system-ui;max-width:1150px;margin:32px auto;padding:20px}a{color:#76dbbf}article{border:1px solid #42596a;padding:18px;margin:18px 0;border-radius:12px}video,img{width:100%}</style><h1>Seed46–48: nhóm đi bộ và đứng trò chuyện</h1><p>Cohort status: "+report['status']+". Native Gazebo RGB thật, phát mặc định1×thời gian mô phỏng. Giữ mọi FAIL, không retry/tuning. Ngưỡng walking là mốc lịch sử seed45; không phải mức cải thiện so với baseline cùng seed.</p><img src='native_motion.png' alt='Native yaw and proximity plots'>"+cards+"<p><a href='results.json'>Results</a> · <a href='audits.json'>Full audits</a> · <a href='media_evidence.json'>Media provenance</a> · <a href='plan.json'>Frozen plan</a></p><script>document.querySelectorAll('video').forEach(v=>v.addEventListener('loadedmetadata',()=>v.playbackRate=.25));</script>")
 print(report['status'],[(v['id'],v['status'],[k for k,b in v['checks'].items() if not b]) for v in report['cases']],report['errors'])

if __name__=='__main__':main()
