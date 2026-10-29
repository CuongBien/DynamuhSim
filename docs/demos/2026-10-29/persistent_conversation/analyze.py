"""Read-only acceptance for explicit conversation vs natural walking contracts."""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('conversation_seed_analysis',HERE.parent/'seed_regression/analyze.py');base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
rows=lambda p:[json.loads(s) for s in p.read_text().splitlines() if s.strip()]

def analyze_case(run,case,plan):
 item,audit,data=base.analyze_case(run,case,plan);result=json.loads((run/'episode/result.json').read_text());checks=item['checks'];worker=[v for v in rows(run/'runtime/worker_trajectory.jsonl') if v.get('active_phase')];members=[len({h for g in v['groups'] for h in g['members']}) for v in worker];item['native_lifecycle']=dict(grouped_member_min=min(members),grouped_member_max=max(members))
 if case['scenario']=='standing':
  checks['persistent_lifecycle_audit']=audit['checks'].get('persistent_conversation_trace') is True
  checks['snapshot_clock_audit']=audit['checks'].get('goal_clock_snapshot_trace') is True
  checks['explicit_snapshot_contract']=result.get('goal_clock_validation')=='standing_snapshot_v1' and result.get('human_lifecycle_profile')=='persistent_conversation_v1'
  checks['query_verifier_not_misreported']= 'goal_clock_trace' not in audit['checks'] and audit['metrics']['goal_clock_snapshot_trace']['arrival_expiry_query_coverage']=='NOT_EXERCISED'
  expected={tuple([1,2,3]),tuple([4,5,6])};gt=rows(run/'episode/ground_truth.jsonl');checks['native_gt_groups_persist']=bool(gt) and all(len(v['groups'])==2 and {tuple(g['members']) for g in v['groups']}==expected and all(g['type']=='standing' for g in v['groups']) for v in gt)
  item['clock_coverage']=audit['metrics']['goal_clock_snapshot_trace'];item['yielding_observation']=dict(steps_with_near_yielding_enabled=sum(bool(v['motion_guard']['reactive_standing_ids']) for v in worker),max_actual_displacement_from_start_m=item['standing_trace']['max_displacement_from_initial_m'],active_sidestep_correction_exercised=item['standing_trace']['max_displacement_from_initial_m']>1e-6,scope='Near-robot eligibility is not proof that displacement correction was exercised.')
 else:
  checks['original_query_verifier_used']=audit['checks'].get('goal_clock_trace') is True and 'goal_clock_snapshot_trace' not in audit['checks'] and 'goal_clock_validation' not in result
  checks['natural_lifecycle_not_overridden']='human_lifecycle_profile' not in result and 'human_lifecycle' not in result['ready'] and not (run/'assets/human_lifecycle.json').exists()
 item['status']='PASS' if all(checks.values()) else 'FAIL';return item,audit,data

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];report=dict(schema='persistent_conversation_result/1',cases=[],errors=[],training_ready=False,defaults_promoted=False);audits={};media={};curves={}
 for case in plan['cases']:
  try:
   run=raw/case['id'];item,audit,data=analyze_case(run,case,plan);report['cases'].append(item);audits[case['id']]=audit;media[case['id']]=base.media.export(run,a.output);curves[case['id']]=data
  except Exception as e:report['errors'].append(dict(id=case['id'],error=type(e).__name__+': '+str(e)))
 report['status']='PASS' if len(report['cases'])==len(plan['cases']) and not report['errors'] and all(c['status']=='PASS' for c in report['cases']) else 'FAIL';report['bounded_collection_profile_eligible']=report['status']=='PASS';report['protected_sources_unchanged']=all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==s for n,s in plan['protected_sha256'].items())
 if not report['protected_sources_unchanged']:report['status']='FAIL';report['bounded_collection_profile_eligible']=False
 for name,value in [('results.json',report),('audits.json',audits),('media_evidence.json',media)]: (a.output/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
 fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
 for ax,case in zip(axes.flat,plan['cases']):
  if case['id'] not in curves:ax.set_title(case['id']+' — NO NATIVE DATA');continue
  run=raw/case['id'];trace=[v for v in rows(run/'runtime/worker_trajectory.jsonl') if v.get('active_phase')];t=[v['state_time']-trace[0]['time'] for v in trace];count=[len({i for g in v['groups'] for i in g['members']}) for v in trace];ax.step(t,count,where='post');ax.set(title=case['id'],ylim=(-.3,6.5),yticks=[0,3,6],xlabel='Active simulation seconds',ylabel='People currently in native groups');ax.grid(alpha=.2)
 fig.savefig(a.output/'group_membership.png',dpi=145);plt.close(fig)
 cards=''
 for c in report['cases']:
  m=c['metric'];coverage=c.get('clock_coverage',{}).get('arrival_expiry_query_coverage','ORIGINAL QUERY VERIFIER');cards+=f"<article><h2>{c['id']}: {c['status']}</h2><p>Goal {m['status']}; {m['active_sim_s']:.2f}s; recovery {m['max_action_recoveries']}; min robot/person surface gap {m['minimum_swept_robot_human_gap_m']*100:.1f}cm. Clock queries: {coverage}.</p><video controls preload='none' src='{c['id']}.webm'></video></article>"
 (a.output/'index.html').write_text("<!doctype html><meta charset='utf-8'><title>Persistent conversation validation</title><style>body{background:#13212b;color:#eaf0f5;font:17px system-ui;max-width:1100px;margin:30px auto;padding:20px}article{border:1px solid #476275;border-radius:12px;margin:20px 0;padding:18px}video,img{width:100%}a{color:#7ee4c2}</style><h1>Giữ nhóm trò chuyện — "+report['status']+"</h1><p>Ba seed trò chuyện giữ nhóm riêng và một lượt walking lifecycle tự nhiên. Video RGB ghi thật từ Gazebo; phát mặc định1×thời gian mô phỏng.</p><p>Cảnh trò chuyện xác minh snapshot tuổi mục tiêu; truy vấn đến đích/hết hạn NOT_EXERCISED. Không giả lập truy vấn để làm đẹp audit. Không thay lỗi và tiêu chí lịch sử.</p><img src='group_membership.png' alt='Native group membership over time'>"+cards+"<p><a href='results.json'>Results</a> · <a href='audits.json'>Audits</a> · <a href='media_evidence.json'>Media provenance</a> · <a href='plan.json'>Frozen plan</a> · <a href='../seed_regression/index.html'>Cohort lỗi trước</a></p><script>document.querySelectorAll('video').forEach(v=>v.addEventListener('loadedmetadata',()=>v.playbackRate=.25));</script>")
 print(report['status'],[(v['id'],v['status'],[k for k,okay in v['checks'].items() if not okay]) for v in report['cases']],report['errors'])

if __name__=='__main__':main()
