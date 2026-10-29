"""Execute each predeclared cell once, without changing runtime or parameters."""
import hashlib,json,os,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent

def main():
 plan=json.loads((HERE/'plan.json').read_text())
 if any(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest for name,digest in plan['protected_sha256'].items()):raise RuntimeError('Protected runtime changed')
 out=ROOT/plan['raw_root'];out.mkdir(parents=True,exist_ok=False);execution=[];blocked=False
 for case in plan['cases']:
  name=case['id'];run=out/name;row=dict(id=name)
  if blocked:
   row.update(status='NOT_RUN',reason='Earlier runtime failed scoped cleanup');execution.append(row);continue
  print('START',name,flush=True)
  cmd=['python3','tools/capture_episode.py','prepare','--run-dir',str(run),'--config',case['config'],'--scenario',case['scenario'],'--seed',str(case['seed']),'--planner-diagnostics','--human-motion-profile',case['human_motion_profile'],'--human-goal-clock',case['human_goal_clock']]
  if 'encounter_profile' in case:cmd+=['--encounter-profile',case['encounter_profile']]
  with (out/(name+'_prepare.log')).open('x') as f:row['prepare_exit']=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT).returncode
  if row['prepare_exit']:
   row.update(status='PREPARE_ERROR');blocked=True
  else:
   with (out/(name+'_run.log')).open('x') as f:row['run_exit']=subprocess.run(['python3','tools/capture_episode.py','run','--run-dir',str(run)],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT).returncode
   with (out/(name+'_audit.log')).open('x') as f:row['audit_exit']=subprocess.run(['python3','tools/audit_episode.py','--run-dir',str(run),'--source-root',str(ROOT),'--output',str(out/(name+'_audit.json'))],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT).returncode
   result_path=run/'episode/result.json';result=json.loads(result_path.read_text()) if result_path.exists() else {};cleanup=result.get('cleanup',{})
   blocked=cleanup.get('status')!='PASS' or bool(cleanup.get('remaining',[True]));row.update(status='CAPTURED' if row['run_exit']==0 else 'RUN_ERROR',scoped_cleanup_empty=not blocked)
  execution.append(row);(out/'execution.json').write_text(json.dumps(execution,indent=2)+'\n');print('FINISH',row,flush=True)
 (out/'execution.json').write_text(json.dumps(execution,indent=2)+'\n')
 return 1 if any(v['status']!='CAPTURED' or v.get('audit_exit') for v in execution) else 0

if __name__=='__main__':raise SystemExit(main())
