"""Four declared captures, once each, gated on preserved component evidence."""
import hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent

def main():
 plan=json.loads((HERE/'plan.json').read_text());preflight=json.loads((ROOT/plan['preflight_root']/'results.json').read_text())
 if preflight['status']!='PASS':raise RuntimeError('Component launch gate failed')
 if any(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest for name,digest in plan['protected_sha256'].items()):raise RuntimeError('Protected behavior/robot inputs changed')
 out=ROOT/plan['raw_root'];out.mkdir(parents=True,exist_ok=False);receipt=[];blocked=False
 for c in plan['cases']:
  name=c['id'];run=out/name;row=dict(id=name)
  if blocked:row.update(status='NOT_RUN',reason='Earlier cleanup not empty');receipt.append(row);continue
  print('START',name,flush=True);command=['python3','tools/capture_episode.py','prepare','--run-dir',str(run),'--config',c['config'],'--scenario',c['scenario'],'--seed',str(c['seed']),'--planner-diagnostics','--human-motion-profile',c['human_motion_profile'],'--human-goal-clock',c['human_goal_clock'],'--human-lifecycle-profile',c['human_lifecycle_profile']]
  if 'encounter_profile' in c:command+=['--encounter-profile',c['encounter_profile']]
  with (out/(name+'_prepare.log')).open('x') as log:row['prepare_exit']=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT).returncode
  if row['prepare_exit']:row.update(status='PREPARE_ERROR');blocked=True
  else:
   with (out/(name+'_run.log')).open('x') as log:row['run_exit']=subprocess.run(['python3','tools/capture_episode.py','run','--run-dir',str(run)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT).returncode
   with (out/(name+'_audit.log')).open('x') as log:row['audit_exit']=subprocess.run(['python3','tools/audit_episode.py','--run-dir',str(run),'--source-root',str(ROOT),'--output',str(out/(name+'_audit.json'))],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT).returncode
   file=run/'episode/result.json';result=json.loads(file.read_text()) if file.exists() else {};cleanup=result.get('cleanup',{});blocked=cleanup.get('status')!='PASS' or bool(cleanup.get('remaining',[True]));row.update(status='CAPTURED' if row['run_exit']==0 else 'RUN_ERROR',cleanup_empty=not blocked)
  receipt.append(row);(out/'execution.json').write_text(json.dumps(receipt,indent=2)+'\n');print('FINISH',row,flush=True)
 (out/'execution.json').write_text(json.dumps(receipt,indent=2)+'\n');return 0 if all(v['status']=='CAPTURED' and v['audit_exit']==0 for v in receipt) else 1

if __name__=='__main__':raise SystemExit(main())
