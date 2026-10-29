"""Same recorded ego inputs: exact natural control and explicit persistent fixture."""
import argparse,copy,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'experiments/social_navigation/navigation/socnavgym_group_nav2')]
from fixture import make_kernel
from dynamuhsim.simulation.conversation import PersistentConversation,verify_conversation
from dynamuhsim.simulation.goal_clock import GoalClock,verify_trace
from dynamuhsim.analysis.standing_clock import verify_snapshots

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());source=ROOT/plan['component_reference'];original=[json.loads(s) for s in (source/'runtime/worker_trajectory.jsonl').read_text().splitlines() if json.loads(s).get('active_phase')][:plan['component_steps']];gate=float((source/'runtime/start_gate').read_text());results={}
 for name in ['native','persistent_conversation_v1']:
  k=make_kernel('standing',46);saved=[];error=None;mode=PersistentConversation(k) if name!='native' else None;clock=GoalClock(k);ready=copy.deepcopy(clock.snapshot())
  try:
   with (a.output/(name+'.jsonl')).open('x') as f:
    for request in original:
     t=request['time'];before=copy.deepcopy(k.humans());groups=copy.deepcopy(k.groups());clock.begin_step(t,gate);k.advance(request['robot_input']);clock.end_step(t+.1);row=dict(time=t,state_time=t+.1,active_phase=True,before=before,humans=k.humans(),groups_before=groups,groups=k.groups(),goal_clock=copy.deepcopy(clock.snapshot()),motion_guard=k.motion.steps[-1])
     if mode is not None:row['human_lifecycle']=mode.snapshot()
     saved.append(row);f.write(json.dumps(row,allow_nan=False)+'\n')
  except Exception as e:error=type(e).__name__+': '+str(e)
  finally:k.close()
  result=dict(error=error,completed_steps=len(saved),minimum_guard_clearance_m=min((min(v['motion_guard']['predicted_clearance'].values()) for v in saved),default=-1))
  if name=='native':
   expected=np.array([[[h[key] for key in ['x','y','yaw']] for h in v['humans']] for v in original]);actual=np.array([[[h[key] for key in ['x','y','yaw']] for h in v['humans']] for v in saved]);result['exact_poses']=expected.shape==actual.shape and bool(np.array_equal(expected,actual));result['exact_groups']=[v['groups'] for v in original]==[v['groups'] for v in saved];result['native_dispersal_exercised']=any(len(v['groups'])!=2 for v in saved);result['clock_pass'],result['clock_metrics']=verify_trace(saved,gate,ready)
  else:result['clock_pass'],result['clock_metrics']=verify_snapshots(saved,gate,ready);result['lifecycle_pass'],result['lifecycle_metrics']=verify_conversation(saved)
  results[name]=result;print(name,json.dumps(result),flush=True)
 okay=results['native'].get('exact_poses') and results['native'].get('exact_groups') and results['native'].get('native_dispersal_exercised') and results['persistent_conversation_v1'].get('lifecycle_pass') and all(not v['error'] and v['completed_steps']==len(original) and v['clock_pass'] and v['minimum_guard_clearance_m']>0 for v in results.values());receipt=dict(status='PASS' if okay else 'FAIL',profiles=results,scope='120 measured ego inputs, not native closed-loop evidence.')
 (a.output/'results.json').write_text(json.dumps(receipt,indent=2)+'\n');return 0 if okay else 1

if __name__=='__main__':raise SystemExit(main())
