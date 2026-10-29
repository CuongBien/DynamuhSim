"""Run paired group proposals on measured ego inputs, preserving every result."""
import argparse,copy,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'experiments/social_navigation/navigation/socnavgym_group_nav2')]
from fixture import make_kernel
from dynamuhsim.simulation.encounters import fixture,initialize
from dynamuhsim.simulation.human_motion_profiles import enable_profile,profile_config
from dynamuhsim.simulation.goal_clock import GoalClock,verify_trace
from dynamuhsim.simulation.group_cohesion_heading import verify_trace as verify_cohesion_trace

def stats(rows):
 xy=np.array([[[h[k] for k in ['x','y']] for h in v['humans']] for v in rows]);yaw=np.array([[h['yaw'] for h in v['humans']] for v in rows]);length=np.linalg.norm(np.diff(xy,axis=0),axis=2).sum(axis=0);turn=np.abs(np.diff(np.unwrap(yaw,axis=0),axis=0)).sum(axis=0);radius=[];heading_error=[]
 for v in rows:
  humans={h['id']:h for h in v['humans']}
  for g in v['groups']:
   if g['type']!='moving':continue
   points=np.array([[humans[hid]['x'],humans[hid]['y']] for hid in g['members']]);radius.append(float(np.linalg.norm(points-points.mean(axis=0),axis=1).max()))
  for h in v['humans']:
   if np.hypot(h['vx'],h['vy'])>.05:heading_error.append(abs(np.arctan2(np.sin(h['yaw']-np.arctan2(h['vy'],h['vx'])),np.cos(h['yaw']-np.arctan2(h['vy'],h['vx'])))))
 return dict(steps=len(rows),yaw_total_rad=turn.tolist(),path_m=length.tolist(),max_member_centroid_radius_m=max(radius,default=None),moving_backwards_fraction=float(np.mean(np.array(heading_error)>np.pi/2)) if heading_error else 0.,minimum_guard_clearance_m=min(min(v['motion_guard']['predicted_clearance'].values()) for v in rows))

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());reference=ROOT/plan['component_reference_root'];results=[]
 for case in plan['cases']:
  source=reference/case['component_reference_id'];trace=[json.loads(s) for s in (source/'runtime/worker_trajectory.jsonl').read_text().splitlines() if json.loads(s).get('active_phase')][:plan['preflight']['steps']];gate=float((source/'runtime/start_gate').read_text());metrics={}
  for profile in plan['preflight']['profiles']:
   k=make_kernel('walking',case['seed']);saved=[];error=None
   try:
    initialize(k,fixture(case['encounter_profile']));motion=enable_profile(k,profile_config(profile));clock=GoalClock(k);ready=copy.deepcopy(clock.snapshot())
    with (a.output/(case['id']+'_'+profile+'.jsonl')).open('x') as f:
     for request in trace:
      t=request['time'];clock.begin_step(t,gate);k.advance(request['robot_input']);clock.end_step(t+.1);row=dict(time=t,state_time=t+.1,active_phase=True,humans=k.humans(),groups=k.groups(),motion_guard=k.motion.steps[-1],goal_clock=copy.deepcopy(clock.snapshot()));row['group_orca' if profile=='group_orca_v1' else 'group_intent']=copy.deepcopy(motion.last);row.update(group_cohesion=copy.deepcopy(motion.cohesion_last)) if hasattr(motion,'cohesion_last') else None;saved.append(row);f.write(json.dumps(row,allow_nan=False)+'\n')
   except Exception as e:error=type(e).__name__+': '+str(e)
   finally:k.close()
   result=dict(error=error,completed_steps=len(saved));result.update(stats(saved)) if len(saved)>1 else None
   if saved:result['clock_trace_pass'],result['clock_trace']=verify_trace(saved,gate,ready)
   if profile=='group_cohesion_v2' and saved:result['cohesion_trace_pass'],result['cohesion_trace']=verify_cohesion_trace(saved)
   if profile==plan['preflight']['baseline_profile']:
    expected=np.array([[[h[key] for key in ['x','y','yaw']] for h in v['humans']] for v in trace]);actual=np.array([[[h[key] for key in ['x','y','yaw']] for h in v['humans']] for v in saved]);result['exact_reference_poses']=expected.shape==actual.shape and bool(np.array_equal(expected,actual));result['max_reference_pose_difference']=float(np.abs(expected-actual).max()) if expected.shape==actual.shape else None
   metrics[profile]=result;print(case['id'],profile,json.dumps(result),flush=True)
  old,new=metrics[plan['preflight']['baseline_profile']],metrics[plan['preflight']['candidate_profile']];gate_pass=old.get('exact_reference_poses',False) and all(not v['error'] and v['completed_steps']==len(trace) and v.get('clock_trace_pass') and v['minimum_guard_clearance_m']>0 and v.get('cohesion_trace_pass',True) for v in metrics.values());screen={}
  if gate_pass:
   screen=dict(yaw_reduction_fraction=1-sum(new['yaw_total_rad'])/sum(old['yaw_total_rad']),cohesion_radius_change_m=new['max_member_centroid_radius_m']-old['max_member_centroid_radius_m'],backward_fraction_change=new['moving_backwards_fraction']-old['moving_backwards_fraction']);screen['motion_screen_pass']=screen['yaw_reduction_fraction']>=.5 and screen['cohesion_radius_change_m']<=.25 and screen['backward_fraction_change']<=.05
  results.append(dict(id=case['id'],profiles=metrics,launch_gate_pass=gate_pass,diagnostic_screen=screen))
 receipt=dict(schema='group_cohesion_heading_preflight/1',status='PASS' if all(v['launch_gate_pass'] for v in results) else 'FAIL',cases=results,scope='Same measured ego inputs. Safety/identity gate separate from motion screen; no closed-loop/native-physics/naturalness claim.')
 (a.output/'results.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
 if receipt['status']!='PASS':raise SystemExit(1)
if __name__=='__main__':main()
