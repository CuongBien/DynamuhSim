"""Recorded-input native controller replay; never injects final human poses."""
import argparse,copy,json,sys,hashlib,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--experiment',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--profile',default='auto');p.add_argument('--input-run',type=Path);a=p.parse_args()
root=Path(__file__).resolve().parents[1];nav=root/'experiments/social_navigation/navigation';sys.path[:0]=[str(root/'python'),str(nav/'socnavgym_group_nav2')]
from fixture import make_kernel
from dynamuhsim.simulation.encounters import load_fixture
from dynamuhsim.simulation.goal_clock import GoalClock
from dynamuhsim.simulation.human_motion_profiles import enable_profile,profile_config
b=a.experiment.resolve();run=(a.input_run or b/'baseline/native').resolve();plan=json.loads((run/'plan.json').read_text());source=run/'runtime';a.profile=plan.get('human_motion_profile','legacy') if a.profile=='auto' else a.profile;rows=[json.loads(l) for l in (source/'worker_trajectory.jsonl').read_text().splitlines() if json.loads(l).get('active_phase')]
gate=float((source/'start_gate').read_text());k=make_kernel(plan['scenario'],plan['seed']);load_fixture(k,source)
controller=enable_profile(k,profile_config(a.profile)) if a.profile!='legacy' else None
clock=GoalClock(k)
from dynamuhsim.simulation.group_diagnostics import GroupDiagnostics
diag=GroupDiagnostics(k) if controller is None else None
initial=k.humans();assert initial==rows[0]['before'],'Initial-state mismatch'
a.output.parent.mkdir(parents=True,exist_ok=True);errors=[];heading=[];events_match=True
with a.output.open('x') as f:
 for row in rows:
  t=row['time'];clock.begin_step(t,gate);before=copy.deepcopy(k.humans());k.advance(row['robot_input']);clock.end_step(t+.1)
  groups=copy.deepcopy(k.groups());goals={int(h.id):[float(h.goal_x),float(h.goal_y)] for h in clock.kernel.motion.env.dynamic_humans}
  from dynamuhsim.simulation.goal_clock import people
  goals={int(h.id):[float(h.goal_x),float(h.goal_y)] for h in people(k)}
  for g in groups:
   members=[h for h in k.humans() if h['id'] in g['members']];g['goal']=goals[g['members'][0]] if g['type']=='moving' else None;g['envelope_m']=max(math.hypot(h['x']-g['center'][0],h['y']-g['center'][1])+h['radius'] for h in members)
  out=dict(time=t,state_time=t+.1,active_phase=True,before=before,humans=k.humans(),groups=groups,robot_input=row['robot_input'],motion_guard=copy.deepcopy(k.motion.steps[-1]),goal_clock=clock.snapshot(),goals=goals)
  if diag:out['native_group_intent']=copy.deepcopy(diag.last)
  if controller:out['behavior']=copy.deepcopy(controller.snapshot());out['group_intent']=copy.deepcopy(controller.last)
  f.write(json.dumps(out,allow_nan=False)+'\n')
  for h,z in zip(out['humans'],row['humans']):errors.append(math.hypot(h['x']-z['x'],h['y']-z['y']));heading.append(abs(math.atan2(math.sin(h['yaw']-z['yaw']),math.cos(h['yaw']-z['yaw']))))
  events_match &= out['goal_clock']==row['goal_clock']
k.close()
receipt=dict(mode='DETERMINISTIC_CONTROLLER_REEXECUTION',profile=a.profile,scenario=plan['scenario']+'/'+plan.get('encounter_profile','default'),seed=plan['seed'],dt_s=.1,steps=len(rows),source=str(source/'worker_trajectory.jsonl'),source_sha256=hashlib.sha256((source/'worker_trajectory.jsonl').read_bytes()).hexdigest(),output_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),start_time_sim_s=rows[0]['time'],end_state_time_sim_s=rows[-1]['state_time'],initial_state_equal=True,baseline_max_xy_error_m=max(errors),baseline_max_yaw_error_rad=max(heading),baseline_goal_clock_events_equal=events_match,comparison_to_baseline_is_reproduction=a.profile==plan.get('human_motion_profile','legacy'),reference_run=str(run))
with a.output.with_suffix('.receipt.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps(receipt,indent=2))
