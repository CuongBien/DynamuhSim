"""Paired native-kernel executions with different synthetic host-time schedules.

This is component determinism verification with recorded ego inputs, not a
Gazebo/closed-loop outcome or replacement for the native capture cohort.
"""
import argparse,copy,json,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'experiments/social_navigation/navigation/socnavgym_group_nav2')]
from fixture import make_kernel
from dynamuhsim.simulation.encounters import fixture,initialize
from dynamuhsim.simulation.human_groups import enable_profile,PROFILE
from dynamuhsim.simulation.goal_clock import GoalClock,verify_trace
import socnavgym.envs.utils.human as native_human

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);source=ROOT/'experiments/social_navigation/benchmark/generated/encounter_regression_20261029';results=[];original=native_human.pytime
 try:
  for scenario in ['head_on','crossing']:
   for variant in ['legacy','group']:
    name=scenario+'_'+variant;trace=[json.loads(s) for s in (source/name/'runtime/worker_trajectory.jsonl').read_text().splitlines() if json.loads(s).get('active_phase')][:60];gate=float((source/name/'runtime/start_gate').read_text());executions={};records={}
    for schedule in ['fast_host','slow_host']:
     host=[1000. if schedule=='fast_host' else 1000000.];native_human.pytime=SimpleNamespace(time=lambda:host[0]);k=make_kernel('walking',45)
     try:
      initialize(k,fixture(scenario))
      if variant=='group':enable_profile(k,PROFILE)
      clock=GoalClock(k);ready=copy.deepcopy(clock.snapshot());poses=[];rows=[]
      for request in trace:
       host[0]+=.01 if schedule=='fast_host' else 100.
       t=request['time'];clock.begin_step(t,gate);k.advance(request['robot_input']);clock.end_step(t+.1);humans=k.humans();poses.append([[h[key] for key in ['x','y','yaw']] for h in humans]);rows.append(dict(time=t,state_time=t+.1,active_phase=True,humans=humans,goal_clock=copy.deepcopy(clock.snapshot())))
      okay,metrics=verify_trace(rows,gate,ready)
      if not okay:raise RuntimeError('Clock trace failed native preflight')
      executions[schedule]=np.array(poses);records[schedule]=dict(clock_trace=metrics,host_start_s=1000. if schedule=='fast_host' else 1000000.,host_end_s=host[0],steps=len(rows),first_active_goal_age_sim_s=rows[0]['goal_clock']['humans']['1']['age_sim_s'])
      with (a.output/(name+'_'+schedule+'.jsonl')).open('x') as f:
       for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')
     finally:k.close()
    equal=bool(np.array_equal(executions['fast_host'],executions['slow_host']));max_error=float(np.abs(executions['fast_host']-executions['slow_host']).max());results.append(dict(id=name,identical_human_xy_yaw=equal,max_abs_mixed_pose_difference=max_error,schedules=records));print(name,equal,max_error,flush=True)
 finally:native_human.pytime=original
 receipt=dict(schema='sim_goal_clock_preflight/1',status='PASS' if all(v['identical_human_xy_yaw'] for v in results) else 'FAIL',cases=results,scope='60 native-kernel steps per cell, same measured ego inputs and RNG seed; wall-clock schedules deliberately differ. No physics/closed-loop/naturalness acceptance.')
 (a.output/'results.json').write_text(json.dumps(receipt,indent=2)+'\n')
 if receipt['status']!='PASS':raise SystemExit(1)
if __name__=='__main__':main()
