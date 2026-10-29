"""Replay a bounded measured ego trace into human kernels; never overwrite output."""
import argparse, json, math, sys, time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'experiments/social_navigation/navigation/socnavgym_group_nav2')]
from fixture import make_kernel
from dynamuhsim.simulation.human_groups import enable_profile,PROFILE

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 plan=json.loads((Path(__file__).parent/'plan.json').read_text());source=ROOT/plan['offline']['source'];trace=[json.loads(s) for s in source.read_text().splitlines() if json.loads(s).get('active_phase')][:plan['offline']['active_steps']]
 results={}
 for profile in plan['offline']['conditions']:
  k=make_kernel('walking',plan['seed']);adapter=enable_profile(k,PROFILE) if profile!='legacy' else None;rows=[];start=time.monotonic()
  try:
   with (a.output/(profile+'.jsonl')).open('x') as f:
    for i,v in enumerate(trace):
     k.advance(v['robot_input']);row=dict(time=(i+1)*.1,humans=k.humans(),groups=k.groups(),motion_guard=k.motion.steps[-1]);row.update(group_orca=adapter.last) if adapter else None;rows.append(row);f.write(json.dumps(row,allow_nan=False)+'\n')
  finally:k.close()
  yaw=np.array([[h['yaw'] for h in v['humans']] for v in rows]);xy=np.array([[[h['x'],h['y']] for h in v['humans']] for v in rows]);turn=np.abs(np.diff(np.unwrap(yaw,axis=0),axis=0)).sum(axis=0);length=np.linalg.norm(np.diff(xy,axis=0),axis=2).sum(axis=0)
  ref=np.array([[[h['x'],h['y'],h['yaw']] for h in v['humans']] for v in trace]);cur=np.dstack([xy,yaw]);rmse=float(np.sqrt(np.mean((ref-cur)**2)))
  results[profile]=dict(steps=len(rows),wall_s=time.monotonic()-start,yaw_total_rad=turn.tolist(),path_m=length.tolist(),yaw_rad_per_m=(turn/length).tolist(),reference_pose_rmse=rmse)
  print(profile,json.dumps(results[profile]),flush=True)
 results['aggregate_yaw_reduction_fraction']=1-sum(results['group_orca_v1']['yaw_total_rad'])/sum(results['legacy']['yaw_total_rad']);results['screen_pass']=results['aggregate_yaw_reduction_fraction']>=.5
 (a.output/'metrics.json').write_text(json.dumps(results,indent=2)+'\n')
if __name__=='__main__':main()
