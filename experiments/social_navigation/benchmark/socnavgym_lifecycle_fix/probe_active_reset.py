#!/usr/bin/env python3
"""Reset while a real native group exists; then compare500 accepted fixed steps."""
import argparse,json,os,random
from pathlib import Path
os.environ['SDL_VIDEODRIVER']='dummy';os.environ['SDL_AUDIODRIVER']='dummy'
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
import gymnasium as gym,numpy as np,socnavgym
from continuous_groups import ContinuousGroups,people
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();r=a.report_dir.resolve()
random.seed(42);np.random.seed(42);env=gym.make('SocNavGym-v2',config=str(r/'formation.yaml'));env.action_space.seed(42);e=env.unwrapped;fix=ContinuousGroups(e)
reference=json.loads((r/'seed42/trajectory.json').read_text())
def snapshot():
    return dict(humans=[dict(id=int(h.id),x=float(h.x),y=float(h.y),yaw=float(h.orientation),speed=float(h.speed)) for h in people(e)],
                groups=sorted([(kind,sorted(int(h.id) for h in g.humans)) for kind,gs in [('moving',e.moving_interactions),('static',e.static_interactions)] for g in gs]),forming=bool(e.crowd_forming))
output={'status':'ERROR','scope':'Supplemental reset of an active group; no forced formation event'}
try:
    env.reset(seed=42)
    for step in range(1,1201):
        env.step(np.zeros(3,dtype=np.float32))
        if e.moving_interactions or e.static_interactions:break
    before=snapshot();assert before['groups'];old={int(h.id):h for h in people(e)}
    env.reset(seed=42);initial=snapshot()
    assert not initial['groups'] and not initial['forming'] and fix.tick==0 and fix.velocities=={} and fix.epoch==1
    assert all(h is not old[int(h.id)] for h in people(e))
    max_error=0.;membership=True
    for i in range(1,501):
        env.step(np.zeros(3,dtype=np.float32));now=snapshot();ref=reference[i]
        max_error=max(max_error,max(abs(h[k]-ref['humans'][j][k]) for j,h in enumerate(now['humans']) for k in ['x','y','yaw','speed']))
        wanted=sorted([(g['kind'],g['members']) for g in ref['groups']]);membership=membership and now['groups']==wanted and now['forming']==ref['forming']
    assert max_error<1e-9 and membership
    output.update(status='PASS',active_group_step=step,groups_before=before['groups'],groups_after=initial['groups'],repeat_steps=500,max_state_error=max_error,membership_match=membership,new_human_objects=True,old_adapter_velocities_cleared=True,total_additional_steps=step+500)
finally:
    env.close();(r/'active_reset_check.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(output))
