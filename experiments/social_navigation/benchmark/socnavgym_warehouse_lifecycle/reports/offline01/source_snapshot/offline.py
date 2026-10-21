#!/usr/bin/env python3
"""Three sequential explicit robot traces; preserve every result before Gazebo."""
import argparse,hashlib,json,math,os,sys,time,traceback
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
os.environ['SDL_VIDEODRIVER']='dummy';os.environ['SDL_AUDIODRIVER']='dummy';os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
from kernel import Kernel
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();out=a.report_dir.resolve();out.mkdir(parents=True,exist_ok=False)
layout=json.loads((SOCIAL/'warehouse/layout.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
(out/'protocol.json').write_text(json.dumps(dict(seed=42,cases=['stationary','forward','lateral_then_forward'],steps=1200,dt=.1,obstacles=42,origin=[-24,0],goal_sampling_window='world x[-27.5,-20.5],y[-1.5,1.5]; no artificial boundary geometry',source_sha256={str(p):sha(p) for p in [HERE/'config.yaml',HERE/'kernel.py',HERE/'motion.py',HERE/'geometry.py',Path(__file__),SOCIAL/'warehouse/layout.json',SOCIAL/'benchmark/socnavgym_lifecycle_fix/continuous_groups.py']}),indent=2))
all_results=[]
for case in ['stationary','forward','lateral_then_forward']:
    print('START',case,flush=True);rows=[];k=None;result=dict(case=case,status='ERROR');started=time.monotonic()
    try:
        k=Kernel(HERE/'config.yaml',layout);initial=k.humans();previous=[]
        for index in range(1200):
            t=index*.1;x=-27.;y=0.;vx=vy=0.
            if case=='forward':x+=.1*min(t,10.);vx=.1 if t<10-1e-8 else 0.
            if case=='lateral_then_forward':
                y=.1*min(t,6.);vy=.1 if t<6-1e-8 else 0.
                x+=.1*max(0.,min(t-6.,6.));vx=.1 if 6-1e-8<=t<12-1e-8 else 0.
            robot=dict(x=x,y=y,yaw=0.,vx=vx,vy=vy,omega=0.);before=k.humans();groups_before=k.groups();k.advance(robot);hs=k.humans();groups=k.groups()
            rows.append(dict(time=t,valid_time=t+.1,robot=robot,before=before,humans=hs,groups_before=groups_before,groups=groups,guard=k.motion.steps[-1]))
        signatures=[[(g['id'],g['type'],g['members']) for g in r['groups']] for r in rows]
        result.update(status='PASS',steps=len(rows),membership_changes=sum(a!=b for a,b in zip([[]]+signatures,signatures)),formations=len(k.motion.events),min_predicted_clearance={key:min(r['guard']['predicted_clearance'][key] for r in rows) for key in ['pair','robot','static']},robot_max_speed=max(math.hypot(r['robot']['vx'],r['robot']['vy']) for r in rows))
        assert result['formations']>0,'No native formation observed in120s'
        assert min(result['min_predicted_clearance'].values())>=.002-1e-8
    except Exception as ex:result.update(status='ERROR',error=str(ex));traceback.print_exc()
    finally:
        result['wall_seconds']=time.monotonic()-started
        (out/(case+'_trajectory.json')).write_text(json.dumps(rows))
        if k:
            (out/(case+'_events.json')).write_text(json.dumps(k.motion.events));k.close()
        (out/(case+'.json')).write_text(json.dumps(result,indent=2));all_results.append(result);print('END',json.dumps(result),flush=True)
(out/'summary.json').write_text(json.dumps(all_results,indent=2));raise SystemExit(0 if all(r['status']=='PASS' for r in all_results) else 1)
