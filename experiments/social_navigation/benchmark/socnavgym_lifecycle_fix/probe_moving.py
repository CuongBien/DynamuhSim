#!/usr/bin/env python3
"""Separate regression: an initially moving native group plus9 independent people."""
import argparse,json,math,os,random,hashlib
from pathlib import Path
os.environ['SDL_VIDEODRIVER']='dummy';os.environ['SDL_AUDIODRIVER']='dummy'
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
import cv2,gymnasium as gym,numpy as np,socnavgym,yaml
from PIL import Image
from continuous_groups import ContinuousGroups,people
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();out=a.report_dir.resolve()/'moving_regression';out.mkdir(exist_ok=False)
config=yaml.safe_load((a.report_dir/'formation.yaml').read_text());config['env'].update(min_dynamic_humans=9,max_dynamic_humans=9,min_h_h_dynamic_interactions=1,max_h_h_dynamic_interactions=1)
(out/'config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
(out/'protocol.json').write_text(json.dumps(dict(seed=44,steps=600,config_overrides='9 independent +1 moving group of3; native generator, no manual events',source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),adapter_sha256=hashlib.sha256(Path(__file__).with_name('continuous_groups.py').read_bytes()).hexdigest()),indent=2))
random.seed(44);np.random.seed(44);env=gym.make('SocNavGym-v2',config=str(out/'config.yaml'));env.action_space.seed(44);e=env.unwrapped;fix=ContinuousGroups(e);calls=[];rows=[];frames=[]
original=e.disperse_moving_crowd
def disperse(*args,**kwargs):calls.append(dict(step=fix.tick,members_before=[int(h.id) for h in e.moving_interactions[args[0]].humans]));return original(*args,**kwargs)
e.disperse_moving_crowd=disperse
result=dict(status='ERROR',seed=44,planned_steps=600,scope='Additional native-generator initial-moving-group fixture, not paired with old formation initial scene')
def state(step):
    return dict(step=step,humans=[dict(id=int(h.id),x=float(h.x),y=float(h.y),yaw=float(h.orientation)) for h in people(e)],groups=[dict(kind=k,members=sorted(int(h.id) for h in g.humans)) for k,gs in [('moving',e.moving_interactions),('static',e.static_interactions)] for g in gs])
def render(step):
    frame=e.render_without_showing('human',draw_human_gaze=True,draw_human_goal=False).copy()
    cv2.putText(frame,'ADAPTER FIX | MOVING GROUP | NOT GAZEBO',(15,40),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),2)
    cv2.putText(frame,f'seed44 | t={step*.1:.1f}s',(15,67),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),1)
    return frame
try:
    env.reset(seed=44);assert len(e.moving_interactions)==1
    initial_group=e.moving_interactions[0];prev=(initial_group.x,initial_group.y);group_path=0.
    rows.append(state(0));frames.append(render(0))
    for step in range(1,601):
        env.step(np.zeros(3,dtype=np.float32));s=state(step);rows.append(s)
        if any(initial_group is g for g in e.moving_interactions):
            center=(initial_group.x,initial_group.y);group_path+=math.dist(prev,center);prev=center
        assert len(s['humans'])==len(set(h['id'] for h in s['humans']))==12
        if step%10==0:frames.append(render(step))
    assert group_path>1.,'Moving group did not travel1m'
    assert calls,'No moving-group dispersal observed'
    result.update(status='PASS',steps=600,initial_group_center_path_m=group_path,moving_dispersals=calls,robot_radius=float(e.robot.radius))
finally:
    env.close();(out/'trajectory.json').write_text(json.dumps(rows));(out/'repair_steps.json').write_text(json.dumps(fix.steps));(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    if frames:
        images=[Image.fromarray(cv2.cvtColor(f,cv2.COLOR_BGR2RGB)).quantize(colors=96) for f in frames[:-1]]
        images[0].save(out/'moving_preview_4x.gif',save_all=True,append_images=images[1:],duration=250,loop=0)
print(json.dumps(result))
