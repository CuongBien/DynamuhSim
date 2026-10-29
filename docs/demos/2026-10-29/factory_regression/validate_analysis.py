"""Known triangle/orientation-speed oracle for the read-only social diagnostic."""
from pathlib import Path
import importlib.util,json,tempfile
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('factory_analysis',HERE/'analyze.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
initial=[dict(id=1,x=0.,y=0.),dict(id=2,x=1.,y=0.),dict(id=3,x=0.,y=1.)];data=[];gt=[]
for t,robot,speed in [(0.,(2.,2.),0.),(.02,(.2,.2),0.),(.04,(.2,.2),.1)]:
 poses={f'common_{h["id"]}_proxy':dict(x=h['x'],y=h['y'],yaw=0.) for h in initial};poses['mecanum_robot']=dict(x=robot[0],y=robot[1],yaw=0.);data.append(dict(time=t,poses=poses));gt.append(dict(time=t,humans=[dict(h,speed=speed if h['id']==1 else 0.) for h in initial],groups=[dict(type='standing',members=[1,2,3],id=1)]))
with tempfile.TemporaryDirectory() as tmp:
 run=Path(tmp);(run/'episode').mkdir();(run/'episode/ground_truth.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in gt));out=m.social_metrics(run,data,{'ready':{'humans':initial}});assert out['robot_center_entry_samples']==2 and out['speed_qualified_entry_samples']==1;assert abs(out['any_entry_duration_approx_s']-.04)<1e-8
print('PASS: outside/inside triangle, speed qualification and sample duration')
