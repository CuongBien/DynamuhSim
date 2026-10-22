#!/usr/bin/env python3
"""One backend at a time; shared inputs, no ROS/Nav2 or synthetic camera claims."""
import argparse,copy,hashlib,json,math,os,random,sys,time,traceback
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
try:os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
except AttributeError:pass
import numpy as np
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--model',required=True);p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args()
protocol=json.loads((HERE/'inputs/protocol.json').read_text());model=protocol['models'][a.model];dt=model['dt'];layout=json.loads((HERE/'inputs/layout.json').read_text());out=a.report_dir;out.mkdir(parents=True,exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

class Runner:
    def __init__(self,case):
        random.seed(42);np.random.seed(42);self.case=case;self.n=0;self.previous=None;self.kind=a.model
        if self.kind.startswith('soc_'):
            social=HERE.parent.parent
            sys.path.insert(0,str(social/'benchmark/socnavgym_warehouse_lifecycle'))
            from kernel import Kernel
            from motion import people
            class WorldKernel(Kernel):origin=(0.,0.)
            self.k=WorldKernel(HERE/'inputs/socnavgym.yaml',layout,seed=42);self.people=people
            for h,f in zip(people(self.k.env),case['humans']):
                h.x=h.initial_x=f['x'];h.y=h.initial_y=f['y'];h.orientation=h.initial_orientation=f['yaw'];h.speed=0.;h.position_history.clear()
                h.set_goal(f['gx'],f['gy']);self.k.env.goals[h.id].x=f['gx'];self.k.env.goals[h.id].y=f['gy']
            if self.kind=='soc_native_orca':
                for name,method in self.k.motion.original.items():setattr(self.k.env,name,method)
            self.meta=dict(dt=dt,static_boxes=len(self.k.env.walls),motion_ceiling=.8,initial_groups=self.k.groups(),native_motion=self.kind=='soc_native_orca')
        elif self.kind.startswith('crowd_'):
            sys.path.insert(0,str(HERE/'generated'));from crowd_kernel_n6 import CrowdKernel
            state=[dict(id=h['id'],x=h['x'],y=h['y'],yaw=h['yaw']) for h in case['humans']]
            scene=dict(condition='benchmark',goal_layout='individual',initial=state,human_routes={str(h['id']):[[h['gx'],h['gy']],[h['x'],h['y']]] for h in case['humans']},robot_start=[case['robot_trace'][0]['x'],case['robot_trace'][0]['y'],0.],robot_goal=[28.,18.,0.])
            self.k=CrowdKernel(scene,layout,True,'sfm' if self.kind=='crowd_sfm' else 'orca_plus');self.meta=self.k.metadata();self.meta['robot_controller']='prescribed shared trace, no ROS/Nav2'
        else:
            sys.path.insert(0,'/geometry')
            from kernel import GeometryKernel
            from native_contract import ContractKernel
            from roadmap import Roadmap
            humans=[dict(h,policy='orca' if i<3 else 'socialforce') for i,h in enumerate(case['humans'])]
            self.k=GeometryKernel(layout,humans) if self.kind=='complex_geometry_mixed' else ContractKernel(humans)
            for h,f in zip(self.k.env.humans,case['humans']):h.radius=.36;h.theta=f['yaw']
            if self.kind=='complex_geometry_mixed':
                self.k.roadmap=Roadmap(layout,.36);self.k.projector.roadmap=self.k.roadmap;self.k.projector.max_speed=.8
            self.meta=dict(dt=dt,static_boxes=42 if self.kind=='complex_geometry_mixed' else 0,interior_boxes_used=self.kind=='complex_geometry_mixed',radius_override=.36,motion_ceiling=.8 if self.kind=='complex_geometry_mixed' else None,native_orca_intent_max_speed=.5,not_trained_RGL=True)
        import inspect
        module=__import__(type(self.k.env).__module__,fromlist=['_'])
        self.meta['env_module_file']=inspect.getfile(module);self.meta['env_module_sha256']=sha(Path(inspect.getfile(module)))
    def state(self):
        if self.kind.startswith('soc_'):
            hs=[dict(id=int(h.id),x=float(h.x),y=float(h.y),radius=float(h.width/2),native_goal=[float(h.goal_x),float(h.goal_y)]) for h in self.people(self.k.env)]
        elif self.kind.startswith('crowd_'):hs=[dict(id=h['id'],x=h['x'],y=h['y'],radius=h['radius'],native_goal=h['goal']) for h in self.k.humans()]
        else:hs=[dict(id=h['id'],x=h['x'],y=h['y'],radius=h['radius'],native_goal=[h['gx'],h['gy']]) for h in self.k.state()['humans']]
        for h in hs:
            old=next((v for v in self.previous if v['id']==h['id']),h) if self.previous else h
            h['vx']=(h['x']-old['x'])/dt;h['vy']=(h['y']-old['y'])/dt
        return dict(time=self.n*dt,step=self.n,humans=hs)
    def advance(self,robot):
        self.previous=copy.deepcopy(self.state()['humans'])
        if self.kind.startswith(('soc_','crowd_')):self.k.advance(robot)
        else:self.k.request(dict(time=self.k.time,epoch=self.k.epoch,frame='world',advance=True,robot=robot))
        self.n+=1;row=self.state();row['robot_input']=robot
        if self.kind=='soc_guarded_orca':row['guard']=self.k.motion.steps[-1]
        elif self.kind=='complex_geometry_mixed':row['guard']=self.k.projector.log[-1]
        return row
    def close(self):
        if self.kind.startswith('soc_'):self.k.close()
        elif hasattr(self.k,'env'):self.k.env.close()

results=[];first_rows=None
for name in protocol['fixtures']:
    case=json.loads((HERE/'inputs'/(name+'.json')).read_text());rows=[];runner=None;start=time.monotonic();error=None
    try:
        runner=Runner(case);rows=[runner.state()]
        for i in range(round(case['seconds']/dt)):
            robot=case['robot_trace'][round(i*dt/.05)].copy();robot.pop('time');rows.append(runner.advance(robot))
        status='COMPLETED'
    except Exception as exc:
        status='ERROR';error=repr(exc);(out/(name+'_error.txt')).write_text(traceback.format_exc())
        if runner is not None:
            try:(out/(name+'_failed_state.json')).write_text(json.dumps(runner.state(),allow_nan=False))
            except Exception:pass
    finally:
        (out/(name+'_trajectory.json')).write_text(json.dumps(rows,allow_nan=False)+'\n')
        result=dict(case=name,model=a.model,status=status,steps=max(0,len(rows)-1),required_steps=round(case['seconds']/dt),wall_seconds=time.monotonic()-start,error=error,metadata=runner.meta if runner else None)
        (out/(name+'_result.json')).write_text(json.dumps(result,indent=2)+'\n');results.append(result);print(json.dumps({k:v for k,v in result.items() if k!='metadata'}),flush=True)
        if name==protocol['fixtures'][0]:first_rows=rows
        if runner:runner.close()
# Same-seed exact20-step prefix, separate from behavior/geometry acceptance.
repeat=dict(status='ERROR')
try:
    case=json.loads((HERE/'inputs'/(protocol['fixtures'][0]+'.json')).read_text());runner=Runner(case);rows=[runner.state()]
    for i in range(20):
        robot=case['robot_trace'][round(i*dt/.05)].copy();robot.pop('time');rows.append(runner.advance(robot))
    assert len(first_rows)>=21 and rows==first_rows[:21]
    repeat=dict(status='PASS',steps=20);(out/'repeat_prefix.json').write_text(json.dumps(rows,allow_nan=False)+'\n');runner.close()
except Exception as exc:repeat=dict(status='ERROR',error=repr(exc));(out/'repeat_error.txt').write_text(traceback.format_exc())
(out/'summary.json').write_text(json.dumps(dict(model=a.model,cases=results,repeat=repeat,worker_sha256=sha(Path(__file__))),indent=2)+'\n')
