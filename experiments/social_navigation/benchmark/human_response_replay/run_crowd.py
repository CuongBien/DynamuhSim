#!/usr/bin/env python3
"""Pinned-container offline probes. No ROS/Gazebo or robot control is started."""
import copy,hashlib,json,math,random,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,'/adapter')
from kernel import CrowdKernel
from crowd_sim_plus.envs.utils.action import ActionXY
R=Path('/runtime');scene=json.loads((R/'inputs/scene.json').read_text());layout=json.loads((R/'inputs/layout.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def seed():random.seed(42);np.random.seed(42)
def state_error(left,right):
    return max(abs(getattr(h,key)-getattr(j,key)) for h,j in zip(left.env.humans,right.env.humans) for key in ['px','py','vx','vy','gx','gy','final_gx','final_gy'])
results=[]
for policy in ['orca_plus','sfm']:
    for case in ['far','head_on','crossing']:
        inputs=R/'inputs'/f'{case}.jsonl';samples=[json.loads(line) for line in inputs.open()];fixture=copy.deepcopy(scene)
        fixture['robot_start']=[samples[0]['robot'][key] for key in ['x','y','yaw']]
        seed();model=CrowdKernel(fixture,layout,human_policy=policy)
        seed();reference=CrowdKernel(fixture,layout,human_policy=policy)
        humans,robot,obstacles=reference.env.humans,reference.env.robot,reference.env.static_obstacles
        reference.env.reset('test',0,return_stat=True)
        reference.env.humans=humans;reference.env.robot=robot;reference.env.static_obstacles=obstacles;reference.env.sim_env='social_warehouse'
        directory=R/'crowdsimplus'/policy/case;directory.mkdir(parents=True,exist_ok=False)
        rows=[];errors=[];robot_errors=[];parity_turns=0;seed();begin=time.monotonic()
        for i,(row,next_row) in enumerate(zip(samples,samples[1:])):
            before=model.humans();rp=row['robot'];nxt=next_row['robot']
            reference.env.robot.set(rp['x'],rp['y'],*reference.robot_goal[:2],rp['vx'],rp['vy'],rp['yaw'])
            rng=(random.getstate(),np.random.get_state())
            model.advance(rp);after_rng=(random.getstate(),np.random.get_state())
            random.setstate(rng[0]);np.random.set_state(rng[1])
            reference.env.step(ActionXY(rp['vx'],rp['vy']))
            # The full native env has no external patrol. Apply the same endpoint
            # switch only after native integration, then compare the full state.
            for index,h in enumerate(reference.env.humans):
                if h.reached_destination():
                    reference.arrivals[index]+=1;reference.targets[index]=1-reference.targets[index]
                    h.final_gx,h.final_gy=reference.routes[index][reference.targets[index]];h.set_g_xy(h.px,h.py);parity_turns+=1
            random.setstate(after_rng[0]);np.random.set_state(after_rng[1])
            errors.append(state_error(model,reference))
            robot_errors.append(math.hypot(reference.env.robot.px-nxt['x'],reference.env.robot.py-nxt['y']))
            rows.append(dict(input_time=row['time'],state_time=next_row['time'],robot_input=rp,robot_next=nxt,human_before=before,humans=model.humans(),actions=model.last_actions))
        assert max(errors)<1e-10,(policy,case,max(errors));assert max(robot_errors)<1e-9,(policy,case,max(robot_errors))
        # Exact second rollout is a determinism check, not a replacement trial.
        seed();repeat=CrowdKernel(fixture,layout,human_policy=policy);seed();repeat_errors=[]
        for row,expected in zip(samples,rows):
            repeat.advance(row['robot'])
            repeat_errors.extend(abs(a[key]-b[key]) for a,b in zip(repeat.humans(),expected['humans']) for key in ['x','y','vx','vy','yaw','speed'])
        assert max(repeat_errors)<1e-10,(policy,case,max(repeat_errors))
        (directory/'trajectory.jsonl').write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in rows))
        metadata=model.metadata()
        assert metadata['source_commit']=='d526a07c013c621fa1ad50894b6ba639f61063de'
        with (directory/'native.config').open('w') as stream:model.config.write(stream)
        result=dict(framework='CrowdSimPlus',policy=policy,case=case,status='PASS',input_sha256=sha(inputs),samples=len(rows),duration_sim_s=20.,wall_s=time.monotonic()-begin,native_full_step_max_state_error=max(errors),native_robot_next_position_error_max_m=max(robot_errors),external_patrol_switches_in_reference=parity_turns,determinism_max_error=max(repeat_errors),metadata=metadata,execution_context='Offline prescribed robot trace. The metadata robot_controller label describes the reusable Gazebo kernel, not this run; no ROS/Nav2/Gazebo was started.',parity_scope='Full upstream env.step human integration, with identical external post-step patrol endpoint changes. Not equivalence of upstream rewards/episode loop or robot planning.')
        (directory/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False));results.append(result)
        print(policy,case,'PASS','parity',max(errors),'repeat',max(repeat_errors),flush=True)
(R/'crowdsimplus_results.json').write_text(json.dumps(results,indent=2));print('Completed six measured probes; native parity and second-rollout checks passed',flush=True)
