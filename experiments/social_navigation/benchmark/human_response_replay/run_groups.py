#!/usr/bin/env python3
"""Offline SocNavGym group ablations on the same prescribed robot input files."""
import argparse,copy,hashlib,json,math,random,sys,time
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent
sys.path.insert(0,str(SOCIAL/'gazebo_adapter'));sys.path.insert(0,str(BENCH))
from kernel import GroupKernel
from circle_contact import CircleContact,geometry_description
from robot_orca import RobotAwareGroups,behavior_description
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();R=a.report_dir.resolve()
layout=json.loads((R/'inputs/layout.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
policies=['native','circle_contact','robot_orca'];cases=['far','head_on','crossing'];results=[]
def build(config,mode,scene,variant):
    k=GroupKernel(config,layout,mode,42);original_radius=k.env.robot.radius
    contact=CircleContact(k,.25) if variant!='native' else None
    avoidance=RobotAwareGroups(k) if variant=='robot_orca' else None
    for h,state in zip(k.group.humans,scene['initial']):
        h.x=h.initial_x=state['x'];h.y=h.initial_y=state['y'];h.orientation=h.initial_orientation=state['yaw'];h.position_history.clear()
    k.group.x=sum(h.x for h in k.group.humans)/3;k.group.y=sum(h.y for h in k.group.humans)/3
    k.route=scene['group_route'];k.group.set_goal(*k.route[0]);k.env.robot.goal_x,k.env.robot.goal_y=scene['robot_goal'][:2]
    return k,contact,avoidance,original_radius
for mode,condition in [('walking','counterflow'),('conversation','standing')]:
    data=R/'group_inputs'/condition;config=data/'config.yaml';scene=json.loads((data/'scene.json').read_text())
    for variant in policies:
        for case in cases:
            samples=[json.loads(line) for line in (R/'inputs'/f'{case}.jsonl').open()]
            directory=R/'socnavgym'/mode/variant/case;directory.mkdir(parents=True,exist_ok=False)
            k,contact,avoidance,original_radius=build(config,mode,scene,variant);trace=[];begin=time.monotonic()
            try:
                metadata=dict(geometry=geometry_description(k,'native' if variant=='native' else 'circle_contact',original_radius),behavior=behavior_description(k,variant=='robot_orca'),execution_context='OFFLINE prescribed robot trace; metadata Gazebo/Nav2 labels describe reusable adapter only, not this run.',initial_humans=k.humans(),native_observation_in_space=k.initial_observation_in_space)
                for row,nxt in zip(samples,samples[1:]):
                    before=k.humans();rp=row['robot']
                    if contact:contact.begin_step()
                    k.advance(rp)
                    trace.append(dict(input_time=row['time'],state_time=nxt['time'],robot_input=rp,robot_next=nxt['robot'],human_before=before,humans=k.humans(),groups=[dict(id=1,members=[1,2,3],type=mode)],contact_ids=sorted(contact.contact_ids) if contact else None,orca=copy.deepcopy(avoidance.last) if avoidance else None))
                (directory/'trajectory.jsonl').write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in trace))
            finally:k.close()
            # Rebuild a separate environment with the same seed for deterministic replay.
            repeat,other_contact,other_avoidance,_=build(config,mode,scene,variant);errors=[]
            try:
                for row,expected in zip(samples,trace):
                    if other_contact:other_contact.begin_step()
                    repeat.advance(row['robot'])
                    errors.extend(abs(h[key]-j[key]) for h,j in zip(repeat.humans(),expected['humans']) for key in ['x','y','vx','vy','yaw','speed'])
            finally:repeat.close()
            assert max(errors)<1e-10,(mode,variant,case,max(errors))
            result=dict(framework='SocNavGym',mode=mode,condition=condition,variant=variant,case=case,status='PASS',samples=len(trace),duration_sim_s=20.,input_sha256=sha(R/'inputs'/f'{case}.jsonl'),scene_sha256=sha(data/'scene.json'),config_sha256=sha(config),determinism_max_error=max(errors),wall_s=time.monotonic()-begin,metadata=metadata,scope='One persistent group3; externally assigned route. No formation/dissolution or speech/dialogue model. Native and modified adapter variants distinguished.')
            (directory/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False));results.append(result)
            print(mode,variant,case,'PASS',flush=True)
(R/'socnavgym_results.json').write_text(json.dumps(results,indent=2));print('18 group probes completed',flush=True)
