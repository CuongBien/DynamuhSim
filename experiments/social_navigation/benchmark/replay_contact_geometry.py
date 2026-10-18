#!/usr/bin/env python3
"""Open-loop geometry ablation with identical prerecorded robot inputs.
This is not a Gazebo navigation trial; native replay must match its saved trace.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'gazebo_adapter'))
from kernel import GroupKernel
from circle_contact import CircleContact
p=argparse.ArgumentParser();p.add_argument('--trial',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
trial=json.loads(a.trial.read_text());runtime=HERE/trial['runtime'];scene=trial['scene']
source=runtime/'native_trajectory.jsonl';logged=[json.loads(l) for l in source.open()];inputs=[r for r in logged if r['active']]
gate=float((runtime/'start_gate').read_text());layout=json.loads((HERE.parent/'warehouse/layout.json').read_text())
def clear(ro,h):
    dx,dy=h['x']-ro['x'],h['y']-ro['y'];c,s=math.cos(ro['yaw']),math.sin(ro['yaw'])
    qx,qy=abs(c*dx+s*dy)-.12,abs(-s*dx+c*dy)-.1
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
results={};parity_errors=[]
for policy in ['native','circle_contact']:
    k=GroupKernel(runtime/'config.yaml',layout,'walking',trial['seed'])
    contact=CircleContact(k,.25) if policy=='circle_contact' else None
    for h,state in zip(k.group.humans,scene['initial']):
        h.x=h.initial_x=state['x'];h.y=h.initial_y=state['y'];h.orientation=h.initial_orientation=state['yaw'];h.position_history.clear()
    k.group.x=sum(h.x for h in k.group.humans)/3;k.group.y=sum(h.y for h in k.group.humans)/3
    k.route=scene['group_route'];k.group.set_goal(*k.route[0]);k.env.robot.goal_x,k.env.robot.goal_y=scene['robot_goal'][:2]
    trace=[]
    try:
        for row in inputs:
            robot=row['robot'];k.env.robot.x,k.env.robot.y,k.env.robot.orientation=robot['x'],robot['y'],robot['yaw']
            ids=[h.id for h in k.group.humans if h.collides(k.env.robot)]
            if contact:contact.begin_step()
            k.advance(robot);humans=k.humans()
            if policy=='native':
                parity_errors.extend(abs(h[key]-saved[key]) for h,saved in zip(humans,row['humans']) for key in ['x','y','yaw','speed','vx','vy'])
            trace.append(dict(time=row['time'],elapsed=row['time']-gate,robot=robot,humans=humans,
                footprint_clearance=min(clear(robot,h) for h in humans),contact_ids=ids))
        results[policy]=trace
    finally:k.close()
max_error=max(parity_errors);assert max_error<1e-10,'Cannot attribute geometry effect: native trace failed parity'
a.output.mkdir(parents=True,exist_ok=True)
summary=dict(source_trial=str(a.trial),source_robot_inputs_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    active_inputs=len(inputs),native_replay_max_state_error=max_error,robot_inputs_identical=True,
    interpretation='Open-loop logged robot poses, not closed-loop Gazebo navigation; only human response differs.',results={})
fig,ax=plt.subplots(figsize=(10,4.5))
for policy,color in [('native','#cf7328'),('circle_contact','#246bb0')]:
    rows=results[policy];event=min(rows,key=lambda r:r['footprint_clearance']);contacts=[r for r in rows if r['contact_ids']]
    summary['results'][policy]=dict(min_footprint_clearance_m=event['footprint_clearance'],closest_elapsed_s=event['elapsed'],
        first_contact_elapsed_s=contacts[0]['elapsed'] if contacts else None,contact_ticks=len(contacts),
        footprint_overlap_gt_1cm_s=sum(b['time']-a['time'] for a,b in zip(rows,rows[1:]) if b['footprint_clearance']<-.01))
    ax.plot([r['elapsed'] for r in rows],[r['footprint_clearance'] for r in rows],color=color,label=policy)
ax.set(xlim=(2,10),ylim=(-.3,1.6),xlabel='simulation seconds after goal',ylabel='rectangle-to-circle clearance [m]',title='Identical recorded robot input: geometry response only (open loop)')
ax.axhline(0,color='red',ls=':');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(a.output/'same_robot_replay.png',dpi=160);plt.close(fig)
(a.output/'same_robot_replay.json').write_text(json.dumps(summary,indent=2));(a.output/'same_robot_replay_traces.json').write_text(json.dumps(results))
print(json.dumps(summary,indent=2))
