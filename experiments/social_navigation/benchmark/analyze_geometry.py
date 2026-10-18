#!/usr/bin/env python3
"""Offline paired geometry ablation, always measuring poses from Gazebo."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle
from matplotlib.transforms import Affine2D
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('report_dir',type=Path);a=p.parse_args()
report=a.report_dir
layout=json.loads((HERE.parent/'warehouse/layout.json').read_text())
def clearance(robot,h,hx=.12,hy=.10):
    dx,dy=h['x']-robot['x'],h['y']-robot['y'];c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    qx,qy=abs(c*dx+s*dy)-hx,abs(-s*dx+c*dy)-hy
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-h.get('radius',.36)
def static_clearance(h,box):
    qx,qy=abs(h['x']-box['x'])-box['sx']/2,abs(h['y']-box['y'])-box['sy']/2
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
def annotate(row):
    robot=row['robot'];humans=row['humans']
    row['footprint_clearance']=min(clearance(robot,h) for h in humans)
    row['padded_clearance']=min(clearance(robot,h,.13,.11) for h in humans)
    row['circle_clearance']=min(math.hypot(h['x']-robot['x'],h['y']-robot['y'])-.61 for h in humans)
    row['pair_clearance']=min(math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(humans) for j in humans[i+1:])
    return row
def overlap(rows,key):
    return sum(b['time']-a['time'] for a,b in zip(rows,rows[1:]) if b[key]<-.01)
entries=[];traces={};events=[]
for file in sorted(report.glob('socnavgym*_s*.json')):
    trial=json.loads(file.read_text())
    if not isinstance(trial,dict) or 'backend' not in trial:continue
    assert trial['protocol_valid'],trial['name']
    runtime=HERE/trial['runtime'];gate=float((runtime/'start_gate').read_text())
    rows=[]
    with Path(trial['diagnostics']).open() as f:
        for line in f:
            raw=json.loads(line)
            if raw['kind']!='gazebo':continue
            t=raw['data']['time'];poses=raw['data']['poses']
            if not gate<=t<=gate+trial['sim_seconds'] or len(poses)<4:continue
            humans=[dict(id=int(k.split('_')[1]),radius=.36,**v) for k,v in poses.items() if k!='mecanum_robot']
            rows.append(annotate(dict(time=t,elapsed=t-gate,robot=poses['mecanum_robot'],humans=humans)))
    assert len(rows)>100
    coarse=[annotate(r) for r in json.loads((report/f'{trial["name"]}_trajectory.json').read_text())]
    event=min(rows,key=lambda r:r['footprint_clearance'])
    native=[json.loads(l) for l in (runtime/'native_trajectory.jsonl').open()]
    active=[r for r in native if r['active']]
    contacts=[r for r in active if r.get('robot_contact_ids')]
    entry=dict(name=trial['name'],geometry=trial['geometry'],condition=trial['condition'],seed=trial['seed'],
        status=trial['status'],protocol_valid=trial['protocol_valid'],goal_verified=trial['status']=='SUCCEEDED' and trial['goal_error_world_m']<.5,
        sim_seconds=trial['sim_seconds'],goal_error_world_m=trial['goal_error_world_m'],robot_path_m=trial['robot_path_m'],observed_rtf=trial['observed_rtf'],
        min_footprint_clearance_m=event['footprint_clearance'],footprint_overlap_gt_1cm_s=overlap(rows,'footprint_clearance'),
        min_padded_footprint_clearance_m=min(r['padded_clearance'] for r in rows),
        min_circle_clearance_m=min(r['circle_clearance'] for r in rows),
        min_human_pair_clearance_m=min(r['pair_clearance'] for r in rows),human_pair_overlap_gt_1cm_s=overlap(rows,'pair_clearance'),
        min_human_factory_clearance_m=min(static_clearance(h,box) for r in rows for h in r['humans'] for box in layout['obstacles']),
        closest_elapsed_s=event['elapsed'],samples_50hz=len(rows),
        coarse_10hz_min_footprint_clearance_m=min(r['footprint_clearance'] for r in coarse),
        coarse_10hz_overlap_gt_1cm_s=overlap(coarse,'footprint_clearance'),
        actual_native_robot_radius=trial['contact_geometry']['effective_robot_radius'],
        worker_compute_ms_mean=sum(r['compute_ms'] for r in active)/len(active),
        circle_contact_active_ticks=len(contacts),first_circle_contact_elapsed_s=contacts[0]['time']-gate if contacts else None,
        human_max_displacement_m=trial['max_human_displacement_m'])
    if trial['geometry']=='native':
        entry['circle_contact_active_ticks']=None
    traces[(trial['condition'],trial['seed'],trial['geometry'])]=(trial,rows)
    entries.append(entry);events.append(dict(name=trial['name'],event=event))
entries.sort(key=lambda r:(r['condition'],r['seed'],r['geometry']))
with (report/'results.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(entries[0]),lineterminator='\n');writer.writeheader();writer.writerows(entries)
summary=dict(trials=len(entries),verified_goals=sum(r['goal_verified'] for r in entries),results=entries,
    notes=['Primary metrics use Gazebo poses captured at >=.019 s; coarse 10 Hz metrics retained for comparison with pilot.',
    'Overlap duration uses later sample state and threshold -0.01 m; it is an estimate, not a contact-force measurement.',
    'Native uses ellipse contact with reset radius jitter; circle_contact uses fixed nominal robot radius .25 and exact circular human predicates.',
    'No new steering, swept clipping, human pair separation or robot agent in group ORCA. Only two human seeds; LiDAR, AMCL and MPPI are not jointly seeded.',
    'The exact analytic predicates also reduce worker compute time. Different RTF/timer scheduling is a closed-loop confound; the identical-input replay isolates contact geometry response.'])
(report/'analysis.json').write_text(json.dumps(summary,indent=2));(report/'closest_events.json').write_text(json.dumps(events,indent=2))
fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True)
for col,seed in enumerate([42,43]):
    for policy,color in [('native','#cf7328'),('circle_contact','#246bb0')]:
        key=('counterflow',seed,policy)
        if key not in traces:continue
        trial,rows=traces[key];x=[r['elapsed'] for r in rows]
        axes[0,col].plot(x,[r['footprint_clearance'] for r in rows],color=color,label=policy,lw=1.3)
        axes[1,col].plot(x,[r['pair_clearance'] for r in rows],color=color,label=policy,lw=1.3)
    axes[0,col].set(title=f'counterflow seed {seed}',ylabel='robot footprint clearance [m]',ylim=(-.28,.8))
    axes[1,col].set(ylabel='human pair clearance [m]',xlabel='simulation seconds after goal')
    for ax in axes[:,col]:ax.axhline(0,color='red',ls=':',lw=1);ax.grid(alpha=.2);ax.set_xlim(0,16)
axes[0,0].legend();fig.suptitle('Native vs circle-contact geometry | first 16 sim seconds; clearance axis zoomed near contact')
fig.tight_layout();fig.savefig(report/'clearance_comparison.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(12,4.7),sharex=True,sharey=True)
for ax,policy in zip(axes,['native','circle_contact']):
    key=('counterflow',43,policy)
    if key not in traces:continue
    trial,rows=traces[key];event=min(rows,key=lambda r:r['footprint_clearance'])
    for box in layout['obstacles']:
        ax.add_patch(Rectangle((box['x']-box['sx']/2,box['y']-box['sy']/2),box['sx'],box['sy'],color='#d6dadd'))
    ax.plot([r['robot']['x'] for r in rows],[r['robot']['y'] for r in rows],color='#246bb0',label='robot')
    for hid,color in zip([1,2,3],['#ce7228','#329c62','#9c65b3']):
        humans=[next(h for h in r['humans'] if h['id']==hid) for r in rows]
        ax.plot([h['x'] for h in humans],[h['y'] for h in humans],color=color,label=f'person {hid}',lw=1)
        h=next(h for h in event['humans'] if h['id']==hid);ax.add_patch(Circle((h['x'],h['y']),.36,color=color,alpha=.3))
    r=event['robot'];transform=Affine2D().rotate(r['yaw']).translate(r['x'],r['y'])+ax.transData
    ax.add_patch(Rectangle((-.12,-.10),.24,.20,transform=transform,color='#246bb0'))
    ax.set(xlim=(-29,-13),ylim=(-3,3),title=f'{policy} | seed 43\nmin footprint clearance {event["footprint_clearance"]:.3f} m',xlabel='world x [m]',ylabel='world y [m]')
    ax.set_aspect('equal');ax.grid(alpha=.2)
axes[0].legend(fontsize=8,ncol=4);fig.tight_layout();fig.savefig(report/'paths_seed43.png',dpi=160);plt.close(fig)
print(json.dumps(summary,indent=2))
