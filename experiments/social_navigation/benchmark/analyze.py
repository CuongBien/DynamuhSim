#!/usr/bin/env python3
"""Analyze paired Gazebo measurements; preserve raw trial reports unchanged."""
import csv
import json
import math
from pathlib import Path
import statistics
import yaml
import ast
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.transforms import Affine2D

HERE=Path(__file__).resolve().parent
REPORT=HERE/'reports'
layout=json.loads((HERE.parent/'warehouse/layout.json').read_text())
params=yaml.safe_load((HERE.parent/'warehouse/nav2_warehouse.yaml').read_text())
for costmap in ['local_costmap','global_costmap']:
    polygon=ast.literal_eval(params[costmap][costmap]['ros__parameters']['footprint'])
    assert set(map(tuple,polygon))=={(.12,.10),(.12,-.10),(-.12,.10),(-.12,-.10)}, 'Update the metric if the Nav2 footprint changes'

def footprint_clearance(robot,human):
    dx,dy=human['x']-robot['x'],human['y']-robot['y']
    c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    qx=abs(c*dx+s*dy)-.12
    qy=abs(-s*dx+c*dy)-.10
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-human['radius']

def box_clearance(h,box):
    qx=abs(h['x']-box['x'])-box['sx']/2
    qy=abs(h['y']-box['y'])-box['sy']/2
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-h['radius']

calibration=[]
for label,robot,human,expected in [
    ('axis-aligned outside',dict(x=0.,y=0.,yaw=0.),dict(x=.60,y=0.,radius=.36),.12),
    ('rotated 90 degrees',dict(x=0.,y=0.,yaw=math.pi/2),dict(x=.60,y=0.,radius=.36),.14),
    ('exact tangent',dict(x=0.,y=0.,yaw=0.),dict(x=.48,y=0.,radius=.36),0.)]:
    actual=footprint_clearance(robot,human)
    assert abs(actual-expected)<1e-12
    calibration.append(dict(case=label,expected_m=expected,measured_m=actual))
(REPORT/'metric_calibration.json').write_text(json.dumps(dict(status='PASS',cases=calibration),indent=2))

records=[]
trajectories={}
for path in sorted(REPORT.glob('*_s*.json')):
    trial=json.loads(path.read_text())
    if not isinstance(trial,dict) or 'backend' not in trial:
        continue
    rows=json.loads((REPORT/f'{trial["name"]}_trajectory.json').read_text())
    if not rows:
        records.append(dict(name=trial['name'],status=trial['status'],protocol_valid=False))
        continue
    for row in rows:
        row['footprint_clearance']=min((footprint_clearance(row['robot'],h) for h in row['humans']),default=None)
    runtime=HERE/trial['runtime']
    gate=runtime/'start_gate'
    start=float(gate.read_text()) if gate.exists() else rows[0]['time']
    trajectories[trial['name']]=(trial,rows,start)
    entry={key:trial.get(key) for key in ['name','backend','condition','seed','status','protocol_valid',
        'sim_seconds','wall_seconds','observed_rtf','robot_path_m','goal_error_world_m',
        'min_robot_human_clearance_m','min_human_pair_clearance_m','robot_human_overlap_s',
        'human_pair_overlap_s','robot_stopped_s','max_human_displacement_m','initial_position_error_m_max']}
    entry['first_human_motion_s']=next((row['time']-start for row in rows if any(math.hypot(h['x']-trial['scene']['initial'][h['id']-1]['x'],h['y']-trial['scene']['initial'][h['id']-1]['y'])>.005 for h in row['humans'])),None)
    entry['goal_verified']=trial['status']=='SUCCEEDED' and trial['goal_error_world_m']<.5
    entry['start_sim_time']=start
    entry['min_nav_footprint_clearance_m']=min((row['footprint_clearance'] for row in rows if row['footprint_clearance'] is not None),default=None)
    entry['nav_footprint_overlap_s']=sum(b['time']-a['time'] for a,b in zip(rows,rows[1:]) if b['footprint_clearance'] is not None and b['footprint_clearance']<0)
    entry['human_pair_overlap_gt_1cm_s']=sum(b['time']-a['time'] for a,b in zip(rows,rows[1:]) if b['human_pair_clearance'] is not None and b['human_pair_clearance']<-.01)
    entry['nav_footprint_overlap_gt_1cm_s']=sum(b['time']-a['time'] for a,b in zip(rows,rows[1:]) if b['footprint_clearance'] is not None and b['footprint_clearance']<-.01)
    entry['circle_overlap_gt_1cm_s']=sum(b['time']-a['time'] for a,b in zip(rows,rows[1:]) if b['robot_human_clearance'] is not None and b['robot_human_clearance']<-.01)
    entry['human_factory_clearance_m_min']=min((box_clearance(h,box) for row in rows for h in row['humans'] for box in layout['obstacles']),default=None)
    entry['max_cohort_radius_m']=max((max(math.hypot(h['x']-statistics.mean(j['x'] for j in row['humans']),h['y']-statistics.mean(j['y'] for j in row['humans'])) for h in row['humans']) for row in rows if row['humans']),default=None)
    native=runtime/'native_trajectory.jsonl'
    if native.exists():
        ticks=[json.loads(line) for line in native.read_text().splitlines()]
        active=[t for t in ticks if t['active']]
        entry['worker_compute_ms_mean']=statistics.mean(t['compute_ms'] for t in active) if active else None
    records.append(entry)

fields=list(dict.fromkeys(key for row in records for key in row))
with (REPORT/'results.csv').open('w',newline='') as stream:
    writer=csv.DictWriter(stream,fieldnames=fields,lineterminator='\n')
    writer.writeheader(); writer.writerows(records)
(REPORT/'analysis.json').write_text(json.dumps(dict(trials=len(records),
    valid_trials=sum(r['protocol_valid'] for r in records),verified_goals=sum(r.get('goal_verified',False) for r in records),
    results=records,metric_notes=dict(circle_robot_radius=.25,human_radius=.36,
        nav_footprint_half_dimensions=[.12,.10],overlap_tolerance_m=.01,physical_contact_sensor_used=False,
        sampling='SceneBroadcaster, at least 0.099 simulation seconds between samples',
        interpretation='Pilot only; two seeds; Nav2 sampling/noise is not seeded by the human seed.')),indent=2))

colors=['#dc7a21','#318b53','#9b59b6']
fig,axes=plt.subplots(2,2,figsize=(14,7.5),sharex=True,sharey=True)
for i,condition in enumerate(['standing','counterflow']):
    for j,backend in enumerate(['rvo2','socnavgym']):
        ax=axes[i,j]
        name=f'{backend}_{condition}_s42'
        for box in layout['obstacles']:
            ax.add_patch(Rectangle((box['x']-box['sx']/2,box['y']-box['sy']/2),box['sx'],box['sy'],facecolor='#d8dce0',edgecolor='#aeb7bf',lw=.5))
        ax.set(xlim=(-29.5,-12),ylim=(-3.5,3.5),title=f'{backend} | {condition} | seed 42',xlabel='world x [m]',ylabel='world y [m]')
        ax.set_aspect('equal'); ax.grid(alpha=.15)
        if name not in trajectories:
            continue
        trial,rows,start=trajectories[name]
        ax.plot([r['robot']['x'] for r in rows],[r['robot']['y'] for r in rows],color='#2369a6',lw=2,label='robot/Nav2')
        closest=min(rows,key=lambda r:r['footprint_clearance'] if r['footprint_clearance'] is not None else 1e9)
        for hid,color in enumerate(colors,1):
            humans=[next(h for h in r['humans'] if h['id']==hid) for r in rows]
            ax.plot([h['x'] for h in humans],[h['y'] for h in humans],color=color,alpha=.7,lw=1,label=f'person {hid}')
            h=next(h for h in closest['humans'] if h['id']==hid)
            ax.add_patch(Circle((h['x'],h['y']),.36,color=color,alpha=.3))
        robot=closest['robot']
        transform=Affine2D().rotate(robot['yaw']).translate(robot['x'],robot['y'])+ax.transData
        ax.add_patch(Rectangle((-.12,-.10),.24,.20,transform=transform,facecolor='#2369a6'))
        ax.scatter([-27,-14],[0,0],marker='x',s=35,c=['#2369a6','#bc3434'])
        ax.text(.02,.03,f'{trial["status"]}; {trial["sim_seconds"]:.1f} sim s',transform=ax.transAxes,fontsize=9)
axes[0,0].legend(loc='upper center',ncol=4,fontsize=8)
fig.suptitle('Same initial geometry and Nav2 parameters; discs show the closest sampled approach')
fig.tight_layout(); fig.savefig(REPORT/'paired_paths.png',dpi=160); plt.close(fig)

fig,axes=plt.subplots(2,2,figsize=(13,7),sharex=True)
for i,condition in enumerate(['standing','counterflow']):
    for backend,color in [('rvo2','#2369a6'),('socnavgym','#dc7a21')]:
        for seed,style in [(42,'-'),(43,'--')]:
            name=f'{backend}_{condition}_s{seed}'
            if name not in trajectories: continue
            trial,rows,start=trajectories[name]
            x=[r['time']-start for r in rows]
            axes[i,0].plot(x,[r['footprint_clearance'] for r in rows],style,color=color,label=f'{backend} s{seed}',lw=1.2)
            axes[i,1].plot(x,[r['human_pair_clearance'] for r in rows],style,color=color,label=f'{backend} s{seed}',lw=1.2)
    for j,label in enumerate(['Robot Nav2 footprint to human proxy','Minimum human-pair clearance']):
        axes[i,j].axhline(0,color='#bc3434',ls=':',lw=1)
        axes[i,j].set(title=f'{condition}: {label}',xlabel='simulation seconds after goal accepted',ylabel='clearance [m]')
        axes[i,j].grid(alpha=.2)
axes[0,0].legend(fontsize=8)
fig.tight_layout(); fig.savefig(REPORT/'clearance_curves.png',dpi=160); plt.close(fig)
print(json.dumps([dict(name=r['name'],status=r['status'],sim_seconds=r.get('sim_seconds'),
    footprint_clearance=r.get('min_nav_footprint_clearance_m'),pair_overlap_s=r.get('human_pair_overlap_s')) for r in records],indent=2))

# Show the closest sampled geometry in robot coordinates, making the two
# clearance conventions visible without relying on camera occlusion.
fig,axes=plt.subplots(1,2,figsize=(10,5))
events=[]
for ax,backend in zip(axes,['rvo2','socnavgym']):
    name=f'{backend}_counterflow_s43'
    if name not in trajectories: continue
    trial,rows,start=trajectories[name]
    event=min(rows,key=lambda r:r['footprint_clearance'])
    robot=event['robot']; c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    ax.add_patch(Rectangle((-.12,-.10),.24,.20,color='#2369a6',label='Nav2 footprint'))
    ax.add_patch(Circle((0,0),.25,fill=False,ls='--',color='#2369a6',label='native robot circle'))
    for h,color in zip(event['humans'],colors):
        dx,dy=h['x']-robot['x'],h['y']-robot['y']
        x,y=c*dx+s*dy,-s*dx+c*dy
        ax.add_patch(Circle((x,y),.36,color=color,alpha=.35))
        ax.text(x,y,str(h['id']),ha='center',va='center')
    ax.set(xlim=(-1.8,1.8),ylim=(-1.8,1.8),xlabel='robot-frame x [m]',ylabel='robot-frame y [m]',
        title=f"{backend} s43; t={event['time']-start:.2f} s\nfootprint clearance {event['footprint_clearance']:.3f} m")
    ax.set_aspect('equal'); ax.grid(alpha=.2)
    events.append(dict(name=name,time_since_goal=event['time']-start,measurement=event,
        contact_sensor_used=False))
axes[0].legend(loc='lower left',fontsize=8)
fig.tight_layout(); fig.savefig(REPORT/'closest_geometry_s43.png',dpi=160); plt.close(fig)
(REPORT/'closest_events.json').write_text(json.dumps(events,indent=2))
