#!/usr/bin/env python3
"""Analyze passive captures, using message stamps and measured Gazebo/odom poses."""
import argparse
from bisect import bisect_right
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

P=argparse.ArgumentParser()
P.add_argument('report_dir',type=Path)
a=P.parse_args()
report=a.report_dir
trial=json.loads(next(report.glob('socnavgym_counterflow_s43.json')).read_text())
raw=Path(trial['diagnostics'])
d=defaultdict(list)
def stamp(m):
    s=m['header']['stamp']; return s['sec']+s['nanosec']/1e9
for line in raw.open():
    row=json.loads(line);kind=row['kind'];m=row['data']
    row['time']=m['time'] if kind in ('gazebo','native') else stamp(m) if 'header' in m else row['receipt_sim']
    d[kind].append(row)
for kind in d:d[kind].sort(key=lambda r:r['time'] or 0.)
times={k:[r['time'] for r in v] for k,v in d.items() if k not in ('tf','tf_static')}
gate=float((raw.parent/'start_gate').read_text())

def previous(kind,t):
    i=bisect_right(times[kind],t)-1
    return d[kind][max(0,i)]
def yaw(q):return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def pose(m):return dict(x=m['position']['x'],y=m['position']['y'],yaw=yaw(m['orientation']))
def odom(t):
    i=bisect_right(times['odom'],t)-1;i=max(0,min(i,len(d['odom'])-2))
    l,r=d['odom'][i:i+2];f=max(0,min(1,(t-l['time'])/(r['time']-l['time'])))
    l,r=pose(l['data']['pose']['pose']),pose(r['data']['pose']['pose'])
    return dict(x=l['x']+f*(r['x']-l['x']),y=l['y']+f*(r['y']-l['y']),yaw=l['yaw']+f*math.atan2(math.sin(r['yaw']-l['yaw']),math.cos(r['yaw']-l['yaw'])))
def local(robot,xy):
    dx,dy=xy[0]-robot['x'],xy[1]-robot['y'];c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    return np.array([c*dx+s*dy,-s*dx+c*dy])
def world(robot,xy):
    c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    return np.array([robot['x']+c*xy[0]-s*xy[1],robot['y']+s*xy[0]+c*xy[1]])
def gz(t):
    # Humans are held between discrete updates; interpolate only the robot's motion.
    i=max(0,bisect_right(times['gazebo'],t)-1)
    out={k:v.copy() for k,v in d['gazebo'][i]['data']['poses'].items()}
    if i+1<len(d['gazebo']):
        l,r=d['gazebo'][i:i+2];f=max(0,min(1,(t-l['time'])/(r['time']-l['time'])))
        l,r=l['data']['poses']['mecanum_robot'],r['data']['poses']['mecanum_robot']
        out['mecanum_robot']={k:l[k]+f*(r[k]-l[k]) for k in ['x','y','z','yaw']}
    return out
def clearance(robot,human,hx=.12,hy=.10):
    xy=np.abs(local(robot,[human['x'],human['y']]))-np.array([hx,hy])
    return float(np.linalg.norm(np.maximum(xy,0))+min(max(xy),0)-.36)
def occupancy(row,xy):
    m=row['data'];info=m['info'];o=info['origin']['position'];res=info['resolution']
    ix,iy=math.floor((xy[0]-o['x'])/res),math.floor((xy[1]-o['y'])/res)
    return m['data'][iy*info['width']+ix] if 0<=ix<info['width'] and 0<=iy<info['height'] else None

active=[r for r in d['gazebo'] if gate<=r['time']<=gate+trial['sim_seconds']]
metrics=[]
for row in active:
    t=row['time'];poses=row['data']['poses'];robot=poses['mecanum_robot'];op=odom(t)
    values=[(clearance(robot,h),int(k.split('_')[1])) for k,h in poses.items() if k!='mecanum_robot']
    cl,hid=min(values);cm=previous('costmap',t)
    cmd=previous('cmd_vel',t)['data']['twist']
    metrics.append(dict(time=t,elapsed=t-gate,clearance=cl,human=hid,
        padded_clearance=min(clearance(robot,h,.13,.11) for k,h in poses.items() if k!='mecanum_robot'),
        odom_position_error=math.hypot(op['x']-27-robot['x'],op['y']-robot['y']),
        odom_yaw_error=abs(math.atan2(math.sin(op['yaw']-robot['yaw']),math.cos(op['yaw']-robot['yaw']))),
        costmap_age=t-cm['time'],center_cost=occupancy(cm,[op['x'],op['y']]),
        command_speed=math.hypot(cmd['linear']['x'],cmd['linear']['y'])))
event=min(metrics,key=lambda r:r['clearance']);first=next((r for r in metrics if r['clearance']<-.01),None)
scan_metrics=[]
for row in d['scan']:
    t=row['time']
    if not gate<=t<=gate+trial['sim_seconds']:continue
    m=row['data'];poses=gz(t);robot=poses['mecanum_robot'];angles=m['angle_min']+np.arange(len(m['ranges']))*m['angle_increment']
    rays=np.stack([np.cos(angles),np.sin(angles)],axis=1);expected=np.full(len(rays),np.inf);ids=np.zeros(len(rays),int)
    for name,h in poses.items():
        if name=='mecanum_robot':continue
        xy=local(robot,[h['x'],h['y']]);projection=rays@xy
        disc=projection**2-(xy@xy-.36**2)
        valid=(disc>=0)&(projection>0)
        dist=projection-np.sqrt(np.maximum(disc,0))
        nearer=valid&(dist>=m['range_min'])&(dist<=m['range_max'])&(dist<expected)
        expected[nearer]=dist[nearer];ids[nearer]=int(name.split('_')[1])
    actual=np.array(m['ranges']);mask=np.isfinite(expected)
    difference=np.full(len(actual),np.inf)
    comparable=mask&np.isfinite(actual)
    difference[comparable]=np.abs(actual[comparable]-expected[comparable])
    matches=comparable&(difference<.05)
    nearest=min((math.hypot(h['x']-robot['x'],h['y']-robot['y'])-.36 for k,h in poses.items() if k!='mecanum_robot'))
    cm=previous('costmap',t+.3)
    costs=[]
    for i in np.flatnonzero(matches):
        point=world(robot,rays[i]*actual[i]);point_odom=world(odom(t),local(robot,point))
        costs.append(occupancy(cm,point_odom))
    scan_metrics.append(dict(time=t,elapsed=t-gate,nearest_surface=nearest,expected_beams=int(mask.sum()),
        matched_beams=int(matches.sum()),human_beams={str(i):int((matches&(ids==i)).sum()) for i in [1,2,3]},
        costmap_marked_matched_beams=sum(c is not None and c>=99 for c in costs),
        comparison_costmap_stamp=cm['time']))

def rates(kind):
    stamps=sorted(set(r['time'] for r in d[kind] if r['time'] is not None and gate<=r['time']<=gate+trial['sim_seconds']))
    gaps=np.diff(stamps)
    return dict(unique_messages=len(stamps),median_period_s=float(np.median(gaps)),max_period_s=float(max(gaps))) if len(gaps) else None

def snapshot(ax,t):
    poses=gz(t);robot=poses['mecanum_robot'];op=odom(t);cm=previous('costmap',t);info=cm['data']['info'];res=info['resolution'];o=info['origin']['position']
    arr=np.array(cm['data']['data']).reshape(info['height'],info['width']);iy,ix=np.where(arr>0)
    pts=np.array([local(op,[o['x']+(x+.5)*res,o['y']+(y+.5)*res]) for y,x in zip(iy,ix)])
    if len(pts):ax.scatter(pts[:,0],pts[:,1],c=arr[iy,ix],cmap='Greys',s=15,vmin=0,vmax=100,marker='s',alpha=.6,label='local costmap')
    scan=previous('scan',t);m=scan['data'];sp=gz(scan['time'])['mecanum_robot'];ranges=np.array(m['ranges']);valid=np.isfinite(ranges)&(ranges>=m['range_min'])&(ranges<=m['range_max'])
    angles=m['angle_min']+np.arange(len(ranges))*m['angle_increment'];pts=[]
    for r,angle in zip(ranges[valid],angles[valid]):pts.append(local(robot,world(sp,[r*math.cos(angle),r*math.sin(angle)])))
    if pts:pts=np.array(pts);ax.scatter(pts[:,0],pts[:,1],c='#188c43',s=9,label='last LiDAR endpoints')
    ax.add_patch(Rectangle((-.12,-.10),.24,.2,color='#2369a6',label='configured footprint'))
    ax.add_patch(Rectangle((-.13,-.11),.26,.22,fill=False,color='#2369a6',ls=':'))
    for name,h in poses.items():
        if name=='mecanum_robot':continue
        xy=local(robot,[h['x'],h['y']]);ax.add_patch(Circle(xy,.36,facecolor='#ed8a35',edgecolor='#a64b00',alpha=.3));ax.text(*xy,name.split('_')[1],ha='center')
    ax.set(xlim=(-1.7,1.7),ylim=(-1.7,1.7),xlabel='robot x [m]',ylabel='robot y [m]',title=f't={t-gate:.3f}s; costmap age {t-cm["time"]:.3f}s; scan age {t-scan["time"]:.3f}s')
    ax.set_aspect('equal');ax.grid(alpha=.15)

fig,axes=plt.subplots(1,2,figsize=(11,5))
snapshot(axes[0],first['time']-.3 if first else event['time']-.3);snapshot(axes[1],event['time']);axes[0].legend(fontsize=8,loc='lower left')
fig.suptitle('Unmodified seed 43 replay: sensors, local costmap and measured human geometry')
fig.tight_layout();fig.savefig(report/'sensor_costmap_geometry.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
x=[r['elapsed'] for r in metrics]
axes[0].plot(x,[r['clearance'] for r in metrics],label='Nav2 rectangle to human');axes[0].axhline(0,color='red',ls=':');axes[0].set(ylabel='clearance [m]',ylim=(-.3,2));axes[0].legend()
axes[1].plot(x,[r['command_speed'] for r in metrics],label='command speed [m/s]');axes[1].plot(x,[r['center_cost']/100 if r['center_cost'] is not None else np.nan for r in metrics],label='cost at robot center / 100');axes[1].legend();axes[1].set(ylabel='command / cost')
axes[2].plot([r['elapsed'] for r in scan_metrics],[r['matched_beams'] for r in scan_metrics],label='beams matching human surface within 5 cm');axes[2].set(ylabel='LiDAR beams',xlabel='simulation seconds after goal accepted');axes[2].legend()
for ax in axes:
    ax.set_xlim(0,12);ax.axvline(event['elapsed'],color='red',ls=':');ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(report/'diagnostic_timeline.png',dpi=160);plt.close(fig)
tf_errors=[]
static_tf=[]
for kind in ('tf','tf_static'):
    for row in d[kind]:
        for tf in row['data']['transforms']:
            if kind=='tf_static':
                static_tf.append(dict(parent=tf['header']['frame_id'],child=tf['child_frame_id'],transform=tf['transform']))
            elif tf['header']['frame_id']=='odom' and tf['child_frame_id']=='base_footprint':
                t=stamp(tf)
                if not gate<=t<=gate+trial['sim_seconds']:continue
                op=odom(t);v=tf['transform']['translation']
                tf_errors.append(math.hypot(op['x']-v['x'],op['y']-v['y']))
near=[r for r in metrics if first and first['time']-.5<=r['time']<=event['time']+.5]
near_scans=[r for r in scan_metrics if first and first['time']-1<=r['time']<=event['time']+.5]
summary=dict(trial_status=trial['status'],scene_sha256=trial['scene_sha256'],map_sha256=trial['map_sha256'],nav_params_sha256=trial['nav_params_sha256'],
    raw_capture=dict(path=str(raw),sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),bytes=raw.stat().st_size,counts=trial['diagnostic_counts']),
    tf=dict(odom_to_base_position_error_vs_odometry_max_m=max(tf_errors),static_transforms=static_tf),
    gate=gate,closest=event,first_overlap_gt_1cm=first,
    footprint_overlap_gt_1cm_s=sum(b['time']-a['time'] for a,b in zip(metrics,metrics[1:]) if b['clearance']<-.01),
    max_odom_position_error_m=max(r['odom_position_error'] for r in metrics),max_odom_yaw_error_rad=max(r['odom_yaw_error'] for r in metrics),
    rates={k:rates(k) for k in ['scan','costmap','cmd_vel','odom','gazebo','native']},
    event_window_center_costs=sorted(set(r['center_cost'] for r in near if r['center_cost'] is not None)),
    event_scans=near_scans,notes=['Configured unpadded rectangle .24 x .20 m; published costmap footprint includes .01 m default padding.',
    'OccupancyGrid values 0-100 are translated Nav2 costs, not raw 0-254 costs.',
    'Gazebo sampled at >=.019s; robot interpolated; humans zero-order held. Scan circle matches use 5 cm tolerance and can include mesh/noise/discrete update error.',
    'Costmap snapshots use odom; human/scan geometry uses actual Gazebo poses. Published costmap age is not the internal controller costmap age.',
    'Human matching costmap counts compare a map up to .3s after scan, and are supporting evidence, not an exact marking success test.'])
(report/'diagnosis.json').write_text(json.dumps(summary,indent=2))
(report/'scan_metrics.json').write_text(json.dumps(scan_metrics,indent=2))
(report/'timeline.json').write_text(json.dumps(metrics))
print(json.dumps(summary,indent=2))
