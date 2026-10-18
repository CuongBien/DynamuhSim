#!/usr/bin/env python3
"""Native SFM vs native ORCA+, using dense Gazebo poses and recorded human velocities."""
import argparse,csv,json,math,statistics
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('report_dir',type=Path);a=p.parse_args();report=a.report_dir
layout=json.loads((HERE.parent/'warehouse/layout.json').read_text())
def rect_circle(robot,h,hx=.12,hy=.10):
    dx,dy=h['x']-robot['x'],h['y']-robot['y'];c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    qx,qy=abs(c*dx+s*dy)-hx,abs(-s*dx+c*dy)-hy
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
def factory(h,b):
    qx,qy=abs(h['x']-b['x'])-b['sx']/2,abs(h['y']-b['y'])-b['sy']/2
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
def duration(rows,key):return sum(b['elapsed']-a['elapsed'] for a,b in zip(rows,rows[1:]) if b[key]<-.01)
entries=[];traces={}
for policy in ['orca_plus','sfm']:
 for seed in [42,43]:
    label='crowdsimplus'+('_sfm' if policy=='sfm' else '');name=f'{label}_counterflow_s{seed}'
    t=json.loads((report/f'{name}.json').read_text());assert t['protocol_valid'] and t['human_policy']==policy
    runtime=HERE/t['runtime'];gate=float((runtime/'start_gate').read_text());rows=[]
    for line in Path(t['diagnostics']).open():
        raw=json.loads(line)
        if raw['kind']!='gazebo':continue
        data=raw['data'];poses=data['poses'];elapsed=data['time']-gate
        if not 0<=elapsed<=t['sim_seconds'] or len(poses)!=4:continue
        hs=[dict(id=int(k.split('_')[1]),**v) for k,v in poses.items() if k!='mecanum_robot'];ro=poses['mecanum_robot']
        rows.append(dict(elapsed=elapsed,robot=ro,humans=hs,clearance=min(rect_circle(ro,h) for h in hs),
            padded=min(rect_circle(ro,h,.13,.11) for h in hs),pair=min(math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(hs) for j in hs[i+1:])))
    assert len(rows)>100
    native=[json.loads(l) for l in (runtime/'native_trajectory.jsonl').open()]
    active=[x for x in native if x['active'] and gate<=x['time']<=gate+t['sim_seconds']]
    acceleration=[math.hypot(h['vx']-j['vx'],h['vy']-j['vy'])/(b['time']-a['time'])
        for a,b in zip(active,active[1:]) if b['step']>a['step'] for h,j in zip(b['humans'],a['humans'])]
    event=min(rows,key=lambda r:r['clearance']);traces[(seed,policy)]=rows
    e=dict(name=name,human_policy=policy,seed=seed,status=t['status'],protocol_valid=True,
        goal_verified=t['status']=='SUCCEEDED' and t['goal_error_world_m']<.5,goal_error_world_m=t['goal_error_world_m'],
        sim_seconds=t['sim_seconds'],robot_path_m=t['robot_path_m'],robot_stopped_s=t['robot_stopped_s'],
        min_footprint_clearance_m=event['clearance'],footprint_overlap_gt_1cm_s=duration(rows,'clearance'),
        min_padded_footprint_clearance_m=min(x['padded'] for x in rows),min_human_pair_clearance_m=min(x['pair'] for x in rows),
        human_pair_overlap_gt_1cm_s=duration(rows,'pair'),min_human_factory_clearance_m=min(factory(h,b) for x in rows for h in x['humans'] for b in layout['obstacles']),
        closest_elapsed_s=event['elapsed'],first_12s_min_footprint_clearance_m=min(x['clearance'] for x in rows if x['elapsed']<=12.),observed_rtf=t['observed_rtf'],worker_compute_ms_mean=statistics.mean(x['compute_ms'] for x in active),
        human_acceleration_mean_m_s2=statistics.mean(acceleration),human_acceleration_p95_m_s2=sorted(acceleration)[int(.95*(len(acceleration)-1))],
        samples_50hz=len(rows),max_human_displacement_m=t['max_human_displacement_m'],
        human_goal_switches=sum(h['goal']!=j['goal'] for a,b in zip(active,active[1:]) for h,j in zip(b['humans'],a['humans'])),
        final_human_speed_mean_m_s=statistics.mean(h['speed'] for h in active[-1]['humans']),
        final_human_goal_distance_mean_m=statistics.mean(math.hypot(h['x']-h['goal'][0],h['y']-h['goal'][1]) for h in active[-1]['humans']))
    entries.append(e)
summary=dict(trials=len(entries),verified_goals=sum(x['goal_verified'] for x in entries),results=entries,
    notes=['Same scene/map/Nav2 hashes, dt .1, preferred speed .8, human radius .36. Native policy parameters unchanged.',
        'SFM is_bottleneck=False in the factory; native special hallway index convention is not applied to factory boxes.',
        'Dense Gazebo geometry samples, -1 cm threshold; not force contact or every physics step.',
        'Recorded velocity change is a descriptive metric, not a validated measure of naturalness.',
        'Two human seeds and one run each; robot/Nav2/AMCL/LiDAR noise not jointly seeded. No statistical ranking.',
        'Original native standalone SFM demo uses .25 s and DWA; Gazebo uses .1 s and Nav2. No group/conversation model.'])
(report/'analysis.json').write_text(json.dumps(summary,indent=2))
with (report/'results.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(entries[0]),lineterminator='\n');w.writeheader();w.writerows(entries)
fig,axes=plt.subplots(2,2,figsize=(12,7))
for col,seed in enumerate([42,43]):
 for policy,color in [('orca_plus','#2774b5'),('sfm','#b75b28')]:
    rows=traces[(seed,policy)];x=[r['elapsed'] for r in rows]
    axes[0,col].plot(x,[r['clearance'] for r in rows],label=policy,color=color)
    axes[1,col].plot(x,[r['pair'] for r in rows],label=policy,color=color)
 axes[0,col].set(title=f'seed {seed}',ylabel='robot footprint clearance [m]')
 axes[1,col].set(ylabel='human pair clearance [m]',xlabel='simulation seconds after goal')
 for ax in axes[:,col]:ax.axhline(0,color='red',ls=':');ax.grid(alpha=.2)
axes[0,0].legend();fig.suptitle('CrowdSimPlus: native ORCA+ vs native SFM | same factory fixture')
fig.tight_layout();fig.savefig(report/'sfm_clearance.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(12,8),sharex=True,sharey=True)
for row,seed in enumerate([42,43]):
 for col,policy in enumerate(['orca_plus','sfm']):
    ax=axes[row,col];rows=traces[(seed,policy)]
    for b in layout['obstacles']:ax.add_patch(Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],color='#d9dde1'))
    ax.plot([x['robot']['x'] for x in rows],[x['robot']['y'] for x in rows],color='#2774b5',label='robot',lw=2)
    for hid,color in zip([1,2,3],['#d37b28','#39946a','#9864b3']):
        hs=[next(h for h in x['humans'] if h['id']==hid) for x in rows]
        ax.plot([h['x'] for h in hs],[h['y'] for h in hs],color=color,label=f'person {hid}')
    ax.set(xlim=(-29,-13),ylim=(-3,3),title=f'{policy}, seed {seed}',xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal');ax.grid(alpha=.2)
axes[0,0].legend(fontsize=8,ncol=4);fig.tight_layout();fig.savefig(report/'sfm_paths.png',dpi=160);plt.close(fig)
print(json.dumps(summary,indent=2))
