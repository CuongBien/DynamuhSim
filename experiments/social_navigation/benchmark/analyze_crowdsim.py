#!/usr/bin/env python3
"""Analyze CrowdSimPlus/native human kernel versus SocNavGym adapter, from Gazebo poses."""
import argparse,csv,json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle
from matplotlib.transforms import Affine2D
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('report_dir',type=Path);a=p.parse_args();report=a.report_dir
layout=json.loads((HERE.parent/'warehouse/layout.json').read_text())
def clearance(robot,h,hx=.12,hy=.10):
    dx,dy=h['x']-robot['x'],h['y']-robot['y'];c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    qx,qy=abs(c*dx+s*dy)-hx,abs(-s*dx+c*dy)-hy
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
def factory(h,b):
    qx,qy=abs(h['x']-b['x'])-b['sx']/2,abs(h['y']-b['y'])-b['sy']/2
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
def annotate(row):
    hs=row['humans'];ro=row['robot'];cx=sum(h['x'] for h in hs)/3;cy=sum(h['y'] for h in hs)/3
    row['clearance']=min(clearance(ro,h) for h in hs)
    row['padded']=min(clearance(ro,h,.13,.11) for h in hs)
    row['pair']=min(math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(hs) for j in hs[i+1:])
    row['cohort_radius']=max(math.hypot(h['x']-cx,h['y']-cy) for h in hs)
    return row
def overlap(rows,key):return sum(b['time']-a['time'] for a,b in zip(rows,rows[1:]) if b[key]<-.01)
entries=[];traces={};events=[]
for file in sorted(report.glob('*_s*.json')):
    t=json.loads(file.read_text())
    if not isinstance(t,dict) or 'backend' not in t:continue
    assert t['protocol_valid'],t['name']
    if t['backend']=='crowdsimplus' and t.get('human_policy',t.get('crowdsim_metadata',{}).get('human_policy','orca_plus'))!='orca_plus':
        raise ValueError('This report compares ORCA+ only; use analyze_sfm.py for SFM trials')
    runtime=HERE/t['runtime'];gate=float((runtime/'start_gate').read_text());rows=[]
    for line in Path(t['diagnostics']).open():
        raw=json.loads(line)
        if raw['kind']!='gazebo':continue
        v=raw['data'];time=v['time'];poses=v['poses']
        if not gate<=time<=gate+t['sim_seconds'] or len(poses)<4:continue
        hs=[dict(id=int(k.split('_')[1]),**h) for k,h in poses.items() if k!='mecanum_robot']
        rows.append(annotate(dict(time=time,elapsed=time-gate,robot=poses['mecanum_robot'],humans=hs)))
    assert len(rows)>100
    policy=('crowdsimplus_visible' if t['robot_visible'] else 'crowdsimplus_blind') if t['backend']=='crowdsimplus' else 'socnavgym_robot_orca';native=[json.loads(l) for l in (runtime/'native_trajectory.jsonl').open()]
    active=[x for x in native if x['active']];contacts=[x for x in active if x.get('robot_contact_ids')]
    seen=[x for x in active if (x.get('orca_telemetry') or {}).get('group_robot_neighbor_seen')]
    coarse=[annotate(x) for x in json.loads((report/f'{t["name"]}_trajectory.json').read_text())]
    event=min(rows,key=lambda r:r['clearance']);pair_event=min(rows,key=lambda r:r['pair'])
    entry=dict(name=t['name'],policy=policy,group_model=t['backend']=='socnavgym',condition=t['condition'],seed=t['seed'],status=t['status'],protocol_valid=t['protocol_valid'],
        goal_verified=t['status']=='SUCCEEDED' and t['goal_error_world_m']<.5,sim_seconds=t['sim_seconds'],goal_error_world_m=t['goal_error_world_m'],
        min_footprint_clearance_m=event['clearance'],footprint_overlap_gt_1cm_s=overlap(rows,'clearance'),
        min_padded_footprint_clearance_m=min(x['padded'] for x in rows),min_human_pair_clearance_m=min(x['pair'] for x in rows),
        human_pair_overlap_gt_1cm_s=overlap(rows,'pair'),min_pair_elapsed_s=pair_event['elapsed'],robot_clearance_at_min_pair_m=pair_event['clearance'],max_population_extent_from_centroid_m=max(x['cohort_radius'] for x in rows),
        min_human_factory_clearance_m=min(factory(h,b) for x in rows for h in x['humans'] for b in layout['obstacles']),
        closest_elapsed_s=event['elapsed'],robot_contact_ticks=len(contacts) if t['backend']=='socnavgym' else None,first_robot_contact_elapsed_s=contacts[0]['time']-gate if contacts else None,
        solver_robot_neighbor_ticks=len(seen) if t['backend']=='socnavgym' else None,first_solver_robot_neighbor_elapsed_s=seen[0]['time']-gate if seen else None,
        observed_rtf=t['observed_rtf'],worker_compute_ms_mean=sum(x['compute_ms'] for x in active)/len(active),
        samples_50hz=len(rows),coarse_10hz_min_clearance_m=min(x['clearance'] for x in coarse),coarse_10hz_overlap_gt_1cm_s=overlap(coarse,'clearance'),
        max_human_displacement_m=t['max_human_displacement_m'])
    entries.append(entry);traces[(t['condition'],t['seed'],policy)]=(t,rows);events.append(dict(name=t['name'],event=event))
entries.sort(key=lambda x:(x['condition'],x['seed'],x['policy']))
summary=dict(trials=len(entries),verified_goals=sum(x['goal_verified'] for x in entries),results=entries,
    notes=['Shared initial poses/map/Nav2, proxy .36, nominal robot radius .25, human timestep .1, preferred speed .8.',
    'CrowdSimPlus uses native ORCAPlus/Human.act/static constraint/Human.step; original demo timestep .25, adapter benchmark .1.',
    'SocNavGym comparator is the modified robot-aware group adapter, not upstream native behavior.',
    'CrowdSimPlus has independent pedestrians and no conversation/group model. Population extent is not a group cohesion score.',
    'Metrics from >=.019 s Gazebo samples for both backends; threshold -1 cm; no force contact sensor.',
    'Two human fixture seeds, one trial per case, Nav2/AMCL/LiDAR noise not jointly seeded; not a statistical ranking.'])
(report/'analysis.json').write_text(json.dumps(summary,indent=2));(report/'closest_events.json').write_text(json.dumps(events,indent=2))
with (report/'results.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(entries[0]),lineterminator='\n');w.writeheader();w.writerows(entries)
fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True)
for col,seed in enumerate([42,43]):
    for policy,color in [('crowdsimplus_visible','#2774b5'),('socnavgym_robot_orca','#27965e'),('crowdsimplus_blind','#c47c28')]:
        if ('counterflow',seed,policy) not in traces:continue
        _,rows=traces[('counterflow',seed,policy)];x=[r['elapsed'] for r in rows]
        axes[0,col].plot(x,[r['clearance'] for r in rows],color=color,label=policy)
        axes[1,col].plot(x,[r['pair'] for r in rows],color=color,label=policy)
    axes[0,col].set(title=f'counterflow seed {seed}',ylabel='footprint clearance [m]',ylim=(-.5,1.8))
    axes[1,col].set(ylabel='human pair clearance [m]',xlabel='simulation seconds after goal')
    for ax in axes[:,col]:ax.axhline(0,color='red',ls=':');ax.set_xlim(0,18);ax.grid(alpha=.2)
axes[0,0].legend();fig.suptitle('CrowdSimPlus individual ORCA+ vs SocNavGym robot-aware group adapter | first 18 s')
fig.tight_layout();fig.savefig(report/'framework_clearance.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(12,4.7),sharex=True,sharey=True)
for ax,policy in zip(axes,['crowdsimplus_visible','socnavgym_robot_orca']):
    if ('counterflow',43,policy) not in traces:continue
    t,rows=traces[('counterflow',43,policy)];event=min(rows,key=lambda r:r['clearance'])
    for b in layout['obstacles']:ax.add_patch(Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],color='#d9dde1'))
    ax.plot([x['robot']['x'] for x in rows],[x['robot']['y'] for x in rows],color='#2774b5',label='robot',lw=2)
    for hid,color in zip([1,2,3],['#d37b28','#39946a','#9864b3']):
        hs=[next(h for h in x['humans'] if h['id']==hid) for x in rows];ax.plot([h['x'] for h in hs],[h['y'] for h in hs],color=color,label=f'person {hid}')
        h=next(h for h in event['humans'] if h['id']==hid);ax.add_patch(Circle((h['x'],h['y']),.36,color=color,alpha=.3))
    ro=event['robot'];transform=Affine2D().rotate(ro['yaw']).translate(ro['x'],ro['y'])+ax.transData
    ax.add_patch(Rectangle((-.12,-.10),.24,.20,transform=transform,color='#2774b5'))
    ax.set(xlim=(-29,-13),ylim=(-3,3),xlabel='world x [m]',ylabel='world y [m]',title=f'{policy} | seed 43\nmin footprint clearance {event["clearance"]:.3f} m');ax.set_aspect('equal');ax.grid(alpha=.2)
axes[0].legend(fontsize=8,ncol=4);fig.tight_layout();fig.savefig(report/'framework_paths_s43.png',dpi=160);plt.close(fig)
print(json.dumps(summary,indent=2))
