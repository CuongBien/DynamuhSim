#!/usr/bin/env python3
"""Extract saved Gazebo poses, measure full windows and render recorded-pose GIFs.

All outputs use exclusive creation. No simulator/controller is launched here.
Requires existing numpy, Pillow, matplotlib; never installs dependencies.
"""
import argparse,bisect,csv,hashlib,json,math
from pathlib import Path
import numpy as np


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


def dump(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)


def write_rows(path,values):
    with Path(path).open('x') as f:
        for value in values:f.write(json.dumps(value,allow_nan=False)+'\n')


def wrap(x):return np.arctan2(np.sin(x),np.cos(x))
def stats(v):
    v=np.asarray(v)
    return dict(p50=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(np.max(v))) if v.size else None

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def native_record(case):
    worker=rows(case/'native/runtime/worker_trajectory.jsonl');active=[r for r in worker if r.get('active_phase')]
    result=json.loads((case/'native/episode/result.json').read_text());gate=result['goal_start_sim']
    raw=rows(case/'native/episode/gazebo_poses.jsonl');wtime=[r['time'] for r in active]
    valid={r['time']:r for r in raw if r['time']>=gate and 'mecanum_robot' in r['poses'] and all(f'common_{i}_proxy' in r['poses'] for i in range(1,7))}
    timeline=[];humans=[];groups=[];robot=[];events=[];intentions=[];safety=[];last=None;previous_near=set()
    for t,r in sorted(valid.items()):
        index=bisect.bisect_right(wtime,t)-1
        if index<0:continue
        w=active[index];members={hid:g['id'] for g in w['groups_before'] for hid in g['members']}
        behavior={h['id']:h for h in w.get('behavior',{}).get('humans',[])}
        stage={h['id']:h for h in w['motion_guard']['motion_contract']};goals=w.get('goal_clock',{}).get('humans',{})
        hs=[]
        for hid in range(1,7):
            pose=r['poses'][f'common_{hid}_proxy'];old=None if last is None else last['humans'][hid-1];dt=None if last is None else t-last['time']
            velocity=None if old is None else [(pose[k]-old[k])/dt for k in ('x','y')]
            rec=dict(time=t,id=hid,**pose,vx=None if velocity is None else velocity[0],vy=None if velocity is None else velocity[1],speed=None if velocity is None else math.hypot(*velocity),group_id=members.get(hid),goal=goals.get(str(hid),{}).get('goal'),behavior_state=behavior.get(hid,{}).get('state'),partner=behavior.get(hid,{}).get('partner'),native_proposed_velocity=stage[hid]['native_proposed_velocity'],post_orca_velocity=stage[hid]['post_orca_velocity'],post_projection_velocity=stage[hid]['final_velocity'],executed_displacement=None if old is None else [pose[k]-old[k] for k in ('x','y')],constraints=w['motion_guard']['projection'],stage_interval_start=w['time'])
            humans.append(rec);hs.append(rec)
        rp=r['poses']['mecanum_robot'];robot.append(dict(time=t,**rp,velocity_source='worker finite difference over planning interval',input=w['robot_input']))
        gs=[]
        for g in w['groups_before']:
            ps=[h for h in hs if h['id'] in g['members']];center=[sum(h[k] for h in ps)/len(ps) for k in ('x','y')]
            d=[math.hypot(h['x']-center[0],h['y']-center[1]) for h in ps]
            grow=dict(time=t,id=g['id'],members=g['members'],type=g['type'],centroid=center,goal=goals.get(str(g['members'][0]),{}).get('goal'),envelope_m=max(d)+.36,member_centroid_distances_m=d)
            groups.append(grow);gs.append(grow)
        frame=dict(time=t,humans=hs,groups=gs,robot=rp);timeline.append(frame);near=set()
        for i,h in enumerate(hs):
            for z in hs[:i]:
                gap=math.hypot(h['x']-z['x'],h['y']-z['y'])-.72
                if gap<.04:safety.append(dict(time=t,event='observed_HH_clearance_alert',ids=[z['id'],h['id']],surface_gap_m=gap,threshold_m=.04))
                if gap<.8:
                    pair=(z['id'],h['id']);near.add(pair)
                    if pair not in previous_near:events.append(dict(event='observed_human_proximity_enter',time=t,ids=pair,surface_gap_m=gap,definition='sampled surface gap < 0.8m; not an avoidance decision'))
            gap=math.hypot(h['x']-rp['x'],h['y']-rp['y'])-.61
            if gap<.12:safety.append(dict(time=t,event='observed_HR_clearance_alert',ids=[h['id'],'robot'],surface_gap_m=gap,threshold_m=.12))
            if gap<.8:
                pair=(h['id'],'robot');near.add(pair)
                if pair not in previous_near:events.append(dict(event='observed_robot_proximity_enter',time=t,ids=pair,surface_gap_m=gap))
        previous_near=near;last=frame
    for w in active:
        robot_input=dict(time=w['time'],state_time=w['state_time'],robot=w['robot_input'])
        intentions.append(dict(time=w['time'],state_time=w['state_time'],humans=w['motion_guard']['motion_contract'],native_group_intent=w.get('native_group_intent'),group_intent=w.get('group_intent'),behavior=w.get('behavior')))
        events.extend(dict(e,worker_time=w['time'],timestamp_semantics='time_active_s since first active controller interval') for e in w.get('behavior',{}).get('events',[]))
        events.extend(dict(e,worker_time=w['time']) for e in w.get('goal_clock',{}).get('events',[]) if e['event']!='checked')
        if w['motion_guard']['projection'].get('motion_limit_relaxed'):safety.append(dict(time=w['time'],event='motion_limit_relaxed',projection=w['motion_guard']['projection']))
    write_rows(case/'human_states.jsonl',humans);write_rows(case/'group_states.jsonl',groups);write_rows(case/'behavior_events.jsonl',events);write_rows(case/'human_intentions.jsonl',intentions);write_rows(case/'safety_events.jsonl',safety)
    write_rows(case/'robot_inputs.jsonl',[dict(time=w['time'],state_time=w['state_time'],**w['robot_input']) for w in worker if 'robot_input' in w]);write_rows(case/'pose_timeline.jsonl',timeline)
    dump(case/'result.json',dict(runtime_status=result['status'],process=json.loads((case/'process.json').read_text()),saved_native_result=str(case/'native/episode/result.json'),human_count=6,human_samples=len(humans),active_pose_frames=len(timeline),active_pose_window_s=[timeline[0]['time'],timeline[-1]['time']],active_pose_duration_s=timeline[-1]['time']-timeline[0]['time'],full_executed_worker_intervals=sum(w['state_time']<=timeline[-1]['time'] for w in active),worker_steps=len(active),observed_proximity_events=sum('proximity' in e['event'] for e in events),cleanup=result.get('cleanup'),gazebo_clean_exit=result.get('gazebo_clean_exit')))
    return timeline


def controller_timeline(path):
    rs=rows(path);out=[]
    def frame(r,before=False):
        hs=r['before'] if before else r['humans'];goals=r.get('goals',{})
        hs=[dict(h,goal=goals.get(str(h['id'])),behavior_state=next((v['state'] for v in r.get('behavior',{}).get('humans',[]) if v['id']==h['id']),None)) for h in hs]
        gs=[]
        for g in r['groups']:
            ps=[h for h in hs if h['id'] in g['members']];center=[sum(h[k] for h in ps)/len(ps) for k in ('x','y')]
            gs.append(dict(id=g['id'],members=g['members'],centroid=center,member_centroid_distances_m=[math.hypot(h['x']-center[0],h['y']-center[1]) for h in ps]))
        return dict(time=r['time'] if before else r['state_time'],humans=hs,groups=gs,robot=r['robot_input'])
    out.append(frame(rs[0],True))
    for i,r in enumerate(rs):
        value=frame(r)
        if i+1<len(rs):value['robot']=rs[i+1]['robot_input']
        else:value['robot']=dict(r['robot_input'],x=r['robot_input']['x']+.1*r['robot_input']['vx'],y=r['robot_input']['y']+.1*r['robot_input']['vy'])
        out.append(value)
    return out


def measure(timeline,root,source,mode):
    import sys
    sys.path.insert(0,str(root/'python'))
    from dynamuhsim.analysis.evidence import module_from_path
    geom=module_from_path(root/'experiments/social_navigation/navigation/socnavgym_warehouse_lifecycle/geometry.py','_diversity_measure_geometry')
    boxes=json.loads((root/'experiments/social_navigation/warehouse/layout.json').read_text())['obstacles']
    ts=np.array([r['time'] for r in timeline]);dt=np.diff(ts);speeds=[];acc=[];errors=[];yaw_total=0.;net_yaw={};path=0.;switches=0;stop_s=0.;stops=0;longest=0.;goal_progress=0.;spread=[];all_dist=[];agents={}
    for hid in [h['id'] for h in timeline[0]['humans']]:
        hs=[next(h for h in r['humans'] if h['id']==hid) for r in timeline];xy=np.array([[h['x'],h['y']] for h in hs]);yaw=np.array([h['yaw'] for h in hs]);ds=np.diff(xy,axis=0);vel=ds/dt[:,None];speed=np.linalg.norm(vel,axis=1);dyaw=wrap(np.diff(yaw));s=sum(np.linalg.norm(ds,axis=1));path+=s;speeds.extend(speed);a=np.linalg.norm(np.diff(vel,axis=0),axis=1)/((dt[1:]+dt[:-1])/2);acc.extend(a);err=abs(wrap(np.arctan2(vel[:,1],vel[:,0])-(yaw[:-1]+dyaw/2)));errors.extend(np.degrees(err[speed>.1]));yaw_total+=sum(abs(dyaw));net_yaw[str(hid)]=float(sum(dyaw));sign=np.sign((dyaw/dt)[abs(dyaw/dt)>.1]);switches+=int(sum(sign[1:]*sign[:-1]<0));run=0.;count=0;person_stop=0.;person_long=0.;progress=0.
        for i,(v,d) in enumerate(zip(speed,dt)):
            if v<.01:
                count+=run==0.;run+=d;person_stop+=d;person_long=max(person_long,run)
            else:run=0.
            g=hs[i].get('goal')
            if g is not None and g==hs[i+1].get('goal'):progress+=math.dist(xy[i],g)-math.dist(xy[i+1],g)
        goal_progress+=progress;stops+=count;stop_s+=person_stop;longest=max(longest,person_long)
        agents[str(hid)]=dict(path_m=float(s),goal_progress_same_target_m=progress,total_absolute_yaw_rad=float(sum(abs(dyaw))),net_yaw_rad=float(sum(dyaw)),stop_duration_s=person_stop,longest_stop_s=person_long,speed_mps=stats(speed),acceleration_mps2=stats(a))
    for r in timeline:
        for g in r['groups']:
            distances=g['member_centroid_distances_m'];spread.append(max(distances));all_dist.extend(distances)
    gaps={k:1e9 for k in ['pair','robot','static']};violations=0
    for a,z in zip(timeline,timeline[1:]):
        starts=[(h['x'],h['y']) for h in a['humans']];ds=[(q['x']-h['x'],q['y']-h['y']) for h,q in zip(a['humans'],z['humans'])];rp=(a['robot']['x'],a['robot']['y']);rd=(z['robot']['x']-rp[0],z['robot']['y']-rp[1]);values=geom.clearance(starts,ds,[.36]*len(starts),rp,rd,.25,boxes)
        for k in gaps:gaps[k]=min(gaps[k],values[k])
        violations+=min(values.values())<.002-1e-8
    return dict(mode=mode,source=str(source),source_sha256=sha(source),time_window_sim_s=[float(ts[0]),float(ts[-1])],duration_s=float(ts[-1]-ts[0]),frames=len(timeline),human_samples=sum(len(r['humans']) for r in timeline),path_length_m=float(path),goal_progress_same_target_m=goal_progress,speed_mps=stats(speeds),sample_vector_acceleration_mps2=stats(acc),total_absolute_yaw_rad=float(yaw_total),net_yaw_rad=net_yaw,heading_travel_error_deg_p95=stats(errors),yaw_sign_switches_gt_01radps=int(switches),stop_duration_person_s=float(stop_s),stop_episodes=int(stops),longest_stop_s=float(longest),group_spread_m=stats(spread),member_centroid_distance_m=stats(all_dist),linear_sample_swept_clearance_m=gaps,clearance_violation_segments=int(violations),per_agent=agents,behavior_occupancy_samples={state:sum(h.get('behavior_state')==state for r in timeline for h in r['humans']) for state in sorted({h.get('behavior_state') for r in timeline for h in r['humans'] if h.get('behavior_state') is not None})},not_measured=['stuck classifier','passing success classifier','regrouping success classifier','full robot mesh/contact clearance','reaction latency to perceived sensor data','human dataset realism'])


def render_pair(b,root,timelines,comparison="comparison",verification="playback_verification.json"):
    from PIL import Image,ImageDraw,ImageFont
    from matplotlib import pyplot as plt
    boxes=json.loads((root/'experiments/social_navigation/warehouse/layout.json').read_text())['obstacles'];colors=['#df453f','#287fc5','#e6a32f','#924bbc','#2e9e6c','#b66835']
    coords=[(h['x'],h['y']) for line in timelines.values() for r in line for h in r['humans']]+[(r['robot']['x'],r['robot']['y']) for line in timelines.values() for r in line]
    xmin=min(x for x,y in coords)-1.;xmax=max(x for x,y in coords)+1.;ymin=min(y for x,y in coords)-1.;ymax=max(y for x,y in coords)+1.
    width=500;height=540;scale=min(460/(xmax-xmin),440/(ymax-ymin));ox=(width-(xmax-xmin)*scale)/2;oy=80
    def transform(x,y):return ox+(x-xmin)*scale,oy+(ymax-y)*scale
    def interpolate(line,t):
        ts=[r['time']-line[0]['time'] for r in line];j=max(0,min(len(line)-2,bisect.bisect_right(ts,t)-1));a,z=line[j:j+2];alpha=max(0.,min(1.,(t-ts[j])/(ts[j+1]-ts[j])));out=[]
        for h,q in zip(a['humans'],z['humans']):out.append(dict(id=h['id'],x=h['x']+alpha*(q['x']-h['x']),y=h['y']+alpha*(q['y']-h['y']),yaw=float(h['yaw']+alpha*wrap(q['yaw']-h['yaw'])),state=h.get('behavior_state')))
        robot={k:a['robot'][k]+alpha*(z['robot'][k]-a['robot'][k]) for k in ['x','y']}
        return out,robot
    duration=max(line[-1]['time']-line[0]['time'] for line in timelines.values());frame_times=np.arange(0,duration+1e-8,.2);frames=[];single={name:[] for name in timelines}
    font=ImageFont.load_default(size=15)
    for t in frame_times:
        panels=[]
        for name,line in timelines.items():
            im=Image.new('RGB',(width,height),'white');d=ImageDraw.Draw(im);d.text((12,8),f'{name.upper()} - recorded Gazebo pose replay',fill='black',font=font);d.text((12,30),f'walking/crossing | seed 45 | active t={t:.1f}s',fill='black',font=font);d.text((12,51),f'absolute sim t={line[0]["time"]+min(t,line[-1]["time"]-line[0]["time"]):.3f}s',fill='black',font=font)
            for box in boxes:
                a=transform(box['x']-box['sx']/2,box['y']+box['sy']/2);z=transform(box['x']+box['sx']/2,box['y']-box['sy']/2);d.rectangle((*a,*z),fill='#dddddd',outline='#aaaaaa')
            hs,rp=interpolate(line,t)
            for h in hs:
                x,y=transform(h['x'],h['y']);rr=.36*scale;c=colors[(h['id']-1)%len(colors)];d.ellipse((x-rr,y-rr,x+rr,y+rr),fill=c,outline='black');d.line((x,y,x+rr*math.cos(h['yaw']),y-rr*math.sin(h['yaw'])),fill='black',width=2);d.text((x+rr,y),str(h['id']),fill='black',font=font)
            x,y=transform(rp['x'],rp['y']);rr=.25*scale;d.rectangle((x-rr,y-rr,x+rr,y+rr),fill='black');d.text((x+rr,y),'robot',fill='black',font=font)
            if t>line[-1]['time']-line[0]['time']:d.text((12,height-30),'END OF RECORDING (last pose held)',fill='red',font=font)
            # Palette frames keep memory bounded and do not claim camera footage.
            panel=im.convert('P',palette=Image.Palette.ADAPTIVE,colors=96);single[name].append(panel);panels.append(im)
        combined=Image.new('RGB',(width*2,height),'white');combined.paste(panels[0],(0,0));combined.paste(panels[1],(width,0));frames.append(combined.convert('P',palette=Image.Palette.ADAPTIVE,colors=128))
    media={}
    for name,fs in single.items():
        path=b/name/'media/recorded_pose_replay.gif';
        if not path.exists():fs[0].save(path,save_all=True,append_images=fs[1:],duration=200,loop=0,optimize=False)
        media[name]=path
    path=b/comparison/'replay_comparison.gif';assert not path.exists();frames[0].save(path,save_all=True,append_images=frames[1:],duration=200,loop=0,optimize=False);media['comparison']=path
    checks={}
    for name,path in media.items():
        with Image.open(path) as im:
            n=im.n_frames;milliseconds=0
            for i in range(n):im.seek(i);im.load();milliseconds+=im.info.get('duration',0)
        assert path.stat().st_size>0 and n>1 and milliseconds>0
        checks[name]=dict(path=str(path),bytes=path.stat().st_size,frames=n,duration_s=milliseconds/1000,sha256=sha(path),source_sha256={k:sha(b/k/'pose_timeline.jsonl') for k in timelines},kind='2D RECORDED GAZEBO POSE REPLAY; not 3D camera footage')
    reconstruction={}
    for name,line in timelines.items():
        xy=[];yaw=[]
        for r in line:
            hs,_=interpolate(line,r['time']-line[0]['time'])
            for h,q in zip(hs,r['humans']):xy.append(math.hypot(h['x']-q['x'],h['y']-q['y']));yaw.append(abs(float(wrap(h['yaw']-q['yaw']))))
        reconstruction[name]=dict(max_xy_error_m=max(xy),max_yaw_error_rad=max(yaw),keyframes=len(line),source_sha256=sha(b/name/'pose_timeline.jsonl'))
    dump(b/'replay'/verification,dict(media=checks,reconstruction=reconstruction,render_dt_s=.2,alignment='each native run aligned to its first active saved Gazebo pose; closed-loop robot inputs differ; not causal A/B'))
    fig,axs=plt.subplots(1,2,figsize=(12,7),sharex=True,sharey=True)
    for ax,(name,line) in zip(axs,timelines.items()):
        for box in boxes:ax.add_patch(plt.Rectangle((box['x']-box['sx']/2,box['y']-box['sy']/2),box['sx'],box['sy'],color='.88'))
        for hid in range(1,7):
            hs=[r['humans'][hid-1] for r in line];ax.plot([h['x'] for h in hs],[h['y'] for h in hs],color=colors[hid-1],label=str(hid))
        ax.plot([r['robot']['x'] for r in line],[r['robot']['y'] for r in line],'k--',label='robot');ax.set(title=name+' actual Gazebo poses',xlabel='World X (m)',ylabel='World Y (m)',xlim=(xmin,xmax),ylim=(ymin,ymax));ax.set_aspect('equal');ax.legend(ncol=4)
    fig.tight_layout();fig.savefig(b/comparison/'trajectories_before_after.png');plt.close(fig)
    for filename,kind in [('group_cohesion.png','spread'),('speed_acceleration.png','speed'),('behavior_timeline.png','behavior')]:
        fig,axs=plt.subplots(1,2,figsize=(12,4),sharey=True)
        for ax,(name,line) in zip(axs,timelines.items()):
            t=np.array([r['time']-line[0]['time'] for r in line])
            if kind=='spread':
                for gid in sorted({g['id'] for r in line for g in r['groups']}):
                    ax.plot(t,[next((max(g['member_centroid_distances_m']) for g in r['groups'] if g['id']==gid),np.nan) for r in line],label=f'group {gid}')
                ax.set_ylabel('Maximum member-centroid distance (m)')
            elif kind=='speed':
                for hid in range(1,7):
                    xy=np.array([[r['humans'][hid-1]['x'],r['humans'][hid-1]['y']] for r in line]);v=np.linalg.norm(np.diff(xy,axis=0),axis=1)/np.diff(t);ax.plot(t[1:],v,alpha=.7,label=f'human {hid}')
                ax.set_ylabel('Observed segment speed (m/s)')
            else:
                states=['walking','passing','overtaking','yielding','waiting','resuming','regrouping','conversation']
                for hid in range(1,7):ax.scatter(t,[states.index(r['humans'][hid-1]['behavior_state']) if r['humans'][hid-1].get('behavior_state') in states else -1 for r in line],s=2,label=f'human {hid}')
                ax.set_yticks(range(-1,len(states)),['NOT MEASURED']+states)
            ax.set_title(name+' actual execution');ax.set_xlabel('Active pose time (s)');ax.legend(fontsize=7)
        fig.tight_layout();fig.savefig(b/comparison/filename);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,4),sharey=True)
    for ax,(name,line) in zip(axs,timelines.items()):
        t=np.array([r['time']-line[0]['time'] for r in line]);dt=np.diff(t)
        for hid in range(1,7):
            xy=np.array([[r['humans'][hid-1]['x'],r['humans'][hid-1]['y']] for r in line]);v=np.diff(xy,axis=0)/dt[:,None];acc=np.linalg.norm(np.diff(v,axis=0),axis=1)/((dt[1:]+dt[:-1])/2);ax.plot(t[2:],acc,alpha=.6,label=f'human {hid}')
        ax.set_title(name+' sampled acceleration');ax.set_xlabel('Active pose time (s)');ax.set_ylabel('Vector acceleration (m/s²)');ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(b/comparison/'acceleration.png');plt.close(fig)
    return checks


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--experiment',type=Path,required=True);parser.add_argument('--skip-render',action='store_true');parser.add_argument('--candidate-case',default='candidate');parser.add_argument('--comparison-dir',default='comparison');parser.add_argument('--verification-name',default='playback_verification.json');a=parser.parse_args();b=a.experiment.resolve();root=Path(__file__).resolve().parents[1]
    native={};paired={}
    for name in ['baseline',a.candidate_case]:
        native[name]=rows(b/name/'pose_timeline.jsonl') if (b/name/'pose_timeline.jsonl').exists() else native_record(b/name);nm=measure(native[name],root,b/name/'native/episode/gazebo_poses.jsonl','ACTUAL_GAZEBO_POSES');
        if not (b/name/'metrics.json').exists():dump(b/name/'metrics.json',nm)
        line=controller_timeline(b/name/'controller.jsonl');m=measure(line,root,b/name/'controller.jsonl','DETERMINISTIC_CONTROLLER_REEXECUTION');rs=rows(b/name/'controller.jsonl');m['interval_acceleration_mps2']=stats([v for r in rs for v in r['motion_guard']['projection']['interval_acceleration_mps2']]);m['guard_residual_clearance_m']={k:min(r['motion_guard']['projection']['predicted_clearance'][k] for r in rs) for k in ['pair','robot','static']};m['predicted_clearance_m']={k:min(r['motion_guard']['projection']['predicted_clearance'][k]+(r['motion_guard'].get('robot_projection_radius_m',.25)-.25 if k=='robot' else 0.) for r in rs) for k in ['pair','robot','static']};m['robot_endpoint_semantics']='recorded next input at t+dt; final future endpoint extrapolated once from measured input velocity';m['motion_relaxation_steps']=sum(r['motion_guard']['projection']['motion_limit_relaxed'] for r in rs);m['behavior_events']=[dict(e,worker_time=r['time']) for r in rs for e in r.get('behavior',{}).get('events',[])];dump(b/a.comparison_dir/(name+'_controller_metrics.json'),m);paired[name]=m
    dump(b/a.comparison_dir/'metrics_before_after.json',dict(matched_controller=paired,native_closed_loop_matching=False,native_limitation='native live robot trajectories differ; compare causal effects only in controller matched-input replay'))
    flat=[('goal_progress_same_target_m',lambda m:m['goal_progress_same_target_m']),('path_length_m',lambda m:m['path_length_m']),('total_absolute_yaw_rad',lambda m:m['total_absolute_yaw_rad']),('heading_error_p95_deg',lambda m:m['heading_travel_error_deg_p95']['p95']),('yaw_sign_switches',lambda m:m['yaw_sign_switches_gt_01radps']),('interval_acceleration_max_mps2',lambda m:m['interval_acceleration_mps2']['max']),('interval_acceleration_p95_mps2',lambda m:m['interval_acceleration_mps2']['p95']),('group_spread_max_m',lambda m:m['group_spread_m']['max']),('group_spread_p95_m',lambda m:m['group_spread_m']['p95']),('predicted_HH_gap_m',lambda m:m['predicted_clearance_m']['pair']),('predicted_HR_gap_m',lambda m:m['predicted_clearance_m']['robot']),('replayed_robot_segment_HR_gap_m',lambda m:m['linear_sample_swept_clearance_m']['robot'])]
    with (b/a.comparison_dir/'metrics_before_after.csv').open('x',newline='') as f:
        w=csv.writer(f);w.writerow(['metric','baseline','candidate','candidate_minus_baseline'])
        for label,get in flat:w.writerow([label,get(paired['baseline']),get(paired[a.candidate_case]),get(paired[a.candidate_case])-get(paired['baseline'])])
    if not a.skip_render:render_pair(b,root,native,a.comparison_dir,a.verification_name)
    print((b/a.comparison_dir/'metrics_before_after.csv').read_text())


if __name__=='__main__':main()
