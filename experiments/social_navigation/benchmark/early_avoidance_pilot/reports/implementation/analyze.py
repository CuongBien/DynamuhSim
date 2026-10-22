#!/usr/bin/env python3
"""Predeclared actual-goal, geometry, group and sensor checks; preserve every failed attempt."""
import argparse,bisect,hashlib,json,math,re,shutil
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from geometry import hull,distance_to_hull
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent

def rows(p):return [json.loads(line) for line in p.read_text().splitlines()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def analyze(root,out):
    out.mkdir(parents=True,exist_ok=False);r=json.loads((root/'result.json').read_text());poses=rows(root/'gazebo_poses.jsonl');gt=rows(root/'ground_truth.jsonl');gtt=[x['time'] for x in gt];protocol=json.loads((HERE/'protocol.json').read_text())
    active=[p for p in poses if p['phase']=='active'];gaps=[];swept=[];group_rows=[];group_coverage=0
    for i,p in enumerate(poses):
        if 'mecanum_robot' not in p['poses']:continue
        robot=p['poses']['mecanum_robot'];humans={int(n[7:-6]):h for n,h in p['poses'].items() if n.startswith('common_') and n.endswith('_proxy')}
        if p['phase'] not in ['active','label_tail']:continue
        for ident,h in humans.items():gaps.append(math.hypot(h['x']-robot['x'],h['y']-robot['y'])-.61)
        if i:
            previous=poses[i-1];rob0=previous['poses'].get('mecanum_robot')
            if rob0:
                for ident,h in humans.items():
                    before=previous['poses'].get(f'common_{ident}_proxy')
                    if before:
                        a=np.array([before['x']-rob0['x'],before['y']-rob0['y']]);b=np.array([h['x']-robot['x'],h['y']-robot['y']]);d=b-a;norm=float(d@d);t=0. if norm==0 else max(0.,min(1.,-float(a@d)/norm))
                        swept.append(float(np.linalg.norm(a+t*d))-.61)
        prior=max(0,bisect.bisect_right(gtt,p['time'])-1);groups=gt[prior].get('groups',[])
        for group in groups:
            if group['type']!='standing' or len(group['members'])<3:continue
            members=[humans[ident] for ident in group['members'] if ident in humans]
            if len(members)!=len(group['members']):continue
            polygon=hull([(h['x'],h['y']) for h in members]);distance=distance_to_hull((robot['x'],robot['y']),polygon)
            group_rows.append(dict(time=p['time'],group_id=group['id'],distance_to_hull_m=distance,center_intrusion=distance==0.,robot_disc_intrusion=distance<=.25,phase=p['phase']))
    index=json.loads((root/'paired_depth/index.json').read_text());stamps=sorted(set(index['on'])&set(index['off']),key=int);effects=[];hashes={}
    for stamp in stamps:
        a,b=[np.load(root/'paired_depth'/name/(stamp+'.npy'),allow_pickle=False) for name in ['on','off']]
        assert a.shape==b.shape==(240,424) and a.dtype==b.dtype==np.float32
        good=np.isfinite(a)&np.isfinite(b);delta=np.zeros_like(a);np.subtract(b,a,out=delta,where=good)
        effects.append(dict(stamp_ns=int(stamp),phase=index['off'][stamp]['phase'],changed_pixels=int(np.sum((good&(np.abs(delta)>.02))|(np.isfinite(a)!=np.isfinite(b))))))
        for name in ['on','off']:
            f=root/'paired_depth'/name/(stamp+'.npy');hashes[str(f.relative_to(root))]=sha(f)
    commands=rows(root/'commands.jsonl');human_cap=[];free_human=[]
    worker=BENCH/'dataset_pilot/generated'/root.name/'worker_trajectory.jsonl'
    for row in rows(worker):
        guard=row.get('motion_guard')
        if not guard:continue
        reactive=set(guard.get('reactive_standing_ids',[]))
        for h in row['humans']:
            (human_cap if h['id'] in reactive else free_human).append(h.get('speed',0.))
    log=(root/'launch.log').read_text(errors='replace');match=re.search(r'teardown pre=(\d+) post=(\d+)',log);balanced=bool(match and match[1]==match[2] and int(match[1])>0)
    events=rows(root/'events.jsonl');eventmap={e['event']:e['stamp_ns']/1e9 for e in events};tail=eventmap.get('observation_end',0)-eventmap.get('action_end',0)
    q=r.get('server_shutdown',{}).get('quiesce',{})
    checks=dict(actual_goal=bool(r.get('actual_goal_reached')),no_adapter_fault=not r['faults'],no_nominal_overlap_over_5mm=bool(swept and min(swept)>=-.005),
        no_standing_group_center_hull_intrusion=bool(group_rows and not any(g['center_intrusion'] for g in group_rows)),
        label_tail=tail>=3.,gpu=any('NVIDIA' in line for line in r['cleanup']['ogre_renderer_lines']),
        clean_shutdown=bool(r['gazebo_clean_exit'] and r['cleanup']['status']=='PASS' and not r['cleanup']['remaining'] and q.get('pause_acknowledged') and q.get('simulation_time_stable')),
        callback_balance=balanced,paired_active_observations=sum(e['phase']=='active' for e in effects)>=50,
        native_standing_speed_cap=not human_cap or max(human_cap)<=.2+1e-7)
    summary=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,navigation_status=r.get('navigation_status'),actual_goal_reached=r.get('actual_goal_reached'),goal_xy_error_m=r.get('goal_xy_error_world_m'),goal_yaw_error_rad=r.get('goal_yaw_error_world_rad'),
        action_active_sim_s=r.get('active_sim_seconds'),budget_sim_s=protocol['budget']['sim_s'],complete_label_tail_s=tail,
        nominal_gap_min_sampled_m=min(gaps) if gaps else None,nominal_gap_min_swept_linear_m=min(swept) if swept else None,
        group_center_intrusion_pose_count=sum(g['center_intrusion'] for g in group_rows),group_disc_intrusion_pose_count=sum(g['robot_disc_intrusion'] for g in group_rows),standing_group_pose_evaluations=len(group_rows),
        paired_frames=len(stamps),active_paired_frames=sum(e['phase']=='active' for e in effects),active_effect_frames=sum(e['phase']=='active' and e['changed_pixels']>=100 for e in effects),
        callback_pre=int(match[1]) if match else None,callback_post=int(match[2]) if match else None,
        max_reactive_standing_human_speed_mps=max(human_cap) if human_cap else 0.,max_other_human_speed_mps=max(free_human) if free_human else 0.,reactive_standing_samples=len(human_cap),
        max_command_speed_mps=max(math.hypot(c['vx'],c['vy']) for c in commands) if commands else None,
        robot_trajectory=[dict(time=x['time'],phase=x['phase'],**x['poses']['mecanum_robot']) for x in poses if 'mecanum_robot' in x['poses']],
        source_invariants='Audit separately; no native policy/radius/sensor threshold changes',training_ready=False,personas_valid=False,
        caveats=protocol['limitations'],paired_raw_sha256=hashes,error=r.get('error'),faults=r['faults'])
    dump(out/'summary.json',summary);dump(out/'group_metrics.json',group_rows);dump(out/'pair_metrics.json',effects);dump(out/'protocol.json',protocol)
    # Plot measured Gazebo positions, not predictions.
    board=Image.new('RGB',(900,700),'white');draw=ImageDraw.Draw(board);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    points=[(v['x'],v['y']) for v in summary['robot_trajectory']];xmin=min(-29.,min(x for x,y in points)-1);xmax=max(-19.,max(x for x,y in points)+1);ymin=min(-2.,min(y for x,y in points)-1);ymax=max(2.,max(y for x,y in points)+1)
    scale=min(800/(xmax-xmin),560/(ymax-ymin))
    def pixel(x,y):return (50+(x-xmin)*scale,630-(y-ymin)*scale)
    for x in np.arange(math.ceil(xmin),xmax):
        a,b=pixel(x,ymin),pixel(x,ymax);draw.line([a,b],fill='#dddddd');draw.text((a[0],640),str(round(x)),font=font,fill='black')
    for y in np.arange(math.ceil(ymin),ymax):draw.line([pixel(xmin,y),pixel(xmax,y)],fill='#dddddd')
    initial=poses[0]['poses']
    for n,h in initial.items():
        if not n.endswith('_proxy'):continue
        x,y=pixel(h['x'],h['y']);rad=.36*scale;draw.ellipse((x-rad,y-rad,x+rad,y+rad),outline='#c45800',width=2);draw.text((x+4,y+4),n[7:-6],font=font,fill='#993300')
    draw.line([pixel(x,y) for x,y in points],fill='#2266cc',width=3)
    for label,x,y,color in [('start',-28,0,'#116622'),('goal',-20,0,'#cc2222')]:
        a,b=pixel(x,y);draw.ellipse((a-5,b-5,a+5,b+5),fill=color);draw.text((a+7,b-22),label,font=font,fill=color)
    draw.text((20,10),'Actual Gazebo path; orange circles = initial people, blue = robot',font=font,fill='black')
    draw.text((20,35),f"{summary['status']}: action={summary['navigation_status']}, actual goal={summary['actual_goal_reached']}",font=font,fill='black')
    draw.text((20,60),'No oracle group information supplied to Nav2; diagnostic metric only.',font=font,fill='black');board.save(out/'actual_path.png')
    for name in ['initial.png','middle.png','final.png','camera_preview_4x.gif','result.json','events.jsonl','live_parameters.json','config_changes.json']:
        if (root/name).exists():shutil.copyfile(root/name,out/name)
    # Keep one actual paired float32 image for audit/review.
    useful=[e for e in effects if e['phase']=='active' and e['changed_pixels']>=100]
    chosen=(useful[len(useful)//2] if useful else effects[len(effects)//2])['stamp_ns']
    np.savez_compressed(out/'representative_depths.npz',stamp_ns=np.array(chosen,dtype=np.int64),**{n:np.load(root/'paired_depth'/n/(str(chosen)+'.npy')) for n in ['on','off']})
    dump(out/'quarantine.json',dict(eligible_for_training=False,complete_navigation=checks['actual_goal'],reason='Experimental configuration; personas/association/future BEV not validated' if summary['status']=='PASS' else 'At least one predeclared acceptance check failed'))
    print(json.dumps({k:v for k,v in summary.items() if k not in ['robot_trajectory','paired_raw_sha256','faults','caveats']},indent=2));return summary
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=analyze(a.capture.resolve(),a.output.resolve())
    raise SystemExit(0 if result['status']=='PASS' else 1)
