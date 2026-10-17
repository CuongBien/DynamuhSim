#!/usr/bin/env python3
"""Analyze recorded runs and make previews; uses the round1 analysis venv only."""
import csv
import json
import math
from pathlib import Path
import statistics
import cv2
from PIL import Image, ImageDraw

HERE=Path(__file__).resolve().parent
REPORT=HERE/'reports'
results=[]
for path in sorted(REPORT.glob('*_dwa_n*_s*_*.json')):
    if path.stem.endswith('_trajectory'):
        continue
    data=json.loads(path.read_text())
    trajectory=json.loads(path.with_name(path.stem+'_trajectory.json').read_text())
    threshold_time=0.
    pair_overlap_steps=0
    for before,after in zip(trajectory,trajectory[1:]):
        dt=after['t']-before['t']
        rob=after['robot']
        near=any(math.hypot(rob['x']-h['x'],rob['y']-h['y'])<1.2 for h in after['humans'])
        if near: threshold_time+=dt
        if (after['min_human_human_clearance_m'] is not None and after['min_human_human_clearance_m']<0):
            pair_overlap_steps+=1
    counts=data['step_counts']
    results.append(dict(name=path.stem,scenario=data['scenario'],human_policy=data['human_policy'],
        test_case=data['test_case'],robot_visible=data['robot_visible'],status=data['status'],
        seconds=data['simulation_seconds'],steps=data['steps'],collision_steps=counts['Collision'],
        wall_constraint_steps=counts['WallCollision'],frozen_steps=counts['Frozen'],
        minimum_clearance_m=data['minimum_robot_human_clearance_m'],
        path_m=data['robot_path_m'],within_1p2m_seconds=threshold_time,human_pair_overlap_steps=pair_overlap_steps,
        collision_free_success=data['status']=='success' and counts['Collision']==0 and counts['WallCollision']==0))
with (REPORT/'results.csv').open('w',newline='') as stream:
    writer=csv.DictWriter(stream,fieldnames=list(results[0]))
    writer.writeheader()
    writer.writerows(results)
aggregates=[]
for scenario in ['circle','hallway','bottleneck']:
    for policy in ['orca_plus','sfm']:
        group=[r for r in results if r['scenario']==scenario and r['human_policy']==policy and r['robot_visible']]
        if group:
            aggregates.append(dict(scenario=scenario,human_policy=policy,cases=len(group),
                reached_goal=sum(r['status']=='success' for r in group),
                collision_free_success=sum(r['collision_free_success'] for r in group),
                timeout=sum(r['status']=='timeout' for r in group),
                sum_collision_steps=sum(r['collision_steps'] for r in group),
                sum_wall_constraint_steps=sum(r['wall_constraint_steps'] for r in group),
                sum_frozen_steps=sum(r['frozen_steps'] for r in group),
                median_time_s=statistics.median(r['seconds'] for r in group),
                minimum_clearance_m=min(r['minimum_clearance_m'] for r in group)))
pairs=[]
for policy in ['orca_plus','sfm']:
    for seed in [0,1,2]:
        prefix=f'circle_{policy}_dwa_n8_s{seed}_'
        vis_path=REPORT/(prefix+'visible.json')
        inv_path=REPORT/(prefix+'blind.json')
        if not vis_path.exists() or not inv_path.exists(): continue
        vis=json.loads(vis_path.read_text())
        inv=json.loads(inv_path.read_text())
        vrows=json.loads(vis_path.with_name(vis_path.stem+'_trajectory.json').read_text())
        irows=json.loads(inv_path.with_name(inv_path.stem+'_trajectory.json').read_text())
        differences=[]
        for v,i in zip(vrows[1:],irows[1:]):
            if abs(v['t']-i['t'])>1e-9: break
            differences.append(sum(math.hypot(a['x']-b['x'],a['y']-b['y']) for a,b in zip(v['humans'],i['humans']))/len(v['humans']))
        pairs.append(dict(policy=policy,test_case=seed,initial_state_identical=vis['initial_state']==inv['initial_state'],
            visible_status=vis['status'],blind_status=inv['status'],
            visible_collision_steps=vis['step_counts']['Collision'],blind_collision_steps=inv['step_counts']['Collision'],
            mean_human_position_difference_m=statistics.mean(differences) if differences else None))
video_stats={}
for path in sorted(REPORT.glob('*.mp4')):
    cap=cv2.VideoCapture(str(path))
    count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps=float(cap.get(cv2.CAP_PROP_FPS))
    okay,frame=cap.read()
    video_stats[path.name]=dict(frames=count,fps=fps,first_frame_decodes=okay)
    if okay:
        cap.set(cv2.CAP_PROP_POS_FRAMES,count//2)
        okay,frame=cap.read()
        if okay:
            cv2.imwrite(str(path.with_suffix('.png')),frame)
    preview_names=['official_demo','circle_orca_plus_dwa_n8_s0_visible','circle_sfm_dwa_n8_s0_visible',
                   'bottleneck_sfm_dwa_n3_s0_visible','bottleneck_orca_plus_dwa_n3_s2_visible','bottleneck_sfm_dwa_n3_s2_visible']
    if path.stem in preview_names:
        frames=[]
        for idx in range(0,count,2):
            cap.set(cv2.CAP_PROP_POS_FRAMES,idx)
            okay,frame=cap.read()
            if not okay: break
            im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)).resize((450,450))
            ImageDraw.Draw(im).text((8,432),'2x simulation speed; native timestep 0.25s',fill='black')
            frames.append(im)
        if frames:
            frames[0].save(path.with_name(path.stem+'_preview.gif'),save_all=True,append_images=frames[1:],
                duration=250,loop=0,optimize=True)
    cap.release()
summary=dict(cases=len(results),aggregates=aggregates,visibility_pairs=pairs,
    note='3 test cases per condition; endpoint/sample metrics, native scenes, DWA unicycle, no ROS integration.')
(REPORT/'analysis.json').write_text(json.dumps(summary,indent=2))
(REPORT/'video_validation.json').write_text(json.dumps(video_stats,indent=2))
print(json.dumps(summary,indent=2))
