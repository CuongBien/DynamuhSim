#!/usr/bin/env python3
"""Analyze native recordings and build compact previews, independently of simulator runtime."""
import csv
import json
import math
from pathlib import Path
import re
import statistics
import cv2
from PIL import Image, ImageDraw

HERE=Path(__file__).resolve().parent
REPORT=HERE/'reports'
results=[]
by_name={}
for path in sorted(REPORT.glob('*.json')):
    if not re.fullmatch(r'.+_s\d+_(visible|blind)',path.stem): continue
    data=json.loads(path.read_text())
    rows=json.loads(path.with_name(path.stem+'_trajectory.json').read_text())
    initial=rows[0]
    valid=initial['min_clearance_m']>=0 and initial['min_human_human_clearance_m']>=0
    pair_overlap=sum(r['min_human_human_clearance_m']<0 for r in rows[1:])
    within_personal=0.
    for before,after in zip(rows,rows[1:]):
        robot=after['robot']
        if any(math.hypot(h['x']-robot['x'],h['y']-robot['y'])<1.2 for h in after['humans']):
            within_personal+=after['t']-before['t']
    static=[]
    for h in initial['humans']:
        if h['policy']=='static':
            end=rows[-1]['humans'][h['id']]
            static.append(math.hypot(end['x']-h['x'],end['y']-h['y']))
    clusters=data['clustering_samples']
    result=dict(name=path.stem,condition=data['name'],seed=data['seed'],visible=data['robot_visible'],
        sampling_seed=data.get('sampling_seed',data['seed']),initial_sampling_attempts=data.get('initial_sampling_attempts',1),width=data['width'],humans=sum(data['counts']),density_per_m2=sum(data['counts'])/(data['width']*10),
        scenario=data['scenario'],status=data['status'],seconds=data['simulation_seconds'],
        min_clearance_m=data['minimum_clearance_m'],min_human_pair_clearance_m=data['min_human_human_clearance_m'],
        human_pair_overlap_steps=pair_overlap,frozen_steps=data['frozen_steps'],wall_block_steps=data['wall_block_steps'],
        initial_geometry_valid=valid,within_1p2m_seconds=within_personal,
        samples_detecting_multi_person_clusters=sum(c['multi_person_clusters']>0 for c in clusters),
        clustering_samples=len(clusters),max_static_displacement_m=max(static) if static else None,
        render_error=data.get('render_error'))
    results.append(result)
    by_name[path.stem]=(data,rows)
with (REPORT/'results.csv').open('w',newline='') as stream:
    writer=csv.DictWriter(stream,fieldnames=list(results[0]))
    writer.writeheader()
    writer.writerows(results)
aggregates=[]
for condition in ['sparse','baseline','dense','narrow','medium_width','orca_only','heterogeneous','passing','crossing']:
    rs=[r for r in results if r['condition']==condition and r['visible']]
    aggregates.append(dict(condition=condition,cases=len(rs),success=sum(r['status']=='Reaching goal' for r in rs),
        collision=sum(r['status']=='Collision' for r in rs),timeout=sum(r['status']=='Timeout' for r in rs),
        invalid_initial_states=sum(not r['initial_geometry_valid'] for r in rs),
        median_seconds=statistics.median(r['seconds'] for r in rs),
        min_clearance_m=min(r['min_clearance_m'] for r in rs),
        min_human_pair_clearance_m=min(r['min_human_pair_clearance_m'] for r in rs),
        sum_human_pair_overlap_steps=sum(r['human_pair_overlap_steps'] for r in rs)))
pairs=[]
for seed in [0,1,2]:
    vis,vr=by_name[f'baseline_s{seed}_visible']
    inv,ir=by_name[f'baseline_s{seed}_blind']
    differences=[]
    for v,i in zip(vr[1:],ir[1:]):
        differences.append(statistics.mean(math.hypot(a['x']-b['x'],a['y']-b['y']) for a,b in zip(v['humans'],i['humans'])))
    pairs.append(dict(seed=1000+seed,initial_state_identical=vis['initial_state']==inv['initial_state'],
        visible_status=vis['status'],blind_status=inv['status'],
        mean_human_position_difference_m=statistics.mean(differences)))
videos={}
for path in sorted(REPORT.glob('*.mp4')):
    cap=cv2.VideoCapture(str(path))
    count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps=cap.get(cv2.CAP_PROP_FPS)
    okay,frame=cap.read()
    videos[path.name]=dict(frames=count,fps=fps,first_frame_decodes=okay)
    if okay:
        cap.set(cv2.CAP_PROP_POS_FRAMES,count//2)
        ok,frame=cap.read()
        if ok: cv2.imwrite(str(path.with_suffix('.png')),frame)
    if path.stem in ['compatible_native_demo','baseline_s0_visible','dense_s0_visible','narrow_s0_visible','heterogeneous_s0_visible']:
        frames=[]
        for idx in range(0,count,2):
            cap.set(cv2.CAP_PROP_POS_FRAMES,idx)
            ok,frame=cap.read()
            if not ok: break
            im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)).resize((450,450))
            panel=Image.new('RGB',(450,480),'white')
            panel.paste(im,(0,0))
            ImageDraw.Draw(panel).text((8,455),path.stem+' | 2x simulation speed',fill='black')
            frames.append(panel)
        if frames:
            frames[0].save(path.with_name(path.stem+'_preview.gif'),save_all=True,append_images=frames[1:],
                duration=250,loop=0,optimize=True)
    cap.release()
log=(REPORT/'compatible_native_demo.log').read_text()
match=re.search(r'It takes ([0-9.]+) seconds to finish\. Final status is ([^,]+)',log)
(REPORT/'compatible_native_demo.json').write_text(json.dumps(dict(
    entrypoint='upstream crowd_nav/test.py',policy='vecmpc with native CV predictor',
    config='external compatible_demo_config.py',source_unmodified=True,
    status=match.group(2) if match else 'unknown',simulation_seconds=float(match.group(1)) if match else None,
    changes=['container-only symlink for hardcoded config path','external concrete config classes and CV settings',
             'external launcher returns env.unwrapped for positional reset']),indent=2))
summary=dict(cases=len(results),aggregates=aggregates,visibility_pairs=pairs,
    invalid_initial_states=[r['name'] for r in results if not r['initial_geometry_valid']],
    rendering_errors=[r['name'] for r in results if r['render_error']],
    note='Small 2D exploratory sample with fixed ORCA robot. Width conditions also differ in human count/density. No cross-framework ranking.')
(REPORT/'analysis.json').write_text(json.dumps(summary,indent=2))
(REPORT/'video_validation.json').write_text(json.dumps(videos,indent=2))
print(json.dumps(summary,indent=2))
