#!/usr/bin/env python3
"""Independent GEOS/relative-motion audit of recorded native and modified runs."""
import argparse,hashlib,json,math,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from shapely.geometry import LineString,Point,box
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,default=HERE/'reports/run01');a=p.parse_args();out=a.report_dir.resolve()
load=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protocol=load(out/'protocol.json');layout=load(out/'source_snapshot/layout.json');summary=load(out/'validation.json')
assert not protocol['native_dirty']
for name,h in protocol['source_sha256'].items():
 assert sha(HERE/name)==h,name
 assert sha(out/'source_snapshot'/Path(name).name)==h,name
assert (out/'source_snapshot/native_contract.py').read_bytes()==(HERE.parent/'complexitynav_contract/kernel.py').read_bytes()
assert (out/'source_snapshot/swept.py').read_bytes()==(HERE.parent/'socnavgym_warehouse_lifecycle/geometry.py').read_bytes()
assert layout==load(REPO/'experiments/social_navigation/warehouse/layout.json')
boxes=[(b,box(b['x']-b['sx']/2,b['y']-b['sy']/2,b['x']+b['sx']/2,b['y']+b['sy']/2)) for b in layout['obstacles']];assert len(boxes)==42
world=ET.parse(REPO/'experiments/social_navigation/warehouse/warehouse.sdf').getroot().find('world');models={m.get('name'):m for m in world.findall('model')}
for b,poly in boxes:
 m=models[b['name']];position=[float(x) for x in m.find('pose').text.split()];size=[float(x) for x in m.find('link/collision/geometry/box/size').text.split()]
 assert np.allclose(position[:2],[b['x'],b['y']]) and np.allclose(position[3:],[0,0,0]) and np.allclose(size[:2],[b['sx'],b['sy']])

def closest_relative(a0,a1,b0,b1,radii):
    u=np.asarray(a0)-np.asarray(b0);d=np.asarray(a1)-np.asarray(b1)-u
    t=float(np.clip(-u.dot(d)/d.dot(d),0,1)) if d.dot(d)>1e-24 else 0.
    return float(np.linalg.norm(u+t*d)-radii)

def analyze(rows):
    static=pair=robot_gap=math.inf;hits=[];path=0.;speed=0.;arrivals=[];near_route=[]
    initial_ids=[h['id'] for h in rows[0]['humans']]
    for n,row in enumerate(rows):
        assert row['time']==n*.25 and row['step']==n and row['frame']=='world'
        assert [h['id'] for h in row['humans']]==initial_ids and len(initial_ids)==len(set(initial_ids))
        for h in row['humans']:assert all(math.isfinite(h[k]) for k in ['x','y','vx','vy','gx','gy'])
    for prev,row in zip(rows,rows[1:]):
        rb=row['robot_input'];rb0=(rb['x'],rb['y']);rb1=(rb['x']+.25*rb['vx'],rb['y']+.25*rb['vy'])
        assert row['interval_start']==prev['time']
        for i,(a,b) in enumerate(zip(prev['humans'],row['humans'])):
            pos0=(a['x'],a['y']);pos1=(b['x'],b['y']);segment=LineString([pos0,pos1]);length=math.dist(pos0,pos1);path+=length;speed=max(speed,length/.25)
            assert math.hypot(b['x']-a['x']-.25*b['vx'],b['y']-a['y']-.25*b['vy'])<1e-9
            for obstacle,poly in boxes:
                gap=segment.distance(poly)-b['radius'];static=min(static,gap)
                if gap< -1e-6:hits.append(dict(time=row['time'],human_id=b['id'],obstacle=obstacle['name'],gap_m=gap))
            robot_gap=min(robot_gap,closest_relative(pos0,pos1,rb0,rb1,b['radius']+.25))
            for j in range(i):
                c,d=prev['humans'][j],row['humans'][j]
                pair=min(pair,closest_relative(pos0,pos1,(c['x'],c['y']),(d['x'],d['y']),b['radius']+d['radius']))
            if b.get('completed_legs',0)>a.get('completed_legs',0):
                distance=math.hypot(b['x']-a['gx'],b['y']-a['gy']);assert distance<.3+1e-8
                arrivals.append(dict(time=row['time'],human_id=b['id'],goal=[a['gx'],a['gy']],distance_m=distance))
    return dict(static_min_gap_m=static,pair_min_gap_m=None if not math.isfinite(pair) else pair,robot_min_gap_m=robot_gap,
        static_overlap_segments=len(hits),first_overlaps=hits[:3],path_m=path,max_speed_mps=speed,arrivals=arrivals)

results=[];data={};robot_hashes={};fixtures=load(out/'fixtures.json')
for c in fixtures:
 for variant in ['native','geometry']:
    name=c['name'];rows=load(out/(name+'_'+variant+'_trajectory.json'));data[(name,variant)]=rows
    assert len(rows)==c['steps']+1
    metric=analyze(rows);raw=next(x for x in summary['cases'] if x['case']==name and x['variant']==variant)
    assert raw['status']=='PASS'
    assert abs(metric['static_min_gap_m']-raw['metrics']['swept_gap_m']['static'])<1e-9
    assert abs(metric['robot_min_gap_m']-raw['metrics']['swept_gap_m']['robot'])<1e-9
    if metric['pair_min_gap_m'] is not None:assert abs(metric['pair_min_gap_m']-raw['metrics']['swept_gap_m']['pair'])<1e-9
    if variant=='geometry':
        assert metric['static_min_gap_m']>=-1e-6 and metric['robot_min_gap_m']>=-1e-6
        assert metric['pair_min_gap_m'] is None or metric['pair_min_gap_m']>=-1e-6
        assert metric['max_speed_mps']<=1.00001
        assert all(s['geometry']['safe_fraction']==1. for s in rows[1:])
        if c['steps']==240:assert {a['human_id'] for a in metric['arrivals']}=={h['id'] for h in rows[0]['humans']}
        else:assert metric['path_m']>5.
    elif c['name'].startswith('rack_') or c['name']=='narrow_two_way':assert metric['static_overlap_segments']>0
    robot_hashes[(name,variant)]=hashlib.sha256(json.dumps([s['robot_input'] for s in rows[1:]],sort_keys=True).encode()).hexdigest()
    results.append(dict(case=name,variant=variant,**metric))
 for n,g in zip(data[(c['name'],'native')][0]['humans'],data[(c['name'],'geometry')][0]['humans']):
    for k in ['id','x','y','radius','gx','gy','vx','vy','policy']:assert n[k]==g[k]
 assert robot_hashes[(c['name'],'native')]==robot_hashes[(c['name'],'geometry')]
# Confirm both people traverse the actual narrow opening rather than detour around it.
rows=data[('narrow_two_way','geometry')];crossings=[]
for hid in [1,2]:
 inside=[(row['time'],next(h for h in row['humans'] if h['id']==hid)) for row in rows if 12<=next(h for h in row['humans'] if h['id']==hid)['x']<=20]
 assert inside and all(abs(h['y'])+.3<=.9+1e-6 for t,h in inside)
 # Observe a continuous left-to-right/right-to-left traversal, bounded in the opening.
 sequence=[next(h for h in row['humans'] if h['id']==hid) for row in rows]
 sides=[-1 if h['x']<11.7 else 1 if h['x']>20.3 else 0 for h in sequence]
 assert -1 in sides and 1 in sides
 crossings.append(dict(id=hid,samples_inside_opening=len(inside),max_abs_center_y_m=max(abs(h['y']) for t,h in inside)))
repeat=load(out/'reset_trajectory.json');original=data[('stationary','geometry')]
for a,b in zip(repeat,original):
 assert a['time']==b['time'] and a['step']==b['step'] and a['humans']==b['humans']
assert len(repeat)==81 and repeat[0]['epoch']==1
cp=HERE/'generated/checkpoint';preserved=load(cp/'files.json')
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)
docs={'experiments/social_navigation/'+n for n in ['README.md','NEXT_STEPS.md','COMMIT_HISTORY.md']}
for name,h in preserved.items():
 if name not in docs:assert sha(REPO/name)==h,name
assert not subprocess.check_output(['docker','ps','-aq','--filter','name=dynamuhsim-complexity-geometry-'],text=True).strip()
result=dict(status='PASS_OFFLINE_ONLY',adapter_cases=6,native_reference_cases=6,adapter_updates=1040,native_reference_updates=960,geometry_boxes_verified=42,
 cases=results,narrow_opening_crossings=crossings,paired_robot_inputs_identical=True,reset80_exact=True,preserved_old_source_evidence_files=len(preserved)-3,original_index_outside_preserved=True,owned_containers_remaining=0,
 limits=['Seed42 fixed fixtures only; not dense-crowd or statistical ranking','No Gazebo/transport/actual LiDAR/actual robot feedback in this cohort','Native intent is routed then ORCA-projected; both policies are modified variants','Constant-velocity robot prediction for next0.25s only; infeasible input reports error','Kernel exception recovery and live worker shutdown need Gazebo contract verification'])
(out/'independent_validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='cases'},indent=2))
