#!/usr/bin/env python3
"""Independent geometry/trace audit; no native simulator, ROS or GPU startup."""
import argparse,hashlib,json,math,subprocess
from pathlib import Path
import numpy as np
from shapely.geometry import LineString,Point,box
import xml.etree.ElementTree as ET
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,default=HERE/'reports/run02');a=p.parse_args();out=a.report_dir.resolve()
load=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
layout=load(REPO/'experiments/social_navigation/warehouse/layout.json');boxes={b['name']:box(b['x']-b['sx']/2,b['y']-b['sy']/2,b['x']+b['sx']/2,b['y']+b['sy']/2) for b in layout['obstacles']}
ledger=load(out/'geometry_ledger.json');assert len(boxes)==len(ledger['boxes'])==42
for b in ledger['boxes']:
 assert np.allclose(np.array(b['vertices']),np.array(list(boxes[b['name']].exterior.coords)[:-1]),atol=1e-12)
# Verify against actual original Gazebo collision geometry too, not only two copies of layout JSON.
world=ET.parse(REPO/'experiments/social_navigation/warehouse/warehouse.sdf').getroot().find('world')
models={m.get('name'):m for m in world.findall('model')}
for b in layout['obstacles']:
 m=models[b['name']];pose=[float(v) for v in m.find('pose').text.split()];size=[float(v) for v in m.find('link/collision/geometry/box/size').text.split()]
 assert np.allclose(pose[:2],[b['x'],b['y']]) and np.allclose(pose[3:],[0,0,0])
 assert np.allclose(size[:2],[b['sx'],b['sy']])
protocol=load(out/'protocol.json')
for name,h in protocol['source_sha256'].items():
 if name.startswith('/work/'):assert sha(HERE/name[len('/work/'):])==h,name
validation=load(out/'validation.json');results=[];cases={}
for name in ['stationary','forward','lateral_then_forward','obstacle_orca','obstacle_socialforce']:
 rows=load(out/(name+'_trajectory.json'));cases[name]=rows;hits=[];min_gap=math.inf
 assert len(rows)==(41 if name.startswith('obstacle_') else 81)
 initial_ids=[h['id'] for h in rows[0]['humans']]
 for n,row in enumerate(rows):
  assert row['frame']=='world' and row['time']==n*.25 and row['step']==n
  assert [h['id'] for h in row['humans']]==initial_ids and len(set(initial_ids))==len(initial_ids)
  for h in row['humans']:assert all(math.isfinite(h[k]) for k in ['x','y','vx','vy','gx','gy'])
 for prev,row in zip(rows,rows[1:]):
  assert row['interval_start']==prev['time']
  for a,b in zip(prev['humans'],row['humans']):
   assert math.hypot(b['x']-a['x']-.25*b['vx'],b['y']-a['y']-.25*b['vy'])<1e-10
   seg=LineString([(a['x'],a['y']),(b['x'],b['y'])])
   for key,poly in boxes.items():
    gap=seg.distance(poly)-b['radius'];min_gap=min(min_gap,gap)
    if gap< -1e-8:hits.append(dict(time=row['time'],id=b['id'],obstacle=key,gap_m=gap))
 if name.startswith('obstacle_'):
  initial=rows[0]['humans'][0]
  assert min(Point(initial['x'],initial['y']).distance(poly)-initial['radius'] for poly in boxes.values())>=0
  target=[h for h in hits if h['obstacle']=='rack_north_0'];assert target
  expected=next(p for p in validation['obstacle_probes'] if p['policy']==name[len('obstacle_'):])
  assert target[0]['time']==expected['first_overlap']['time'] and len(target)==expected['overlap_segments']
 else:assert not hits
 results.append(dict(case=name,static_min_gap_m=min_gap,static_overlap_segments=len(hits),first_overlap=hits[0] if hits else None))
for name,rows in cases.items():
 for prev,row in zip(rows[1:-1],rows[2:]):
  a,b=prev['robot_input'],row['robot_input']
  assert math.hypot(b['x']-a['x']-.25*a['vx'],b['y']-a['y']-.25*a['vy'])<1e-10
# Input intervention changes human trajectories with identical initial conditions.
response={}
for name in ['forward','lateral_then_forward']:
 rows=cases[name];base=cases['stationary'];assert rows[0]['humans']==base[0]['humans']
 response[name]=max(math.hypot(a['x']-b['x'],a['y']-b['y']) for u,v in zip(rows,base) for a,b in zip(u['humans'],v['humans']))
 assert response[name]>.001
cp=HERE/'generated/checkpoint'
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)
intentional_docs={'experiments/social_navigation/'+n for n in ['README.md','NEXT_STEPS.md','COMMIT_HISTORY.md']}
preserved=load(cp/'files.json')
for name,h in preserved.items():
 if name not in intentional_docs:assert sha(REPO/name)==h,name
assert not subprocess.check_output(['docker','ps','-aq','--filter','name=dynamuhsim-complexity-contract-'],text=True).strip()
result=dict(status='NOT_READY_FOR_GAZEBO',trace_and_geometry_audit='PASS',geometry_ledger_boxes=42,native_internal_boxes_consumed=0,
 cases=results,robot_input_intervention_max_human_position_difference_m=response,
 original_index_and_outside_patch_preserved=True,preserved_source_and_evidence_files=len(preserved)-len(intentional_docs),
 intended_status_doc_updates=sorted(intentional_docs),owned_containers_remaining=0,
 parity_note='Strict same-frame native parity is asserted by the executed probe; reference trajectories were not separately exported. Frame translation differs by float32 rounding and is reported independently.')
(out/'independent_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
