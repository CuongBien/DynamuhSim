#!/usr/bin/env python3
"""Independent label/metric preservation checks, no live simulator."""
import argparse,hashlib,json,math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from fixture import HERE,REPO,BENCH

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(capture,analysis,out):
 out.mkdir(parents=True,exist_ok=False);result=json.loads((capture/'result.json').read_text());ev=json.loads((analysis/'evaluation.json').read_text());assert result['capture_accepted']
 protocol=json.loads((HERE/'protocol.json').read_text());eps=protocol['depth_equality_tolerance_m'];owners=np.load(analysis/'person_instance_LABEL_ONLY.npy',allow_pickle=False)
 arrays={m:np.load(capture/(m+'.npy'),allow_pickle=False) for m in protocol['modes']};stamps=np.load(capture/'stamp_ns.npy',allow_pickle=False);assert len(stamps)==protocol['frames'] and np.all(np.diff(stamps)>0)
 scalar_checks=0;coordinate_sets=0
 for frame in range(len(stamps)):
  full=arrays['full'][frame];assert full.tobytes()==arrays['control'][frame].tobytes();assert np.array_equal(np.isfinite(full),np.isfinite(arrays['full'][0]))
  for mode,a in arrays.items():
   finite=np.isfinite(a[frame]);assert np.array_equal(finite,np.isfinite(a[0]));assert np.max(np.abs(a[frame][finite]-a[0][finite]),initial=0)<=protocol['static_depth_tolerance_m']
  # Scalar implementation of ray evidence on every changed/owned pixel, plus fixed background grid.
  select=owners!=0
  for i in range(1,7):
   d=arrays['drop'+str(i)][frame];finite=np.isfinite(full);delta=np.zeros(full.shape);np.subtract(d,full,out=delta,where=finite&np.isfinite(d));select|=finite&((~np.isfinite(d))|(delta>eps))
  select[::11,::11]=True
  for v,u in np.argwhere(select):
   f=float(full[v,u]);possible=[];contradiction=False
   for i in range(1,7):
    d=float(arrays['drop'+str(i)][frame,v,u]);a=float(arrays['only'+str(i)][frame,v,u])
    effect=math.isfinite(f) and (not math.isfinite(d) or d-f>eps)
    match=math.isfinite(f) and math.isfinite(a) and abs(f-a)<=eps
    if effect and not match:contradiction=True
    if effect and match:possible.append(i)
   expected=-1 if contradiction or len(possible)>1 else possible[0] if possible else 0
   assert int(owners[v,u])==expected
   scalar_checks+=1
 # Recount proposal categories and metrics from saved point-label histograms.
 metric_checks=0
 for name,variant in ev['evaluation'].items():
  rows=variant['proposals'];clean=[];categories={k:0 for k in variant['counts']}
  for row in rows:
   counts={int(k):v for k,v in row['label_counts'].items()};assert sum(counts.values())==row['point_count'];humans=[i for i in range(1,7) if counts.get(i,0)>=protocol['proposal_person_support_min_points']]
   purity=max([counts.get(i,0) for i in range(1,7)],default=0)/row['point_count']
   category='merged_humans' if len(humans)>1 else 'pure_person' if len(humans)==1 and purity>=protocol['proposal_purity'] else 'non_person' if counts.get(0,0)/row['point_count']>=protocol['proposal_purity'] else 'mixed_or_uncertain'
   assert row['classification']==category;categories[category]+=1
   if category=='pure_person':clean.append(humans[0])
   metric_checks+=1
  assert categories==variant['counts'];expected_precision=len(clean)/len(rows) if rows else None;assert variant['instance_precision']==expected_precision
  if not name.startswith('null'):
   detected=sorted(set(clean)&set(variant['eligible_people']));assert detected==variant['detected_people'];assert sorted(set(variant['eligible_people'])-set(detected))==variant['missed_people']
 frozen=json.loads((BENCH/'depth_identity_guard/reports/final_engine/frozen.json').read_text());
 # All sources recorded before accepted capture still byte-identical.
 assert all(sha(REPO/p)==h for p,h in result['source_sha256'].items())
 assert all(sha(REPO/p)==h for p,h in ev['source_sha256'].items())
 assert all(sha(capture/p)==h for p,h in ev['capture_files_sha256'].items())
 # Camera physical origin and rotation are inherited from original robot and static robot z=.04.
 urdf=ET.parse(capture/'runtime/robot_low.urdf').getroot();assert np.array_equal(np.fromstring(urdf.find("joint[@name='camera_joint']/origin").get('xyz'),sep=' '),[.12,0,.08])
 world=ET.parse(capture/'runtime/warehouse.sdf').getroot().find('world');assert world.find("model[@name='mecanum_robot']").findtext('static')=='true';assert len(world.findall('actor'))==6
 original=json.loads((HERE/'generated/checkpoint/files.json').read_text());assert all(sha(REPO/p)==h for p,h in original.items())
 report=dict(status='PASS',scalar_pixel_checks=scalar_checks,proposal_histograms_recounted=metric_checks,frames_checked=len(stamps),images_checked=sum(len(a) for a in arrays.values()),pre_capture_source_hashes_verified=len(result['source_sha256']),evaluation_source_hashes_verified=len(ev['source_sha256']),preexisting_files_unchanged=len(original),paired_control_bitwise=True,strict_interframe_bitwise_gate_pass=result['strict_interframe_bitwise_gate_pass'],numeric_static_gate_pass=True,calibrated_camera_origin_verified=True,semantic_person_validated=False,association_accepted=False,training_ready=False,scope='Pixel ownership and saved diagnostic metric integrity only; not semantic model acceptance',capture_result_sha256=sha(capture/'result.json'),evaluation_sha256=sha(analysis/'evaluation.json'))
 (out/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--analysis',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.capture.resolve(),a.analysis.resolve(),a.output.resolve())
