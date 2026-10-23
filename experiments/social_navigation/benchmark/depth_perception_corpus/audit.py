#!/usr/bin/env python3
"""Automated integrity/label QA, including sealed test; never model evaluation."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from export import HERE,BENCH,REPO,label_pixels,clusters,sha
from loader import load_input,load_label,INPUT_KEYS

def run(root,out):
 root=Path(root);cfg=json.loads((HERE/'protocol.json').read_text());scenes=json.loads((HERE/'scenes_LABEL_ONLY.json').read_text());manifest=json.loads((root/'manifest.json').read_text());frozen=json.loads((HERE/'frozen.json').read_text());assert all(sha(REPO/k)==h for k,h in frozen['source_sha256'].items());assert manifest['source_frozen_sha256']==sha(HERE/'frozen.json')
 reports=[];family_splits={};hash_split={};cal=json.loads((root/'inputs/calibration_SENSOR_ONLY.json').read_text());valid_scans=0;sample_count=0;max_unknown=0.;occluded=0
 for scene,summary in zip(scenes,manifest['scenes'],strict=True):
  sid=scene['scene_id'];assert summary['scene_id']==sid and sid in cfg['splits'][scene['split']];family_splits.setdefault(scene['family'],set()).add(scene['split']);raw=HERE/'generated/runs'/sid;cap=json.loads((raw/'result.json').read_text());assert cap['capture_accepted'] and cap['gazebo_exit']==0 and cap['cleanup']['remaining']==[] and cap['actual_nvidia_renderer'] and cap['callbacks_balanced'] and cap['full_control_bitwise'] and cap['motion_observer']['accepted'];assert sha(raw/'runtime/ogre2_session.log')==cap['cleanup']['ogre_log_sha256']
  prov=json.loads((root/'labels'/sid/'source_provenance_LABEL_ONLY.json').read_text());assert all(sha(raw/k)==h for k,h in prov['sha256'].items());assert all(sha(REPO/k)==h for k,h in cap['source_sha256'].items());assert cap['source_unchanged']
  d=np.load(root/'inputs'/sid/'depth_SENSOR_ONLY.npz');p=np.load(root/'inputs'/sid/'proposals_SENSOR_ONLY.npz');labels=np.load(root/'labels'/sid/'proposal_labels_LABEL_ONLY.npz');mask=np.load(root/'labels'/sid/'instance_pixels_LABEL_ONLY.npz')['person_instance'];laser=np.load(root/'inputs'/sid/'lidar_SENSOR_ONLY.npz');assert (root/'inputs'/sid/'lidar_SENSOR_ONLY.npz').read_bytes()==(raw/'scan_SENSOR_ONLY.npz').read_bytes();assert laser['ranges'].shape[1]==360 and np.all(laser['available_ns']>=laser['stamp_ns']) and np.all(np.diff(laser['stamp_ns'])>0)
  assert not labels['persona_valid'].any() and np.all(labels['persona_index']==-1);assert not manifest['world_model_training_ready'] and not manifest['classifier_validated'];assert p['point_offsets'][0]==0 and p['point_offsets'][-1]==len(p['points_xyz'])
  stamps=np.load(raw/'stamp_ns.npy');availability=json.loads((raw/'depth_available.json').read_text())['full'];counts={-1:0,0:0,1:0};cursor=0;fractions=[];unknowns=[]
  full=np.load(raw/'full.npy');assert full.tobytes()==np.load(raw/'control.npy').tobytes()
  for frame,index in enumerate(scene['selected_depth_indices']):
   assert d['depth'][frame].tobytes()==full[index].tobytes() and int(d['stamp_ns'][frame])==int(stamps[index]) and int(d['available_ns'][frame])==availability[index]
   # Millimetre-quantized fingerprint detects exact/near-identical whole frames across splits.
   fingerprint=np.where(np.isfinite(d['depth'][frame]),np.rint(np.minimum(d['depth'][frame],100)*1000),-1).astype('<i4');h=hashlib.sha256(fingerprint.tobytes()).hexdigest();assert h not in hash_split or hash_split[h]==scene['split'];hash_split[h]=scene['split']
   owner,qa=label_pixels(full[index],[np.load(raw/('drop'+str(i)+'.npy'),mmap_mode='r')[index] for i in range(1,7)],[np.load(raw/('only'+str(i)+'.npy'),mmap_mode='r')[index] for i in range(1,7)],np.load(raw/'null.npy',mmap_mode='r')[index]);np.testing.assert_array_equal(owner,mask[frame]);unknown=float(np.mean(owner<0));assert unknown<=cfg['label_rules']['max_unknown_pixel_fraction'];max_unknown=max(max_unknown,unknown);unknowns.append(unknown)
   for hid in range(1,7):
    alone=np.load(raw/('only'+str(hid)+'.npy'),mmap_mode='r')[index];denom=int((np.isfinite(alone)&(alone>=.28)&(alone<=5)).sum());visible=int((owner==hid).sum())
    if denom and visible>=20:
     fraction=visible/denom;fractions.append(fraction);occluded+=int(fraction<.9)
   groups=clusters(full[index],cal)
   for local,(points,uv) in enumerate(groups):
    a,b=p['point_offsets'][cursor:cursor+2];assert p['frame_index'][cursor]==frame and p['proposal_in_frame'][cursor]==local;np.testing.assert_array_equal(points.astype(np.float32),p['points_xyz'][a:b]);np.testing.assert_array_equal(uv,p['point_pixel_uv'][a:b]);owners=owner[uv[:,1],uv[:,0]];ids=np.unique(owners[owners>0]);nunknown=int((owners<0).sum());purity=max([int((owners==i).sum()) for i in ids],default=0)/len(owners)
    # Recompute class rules independently of exporter.classify.
    expected=0 if np.all(owners==0) else (1 if nunknown==0 and len(ids)==1 and (owners==ids[0]).sum()>=5 and purity>=.9 else -1);assert labels['class_index'][cursor]==expected and labels['label_valid'][cursor]==(expected>=0);assert labels['person_id'][cursor]==(ids[0] if expected==1 else -1);assert labels['unknown_point_count'][cursor]==nunknown;assert abs(float(labels['person_purity'][cursor])-purity)<1e-6
    sample=load_input(root,sid,cursor,allow_test=True);assert set(sample)==INPUT_KEYS;assert np.array_equal(sample['depth'],full[index],equal_nan=True);assert sample['depth_available_ns']>=sample['depth_stamp_ns'];valid_scans+=int(sample['scan_valid']);sample_count+=1
    if sample['scan_valid']:assert sample['scan_stamp_ns']<=sample['depth_stamp_ns'] and sample['scan_available_ns']<=sample['depth_available_ns'] and sample['depth_stamp_ns']-sample['scan_stamp_ns']<=250000000
    counts[expected]+=1;cursor+=1
  assert cursor==summary['proposals']==len(labels['class_index']);assert counts[1]==summary['person_samples'] and counts[0]==summary['environment_samples'] and counts[-1]==summary['ambiguous_samples'];assert len(d['depth'])==(5 if scene['moving'] else 1)
  if scene['split']=='test':
   try:load_input(root,sid,0)
   except PermissionError:pass
   else:raise AssertionError('Test not sealed')
  reports.append(dict(scene_id=sid,split=scene['split'],frames=len(d['depth']),proposals=cursor,person=counts[1],environment=counts[0],ambiguous=counts[-1],unknown_pixel_fraction=unknowns,motion=cap['motion_observer'],renderer=cap['cleanup']['ogre_renderer_lines'],source_sha256=sha(raw/'result.json'),min_in_view_person_visible_fraction=min(fractions,default=None)))
 assert all(len(v)==1 for v in family_splits.values());assert len(scenes)==len(manifest['scenes'])==7;assert sample_count>0 and valid_scans==sample_count
 # Input branch contains only declared sensor files, never counterfactual or world-state files.
 expected_files={'calibration_SENSOR_ONLY.json'}|{sid+'/'+name for sid in [s['scene_id'] for s in scenes] for name in ['depth_SENSOR_ONLY.npz','lidar_SENSOR_ONLY.npz','lidar_geometry_SENSOR_ONLY.json','proposals_SENSOR_ONLY.npz']};assert {str(p.relative_to(root/'inputs')) for p in (root/'inputs').rglob('*') if p.is_file()}==expected_files
 result=dict(status='PASS',scenes=reports,total_frames=sum(v['frames'] for v in reports),total_proposals=sample_count,causal_lidar_samples=valid_scans,max_unknown_pixel_fraction=max_unknown,visible_person_frame_instances_below_90pct_in_view=occluded,whole_family_splits=True,millimetre_quantized_cross_split_duplicate_frames=0,test_model_evaluations=0,test_previews=0,classifier_validated=False,world_model_training_ready=False)
 out=Path(out);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='scenes'},indent=2));return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--corpus',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.corpus,a.output)
