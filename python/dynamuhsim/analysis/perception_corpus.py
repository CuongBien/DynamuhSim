"""Read-only final corpus01 QA: retain missing-scan masks; no model evaluation."""
import hashlib,json
from pathlib import Path
from dynamuhsim.workspace import source_path, stage_directory
import numpy as np
from .evidence import load_json,sha256_file as sha,resolve_sources,module_from_path
from dynamuhsim.perception.corpus import STAGE,corpus_math
RECORDED_SOURCE_REVISION='9833565fa546d55cb0e553f8388bf4a703c59583'

def audit_corpus(root, *, repository, raw_root, source_roots=(), source_revision=RECORDED_SOURCE_REVISION):
 root=Path(root).resolve();repository=Path(repository).resolve();stage=stage_directory(repository,"depth_perception_corpus");raw_root=Path(raw_root).resolve();freeze=load_json(stage/'frozen.json');manifest=load_json(root/'manifest.json');origins={}
 if manifest['source_frozen_sha256']!=sha(stage/'frozen.json'):raise ValueError('Corpus source receipt identity differs')
 def verify(expected=None):
  expected=freeze['source_sha256'] if expected is None else expected
  found,missing=resolve_sources(expected,roots=[repository,*source_roots],git_repo=repository,commit=source_revision)
  if missing:raise ValueError('Historical source unavailable: '+', '.join(missing))
  origins.update({name:value['origin'] for name,value in found.items()})
 verify()
 for name in ['loader.py','protocol.json','scenes_LABEL_ONLY.json']:
  path=STAGE/name
  if sha(source_path(repository,str(path)))!=freeze['source_sha256'][str(path)]:raise ValueError('Active corpus contract differs from frozen source: '+name)
 math=corpus_math(repository);loader=module_from_path(stage/'loader.py','verified_corpus_loader')
 with np.errstate(invalid='ignore'):
  result=_audit(root,stage,raw_root,math.label_pixels,math.clusters,loader.load_input,loader.INPUT_KEYS,verify)
 verify();return result,dict(recorded_source_revision=source_revision,source_origins=origins,historical_source_executed=False,active_math='Current tracked loader and frozen-hash label/proposal math')

def _audit(root, HERE, RAW, label_pixels, clusters, load_input, INPUT_KEYS, verify):
 root=Path(root);cfg=json.loads((HERE/'protocol.json').read_text());scenes=json.loads((HERE/'scenes_LABEL_ONLY.json').read_text());manifest=json.loads((root/'manifest.json').read_text());verify();assert manifest['source_frozen_sha256']==sha(HERE/'frozen.json')
 reports=[];family_splits={};hash_split={};cal=json.loads((root/'inputs/calibration_SENSOR_ONLY.json').read_text());valid_scans=0;sample_count=0;max_unknown=0.;occluded=0
 for scene,summary in zip(scenes,manifest['scenes'],strict=True):
  sid=scene['scene_id'];assert summary['scene_id']==sid and sid in cfg['splits'][scene['split']];family_splits.setdefault(scene['family'],set()).add(scene['split']);raw=RAW/sid;cap=json.loads((raw/'result.json').read_text());assert cap['capture_accepted'] and cap['gazebo_exit']==0 and cap['cleanup']['remaining']==[] and cap['actual_nvidia_renderer'] and cap['callbacks_balanced'] and cap['full_control_bitwise'] and cap['motion_observer']['accepted'];assert sha(raw/'runtime/ogre2_session.log')==cap['cleanup']['ogre_log_sha256']
  prov=json.loads((root/'labels'/sid/'source_provenance_LABEL_ONLY.json').read_text());assert all(sha(raw/k)==h for k,h in prov['sha256'].items());verify(cap['source_sha256']);assert cap['source_unchanged']
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
    eligible=np.flatnonzero((laser['stamp_ns']<=sample['depth_stamp_ns'])&(laser['available_ns']<=sample['depth_available_ns'])&(sample['depth_stamp_ns']-laser['stamp_ns']<=250000000));assert sample['scan_valid']==bool(len(eligible))
    if sample['scan_valid']:
     j=eligible[np.argmax(laser['stamp_ns'][eligible])];assert sample['scan_stamp_ns']==int(laser['stamp_ns'][j]) and sample['scan_available_ns']==int(laser['available_ns'][j]);np.testing.assert_array_equal(sample['scan_ranges'],laser['ranges'][j])
    else:assert sample['scan_stamp_ns']==-1 and sample['scan_available_ns']==-1 and np.isnan(sample['scan_ranges']).all()
    counts[expected]+=1;cursor+=1
  assert cursor==summary['proposals']==len(labels['class_index']);assert counts[1]==summary['person_samples'] and counts[0]==summary['environment_samples'] and counts[-1]==summary['ambiguous_samples'];assert len(d['depth'])==(5 if scene['moving'] else 1)
  if scene['split']=='test':
   try:load_input(root,sid,0)
   except PermissionError:pass
   else:raise AssertionError('Test not sealed')
  reports.append(dict(scene_id=sid,split=scene['split'],frames=len(d['depth']),proposals=cursor,person=counts[1],environment=counts[0],ambiguous=counts[-1],unknown_pixel_fraction=unknowns,motion=cap['motion_observer'],renderer=cap['cleanup']['ogre_renderer_lines'],source_sha256=sha(raw/'result.json'),min_in_view_person_visible_fraction=min(fractions,default=None)))
 assert all(len(v)==1 for v in family_splits.values());assert len(scenes)==len(manifest['scenes'])==7;assert sample_count>0 and valid_scans>0
 # Input branch contains only declared sensor files, never counterfactual or world-state files.
 expected_files={'calibration_SENSOR_ONLY.json'}|{sid+'/'+name for sid in [s['scene_id'] for s in scenes] for name in ['depth_SENSOR_ONLY.npz','lidar_SENSOR_ONLY.npz','lidar_geometry_SENSOR_ONLY.json','proposals_SENSOR_ONLY.npz']};assert {str(p.relative_to(root/'inputs')) for p in (root/'inputs').rglob('*') if p.is_file()}==expected_files
 result=dict(status='PASS',scenes=reports,total_frames=sum(v['frames'] for v in reports),total_proposals=sample_count,causal_lidar_samples=valid_scans,masked_missing_lidar_samples=sample_count-valid_scans,missing_scan_policy_checked=True,auditor_version=2,max_unknown_pixel_fraction=max_unknown,visible_person_frame_instances_below_90pct_in_view=occluded,whole_family_splits=True,millimetre_quantized_cross_split_duplicate_frames=0,test_model_evaluations=0,test_previews=0,classifier_validated=False,world_model_training_ready=False)
 return result
