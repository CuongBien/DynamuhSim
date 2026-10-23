#!/usr/bin/env python3
"""Restore all uploaded ZIP parts and validate source checksums + portable reader."""
from pathlib import Path
import json,zipfile,hashlib,sys
HERE=Path(__file__).resolve().parent
OUT=HERE/'generated/archives/wm_dataset01_backup_20261023'
parts=OUT/'drive_parts_80m';restore=OUT/'restore_test';restore.mkdir(exist_ok=False)
m=json.loads((parts/'parts_manifest.json').read_text());seen=set()
for part in m['parts']:
 with zipfile.ZipFile(parts/part['file']) as z:
  for entry in z.infolist():
   assert entry.filename not in seen;seen.add(entry.filename)
   dest=(restore/entry.filename).resolve();assert dest.is_relative_to(restore.resolve())
   z.extract(entry,restore)
manifest=json.loads((restore/'backup_manifest.json').read_text());assert len(seen)==len(manifest['files'])+1
for entry in manifest['files']:
 path=restore/entry['path'];h=hashlib.sha256()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
 assert path.stat().st_size==entry['bytes'] and h.hexdigest()==entry['sha256'],entry['path']
bundle=restore/'wm_episode_batch/generated/dataset01';sys.path.insert(0,str(bundle/'tools'));from loader import Dataset
e=Dataset(bundle).episode('wm_standing_45');x=e.observation(50);y=e.supervision(50)
assert x['depth_history'].shape==(20,240,424) and y['future_bev']['occupancy'].shape==(3,64,64)
result={'status':'PASS','parts':len(m['parts']),'restored_entries':len(seen),'all_original_file_sha256_match':True,'restored_depth_shape':list(x['depth_history'].shape),'restored_bev_label_shape':list(y['future_bev']['occupancy'].shape),'source_archive_sha256':m['source_archive_sha256'],'restore_dir':str(restore)}
(OUT/'restore_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
