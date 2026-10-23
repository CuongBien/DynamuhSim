#!/usr/bin/env python3
"""Independent ZIP parts for the connector upstream 100 MiB transfer limit."""
from pathlib import Path
import zipfile,shutil,hashlib,json
HERE=Path(__file__).resolve().parent
OUT=HERE/'generated/archives/wm_dataset01_backup_20261023'
def digest(p,alg):
 h=hashlib.new(alg)
 with p.open('rb') as f:
  for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
 return h.hexdigest()
def main():
 source=OUT/'DynamuhSim_wm_dataset01_backup_20261023.zip'
 parts=OUT/'drive_parts_80m';parts.mkdir(exist_ok=False)
 groups=[[]];size=0
 with zipfile.ZipFile(source) as src:
  for entry in src.infolist():
   if size+entry.compress_size>80*1024*1024 and groups[-1]:groups.append([]);size=0
   groups[-1].append(entry);size+=entry.compress_size
  result=[]
  for n,group in enumerate(groups,1):
   dest=parts/f'DynamuhSim_wm_dataset01_backup_part{n:03d}.zip'
   with zipfile.ZipFile(dest,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=1,allowZip64=True) as z:
    for entry in group:
     with src.open(entry) as a,z.open(entry.filename,'w',force_zip64=True) as b:shutil.copyfileobj(a,b,8*1024*1024)
   with zipfile.ZipFile(dest) as z:
    assert z.testzip() is None
   assert dest.stat().st_size<100*1024*1024
   item={'file':dest.name,'bytes':dest.stat().st_size,'sha256':digest(dest,'sha256'),'md5':digest(dest,'md5'),'entries':len(group)}
   result.append(item);print(json.dumps(item),flush=True)
 assert sum(x['entries'] for x in result)==1022
 (parts/'parts_manifest.json').write_text(json.dumps({'format':'independent ZIP files, NOT binary split fragments','restore':'Extract all part ZIPs into the SAME new empty folder. Preserve nested paths. The combined backup_manifest.json inventories SHA256 for all original files. No concatenation is needed.','source_archive_sha256':digest(source,'sha256'),'parts':result,'part_count':len(result)},indent=2)+'\n')
 (parts/'README_RESTORE.txt').write_text('DynamuhSim WM dataset01 backup\n\nDownload ALL ZIP parts listed in parts_manifest.json. Extract each ZIP into the SAME new folder, keeping the directory structure. These are independent ZIP archives: do NOT concatenate them. The combined folder contains wm_episode_batch/generated/dataset01 (portable training data), raw MCAP captures (including 2 quarantined runs), configs, source and reports. backup_manifest.json lists every source file and SHA256.\n\n8 accepted episodes; 5094 valid WM observed BEV windows. Train/validation/test are fixed. Quarantined raw runs are NOT training data. WM/RL has not been trained.\n\nThis backup excludes duplicate normalized intermediate files and diagnostic paired camera arrays. It is not a full ROS installation or complete repository image. See README.md and reports/SCHEMA.md.\n',encoding='utf8')
if __name__=='__main__':main()
