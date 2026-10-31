#!/usr/bin/env python3
"""Non-destructive portable dataset + native MCAP/evidence backup, ZIP64."""
import hashlib,json,os,subprocess,time,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
OUT=HERE/'generated/archives/wm_dataset01_backup_20261023'
def digest(path, algorithm='sha256'):
    h=hashlib.new(algorithm)
    with path.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''): h.update(block)
    return h.hexdigest()
def main():
    OUT.mkdir(parents=True,exist_ok=False)
    files=[]
    for f in HERE.rglob('*'):
        if not f.is_file(): continue
        rel=f.relative_to(HERE)
        if '__pycache__' in rel.parts: continue
        if rel.parts[0]=='generated':
            if len(rel.parts)<3 or rel.parts[1] not in {'dataset01','runs','runtime'}: continue
            if rel.parts[1]=='runs' and any(x in rel.parts[3:] for x in ['normalized','paired_depth']): continue
        files.append((f,Path('wm_episode_batch')/rel))
    files.sort(key=lambda pair:str(pair[1]))
    inventory=[]
    for f,rel in files: inventory.append({'path':str(rel),'bytes':f.stat().st_size,'sha256':digest(f)})
    info={'schema':'wm_backup/1','source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
          'source_git_dirty':True,'created_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
          'scope':'Complete dataset01, stage code/reports/examples, all ten native MCAP bags and auxiliary capture traces, runtime configuration/logs.',
          'excluded':['Duplicated runs/*/normalized (canonical dataset01 included)',
                      'runs/*/paired_depth diagnostic camera on/off arrays (native recorded sensor MCAP retained)',
                      'Other stages, dependency installs, build binaries, Git internals, cache/checkpoint files'],
          'accepted_episodes':8,'quarantined_captures':2,'wm_valid_windows':5094,
          'restore':'Extract ZIP keeping paths. dataset01 is self-contained: Python3 + NumPy. See wm_episode_batch/README.md and reports/SCHEMA.md. Capture runtime absolute asset paths require the source repository and dependencies; this archive is not a full environment image.',
          'files':inventory,'uncompressed_bytes':sum(x['bytes'] for x in inventory)}
    (OUT/'backup_manifest.json').write_text(json.dumps(info,indent=2)+'\n')
    archive=OUT/'DynamuhSim_wm_dataset01_backup_20261023.zip'
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=1,allowZip64=True) as z:
        z.write(OUT/'backup_manifest.json','backup_manifest.json')
        for i,(f,rel) in enumerate(files):
            z.write(f,str(rel),compress_type=zipfile.ZIP_STORED if f.suffix.lower() in {'.mcap','.gif','.png'} else zipfile.ZIP_DEFLATED)
            if i%100==0: print(f'Archived {i+1}/{len(files)}',flush=True)
    print('Checking every archived file against original SHA256...',flush=True)
    with zipfile.ZipFile(archive) as z:
        assert len(z.namelist())==len(inventory)+1
        for entry in inventory:
            h=hashlib.sha256()
            with z.open(entry['path']) as f:
                for block in iter(lambda:f.read(8*1024*1024),b''): h.update(block)
            assert h.hexdigest()==entry['sha256'],entry['path']
    result={k:v for k,v in info.items() if k!='files'}
    result.update(archive=str(archive),archive_bytes=archive.stat().st_size,archive_sha256=digest(archive),archive_md5=digest(archive,'md5'),file_count=len(inventory)+1,verified_all_entries_sha256=True)
    (OUT/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    (OUT/(archive.name+'.sha256')).write_text(result['archive_sha256']+'  '+archive.name+'\n')
    print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__': main()
