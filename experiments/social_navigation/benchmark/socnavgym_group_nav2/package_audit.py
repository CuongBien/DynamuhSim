#!/usr/bin/env python3
import ast,gzip,hashlib,json,os,subprocess,sys
from pathlib import Path
from PIL import Image
import yaml
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=HERE.parents[3];OUT=HERE/'reports';load=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();plan=load(HERE/'protocol.json');trials=[dict(name=n) for n in plan['previous_valid_walking']]+load(OUT/'headless_amendment.json')['plan']
allowed={'experiments/social_navigation/'+n for n in ['README.md','NEXT_STEPS.md','COMMIT_HISTORY.md','FRAMEWORK_COMPARISON.md']};cp=HERE/'generated/checkpoint';old=load(cp/'files.json')
for name,digest in old.items():
    if name not in allowed:assert sha(REPO/name)==digest,name
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary']) and (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'])
# Source versions match their corresponding saved snapshots, not today's source by assumption.
snapshot_pairs=[('prelive_source_manifest.json','prelive_source'),('live_source_manifest.json','live_source_snapshot'),('v2_source_manifest.json','v2_source_snapshot'),('v2_final_prelive_manifest.json','v2_final_prelive_source'),('v3_prelive_manifest.json','v3_prelive_source')];historical=set()
for manifest,folder in snapshot_pairs:
    for name,digest in load(OUT/manifest).items():
        q=OUT/folder/Path(name).name;assert sha(q)==digest,(manifest,name);historical.add((name,digest))
sys.path.insert(0,str(BENCH/'gpu_runtime'));from runtime_guard import RuntimeGuard
archives=0;audits=[]
for trial in trials:
    d=OUT/trial['name'];r=load(d/'result.json');v=load(d/'validation.json');assert r['status']=='MEASURED' and v['status']=='AUDITED_NAV2_PILOT' and v['nvidia'] and v['shutdown_clean'] and not r['faults']
    for name,digest in r['source_sha256'].items():assert sha(REPO/name)==digest or (name,digest) in historical,(trial['name'],name)
    guard=RuntimeGuard.__new__(RuntimeGuard);guard.token=r['cleanup']['session'];assert not guard.members()
    if trial['name'].endswith('headless'):assert not r['gui'] and v['standing_reaction']['planned_peak_speed_mps']<=.200001
    audits.append(dict(name=trial['name'],actual_goal_reached=v['actual_goal_reached'],source_version_verified=True,owned_processes_remaining=0))
# All attempted sessions, including crashes/terminal stop, preserve exact raw archives.
for name in [t['name'] for t in trials]+['walking01','standing01_v2','standing01_v3']:
    d=OUT/name;r=load(d/'result.json');runtime=REPO/r['runtime'];guard=RuntimeGuard.__new__(RuntimeGuard);guard.token=r['cleanup']['session'];assert not guard.members()
    for q in [*d.glob('*.jsonl'),d/'launch.log',runtime/'warehouse.sdf',runtime/'nav2_runtime.yaml',runtime/'worker_trajectory.jsonl',runtime/'worker.log',runtime/'ogre2_session.log',runtime/'gui.log']:
        if q.exists():target=d/(q.name+'.gz');target.write_bytes(gzip.compress(q.read_bytes(),mtime=0));assert gzip.decompress(target.read_bytes())==q.read_bytes();archives+=1
    for q in d.glob('*.png'):
        with Image.open(q) as im:im.load();assert im.size==(640,480)
    if (d/'camera_preview_4x.gif').exists():
        with Image.open(d/'camera_preview_4x.gif') as im:
            duration=0
            for i in range(im.n_frames):im.seek(i);im.load();duration+=im.info.get('duration',0)
            assert duration==r['media']['gif_duration_ms']
for gate in ['gate01','gate02','gate03']:
    for item in load(OUT/gate/'validation.json')['cases']:
        name=item['scenario']+str(item['seed']);runtime=HERE/'generated'/((('gate_' if gate=='gate01' else gate+'_')+name));dest=OUT/gate/name;dest.mkdir(exist_ok=True)
        for q in runtime.iterdir():
            if q.is_file():target=dest/(q.name+'.gz');target.write_bytes(gzip.compress(q.read_bytes(),mtime=0));assert gzip.decompress(target.read_bytes())==q.read_bytes()
        q=OUT/gate/(name+'.log');q.with_suffix('.log.gz').write_bytes(gzip.compress(q.read_bytes(),mtime=0))
base=yaml.safe_load((BENCH/'socnavgym_warehouse_lifecycle/config.yaml').read_text());config=yaml.safe_load((HERE/'config.yaml').read_text());check=yaml.safe_load((HERE/'config.yaml').read_text())
for name in ['min_map_x','max_map_x','min_map_y','max_map_y']:check['env'][name]=base['env'][name]
assert check==base and config['env']['min_map_x']==60 and config['env']['min_map_y']==40
for q in HERE.rglob('*.py'):
    if 'generated' not in q.relative_to(HERE).parts:ast.parse(q.read_text())
for q in OUT.rglob('*.json'):load(q)
snapshot=OUT/'final_source';snapshot.mkdir(exist_ok=True);sources={}
for q in [*HERE.glob('*.py'),HERE/'run.sh',HERE/'protocol.json',HERE/'config.yaml',HERE/'README.md']:
    (snapshot/q.name).write_bytes(q.read_bytes());sources[str(q.relative_to(REPO))]=sha(q)
(OUT/'final_source_manifest.json').write_text(json.dumps(sources,indent=2)+'\n')
(OUT/'package_validation.json').write_text(json.dumps(dict(status='PASS_EVIDENCE_PACKAGE_NOT_ALL_NAVIGATION_GOALS',complete_trials=audits,retained_failed_attempts=['walking01:GUI driver','standing01_v2:terminal fixed-standing','standing01_v3:Qt context'],v2_standing_seed43_not_run=True,raw_archives_byte_identical=archives,preserved_old_files=len(old)-len(allowed),original_user_staging_and_outside_byte_identical=True,source_snapshots_verified=True,reactive_speed_cap=.2),indent=2)+'\n')
listed=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','--',str(HERE.relative_to(REPO))],text=True).splitlines();files={name:sha(REPO/name) for name in listed if not name.endswith('/reports/artifact_manifest.json')}
(OUT/'artifact_manifest.json').write_text(json.dumps(dict(files=files,scope='New group Nav2 experiment; generated/runtime and local plain log/jsonl ignored; exact gzip artifacts retained; self excluded'),indent=2)+'\n')
print(json.dumps(dict(status='PACKAGED',artifacts=len(files),archives=archives,preserved_old_files=len(old)-len(allowed)),indent=2))
