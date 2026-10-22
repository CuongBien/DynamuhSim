#!/usr/bin/env python3
# Read-only verification followed by local evidence manifest/snapshots.
import ast,gzip,hashlib,json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=HERE.parents[3];OUT=HERE/'reports';load=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();plan=load(HERE/'protocol.json')
cp=HERE/'generated/checkpoint';allowed={'experiments/social_navigation/'+n for n in ['README.md','NEXT_STEPS.md','COMMIT_HISTORY.md','FRAMEWORK_COMPARISON.md']};old=load(cp/'files.json')
for name,digest in old.items():
    if name not in allowed:assert sha(REPO/name)==digest,name
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'])
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'])
# Snapshot captured during first live session; only analyzer was later repaired.
for name,digest in load(OUT/'source_manifest.json').items():
    snap=OUT/'source_snapshot'/Path(name).relative_to('experiments/social_navigation/benchmark');assert sha(snap)==digest
    if name not in {str((HERE/n).relative_to(REPO)) for n in ['analyze.py','cohort.py']}:assert sha(REPO/name)==digest,name
final_sources={}
for q in [*HERE.glob('*.py'),HERE/'run.sh',HERE/'protocol.json',HERE/'README.md']:
    target=OUT/'final_source_snapshot'/q.name;target.parent.mkdir(exist_ok=True);target.write_bytes(q.read_bytes());final_sources[str(q.relative_to(REPO))]=sha(q)
(OUT/'final_source_manifest.json').write_text(json.dumps(final_sources,indent=2)+'\n')
receipts=load(OUT/'initial_execution.json');assert [r['name'] for r in receipts]==[r['name'] for r in plan['plan']] and all(r['returncode']==0 for r in receipts);assert receipts[1]['audit_returncode']==1
rows=[];archives=0
sys.path.insert(0,str(BENCH/'gpu_runtime'));from runtime_guard import RuntimeGuard
for trial in plan['plan']:
    d=OUT/trial['name'];result=load(d/'result.json');v=load(d/'validation.json');assert result['status']=='MEASURED' and v['status']=='AUDITED_NAV2_PILOT';assert result['model']==trial['model'] and result['gui']==trial['gui']
    poses=[json.loads(line) for line in gzip.decompress((d/'gazebo_poses.jsonl.gz').read_bytes()).decode().splitlines()];assert any(r['time']==v['goal_sample_sim'] and r['poses'].get('mecanum_robot')==result['final_world_pose'] and r['phase']=='active' for r in poses)
    runtime=REPO/result['runtime'];guard=RuntimeGuard.__new__(RuntimeGuard);guard.token=result['cleanup']['session'];assert not guard.members()
    assert not subprocess.check_output(['docker','ps','-aq','--filter','name=^/dynamuhsim-nav2-shared-'+guard.token[:12]+'$'],text=True).strip()
    assert v['actual_goal_reached']==result['actual_goal_reached'];assert v['nvidia'] and v['shutdown_clean'] and not result['faults']
    for q in d.glob('*.gz'):
        raw=gzip.decompress(q.read_bytes());source=d/q.name[:-3]
        if not source.exists():source=runtime/q.name[:-3]
        assert source.exists() and raw==source.read_bytes(),q;archives+=1
    rows.append(dict(name=trial['name'],status=v['status'],actual_goal_reached=v['actual_goal_reached'],final_analyzer_sha256=sha(HERE/'analyze.py'),owned_processes_and_container_remaining=0))
for q in HERE.rglob('*.py'):
    if 'generated' not in q.relative_to(HERE).parts:ast.parse(q.read_text())
json_files=0
for q in OUT.rglob('*.json'):load(q);json_files+=1
(OUT/'final_audit.json').write_text(json.dumps(dict(status='PASS',trials=rows,scope='Final artifacts verified after analyzer-only repair; no live rerun',preserved_old_files=len(old)-len(allowed),initial_staged_entries=len(subprocess.check_output(['git','diff','--cached','--name-only'],text=True).splitlines()),outside_and_staging_byte_identical=True,raw_gzip_byte_identical_archives=archives,json_files_parsed=json_files),indent=2)+'\n')
# Exclude only this self-describing manifest to avoid hash recursion and local ignored streams.
listed=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','--',str(HERE.relative_to(REPO))],text=True).splitlines();files={name:sha(REPO/name) for name in listed if not name.endswith('/reports/artifact_manifest.json')}
(OUT/'artifact_manifest.json').write_text(json.dumps(dict(status='PACKAGED',files=files,excluded='generated, pycache, local raw *.jsonl/*.log; exact gzip archives included; manifest excludes itself'),indent=2)+'\n')
print(json.dumps(dict(status='PASS',artifacts=len(files),archives=archives,preserved_old_files=len(old)-len(allowed),staged_entries=len(subprocess.check_output(['git','diff','--cached','--name-only'],text=True).splitlines())),indent=2))
