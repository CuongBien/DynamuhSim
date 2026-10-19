#!/usr/bin/env python3
"""Integrity, native clustering, capture schema and source-preservation checks."""
import argparse,ast,base64,hashlib,json,math,os,subprocess,sys,zlib
from collections import Counter
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
summary=json.loads((a.report_dir/'summary.json').read_text());manifest=json.loads((a.report_dir/'manifest.json').read_text());timeline=json.loads((a.report_dir/'timeline.json').read_text())
assert len(summary)==2
checks={}
# A deliberately bimodal cloud must remain two native clusters, without random draws.
env=os.environ.copy();env['LD_LIBRARY_PATH']='/opt/ros/jazzy/lib'
r=subprocess.run([str(HERE/'generated/native_clusters')],input='4\n0 0 0 0.25\n0.1 0 0 0.25\n5 0 0 0.25\n5.1 0 0 0.25\n',text=True,capture_output=True,check=True,env=env)
v=list(map(float,r.stdout.split()));assert v[0]==2 and len(v)==9
clusters=sorted([v[1:5],v[5:9]],key=lambda c:c[1]);assert np.allclose(clusters,[[.5,.05,0,0],[.5,5.05,0,0]],atol=1e-12)
checks['native_two_disconnected_clusters']='PASS'
for su in summary:
 tr=timeline[su['name']];saved=json.loads((a.report_dir/(su['name']+'.json')).read_text());runtime=BENCH/saved['runtime'];counts=Counter();grid_count=0
 for line in (runtime/'diagnostics.jsonl').open():
  r=json.loads(line);counts[r['kind']]+=1
  if r['kind']=='global_costmap':
   d=r['data'];raw=zlib.decompress(base64.b64decode(d['data_b64']));grid=np.frombuffer(raw,dtype=np.int8)
   assert len(raw)==d['info']['width']*d['info']['height'];assert grid.min()>=-1 and grid.max()<=100
   assert zlib.decompress(zlib.compress(raw))==raw;grid_count+=1
  if r['kind']=='particle_cloud':
   parts=r['data']['particles'];assert 500<=len(parts)<=2000
   assert all(math.isfinite(x['weight']) and x['weight']>=0 and abs(x['pose']['position']['z'])<1e-12 for x in parts)
   assert abs(sum(x['weight'] for x in parts)-1)<1e-9
 assert dict(counts)==saved['diagnostic_counts']==su['counts'];assert grid_count>20
 assert su['native_cluster_matches']==su['particle_cloud_samples'] and su['native_cluster_match_error_max_m']<1e-10
 assert su['channel_traversals']['0']==1 and su['bypass_events']==0
 assert su['min_robot_human_footprint_m']>0 and su['min_human_pair_m']>0 and su['min_robot_machine_footprint_m']>0
 assert su['odom_error_max_m']<.02
 baseline=json.loads((BENCH/'bottleneck/reports/run01'/(su['name']+'.json')).read_text());assert baseline['status']=='ABORTED'
 for k in ['scene_sha256','nav_params_sha256','runtime_nav_params_sha256','map_sha256','crowdsim_image_id']:assert saved[k]==baseline[k]
 entry=next(x for x in manifest['trials'] if x['name']==su['name'])
 for name,digest in entry['raw_sha256'].items():assert sha(runtime/name)==digest
checks['all_recorded_messages_grids_weights_counts_inputs']='PASS'
checks['all_325_native_max_cluster_means_match_amcl']='PASS'
checks['same_scene_map_config_and_native_image_as_old_failures']='PASS'
# Collect every attempt, including the two warmup instrumentation failures.
attempts=[]
for rd in ['run01','run02','run03']:
 for path in sorted((HERE/'reports'/rd).glob('crowdsimplus*.json')):
  if path.name.endswith('_trajectory.json'):continue
  r=json.loads(path.read_text());runtime=BENCH/r['runtime']
  attempts.append(dict(report=str(path.relative_to(HERE)),status=r['status'],protocol_valid=r['protocol_valid'],samples=r['samples'],error=r.get('error'),report_sha256=sha(path),raw_diagnostics_sha256=sha(runtime/'diagnostics.jsonl')))
assert len(attempts)==4 and sum(r['protocol_valid'] for r in attempts)==2
assert all(r['samples']==0 and r['status']=='ERROR' for r in attempts if not r['protocol_valid'])
(a.report_dir/'attempts.json').write_text(json.dumps(attempts,indent=2));checks['all_attempts_retained']='PASS'
# Passive capture must never create publishers or call services/action clients.
tree=ast.parse((HERE/'capture.py').read_text())
for node in ast.walk(tree):
 if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):assert node.func.attr not in ['create_publisher','create_client','set_parameters','publish','send_goal_async']
checks['capture_has_no_write_to_ros']='PASS'
# Verify only this phase's four runtimes; never stop unrelated ROS/Gazebo sessions.
owned_runtimes={str((BENCH/json.loads((HERE/r['report']).read_text())['runtime']).resolve()) for r in attempts}
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:
  environ=dict(v.split(b'=',1) for v in (proc/'environ').read_bytes().split(b'\0') if b'=' in v)
  args=(proc/'cmdline').read_bytes()
 except (OSError,PermissionError):continue
 logdir=environ.get(b'ROS_LOG_DIR',b'').decode(errors='replace')
 assert not (args and any(logdir==runtime+'/ros_logs' for runtime in owned_runtimes)),proc.name
checks['owned_runtime_processes_cleaned_up']='PASS'

# All original input captures from the previous offline diagnosis remain byte-identical.
old=json.loads((BENCH/'localization_diagnosis/reports/run01/manifest.json').read_text())
for r in old['inputs']:assert sha(Path(r['runtime'])/'diagnostics.jsonl')==r['diagnostics_sha256']
assert sha(Path(manifest['native_library']))==manifest['native_library_sha256']=='5742b1e7f4a91519bf78f35850bc0e130a855d250410f38077a03ff292eb1a71'
checks['old_captures_and_installed_amcl_unchanged']='PASS'
checkpoint=BENCH/'generated/global_diagnostics_checkpoint';snapshot=json.loads((checkpoint/'tracked_hashes.json').read_text());changed=[]
allowed={'experiments/social_navigation/benchmark/trial.py','experiments/social_navigation/benchmark/README.md','experiments/social_navigation/README.md','experiments/social_navigation/FRAMEWORK_COMPARISON.md'}
for name,digest in snapshot.items():
 path=REPO/name;current=sha(path) if path.is_file() else None
 if current!=digest:changed.append(name)
assert set(changed)<=allowed,changed
assert subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)==(checkpoint/'index.patch').read_bytes()
assert subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)==(checkpoint/'outside.patch').read_bytes()
for path in list(HERE.glob('*.py'))+[BENCH/'trial.py']:
 source=path.read_text();compile(source,str(path),'exec');assert all(line==line.rstrip() for line in source.splitlines())
checks['python_and_whitespace']='PASS'
checks['preexisting_staged_and_outside_changes_preserved']='PASS'
checks['tracked_sources_and_old_reports_preserved']='PASS'
preservation=dict(tracked_entries=len(snapshot),changed_since_phase_start=changed,allowed_changes=sorted(allowed),all_other_tracked_bytes_preserved=True,staged_diff_identical=True,outside_diff_identical=True,original_five_diagnostic_captures_preserved=True)
(a.report_dir/'source_preservation.json').write_text(json.dumps(preservation,indent=2))
(a.report_dir/'checks.json').write_text(json.dumps(dict(status='PASS',checks=checks,source_sha256={str(x.relative_to(REPO)):sha(x) for x in [*HERE.glob('*.py'),HERE/'native_clusters.cpp',BENCH/'trial.py']}),indent=2))
print(json.dumps(dict(status='PASS',checks=checks,tracked_entries=len(snapshot)),indent=2))
