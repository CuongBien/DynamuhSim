#!/usr/bin/env python3
"""Validate the six-trial evidence without starting another ROS/Gazebo session."""
import argparse,hashlib,json,math,os,re,subprocess,sys,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
from recluster import recluster
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();report=a.report_dir
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
summary=json.loads((report/'summary.json').read_text());protocol=json.loads((report/'protocol.json').read_text());progress=json.loads((report/'progress.json').read_text());manifest=json.loads((report/'manifest.json').read_text());events=json.loads((report/'events.json').read_text());checks={}
assert progress['completed']==6 and progress['stopped'] is None and len(summary['trials'])==6
assert [r['policy'] for r in summary['trials']]==[p['policy'] for p in protocol['plan']]
for source,digest in protocol['source_sha256'].items():assert sha(REPO/source)==digest,source
for source,digest in manifest['analysis_sources'].items():assert sha(REPO/source)==digest,source
for r in manifest['raw_inputs']:
 for name,digest in r['raw_sha256'].items():assert sha(BENCH/r['runtime']/name)==digest
assert sum(r['status']=='SUCCEEDED' for r in summary['trials'])==3
assert all(not r['infrastructure_errors'] and not r['remaining_nodes'] for r in progress['attempts'])
assert summary['checks']['all_native_cluster_matches']==819 and summary['checks']['native_max_cluster_amcl_error_max_m']<1e-10
checks['six_fixed_attempts_no_replacement_all_source_and_input_hashes']='PASS'
checks['819_cloud_means_match_after_independent_circular_lift']='PASS'
# Synthetic physical invariant: close headings across +/-pi are one coherent mode.
(HERE/'generated').mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(dir=HERE/'generated') as directory:
 runtime=Path(directory);parts=[]
 for x,angle in [(0.,3.13),(.1,-3.13)]:
  parts.append(dict(pose=dict(position=dict(x=x,y=0,z=0),orientation=dict(x=0,y=0,z=math.sin(angle/2),w=math.cos(angle/2))),weight=.5))
 (runtime/'diagnostics.jsonl').write_text(json.dumps(dict(kind='particle_cloud',data=dict(header=dict(stamp=dict(sec=0,nanosec=0),frame_id='map'),particles=parts)))+'\n')
 row=dict(time=0.,elapsed=0.,clusters=[dict(weight=.5,x=0,y=0,yaw=3.13),dict(weight=.5,x=.1,y=0,yaw=-3.13)],amcl=dict(x=.05,y=0,yaw=math.pi),max_cluster_amcl_error_m=.05)
 result=recluster(runtime,[row])[0];assert len(result['principal_yaw_clusters'])==2 and len(result['clusters'])==1
 assert abs(result['clusters'][0]['x']-.05)<1e-12 and abs(result['clusters'][0]['weight']-1)<1e-12 and result['max_cluster_amcl_error_m']<1e-12
checks['circular_yaw_seam_regression_through_actual_native_library']='PASS'
e=next(e for e in events if e['round']==1 and e['policy']=='orca_plus');before=e['largest_jump_clouds']['before_largest_jump']['clusters'];after=e['largest_jump_clouds']['after_largest_jump']['clusters']
assert not before[0]['mean_static_occupied'] and after[0]['mean_static_occupied']
assert before[0]['weight']>.59 and .50<after[0]['weight']<.52
assert .03<math.hypot(e['jump_after']['actual']['x']-e['jump_before']['actual']['x'],e['jump_after']['actual']['y']-e['jump_before']['actual']['y'])<.05
checks['recorded_orca_mode_switch_not_robot_motion']='PASS'
e=next(e for e in events if e['round']==3 and e['policy']=='sfm');assert e['first_plan_leaves_channel_s']<e['first_estimated_center_occupied_s']<e['first_persistent_translation_stop_s']<e['first_backup_executing_status_s']
checks['sfm_global_reroute_precedes_occupied_pose_stop_and_backup']='PASS'
r=next(r for r in summary['trials'] if r['round']==2 and r['policy']=='sfm');assert r['min_human_pair_m']<-.02 and abs(r['human_pair_overlap_over_1cm_s']-.1)<1e-10
checks['human_overlap_retained_even_when_goal_succeeds']='PASS'
# Preserve all original user's staged changes; phase commits were scoped separately.
checkpoint=BENCH/'generated/commit_diagnostics_gpu'
assert subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)==(checkpoint/'index.patch').read_bytes()
assert subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)==(checkpoint/'outside.patch').read_bytes()
allowed={'experiments/social_navigation/README.md','experiments/social_navigation/FRAMEWORK_COMPARISON.md','experiments/social_navigation/COMMIT_HISTORY.md','experiments/social_navigation/benchmark/README.md','experiments/social_navigation/benchmark/navigation_diagnostics/README.md'}
changed=subprocess.check_output(['git','diff','--name-only','--','experiments/social_navigation'],text=True,cwd=REPO).splitlines();assert set(changed)<=allowed,changed
for path in HERE.glob('*.py'):
 source=path.read_text();compile(source,str(path),'exec');assert all(line==line.rstrip() for line in source.splitlines())
missing=[]
for path in HERE.rglob('*.md'):
 for target in re.findall(r'\]\(([^)]+)\)',path.read_text()):
  if '://' not in target and not (path.parent/target).exists() and (path.parent/target).resolve()!=(report/'validation.json').resolve():missing.append([str(path),target])
assert not missing,missing
# No owned runtime process, including detached children, may remain.
remaining=[]
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:
  env=dict(v.split(b'=',1) for v in (proc/'environ').read_bytes().split(b'\0') if b'=' in v);args=(proc/'cmdline').read_bytes()
 except OSError:continue
 if args and (b'social_navigation/' in env.get(b'ROS_LOG_DIR',b'') or env.get(b'DYNAMUHSIM_EXPERIMENT_SESSION')):remaining.append(proc.name)
assert not remaining,remaining
checks['original_staged_outside_and_committed_runtime_sources_preserved']='PASS';checks['syntax_whitespace_links_no_live_experiment_processes']='PASS'
(report/'validation.json').write_text(json.dumps(dict(status='PASS',checks=checks,remaining_processes=remaining,changed_existing_files=changed,source_sha256={p.name:sha(p) for p in HERE.glob('*.py')},plot_source_sha256=sha(HERE/'plot_failures.py')),indent=2));print(json.dumps(dict(status='PASS',checks=checks),indent=2))
