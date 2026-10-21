#!/usr/bin/env python3
"""Validate completed experiment and preserve existing user Git state."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();report=a.report_dir.resolve()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checks=json.loads((report/'checks.json').read_text());summary=json.loads((report/'summary.json').read_text());protocol=json.loads((report/'protocol.json').read_text());manifest=json.loads((report/'manifest.json').read_text())
assert checks['status']=='PASS' and checks['trials']==len(summary['trials'])==6
for source,digest in protocol['source_sha256'].items():assert sha(REPO/source)==digest,source
for item in manifest['trials']:
    runtime=BENCH/item['runtime']
    for name,digest in item['raw_sha256'].items():assert sha(runtime/name)==digest
    directory=report/item['profile'];name='crowdsimplus'+('_sfm' if item['policy']=='sfm' else '')+'_individual_goals_bottleneck_s42'
    assert sha(directory/(name+'.json'))==item['trial_sha256']
    assert sha(directory/(name+'.log'))==item['launch_log_sha256']
checkpoint=BENCH/'generated/shutdown_localization_checkpoint'
assert subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)==(checkpoint/'index.patch').read_bytes(),'Existing staged data changed'
assert subprocess.check_output(['git','diff','--binary','--','.',' :(exclude)experiments/social_navigation'.strip()],cwd=REPO)==(checkpoint/'outside.patch').read_bytes(),'Outside experiment worktree changed'
left=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        env=dict(x.split(b'=',1) for x in (proc/'environ').read_bytes().split(b'\0') if b'=' in x);args=(proc/'cmdline').read_bytes()
    except OSError:continue
    if args and (env.get(b'DYNAMUHSIM_EXPERIMENT_SESSION') or b'social_navigation/' in env.get(b'ROS_LOG_DIR',b'')):left.append(int(proc.name))
assert not left,left
containers=subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines();assert not any(x.startswith('dynamuhsim_crowd_') for x in containers)
probes=json.loads((BENCH/'shutdown_fix/reports/probes.json').read_text());verified=[x for x in probes if x['probe'].startswith('verified')];assert len(verified)==3
assert all(x['shutdown_check']=='PASS' and x['gazebo_clean'] and not x['segfault'] for x in verified)
assert all(r['gazebo_clean'] for r in summary['trials'])
# Compile without generating tracked bytecode or invoking ROS.
compiled=[]
for relative in ['trial.py','navigation_diagnostics/analyze.py','shutdown_fix/ordered_stop.py','localization_trials/profiles.py','localization_trials/run.py','localization_trials/analyze.py','localization_trials/validate.py']:
    path=BENCH/relative;compile(path.read_text(),str(path),'exec');compiled.append(relative)
r=subprocess.run(['git','diff','--check','--','experiments/social_navigation/benchmark/trial.py','experiments/social_navigation/benchmark/navigation_diagnostics/analyze.py'],cwd=REPO,capture_output=True,text=True);assert r.returncode==0,r.stdout+r.stderr
value=dict(status='PASS',fixed_plan_trials=6,short_shutdown_checks=3,gazebo_clean_in_pilot=6,raw_hashes_verified=True,source_hashes_verified=True,existing_git_index_unchanged=True,outside_experiment_worktree_unchanged=True,owned_nodes_remaining=left,owned_containers_remaining=[],compiled_sources=compiled,particle_clouds=checks['particle_clouds'],matched_clouds=checks['matched_clouds'],max_cluster_match_error_m=checks['max_cluster_match_error_m'],baseline_robot_config_sha256=sha(BENCH.parent/'warehouse/nav2_warehouse.yaml'))
(report/'validation.json').write_text(json.dumps(value,indent=2));print(json.dumps(value,indent=2))
