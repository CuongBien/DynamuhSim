#!/usr/bin/env python3
"""Validate completed experiment and preserve existing user Git state."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();report=a.report_dir.resolve()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checks=json.loads((report/'checks.json').read_text());summary=json.loads((report/'summary.json').read_text());protocol=json.loads((report/'protocol.json').read_text());manifest=json.loads((report/'manifest.json').read_text())
assert checks['status']=='PASS' and checks['trials']==len(summary['trials'])==12
for source,digest in protocol['source_sha256'].items():assert sha(REPO/source)==digest,source
assert sha(HERE/'analyze.py')==manifest['analysis_source_sha256']
for source,digest in manifest['analysis_helpers_sha256'].items():assert sha(REPO/source)==digest,source
import xml.etree.ElementTree as ET
worlds={}
for item in manifest['trials']:
    runtime=BENCH/item['runtime']
    for name,digest in item['raw_sha256'].items():assert sha(runtime/name)==digest
    tree=ET.parse(runtime/'warehouse.sdf')
    for node in tree.iter():
        if 'socket' in node.tag.lower():node.text='<OWNED_SOCKET>'
        if node.text:node.text=node.text.replace(str(runtime.resolve()),'<RUNTIME>')
        for key,value in list(node.attrib.items()):node.set(key,value.replace(str(runtime.resolve()),'<RUNTIME>'))
    worlds.setdefault(item['seed'],set()).add(hashlib.sha256(ET.tostring(tree.getroot())).hexdigest())
    directory=report/f"seed{item['seed']}"/item['profile'];name='crowdsimplus'+('_sfm' if item['policy']=='sfm' else '')+f"_individual_goals_bottleneck_s{item['seed']}"
    assert sha(directory/(name+'.json'))==item['trial_sha256']
    assert sha(directory/(name+'.log'))==item['launch_log_sha256']
assert set(worlds)=={42,43,44} and all(len(values)==1 for values in worlds.values()),worlds
checkpoint=BENCH/'generated/commit_stability_shutdown_amcl'
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
assert len({(r['seed'],r['profile'],r['policy']) for r in summary['trials']})==12
assert {r['seed'] for r in summary['trials']}=={42,43,44}
assert {r['profile'] for r in summary['trials']}=={'baseline','lf_prob_beamskip'}
expected_head=json.loads((report/'commit_checkpoint.json').read_text())[-1]['commit']
assert protocol['head']==expected_head
# Robot, map, native human policy and actual shared trial runner stay at committed version.
for path in ['experiments/social_navigation/warehouse/nav2_warehouse.yaml','experiments/social_navigation/warehouse/warehouse.pgm','experiments/social_navigation/crowdsimplus_adapter/kernel.py','experiments/social_navigation/crowdsimplus_adapter/adapter.cc','experiments/social_navigation/crowdsimplus_adapter/worker.py','experiments/social_navigation/benchmark/trial.py','experiments/social_navigation/benchmark/localization_trials/profiles.py']:
    expected=subprocess.check_output(['git','show',expected_head+':'+path],cwd=REPO)
    assert (REPO/path).read_bytes()==expected,path
fixtures=json.loads((report/'fixture_checks.json').read_text());assert fixtures['status']=='PASS'
for fixture in fixtures['fixtures']:
    cases=[json.loads((report/item['report']).read_text()) for item in json.loads((report/'progress.json').read_text())['attempts'] if item['seed']==fixture['seed']]
    assert len(cases)==4 and all(case['scene_sha256']==fixture['sha256'] for case in cases)
aggregates=json.loads((report/'aggregate.json').read_text())
for row in aggregates:
    cases=[r for r in summary['trials'] if r['profile']==row['profile'] and r['policy']==row['policy']]
    assert row['reported_success']==sum(r['status']=='SUCCEEDED' for r in cases)
    assert row['actual_success']==sum(r['status']=='SUCCEEDED' and r['goal_error_world_m']<=.20 for r in cases)
    assert row['false_success']==sum(r['status']=='SUCCEEDED' and r['goal_error_world_m']>.20 for r in cases)

# Compile without generating tracked bytecode or invoking ROS.
compiled=[]
for relative in ['trial.py','navigation_diagnostics/analyze.py','localization_validation/run.py','localization_validation/analyze.py','localization_validation/validate.py','shutdown_fix/ordered_stop.py']:
    path=BENCH/relative;compile(path.read_text(),str(path),'exec');compiled.append(relative)
r=subprocess.run(['git','diff','--check','--','experiments/social_navigation/benchmark/trial.py','experiments/social_navigation/benchmark/navigation_diagnostics/analyze.py'],cwd=REPO,capture_output=True,text=True);assert r.returncode==0,r.stdout+r.stderr
from PIL import Image
media=[]
for item in json.loads((report/'progress.json').read_text())['attempts']:
    trial=json.loads((report/item['report']).read_text());base=(report/item['report']).with_suffix('')
    png=base.with_suffix('.png');gif=base.with_suffix('.gif')
    with Image.open(png) as image:image.verify()
    with Image.open(gif) as image:
        assert image.format=='GIF' and image.n_frames>1
        image.seek(image.n_frames-1);image.load();frames=image.n_frames;size=list(image.size)
    assert trial['observer_frames']>0 and frames<=(trial['observer_frames']+1)//2
    media.append(dict(seed=item['seed'],profile=item['profile'],policy=item['policy'],observer_frames=trial['observer_frames'],gif_frames=frames,size=size,png_sha256=sha(png),gif_sha256=sha(gif),png=str(png.relative_to(report)),gif=str(gif.relative_to(report))))
(report/'media_validation.json').write_text(json.dumps(dict(status='PASS',trials=12,method='Decode PNG and last GIF frame; verify sampled frame counts against live Gazebo observer buffer count in trial report. File integrity, not replay of raw camera packets.',cases=media),indent=2))
value=dict(status='PASS',fixed_plan_trials=12,media_files_verified=24,prior_short_shutdown_checks=3,gazebo_clean_in_cohort=12,paired_scenes_verified=True,paired_gazebo_worlds_verified=True,normalized_world_sha256={seed:next(iter(values)) for seed,values in worlds.items()},actual_goal_counts_verified=True,baseline_robot_and_human_sources_unchanged=True,raw_hashes_verified=True,source_hashes_verified=True,existing_git_index_unchanged=True,outside_experiment_worktree_unchanged=True,owned_nodes_remaining=left,owned_containers_remaining=[],compiled_sources=compiled,particle_clouds=checks['particle_clouds'],matched_clouds=checks['matched_clouds'],max_cluster_match_error_m=checks['max_cluster_match_error_m'],baseline_robot_config_sha256=sha(BENCH.parent/'warehouse/nav2_warehouse.yaml'))
(report/'validation.json').write_text(json.dumps(value,indent=2));print(json.dumps(value,indent=2))
