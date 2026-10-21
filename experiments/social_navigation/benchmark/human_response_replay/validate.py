#!/usr/bin/env python3
"""Validate replay evidence and keep the original robot/worktree intact."""
import argparse,hashlib,json,subprocess,math
from pathlib import Path
from PIL import Image
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2];SOCIAL=BENCH.parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();R=a.report_dir.resolve();sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protocol=json.loads((R/'protocol.json').read_text());groups=json.loads((R/'group_protocol.json').read_text());manifest=json.loads((R/'manifest.json').read_text());summary=json.loads((R/'summary.json').read_text())
assert len(protocol['plan'])==6 and len(summary['results'])==24
for part in [protocol,groups]:
    for source,digest in part['source_sha256'].items():assert sha(REPO/source)==digest,source
    for name,digest in part['input_sha256'].items():assert sha(R/'inputs'/name)==digest,name
assert sha(HERE/'analyze.py')==manifest['analysis_source_sha256']
assert json.loads((R/'fixture_checks.json').read_text())['status']=='PASS'
parity=[];repeats=[];steps=0
for record in manifest['trials']:
    directory=R/record['key'];result=json.loads((directory/'result.json').read_text());rows=[json.loads(line) for line in (directory/'trajectory.jsonl').open()];inputs=[json.loads(line) for line in (R/'inputs'/f"{result['case']}.jsonl").open()]
    assert sha(directory/'result.json')==record['result_sha256'];assert sha(directory/'trajectory.jsonl')==record['trajectory_sha256']
    assert result['input_sha256']==record['input_sha256']==sha(R/'inputs'/f"{result['case']}.jsonl")
    assert result['status']=='PASS' and result['samples']==len(rows)==200 and len(inputs)==201
    assert result['determinism_max_error']<1e-10;repeats.append(result['determinism_max_error'])
    if result['framework']=='CrowdSimPlus':
        assert result['native_full_step_max_state_error']<1e-10;parity.append(result['native_full_step_max_state_error'])
        assert result['metadata']['source_commit']=='d526a07c013c621fa1ad50894b6ba639f61063de'
    for row,inp,nxt in zip(rows,inputs,inputs[1:]):
        assert row['robot_input']==inp['robot'] and row['robot_next']==nxt['robot']
        assert row['input_time']==inp['time'] and row['state_time']==nxt['time']
        assert [h['id'] for h in row['humans']]==[1,2,3]
        assert all(math.isfinite(h[key]) for h in row['humans'] for key in ['x','y','vx','vy','yaw'])
        steps+=1
assert len(parity)==6 and len(repeats)==24 and steps==4800
for check in summary['group_controls']:
    if check['negative_control']:assert check['circle_vs_robot_orca_max_state_difference']<1e-10
for row in summary['results']:
    if row['mode']=='conversation':assert row['max_displacement_m']<1e-10 and row['max_position_difference_from_far_m']<1e-10
    if row['case']=='far':assert row['max_position_difference_from_far_m']==0 and row['first_velocity_divergence'] is None
media=[]
for path in sorted(R.glob('*.gif')):
    with Image.open(path) as image:
        duration=0
        for index in range(image.n_frames):image.seek(index);image.load();duration+=image.info['duration']
        assert 1<image.n_frames<=100 and duration==20000
        media.append(dict(name=path.name,frames=image.n_frames,duration_ms=duration,sha256=sha(path)))
assert len(media)==6
for path in R.glob('*.png'):
    with Image.open(path) as image:image.verify()
checkpoint=BENCH/'generated/human_replay_checkpoint'
assert subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)==(checkpoint/'index.patch').read_bytes()
assert subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)==(checkpoint/'outside.patch').read_bytes()
head=(checkpoint/'head').read_text().strip()
for relative in ['crowdsimplus_adapter/kernel.py','crowdsimplus_adapter/worker.py','gazebo_adapter/kernel.py','benchmark/circle_contact.py','benchmark/robot_orca.py','warehouse/nav2_warehouse.yaml','warehouse/warehouse.pgm']:
    assert (SOCIAL/relative).read_bytes()==subprocess.check_output(['git','show',head+':experiments/social_navigation/'+relative],cwd=REPO),relative
for path in HERE.glob('*.py'):compile(path.read_text(),str(path),'exec')
result=dict(status='PASS',crowd_probes=6,group_probes=18,updates=steps,native_parity_updates=1200,native_full_step_max_error=max(parity),second_rollout_updates=4800,second_rollout_max_error=max(repeats),identical_robot_inputs_verified=True,group_far_and_conversation_controls_verified=True,unchanged_native_human_and_robot_sources=True,original_git_index_and_outside_worktree_preserved=True,media=media,media_provenance='OFFLINE replay diagrams from simulator states, not Gazebo recordings or AI images.')
(R/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps({key:value for key,value in result.items() if key!='media'},indent=2))
