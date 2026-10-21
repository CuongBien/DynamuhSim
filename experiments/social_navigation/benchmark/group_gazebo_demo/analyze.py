#!/usr/bin/env python3
"""Combine retained attempts and compute sampled geometric metrics; no simulator calls."""
import argparse,ast,hashlib,json,math,subprocess,sys
from pathlib import Path
from PIL import Image
import yaml
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parent.parent.parent
p=argparse.ArgumentParser();p.add_argument('--initial',type=Path,required=True);p.add_argument('--recovery',type=Path,required=True);a=p.parse_args()
initial=a.initial.resolve();out=a.recovery.resolve();sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
load=lambda p:json.loads(p.read_text())
progress=load(out/'progress.json');protocol=load(out/'protocol.json')
assert progress['completed']==progress['planned']==3 and progress['stopped'] is None
assert protocol['amcl_observer_qos']=='transient_local' and protocol['amcl_late_join_check']
assert protocol['continuation_of']['sha256']==sha(initial/'progress.json')
for source,digest in protocol['source_sha256'].items():assert sha(REPO/source)==digest,source
config=yaml.safe_load((BENCH.parent/'warehouse/nav2_warehouse.yaml').read_text())
poly=ast.literal_eval(config['local_costmap']['local_costmap']['ros__parameters']['footprint'])
hx=max(abs(x) for x,y in poly);hy=max(abs(y) for x,y in poly)
assert abs(hx-.12)<1e-9 and abs(hy-.10)<1e-9
radius=.36;tol=config['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']
def footprint(robot,h):
    dx,dy=h['x']-robot['x'],h['y']-robot['y'];c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    qx,qy=abs(c*dx+s*dy)-hx,abs(-s*dx+c*dy)-hy
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-radius
rows=[];manifest=[]
for directory in [initial,out]:
    prog=load(directory/'progress.json')
    for item in prog['attempts']:
        trial_path=directory/item['report'];v=load(trial_path);runtime=BENCH/v['runtime']
        row={k:v.get(k) for k in ['name','condition','geometry','robot_avoidance','status','protocol_valid','error','goal_error_world_m','sim_seconds','wall_seconds','observed_rtf','min_robot_human_clearance_m','min_human_pair_clearance_m']}
        row.update(cohort=directory.name,report=str(trial_path.relative_to(HERE)),infra_errors=item['infrastructure_errors'])
        files=[trial_path,directory/(v['name']+'_trajectory.json'),directory/(v['name']+'.log'),runtime/'diagnostics.jsonl',runtime/'scene.json',runtime/'worker.log',runtime/'ogre2_session.log']
        if v['protocol_valid']:
            assert not item['infrastructure_errors'] and not item['remaining_nodes']
            assert v['gazebo_exit']['clean'] and v['server_shutdown']['status']=='STOPPED'
            assert v['process_cleanup']['status']=='PASS' and not v['process_cleanup']['remaining']
            assert any('NVIDIA' in line for line in v['process_cleanup']['ogre_renderer_lines'])
            assert v['localization_profile']=='baseline' and v['amcl_parameters_readback']
            assert not any(x in (runtime/'worker.log').read_text() for x in ['Traceback','ConnectionResetError','BrokenPipeError'])
            if directory==out:
                assert v['amcl_observer_qos']=='transient_local'
                assert any(e['node_name']=='amcl' and e['durability']=='TRANSIENT_LOCAL' and e['reliability']=='RELIABLE' for e in v['amcl_publishers'])
                late=v['amcl_late_join_check']['samples'];assert late['transient_local']
                row['late_join_counts']={mode:len(messages) for mode,messages in late.items()}
            gate=float((runtime/'start_gate').read_text());end=gate+v['sim_seconds'];dense=[]
            for line in (runtime/'diagnostics.jsonl').open():
                d=json.loads(line)
                if d['kind']!='gazebo' or not gate<=d['data']['time']<=end:continue
                poses=d['data']['poses'];robot=poses.get('mecanum_robot');humans=[h for name,h in poses.items() if name.startswith('socnavgym_')]
                if robot is None or len(humans)!=3:continue
                pairs=[math.hypot(h['x']-j['x'],h['y']-j['y'])-2*radius for i,h in enumerate(humans) for j in humans[i+1:]]
                dense.append(dict(time=d['data']['time'],footprint_gap=min(footprint(robot,h) for h in humans),pair_gap=min(pairs)))
            assert len(dense)>100
            row.update(actual_xy_within_020m=v['goal_error_world_m']<=tol,dense_samples=len(dense),min_footprint_human_m=min(d['footprint_gap'] for d in dense),min_human_pair_dense_m=min(d['pair_gap'] for d in dense),
                footprint_overlap_beyond_1cm_s=sum(n['time']-u['time'] for u,n in zip(dense,dense[1:]) if n['footprint_gap']<-.01),
                pair_overlap_beyond_1cm_s=sum(n['time']-u['time'] for u,n in zip(dense,dense[1:]) if n['pair_gap']<-.01))
            for ext in ['png','gif']:
                path=directory/(v['name']+'.'+ext);files.append(path)
                with Image.open(path) as im:
                    if ext=='png':im.verify()
                    else:
                        duration=0
                        for frame in range(im.n_frames):im.seek(frame);im.load();duration+=im.info.get('duration',0)
                        row['gif_decoded_frames']=im.n_frames;row['gif_duration_ms']=duration
            files.append(runtime/'start_gate')
        rows.append(row)
        manifest.append(dict(cohort=directory.name,name=v['name'],files={str(f.relative_to(REPO)):sha(f) for f in files if f.exists()}))
valid=[r for r in rows if r['protocol_valid']];assert len(rows)==7 and len(valid)==6
assert len({(r['condition'],r['geometry'],r['robot_avoidance']) for r in valid})==6
assert sum(not r['protocol_valid'] for r in rows)==1
cp=HERE/'generated/amcl_qos_checkpoint'
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)
for name,digest in load(cp/'preserved_hashes.json').items():assert sha(REPO/name)==digest,name
validation=dict(status='PASS',attempts_retained=7,valid_trials=6,original_startup_errors=1,recovery_trials=3,late_join_checks=[dict(name=r['name'],counts=r['late_join_counts']) for r in rows if 'late_join_counts' in r],media_files_decoded=12,
    preserved_files=len(load(cp/'preserved_hashes.json')),original_index_unchanged=True,outside_worktree_unchanged=True,
    limitations='One seed, separate cohorts and manual GUI interaction in native recovery. Finite sampled gaps; no continuous collision or simulator ranking guarantee.')
(out/'summary.json').write_text(json.dumps(dict(trials=rows,actual_goal_xy_tolerance_m=tol,validation=validation),indent=2,allow_nan=False)+'\n')
(out/'manifest.json').write_text(json.dumps(dict(analysis_source_sha256=sha(Path(__file__)),trials=manifest),indent=2)+'\n')
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
lines=['# SocNavGym: Gazebo continuation and AMCL observer fix','','All seven attempts are retained: three standing trials, the original counterflow startup error, then three counterflow trials under the explicit observer-QoS amendment. Six scene/variant combinations now have valid measurements; the original six-trial cohort itself remains stopped. This is one seed, not a framework ranking.','','| Cohort | Scene | Variant | Nav2 | Actual XY error (m) | Min footprint–human (m) | Footprint overlap >1cm (s) | Min human pair (m) | Pair overlap >1cm (s) |','|---|---|---|---|---:|---:|---:|---:|---:|']
for r in rows:
    variant='robot_orca' if r['robot_avoidance'] else r['geometry']
    values=[f"{r[k]:.4f}" if r.get(k) is not None else '—' for k in ['goal_error_world_m','min_footprint_human_m','footprint_overlap_beyond_1cm_s','min_human_pair_dense_m','pair_overlap_beyond_1cm_s']]
    lines.append('| '+' | '.join([r['cohort'],r['condition'],variant,r['status']]+values)+' |')
lines+=['',f"Nav2 reported SUCCEEDED in {sum(r['status']=='SUCCEEDED' for r in valid)}/6 valid trials; {sum(r['status']=='SUCCEEDED' and r['actual_xy_within_020m'] for r in valid)}/6 also meet actual XY error ≤{tol:.2f}m. Goal success and geometric overlap are separate results.",'','AMCL was active in the original failed startup: logs show initial pose and scan/current-time map TF, but the observer received zero AMCL messages. The old subscriber was best-effort/volatile. Jazzy AMCL offers reliable/transient-local pose samples. An isolated synthetic probe recovered a pre-published sample in3/3 durable late subscriptions and0/3 sensor-data subscriptions. All three recovery trials also record the actual AMCL publisher QoS and real late-subscriber samples. This supports a missed-observation mechanism; it does not prove the exact historical DDS packet/discovery sequence.','', 'Jazzy upstream source: [AMCL pose publisher and stationary update path](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp). The running publisher QoS was also observed directly, rather than inferred solely from this source.', '', 'Only experiment subscriber QoS is changed with --amcl-observer-qos transient_local; historical default sensor_data remains available and unchanged. The late-join check is read-only, before goal/start gate: no localization reset, forced update, initial-pose republish, AMCL parameter tuning or robot/controller change. The original failed attempt is not overwritten.','', 'The user confirmed GUI drag/zoom and Pause/Play in the native counterflow recovery trial. Manual pause duration is not instrumented; do not compare that trial’s wall time/RTF as unattended throughput. Qt uses xcb; all completed attempts verified fresh NVIDIA Ogre rendering and clean shutdown/cleanup.','', 'Dense measurements use recorded Gazebo poses during the active gate, the configured0.24×0.20m rotated Nav2 footprint and human circles r0.36m. Signed gaps retain all penetrations. Durations use the right endpoint of successive sampled intervals; they do not establish continuous collision freedom. The older trial circle-proxy metrics remain in raw JSON and are not interchangeable with footprint gaps.','', 'In these six valid trials, all sampled robot-footprint/human gaps are positive; this is narrower than a safety guarantee. Native walking has human–human penetration of8.86cm for2.084s beyond1cm, geometry-only2.41cm for0.400s, and robot-aware minimum pair gap44.29cm with no sampled pair penetration. Robot-aware minimum footprint gap is21.08cm, compared with1.64cm native and3.81cm geometry-only. These are observations from one seed and different closed-loop robot trajectories, not a statistical ranking.', '', 'Group membership is fixed at three. Standing conversation is an inward-facing stationary fixture; robot_orca changes the walking solver only. Native and geometry-only walking may retain human overlap; inspect the measured minima rather than interpreting animation as safety. Robot input differs in closed-loop Nav2, so these are not exact-input replay results.','', 'The PNG/GIF files are previews from the live Gazebo observer RGB camera pipeline, not AI images and not screen recordings. Decode and SHA256 checks cover all12 files across the6 valid trials. Existing GIFs subsample camera frames with nominal667ms playback; this is a preview and not a measurement of wall-time performance. No media exist for the startup failure.','', 'Protocols, raw trial results/trajectories, runtime diagnostic hashes, summary.json, manifest.json, GUI confirmation and validation.json retain provenance. ComplexityNav remains standalone; no World Model or Isaac migration was implemented.','', '![Native walking — Gazebo observer](socnavgym_counterflow_s42.gif)','','![Robot-aware walking — Gazebo observer](socnavgym_circle_contact_robot_orca_counterflow_s42.gif)']
(out/'RECOVERY_REPORT.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(validation));print(json.dumps(valid,indent=2))
