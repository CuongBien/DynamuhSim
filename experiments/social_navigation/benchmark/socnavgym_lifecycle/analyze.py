#!/usr/bin/env python3
"""Validate recorded lifecycle contracts and expose all native geometry discontinuities."""
import argparse,collections,hashlib,json,math,subprocess
from pathlib import Path
from PIL import Image
import yaml
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3];SOCIAL=HERE.parent.parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();out=a.report_dir.resolve()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();load=lambda p:json.loads(p.read_text())
protocol=load(out/'protocol.json');progress=load(out/'progress.json')
assert len(progress['attempts'])==progress['planned']==3
for name,digest in protocol['source_sha256'].items():assert sha(REPO/name)==digest,name
config=yaml.safe_load((out/'formation.yaml').read_text());dt=config['episode']['time_step'];r=config['human']['human_diameter']/2
assert dt==.1 and config['env']['set_shape']=='square'
inner_x=config['env']['min_map_x']/2-config['wall']['wall_thickness'];inner_y=config['env']['min_map_y']/2-config['wall']['wall_thickness']
rows=[];manifest=[];total_native_steps=0
for attempt in progress['attempts']:
    d=out/f'seed{attempt["seed"]}';result=load(d/'result.json');states=load(d/'trajectory.json');calls=load(d/'native_calls.json')
    assert result['seed']==attempt['seed']
    initial=states[0];expected={h['id'] for h in initial['humans']};assert len(expected)==12
    pair_min=math.inf;robot_min=math.inf;wall_min=math.inf;largest_step=0.;pair_overlap=0.;robot_overlap=0.;wall_overlap=0.
    for i,state in enumerate(states):
        hs=state['humans'];ids=[h['id'] for h in hs];assert len(ids)==len(set(ids))==12 and set(ids)==expected
        groups=[j for g in state['groups'] for j in g['members']];assert len(groups)==len(set(groups)) and set(groups)<=expected
        assert state['epoch']==0 and state['step']==i and abs(state['time']-i*dt)<1e-8
        pairs=[math.hypot(h['x']-j['x'],h['y']-j['y'])-2*r for n,h in enumerate(hs) for j in hs[n+1:]]
        pg=min(pairs);rg=min(math.hypot(h['x']-state['robot']['x'],h['y']-state['robot']['y'])-r-config['robot']['robot_radius'] for h in hs)
        wg=min(min(inner_x-abs(h['x']),inner_y-abs(h['y']))-r for h in hs)
        pair_min=min(pair_min,pg);robot_min=min(robot_min,rg);wall_min=min(wall_min,wg)
        if i:
            pair_overlap+=dt*(pg<-.01);robot_overlap+=dt*(rg<-.01);wall_overlap+=dt*(wg<-.01)
            largest_step=max(largest_step,max(math.hypot(h['x']-states[i-1]['humans'][n]['x'],h['y']-states[i-1]['humans'][n]['y']) for n,h in enumerate(hs)))
        assert all(abs(state['robot'][k]-initial['robot'][k])<1e-9 for k in ['x','y','yaw'])
    native_counts=dict(collections.Counter(c['native_method'] for c in calls if c['epoch']==0))
    finish=[c for c in calls if c['epoch']==0 and c['native_method']=='finish_human_crowd_formation']
    snaps=[dict(step=c['step'],time=c['step']*dt,approximate=c['kwargs'].get('make_approx_crowd',False),max_position_delta_m=max(d['position_delta_m'] for d in c['deltas']),max_yaw_delta_rad=max(abs(d['yaw_delta_rad']) for d in c['deltas'])) for c in finish]
    row=dict(seed=attempt['seed'],status=result['status'],steps=len(states)-1,events=result['events'],native_calls=native_counts,finish_events=snaps,
        min_human_pair_m=pair_min,pair_overlap_beyond_1cm_s=pair_overlap,min_nominal_robot_human_m=robot_min,robot_overlap_beyond_1cm_s=robot_overlap,min_wall_human_m=wall_min,wall_overlap_beyond_1cm_s=wall_overlap,max_one_step_displacement_m=largest_step,
        id_count=12,ids_preserved=True,reset_check=result.get('reset_check'),observation_outside_space_steps=result.get('observation_outside_space_steps'))
    if result['status']=='PASS':assert len(states)==1201 and attempt['returncode']==0
    else:row['error']=result.get('error')
    if attempt['seed']==42 and 'reset_check' in result:
        check=result['reset_check'];assert check['max_state_error']<1e-9 and check['membership_match']
        replay=load(d/'reset_repeat.json');assert len(replay['rows'])==500
        total_native_steps+=500
        path=d/'native_preview_4x.gif';duration=0
        with Image.open(path) as im:
            for n in range(im.n_frames):im.seek(n);im.load();duration+=im.info.get('duration',0)
            assert duration==30000
        row['gif_duration_ms']=duration
    for path in d.glob('*.png'):
        with Image.open(path) as im:im.verify()
    rows.append(row);total_native_steps+=len(states)-1
    manifest.append(dict(seed=attempt['seed'],files={str(p.relative_to(REPO)):sha(p) for p in sorted(d.iterdir()) if p.is_file()}))
cp=HERE/'generated/checkpoint'
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)
for name,digest in load(cp/'preserved_hashes.json').items():assert sha(REPO/name)==digest,name
# Historical seed42 recording is preserved, not replaced by this new run.
old=load(SOCIAL/'reports/socnavgym_formation_s42_trajectory.json')
new=load(out/'seed42/trajectory.json')[1:];first_difference=None
for index,(earlier,current) in enumerate(zip(old,new)):
    previous={h['id']:h for h in earlier['humans']};now={h['id']:h for h in current['humans']}
    error=max(math.hypot(previous[k]['x']-now[k]['x'],previous[k]['y']-now[k]['y']) for k in previous)
    if error>1e-8:
        first_difference=dict(old_zero_based_step=index,new_completed_step=index+1,max_position_difference_m=error);break
historical=dict(first_position_difference=first_difference,old_membership_changes=len(load(SOCIAL/'reports/socnavgym_formation_s42.json')['group_membership_changes']),new_membership_changes=len(rows[0]['events']),
    note='Same seed does not establish full1200-step historical reproduction. New run has passive instrumentation and different render cadence; cause of later trajectory difference is not established. Current reset check establishes only a500-step prefix.')
(out/'historical_comparison.json').write_text(json.dumps(historical,indent=2)+'\n')
snaps=[s for r in rows for s in r['finish_events'] if s['max_position_delta_m']>1e-6]
checks=dict(status='PASS' if all(r['status']=='PASS' for r in rows) else 'PARTIAL',native_steps=total_native_steps,seed_count=3,ids_unique_and_preserved=True,reset_prefix_checked=bool(rows[0]['reset_check']),historical_full_reproduction=False,native_finish_calls_with_position_snap=len(snaps),largest_finish_snap_m=max([s['max_position_delta_m'] for s in snaps],default=0),preserved_files=len(load(cp/'preserved_hashes.json')),user_index_and_outside_worktree_unchanged=True,
    gazebo_lifecycle_acceptance='NOT_IMPLEMENTED: standalone contract measurements only; native snaps, overlaps and mapping to warehouse need explicit handling/evidence')
(out/'summary.json').write_text(json.dumps(dict(checks=checks,trials=rows),indent=2,allow_nan=False)+'\n')
(out/'manifest.json').write_text(json.dumps(dict(analysis_source_sha256=sha(Path(__file__)),trials=manifest),indent=2)+'\n')
(out/'validation.json').write_text(json.dumps(checks,indent=2)+'\n')
lines=['# SocNavGym native group lifecycle validation','','Three predeclared seeds42/43/44,1200 native steps each (120s), with a500-step same-seed reset prefix for seed42. Existing14×14m formation demo config is copied byte-for-byte. The robot remains stationary; there is no ROS/Gazebo or warehouse mapping. No group event is injected and native source/policy parameters are unchanged.','','| Seed | Status | Membership changes | Native formations finished | Min pair gap (m) | Pair overlap >1cm (s) | Largest finish snap (m) |','|---|---|---:|---:|---:|---:|---:|']
for r0 in rows:lines.append(f"| {r0['seed']} | {r0['status']} | {len(r0['events'])} | {len(r0['finish_events'])} | {r0['min_human_pair_m']:.4f} | {r0['pair_overlap_beyond_1cm_s']:.1f} | {max([s['max_position_delta_m'] for s in r0['finish_events']],default=0):.4f} |")
lines+=['','Each state has12 unique human IDs, with no lost/duplicated member across independent, pending and active-group ownership. The worker also checks that each numeric human ID retains the same actual Human object within an epoch. Native group objects lack a persistent numeric group-ID contract; observer labels are assigned from object identity and stay stable for the life of that object. They are external measurement labels, not new upstream IDs.','', 'Reset is a new epoch: numeric human IDs are reused and objects are recreated. The seed42 reset after the120s episode clears active/pending groups; its first500 steps are compared with the original prefix, including positions/headings/speeds and group/pending membership. Recorded error and outcome remain in summary.json. A future adapter needs an epoch/reset handshake, rather than assuming human ID alone is globally unique.','', 'Native finish_human_crowd_formation may call arrange_humans, changing positions instantly inside that method. The passive method wrappers record before/after states and per-human deltas without changing RNG or return values. The approximate branch changes headings instead. Finish snapshots are separated from ordinary per-step integration; all measured snaps remain in summary.json/native_calls.json. No smoothing or clipping is applied to make them appear continuous.','', 'Sampled geometric gaps use human circles of radius0.36m, nominal robot radius0.25m, and the inner edges of the native square walls (MAP/2−wall_thickness). Minimum signed gaps and durations beyond1cm are retained, including overlaps. This is neither a collision-free guarantee nor validation in warehouse geometry.','', 'The historical seed42 recording agrees in position through666 completed steps, then differs at step667 by about1.06cm and eventually has4 membership changes versus5 here. Historical data are retained in place; the cause is not established. Do not claim full1200-step historical determinism from the successful500-step reset prefix. See historical_comparison.json.', '', 'Measured human circle-proxy penetration reaches0.56–0.70m across the three seeds. These are circle geometries proposed for Gazebo proxies, not a direct measurement of native ellipse contacts or animated-body mesh collisions. Seed44 has two instantaneous native formation position snaps,0.3264m and0.2049m, plus approximate-formation yaw snaps. On this evidence, automatic formation is NOT promoted into Gazebo in this milestone; fixed-group Gazebo demonstrations remain available. This closes the native capability/limitation check without hiding overlaps or adding new avoidance rules.', '', 'These data validate ownership/reset observability; they do not yet accept the formation lifecycle in Gazebo. The current Gazebo GroupKernel supports one fixed group and omits full native formation/episode logic. Extending it needs all individual and group update paths, stable actor/proxy IDs, reset/clock synchronization and an explicit decision about representing native arrangement snaps. Do not silently reset the scene or teleport actors and call the result natural formation.','', 'Gymnasium observation-space warnings are retained. The full native step path was exercised, but invalid RL observations are not certified for training. Preview GIF uses native2D render frames with explicit NOT GAZEBO label, every1sim second at4× playback (30s for120sim seconds); no AI image generation or Gazebo camera is involved.','', 'All source hashes, native method calls, episode trajectories, reset prefix, event PNGs, validation and image hashes are saved. Source vendor, original robot source, current Gazebo adapter and warehouse files match the audit checkpoint. ComplexityNav is unchanged.','', '![Native formation — standalone2D,4×](seed42/native_preview_4x.gif)']
(out/'LIFECYCLE_REPORT.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(checks));print(json.dumps([{k:r[k] for k in ['seed','status','native_calls','finish_events','min_human_pair_m','max_one_step_displacement_m','reset_check']} for r in rows],indent=2))
