#!/usr/bin/env python3
"""Independent measured acceptance of the adapter repair, including swept paths."""
import argparse,collections,hashlib,json,math,subprocess
from pathlib import Path
from PIL import Image
import yaml
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3];SOCIAL=HERE.parent.parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();out=a.report_dir.resolve()
load=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protocol=load(out/'protocol.json');progress=load(out/'progress.json')
assert len(progress['attempts'])==progress['planned']==3
for name,digest in protocol['source_sha256'].items():assert sha(REPO/name)==digest,name
cfg=yaml.safe_load((out/'formation.yaml').read_text());dt=cfg['episode']['time_step'];radius=cfg['human']['human_diameter']/2
limit=cfg['env']['min_map_x']/2-cfg['wall']['wall_thickness'];vmax=cfg['human']['max_advance_human'];wmax=cfg['human']['max_rotation_human']
reference=Path(protocol['native_reference']);manifest=[];summaries=[];failures=[]
def swept_gap(a,b,c,d,combined_radius):
    # Closest point of a relative-motion segment; independent of repair's root solver.
    px,py=a[0]-c[0],a[1]-c[1];dx,dy=(b[0]-a[0])-(d[0]-c[0]),(b[1]-a[1])-(d[1]-c[1])
    norm=dx*dx+dy*dy;t=0. if norm<1e-30 else max(0.,min(1.,-(px*dx+py*dy)/norm))
    return math.hypot(px+t*dx,py+t*dy)-combined_radius
for attempt in progress['attempts']:
    seed=attempt['seed'];d=out/f'seed{seed}';res=load(d/'result.json');states=load(d/'trajectory.json');native=load(reference/f'seed{seed}/trajectory.json')
    issues=[]
    if res['status']!='PASS' or len(states)!=1201:issues.append('Native step/reset worker did not complete')
    assert states[0]==native[0],'Initial scene must exactly match native reference'
    rrobot=res.get('repair_effective_robot_radius',cfg['robot']['robot_radius'])
    paths={h['id']:0. for h in states[0]['humans']};minimum_pair=1e9;minimum_robot=1e9;minimum_wall=1e9;max_move=0.;max_yaw=0.
    pair_s=0.;robot_s=0.;wall_s=0.;robot0=states[0]['robot'];ids=set(paths)
    for previous,current in zip(states,states[1:]):
        hs=current['humans'];old=previous['humans'];assert [h['id'] for h in hs]==[h['id'] for h in old] and set(h['id'] for h in hs)==ids
        group_members=[i for g in current['groups'] for i in g['members']];assert len(group_members)==len(set(group_members)) and set(group_members)<=ids
        pair_gap=1e9;robot_gap=1e9;wall_gap=1e9
        for i,(h,o) in enumerate(zip(hs,old)):
            hp=(h['x'],h['y']);op=(o['x'],o['y']);move=math.dist(hp,op);paths[h['id']]+=move;max_move=max(max_move,move)
            yaw=abs(math.atan2(math.sin(h['yaw']-o['yaw']),math.cos(h['yaw']-o['yaw'])));max_yaw=max(max_yaw,yaw)
            wall_gap=min(wall_gap,limit-max(abs(h['x']),abs(h['y']),abs(o['x']),abs(o['y']))-radius)
            robot_gap=min(robot_gap,swept_gap(op,hp,(robot0['x'],robot0['y']),(robot0['x'],robot0['y']),radius+rrobot))
            for j in range(i):pair_gap=min(pair_gap,swept_gap(op,hp,(old[j]['x'],old[j]['y']),(hs[j]['x'],hs[j]['y']),2*radius))
        minimum_pair=min(minimum_pair,pair_gap);minimum_robot=min(minimum_robot,robot_gap);minimum_wall=min(minimum_wall,wall_gap)
        pair_s+=dt*(pair_gap<-.01);robot_s+=dt*(robot_gap<-.01);wall_s+=dt*(wall_gap<-.01)
    calls=load(d/'native_calls.json');finish=[c for c in calls if c['epoch']==0 and c['native_method']=='finish_human_crowd_formation'];disperse=[c for c in calls if c['epoch']==0 and c['native_method'].startswith('disperse_')]
    # Reject a nominally safe but frozen or disabled-formation result.
    if not finish:issues.append('No completed formation')
    if not disperse:issues.append('No native dispersal')
    if min(paths.values())<1.:issues.append('At least one initially moving human travelled less than1m')
    if sum(v>5. for v in paths.values())<6:issues.append('Fewer than6 humans travelled5m')
    if sum(paths.values())<100.:issues.append('Total human path below100m')
    if min(minimum_pair,minimum_robot,minimum_wall)<-1e-7:issues.append('Swept geometric penetration')
    if max_move>vmax*dt+1e-7:issues.append('Translation speed bound violated')
    if max_yaw>wmax*dt+1e-7:issues.append('Yaw speed bound violated')
    finish_snap=max([max(x['position_delta_m'] for x in c['deltas']) for c in finish],default=0.)
    if finish_snap>1e-8:issues.append('Formation commit still changes positions instantly')
    repair_steps=[x for x in load(d/'repair_steps.json') if x['epoch']==0]
    native_summary=next(x for x in load(reference/'summary.json')['trials'] if x['seed']==seed)
    row=dict(seed=seed,status='PASS' if not issues else 'FAIL',issues=issues,steps=len(states)-1,
        formations=len(finish),dispersal_calls=len(disperse),membership_changes=len(res['events']),
        min_swept_pair_m=minimum_pair,min_swept_robot_m=minimum_robot,min_swept_wall_m=minimum_wall,pair_overlap_beyond_1cm_s=pair_s,
        max_displacement_per_step_m=max_move,max_yaw_per_step_rad=max_yaw,max_formation_commit_position_change_m=finish_snap,
        path_by_human_m=paths,total_path_m=sum(paths.values()),sweep_limited_steps=sum(x['safe_fraction']<1. for x in repair_steps),
        full_stop_steps=sum(x['safe_fraction']<1e-6 for x in repair_steps),native_min_sampled_pair_m=native_summary['min_human_pair_m'],
        reset_check=res.get('reset_check'),events=res['events'])
    if seed==42:
        if not res.get('reset_check') or res['reset_check']['max_state_error']>=1e-9 or not res['reset_check']['membership_match']:row['issues'].append('Reset repeat not verified');row['status']='FAIL'
        with Image.open(d/'native_preview_4x.gif') as image:
            duration=0
            for i in range(image.n_frames):image.seek(i);image.load();duration+=image.info.get('duration',0)
            assert duration==30000
    for image_path in d.glob('*.png'):
        with Image.open(image_path) as image:image.verify()
    summaries.append(row);failures.extend([dict(seed=seed,reason=x) for x in row['issues']])
    manifest.append(dict(seed=seed,files={str(p.relative_to(REPO)):sha(p) for p in d.iterdir() if p.is_file()}))
active_reset=load(out/'active_reset_check.json')
assert active_reset['status']=='PASS' and active_reset['groups_before'] and not active_reset['groups_after']
assert active_reset['max_state_error']==0. and active_reset['membership_match'] and active_reset['old_adapter_velocities_cleared']
md=out/'moving_regression';moving=load(md/'result.json');mstates=load(md/'trajectory.json')
assert moving['status']=='PASS' and len(mstates)==601 and moving['initial_group_center_path_m']>1. and moving['moving_dispersals']
mp=1e9;mw=1e9;mm=0.;my=0.
for previous,current in zip(mstates,mstates[1:]):
    old=previous['humans'];hs=current['humans'];assert len(hs)==len(set(h['id'] for h in hs))==12
    for i,(h,o) in enumerate(zip(hs,old)):
        assert h['id']==o['id'];mm=max(mm,math.hypot(h['x']-o['x'],h['y']-o['y']))
        my=max(my,abs(math.atan2(math.sin(h['yaw']-o['yaw']),math.cos(h['yaw']-o['yaw']))))
        mw=min(mw,limit-max(abs(h['x']),abs(h['y']),abs(o['x']),abs(o['y']))-radius)
        for j in range(i):mp=min(mp,swept_gap((o['x'],o['y']),(h['x'],h['y']),(old[j]['x'],old[j]['y']),(hs[j]['x'],hs[j]['y']),2*radius))
assert min(mp,mw)>=-1e-7 and mm<=vmax*dt+1e-7 and my<=wmax*dt+1e-7
moving.update(min_swept_pair_m=mp,min_swept_wall_m=mw,max_step_m=mm,max_yaw_rad=my,
    verification_scope='Independent pair/wall swept checks and speed bounds; stationary robot is checked by adapter, this supplemental trajectory does not record robot pose')
with Image.open(md/'moving_preview_4x.gif') as image:
    duration=0
    for i in range(image.n_frames):image.seek(i);image.load();duration+=image.info.get('duration',0)
    assert duration==15000
(out/'moving_regression_check.json').write_text(json.dumps(moving,indent=2)+'\n')
cp=HERE/'generated/checkpoint'
assert (cp/'index.patch').read_bytes()==subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
assert (cp/'outside.patch').read_bytes()==subprocess.check_output(['git','diff','--binary','--','.',':(exclude)experiments/social_navigation'],cwd=REPO)
for name,digest in load(cp/'preserved_hashes.json').items():assert sha(REPO/name)==digest,name
validation=dict(status='PASS' if not failures else 'FAIL',trials=3,steps=sum(r['steps'] for r in summaries)+500,additional_active_reset_steps=active_reset['total_additional_steps'],additional_moving_steps=600,total_checked_steps=sum(r['steps'] for r in summaries)+500+active_reset['total_additional_steps']+600,active_group_reset='PASS',moving_group_regression='PASS',failures=failures,
    original_initial_states_exactly_equal=True,native_vendor_and_robot_unchanged=True,preserved_files=len(load(cp/'preserved_hashes.json')),index_and_outside_worktree_preserved=True,
    scope='Standalone stationary-robot square-room adapter only; no moving-robot/Gazebo/warehouse acceptance')
(out/'summary.json').write_text(json.dumps(dict(validation=validation,trials=summaries),indent=2)+'\n')
(out/'manifest.json').write_text(json.dumps(dict(analysis_sha256=sha(Path(__file__)),supplemental_source_sha256={p.name:sha(p) for p in [HERE/'probe_active_reset.py',HERE/'probe_moving.py',HERE/'test_swept.py']},controls_sha256={str(p.relative_to(REPO)):sha(p) for p in [out/'protocol.json',out/'progress.json',out/'formation.yaml',out/'active_reset_check.json',out/'moving_regression_check.json',out/'geometry_fixture_check.json',*list(md.iterdir())] if p.is_file()},trials=manifest),indent=2)+'\n')
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
lines=['# Formation repair: measured standalone acceptance','','The repair is an opt-in experiment adapter, not an upstream SocNavGym fix and not a Gazebo integration. Vendor code, baseline fixed-group Gazebo adapter and ROS robot source are unchanged. Original native results remain under socnavgym_lifecycle/reports/run01. Three fixed-adapter120s runs use the exact same initial state and copied config as seeds42/43/44 in that reference.','','| Seed | Native min sampled pair gap (m) | Fixed min swept pair gap (m) | Formations / dispersal calls | Largest step (m) | Human path total (m) |','|---|---:|---:|---|---:|---:|']
for r in summaries:lines.append(f"| {r['seed']} | {r['native_min_sampled_pair_m']:.4f} | {r['min_swept_pair_m']:.4f} | {r['formations']} / {r['dispersal_calls']} | {r['max_displacement_per_step_m']:.4f} | {r['total_path_m']:.1f} |")
lines+=['', 'Repair1: keep current member positions and original Human objects when native formation commits. The native method still selects membership/type/goals and consumes its original RNG draws, but its immediate arrangement positions and heading snap are not exposed. Group centers are recomputed from accepted members. Final heading changes are bounded to1.57rad/s (0.157rad per0.1s tick). Native instantaneous normal-arrangement proposals are recorded separately in repair_events.json; measured formation commit position change is zero.','', 'Repair2: feed native intended motion into a global individual-disc ORCA pass, with current accepted positions/velocities, all humans, room walls and a stationary robot. Human radius remains0.36m, ORCA padding5mm per agent. Standing group members are held in place and turn gradually inward; native individual and moving-group paths still supply intended motion. This explicitly changes the movement policy and stationary-group treatment. Do not describe it as unmodified native behavior.','', 'An analytic swept-disc limiter is an emergency fallback over each full linear tick (2mm clearance margin), covering pair, fixed robot and square-wall boundaries. It shortens the common integration interval before first contact; it never pushes already-overlapping people apart. Initially overlapping scenes and moving-robot use are rejected. In the3 measured episodes this fallback never shortened a step; unit fixtures separately exercised crossing paths that have safe endpoints but collide between ticks, wall crossing, robot crossing, free motion and rejection of an invalid start.','', 'Independent analysis uses closest approach of relative line segments rather than the fallback root solver. Main runs have no swept proxy penetration; maximum displacement is0.080000011m per0.1s tick (RVO float tolerance), yaw0.157rad per tick. The fixed runs still form/disperse groups and travel358–441m in aggregate per episode; there are no common-interval full-stop steps. This does not prove subjective naturalness or general crowd safety.','', 'All spontaneously completed formations in these3 repaired runs are standing groups. A separate initial-moving-group fixture (9 independent people+1 group of3, seed44,600 steps) verifies walking and native moving-group dispersal: initial group center travels15.12m before/while remaining moving; a moving dispersal occurs at step330. Pair/wall swept checks and speed/yaw bounds pass. Its initial scene differs from the primary paired trials and is not included as another paired native comparison. Supplemental robot pose is not logged, so independent robot-gap claims apply to the3 primary runs only.','', 'Reset checks: main seed42 repeats500 steps exactly after the full episode. A supplemental probe resets at the first actually formed active group (step276), clears group membership and adapter velocity state, creates new Human objects with reused numeric IDs, then repeats500 steps with zero measured state error and matching membership. Use (reset_epoch,human_id), not human_id alone.','', f"Validation covers {validation['total_checked_steps']} fixed-adapter updates:4100 primary/reset-prefix,776 active-group reset probe and600 moving regression. IDs, exact initial-state equality, original user index/outside-worktree and11298 preserved files are checked. GIFs decode to30s and15s, both4× simulation time. The2D renders are from recorded simulations, labelled ADAPTER FIX / NOT GAZEBO, with no AI image generation.",'', 'Limits: stationary robot, empty14×14m native square room only; furniture/warehouse obstacles and measured moving robot are not supported by this prototype. No Gazebo lifecycle acceptance is claimed. This validation does not resolve native RL observation-space warnings or full historical1200-step determinism. Formation/dispersal timing and human trajectories differ from native after the adapter changes dynamics. No framework ranking is implied.','', 'Next: extend and validate the adapter contract for warehouse obstacles and measured robot motion, then run a small live Gazebo formation test while retaining this standalone and native evidence. ComplexityNav work remains deferred until this step is clear.','', '![Fixed formation — recorded2D,4×](seed42/native_preview_4x.gif)','','![Moving group regression — recorded2D,4×](moving_regression/moving_preview_4x.gif)']
(out/'FIX_REPORT.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(validation));print(json.dumps([{k:r[k] for k in ['seed','status','issues','formations','dispersal_calls','min_swept_pair_m','max_displacement_per_step_m','max_yaw_per_step_rad','total_path_m','full_stop_steps']} for r in summaries],indent=2))
raise SystemExit(0 if validation['status']=='PASS' else 1)
