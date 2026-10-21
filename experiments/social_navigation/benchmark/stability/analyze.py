#!/usr/bin/env python3
"""Analyze the fixed six-trial cohort after simulation has finished."""
import argparse,csv,hashlib,json,math,subprocess,sys
from collections import Counter
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);p.add_argument('--skip-round-analysis',action='store_true');a=p.parse_args();report=a.report_dir.resolve()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0,str(BENCH/'localization_diagnosis'))
from geometry import MapField
from recluster import recluster
field=MapField(BENCH.parent/'warehouse/warehouse.yaml')
protocol=json.loads((report/'protocol.json').read_text());progress=json.loads((report/'progress.json').read_text())
assert progress['completed']==progress['planned']==6 and progress['stopped'] is None
assert len(protocol['plan'])==6
for source,digest in protocol['source_sha256'].items():assert sha(REPO/source)==digest,source
assert all(not item['infrastructure_errors'] and not item['remaining_nodes'] for item in progress['attempts'])
for roundnum in [1,2,3]:
 directory=report/f'round{roundnum:02d}'
 if not a.skip_round_analysis:
  with (directory/'offline_analysis.log').open('w') as log:
   subprocess.run([sys.executable,str(BENCH/'navigation_diagnostics/analyze.py'),'--report-dir',str(directory)],cwd=REPO,stdout=log,stderr=subprocess.STDOUT,check=True)
rows=[];events=[];shutdown=[];raws=[]
for item in progress['attempts']:
 directory=report/f"round{item['round']:02d}";trial=json.loads((report/item['report']).read_text());runtime=BENCH/trial['runtime']
 summaries=json.loads((directory/'summary.json').read_text());row=next(r for r in summaries if r['policy']==item['policy']);row.update(round=item['round'],sequence=item['sequence'],report=item['report'])
 assert trial['commit']==protocol['head'] and trial['seed']==42 and trial['navigation_capture']['counts']
 assert trial['rendering']['hardware_verified'] and trial['process_cleanup']['status']=='PASS' and not trial['process_cleanup']['remaining']
 assert any('NVIDIA' in line for line in trial['process_cleanup']['ogre_renderer_lines'])
 # Keep complete raw message counts and hashes from both recorder and dense geometry analysis.
 counts=Counter();global_grid_ok=0;dense_pairs=[];first_backup_executing=None;gate=float((runtime/'start_gate').read_text())
 import base64,zlib
 for line in (runtime/'diagnostics.jsonl').open():
  raw=json.loads(line);counts[raw['kind']]+=1
  if raw['kind']=='gazebo' and gate<=raw['data']['time']<=gate+trial['sim_seconds']:
   people=[dict(id=int(k.split('_')[1]),**v) for k,v in raw['data']['poses'].items() if k!='mecanum_robot']
   if len(people)==3:dense_pairs.append(dict(elapsed=raw['data']['time']-gate,clearance_m=min(math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(people) for j in people[i+1:]),humans=people))
  if raw['kind']=='behavior_backup_status' and first_backup_executing is None and any(v['status']==2 for v in raw['data']['status_list']):first_backup_executing=raw['receipt_sim']-gate
  if raw['kind']=='global_costmap':
   d=raw['data'];data=zlib.decompress(base64.b64decode(d['data_b64']));assert len(data)==d['info']['width']*d['info']['height'];global_grid_ok+=1
 assert dict(counts)==trial['diagnostic_counts']==row['counts'] and global_grid_ok>0
 tr=json.loads((directory/'timeline.json').read_text())[row['name']]
 ar=tr['amcl'];largest=max(ar,key=lambda r:r['error_m']);largest_jump=tr['jump_after']
 principal_error=row['native_cluster_match_error_max_m'];pc=recluster(runtime,tr['particle_cloud'])
 row['principal_yaw_cluster_error_max_m']=principal_error
 row['native_cluster_match_error_max_m']=max(r['max_cluster_amcl_error_m'] for r in pc if 'max_cluster_amcl_error_m' in r)
 row['max_native_cluster_count']=max(len(r['clusters']) for r in pc)
 (directory/(row['name']+'_circular_clusters.json')).write_text(json.dumps(pc,indent=2,allow_nan=False))
 maximum_x=-float('inf');drawdown=0.
 for r in ar:
  maximum_x=max(maximum_x,r['actual']['x']);drawdown=max(drawdown,maximum_x-r['actual']['x'])
 row['max_actual_world_x_drawdown_m']=drawdown
 row['world_goal_error_m']=trial['goal_error_world_m']
 ev=dict(round=item['round'],policy=item['policy'],status=trial['status'],largest_error=largest,jump_before=tr['jump_before'],jump_after=largest_jump,
  first_estimated_center_occupied_s=row['first_estimated_center_occupied_s'],first_persistent_translation_stop_s=row['first_persistent_translation_stop_s'],max_native_cluster_count=row['max_native_cluster_count'],
  max_occupied_weight_cloud=max(pc,key=lambda r:r['occupied_weight']),
  occupied_samples=[r for r in ar if r['occupied']],warning_error_logs=[r for r in tr['rosout'] if r['level']>=30])
 cloud_window={}
 for label,anchor in [('before_largest_jump',tr['jump_before']),('after_largest_jump',tr['jump_after'])]:
  cloud=min(pc,key=lambda r:abs(r['elapsed']-anchor['elapsed']))
  cloud_window[label]=dict(requested_elapsed_s=anchor['elapsed'],cloud_elapsed_s=cloud['elapsed'],cloud_time_offset_s=cloud['elapsed']-anchor['elapsed'],occupied_weight=cloud['occupied_weight'],clusters=[dict(**c,mean_static_occupied=bool(field.occupied_at(c['x'],c['y']))) for c in cloud['clusters']])
 ev['largest_jump_clouds']=cloud_window
 assert abs(min(d['clearance_m'] for d in dense_pairs)-row['min_human_pair_m'])<1e-12
 row['human_pair_overlap_over_1cm_s']=sum(v['elapsed']-u['elapsed'] for u,v in zip(dense_pairs,dense_pairs[1:]) if v['clearance_m']<-.01)
 ev['minimum_human_pair_event']=min(dense_pairs,key=lambda d:d['clearance_m'])
 ev['first_backup_executing_status_s']=first_backup_executing
 plans=tr['global_plans'];ev['first_plan_leaves_channel_s']=next((pl['elapsed'] for pl in plans if pl['leaves_channel_core']),None)
 initial_dx=[]
 for pl in plans:
  length=0.;points=pl['points'];look=points[-1]
  for first,second in zip(points,points[1:]):
   length+=((first[0]-second[0])**2+(first[1]-second[1])**2)**.5;look=second
   if length>=1.:break
  initial_dx.append(dict(elapsed=pl['elapsed'],initial_1m_dx=look[0]-points[0][0],length_m=pl['length_m'],leaves_channel_core=pl['leaves_channel_core'],start=points[0],lookahead=look))
 ev['minimum_plan_initial_1m_dx']=min(initial_dx,key=lambda pl:pl['initial_1m_dx'])
 row['minimum_plan_initial_1m_dx_m']=ev['minimum_plan_initial_1m_dx']['initial_1m_dx']
 for label,anchor in [('before_jump',tr['jump_after']['elapsed']),('before_persistent_stop',row['first_persistent_translation_stop_s'])]:
  previous=[pl for pl in plans if anchor is not None and pl['elapsed']<=anchor]
  ev['last_global_plan_'+label]=dict(elapsed=previous[-1]['elapsed'],length_m=previous[-1]['length_m'],leaves_channel_core=previous[-1]['leaves_channel_core'],first_waypoints=previous[-1]['points'][:20]) if previous else None
 events.append(ev)
 # Teardown warnings are retained separately from runtime goal results.
 log=(directory/(row['name']+'.log')).read_text(errors='replace');worker=(runtime/'worker.log').read_text(errors='replace')
 notes=[line for line in log.splitlines() if any(term in line for term in ["exit code -11","failed to terminate","escalating to 'SIGTERM'","process has died"])];worker_reset='ConnectionResetError' in worker
 shutdown.append(dict(round=item['round'],policy=item['policy'],launch_warnings=notes,worker_connection_reset=worker_reset,process_cleanup=trial['process_cleanup']))
 raw_entry=dict(round=item['round'],policy=item['policy'],runtime=str(runtime.relative_to(BENCH)),raw_sha256={file:sha(runtime/file) for file in ['diagnostics.jsonl','start_gate','scene.json','nav2_scenario.yaml','crowdsim_metadata.json','process_cleanup.json','rendering.json','ogre2_session.log']})
 raws.append(raw_entry);rows.append(row)
rows.sort(key=lambda r:r['sequence']);checks=dict(all_six_planned_trials_retained=True,all_recorded_message_counts_match=True,all_global_grids_lossless_size_valid=True,all_source_hashes_fixed_through_cohort=True,all_trial_commits_match_protocol=True,all_nvidia_preflight_and_fresh_ogre_logs=True,all_owned_processes_cleaned=True,all_native_cluster_matches=sum(r['native_cluster_matches'] for r in rows),native_max_cluster_amcl_error_max_m=max(r['native_cluster_match_error_max_m'] for r in rows))
assert checks['native_max_cluster_amcl_error_max_m']<1e-10
policy_stats={}
for policy in ['sfm','orca_plus']:
 selected=[r for r in rows if r['policy']==policy]
 assert len(selected)==3
 policy_stats[policy]=dict(trials=3,status_counts=dict(Counter(r['status'] for r in selected)),amcl_error_max_m=max(r['amcl_error_max_m'] for r in selected),largest_amcl_step_m=max(r['largest_amcl_step_m'] for r in selected),minimum_robot_human_footprint_m=min(r['min_robot_human_footprint_m'] for r in selected),minimum_human_pair_m=min(r['min_human_pair_m'] for r in selected))
with (report/'results.csv').open('w',newline='') as f:
 keys=['sequence','round','policy','status','sim_seconds','world_goal_error_m','amcl_error_max_m','largest_amcl_step_m','max_actual_world_x_drawdown_m','odom_error_max_m','min_robot_human_footprint_m','min_human_pair_m','min_robot_machine_footprint_m','human_pair_overlap_over_1cm_s','max_particle_occupied_weight','max_native_cluster_count','native_cluster_matches','native_cluster_match_error_max_m']
 writer=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore',lineterminator='\n');writer.writeheader();writer.writerows(rows)
labels=[f"R{r['round']} {r['policy']}" for r in rows];colors=['#4d83b5' if r['policy']=='sfm' else '#bd793d' for r in rows]
fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
for ax,key,title in zip(axes,['amcl_error_max_m','largest_amcl_step_m','min_robot_human_footprint_m'],['Maximum AMCL position error [m]','Largest AMCL publication step [m]','Minimum robot-human footprint clearance [m]']):
 vals=[r[key] for r in rows];ax.bar(labels,vals,color=colors);ax.set_ylabel(title);ax.grid(axis='y',alpha=.2)
 for i,v in enumerate(vals):ax.text(i,v,f'{v:.3f}',ha='center',va='bottom',fontsize=8)
 ax.margins(y=.18)
fig.suptitle('Fixed seed42 repetitions | diagnostic cohort after cleanup | no policy ranking');fig.tight_layout();fig.savefig(report/'stability_metrics.png',dpi=160,bbox_inches='tight');plt.close(fig)
manifest=dict(protocol_head=protocol['head'],analysis_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),protocol_sha256=sha(report/'protocol.json'),progress_sha256=sha(report/'progress.json'),raw_inputs=raws,angle_reference_sources=json.loads((HERE/'generated/reference/manifest.json').read_text()),
 analysis_sources={str(p.relative_to(REPO)):sha(p) for p in [Path(__file__),BENCH/'navigation_diagnostics/analyze.py',BENCH/'navigation_diagnostics/native_clusters.cpp',BENCH/'localization_diagnosis/geometry.py',HERE/'recluster.py']},method='Same committed native geometry and native AMCL clustering as navigation_diagnostics, with independent circular-mean yaw lift before reclustering to avoid quaternion +/-pi seam. Principal-yaw output retained in round analyses. Six fixed repetitions, no source/controller/policy tuning or retry replacement. All analysis ran after simulation.')
for name,data in [('summary.json',dict(trials=rows,policies=policy_stats,checks=checks)),('events.json',events),('shutdown_notes.json',shutdown),('manifest.json',manifest),('checks.json',dict(status='PASS',checks=checks))]:(report/name).write_text(json.dumps(data,indent=2,allow_nan=False))
print(json.dumps(dict(policies=policy_stats,checks=checks),indent=2))
