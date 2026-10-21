#!/usr/bin/env python3
"""Offline paired analysis of twelve multi-seed AMCL trials; all outcomes retained."""
import argparse,csv,hashlib,json,math,subprocess,sys
from collections import Counter
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
sys.path.insert(0,str(BENCH));sys.path.insert(0,str(BENCH/'stability'))
from recluster import recluster
sys.path.insert(0,str(BENCH/'localization_trials'))
from profiles import PROFILES as ALL_PROFILES,configure
PROFILES={key:ALL_PROFILES[key] for key in ['baseline','lf_prob_beamskip']}
import yaml
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();report=a.report_dir.resolve()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protocol=json.loads((report/'protocol.json').read_text());progress=json.loads((report/'progress.json').read_text())
assert progress['completed']==progress['planned']==12 and progress['stopped'] is None
assert protocol['profiles']==PROFILES and len(protocol['plan'])==12 and protocol['seeds']==[42,43,44]
for source,digest in protocol['source_sha256'].items():assert sha(REPO/source)==digest,source
assert all(not x['infrastructure_errors'] and not x['remaining_nodes'] for x in progress['attempts'])
for seed in protocol['seeds']:
    for profile in PROFILES:
        directory=report/f'seed{seed}'/profile
        with (directory/'offline_analysis.log').open('w') as log:
            subprocess.run([sys.executable,str(BENCH/'navigation_diagnostics/analyze.py'),
                '--localization-ablation','--seed',str(seed),'--paired-reference',str(report/f'seed{seed}'/'baseline'),
                '--report-dir',str(directory)],stdout=log,stderr=subprocess.STDOUT,check=True,cwd=REPO)
rows=[];manifest=[];checks=[];readbacks=[]
for item in progress['attempts']:
    directory=report/f"seed{item['seed']}"/item['profile'];trial_path=report/item['report'];trial=json.loads(trial_path.read_text());runtime=BENCH/trial['runtime']
    assert trial['commit']==protocol['head'] and trial['protocol_valid'] and trial['gazebo_exit']['clean']
    assert trial['seed']==item['seed']
    assert trial['localization_profile']==item['profile'] and trial['human_policy']==item['policy']
    expected=configure(yaml.safe_load((BENCH.parent/'warehouse/nav2_warehouse.yaml').read_text()),item['profile'],trial['robot_start'])
    assert yaml.safe_load((runtime/'nav2_scenario.yaml').read_text())==expected
    assert trial['localization_parameter_overrides']==PROFILES[item['profile']]
    readbacks.append(dict(seed=item['seed'],profile=item['profile'],policy=item['policy'],values=trial['amcl_parameters_readback']))
    row=next(x for x in json.loads((directory/'summary.json').read_text()) if x['policy']==item['policy'])
    row.update(seed=item['seed'],profile=item['profile'],sequence=item['sequence'],report=item['report'],gazebo_clean=True)
    tolerance=expected['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']
    row['actual_goal_xy_tolerance_m']=tolerance
    row['actual_goal_xy_within_tolerance']=trial['goal_error_world_m']<=tolerance
    row['reported_success_outside_actual_xy_tolerance']=trial['status']=='SUCCEEDED' and not row['actual_goal_xy_within_tolerance']
    timeline=json.loads((directory/'timeline.json').read_text())[trial['name']]
    circular=recluster(runtime,timeline['particle_cloud'])
    row['principal_yaw_cluster_error_max_m']=row['native_cluster_match_error_max_m']
    errors=[x['max_cluster_amcl_error_m'] for x in circular if 'max_cluster_amcl_error_m' in x]
    row['native_cluster_match_error_max_m']=max(errors)
    row['max_native_cluster_count']=max(len(x['clusters']) for x in circular)
    (directory/(trial['name']+'_circular_clusters.json')).write_text(json.dumps(circular,indent=2,allow_nan=False))
    counts=Counter();dense=[];gate=float((runtime/'start_gate').read_text())
    for line in (runtime/'diagnostics.jsonl').open():
        raw=json.loads(line);counts[raw['kind']]+=1
        if raw['kind']=='gazebo' and gate<=raw['data']['time']<=gate+trial['sim_seconds']:
            people=[dict(id=int(k.split('_')[1]),**v) for k,v in raw['data']['poses'].items() if k!='mecanum_robot']
            if len(people)==3:
                pairs=[dict(ids=[h['id'],j['id']],gap=math.hypot(h['x']-j['x'],h['y']-j['y'])-.72) for i,h in enumerate(people) for j in people[i+1:]]
                dense.append(dict(elapsed=raw['data']['time']-gate,**min(pairs,key=lambda x:x['gap'])))
    assert dict(counts)==trial['diagnostic_counts']==row['counts']
    row['human_pair_overlap_beyond_1cm_s']=sum(v['elapsed']-u['elapsed'] for u,v in zip(dense,dense[1:]) if v['gap']<-.01)
    assert abs(row['min_human_pair_m']-min(x['gap'] for x in dense))<1e-8
    row['human_pair_min_event']=min(dense,key=lambda x:x['gap'])
    launch_log=(directory/(trial['name']+'.log')).read_text()
    row['beam_skip_temp_allocation_log_count']=launch_log.count('Reallocing temp weights')
    row['beam_skip_fallback_log_count']=launch_log.count('integrating all observations')
    assert all(x not in (runtime/'worker.log').read_text() for x in ['Traceback','ConnectionResetError','BrokenPipeError'])
    for key,value in PROFILES[item['profile']].items():assert trial['amcl_parameters_readback'][key]==value
    assert trial['process_cleanup']['status']=='PASS' and not trial['process_cleanup']['remaining']
    assert any('NVIDIA' in x for x in trial['process_cleanup']['ogre_renderer_lines'])
    assert set(row['plan_publishers_at_snapshots'])==set(row['amcl_publishers_at_snapshots'])=={1}
    assert max(errors)<.15,'Circular cluster correspondence requires investigation'
    checks.append(dict(seed=item['seed'],profile=item['profile'],policy=item['policy'],clouds=len(circular),matches=len(errors),cluster_match_error_max_m=max(errors),counts=dict(counts),config_verified=True,parameters_readback_verified=True,gpu_verified=True,shutdown_verified=True,cleanup_verified=True))
    manifest.append(dict(seed=item['seed'],profile=item['profile'],policy=item['policy'],trial_sha256=sha(trial_path),runtime=trial['runtime'],raw_sha256={f:sha(runtime/f) for f in ['diagnostics.jsonl','start_gate','scene.json','warehouse.sdf','nav2_scenario.yaml','crowdsim_metadata.json','ogre2_session.log','worker.log']},launch_log_sha256=sha(directory/(trial['name']+'.log'))))
    rows.append(row)
baseline_readback=next(x['values'] for x in readbacks if x['profile']=='baseline')
for case in readbacks:
    for key,value in baseline_readback.items():
        if key not in ['laser_model_type','do_beamskip']:
            assert case['values'][key]==value,(case['profile'],key,value,case['values'][key])
(report/'amcl_readbacks.json').write_text(json.dumps(readbacks,indent=2))
summary=dict(trials=rows,scope=protocol['comparison_scope'],validation=checks)
(report/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False));(report/'manifest.json').write_text(json.dumps(dict(trials=manifest,analysis_source_sha256=sha(Path(__file__)),analysis_helpers_sha256={str(path.relative_to(REPO)):sha(path) for path in [BENCH/'stability/recluster.py',BENCH/'footprint_geometry.py',BENCH/'localization_diagnosis/geometry.py',BENCH/'navigation_diagnostics/native_clusters.cpp']},cluster_method='Independent circular-mean yaw lift, then installed libpf clustering; quaternion streams cannot recover hidden whole-turn state or PF converged flag.'),indent=2))
(report/'checks.json').write_text(json.dumps(dict(status='PASS',trials=len(rows),particle_clouds=sum(x['clouds'] for x in checks),matched_clouds=sum(x['matches'] for x in checks),max_cluster_match_error_m=max(x['cluster_match_error_max_m'] for x in checks),cases=checks),indent=2))
with (report/'results.csv').open('w',newline='') as f:
    keys=[k for k,v in rows[0].items() if not isinstance(v,(dict,list))];writer=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore',lineterminator='\n');writer.writeheader();writer.writerows(rows)
# Keep individual paired points visible rather than imply a population estimate.
from matplotlib.lines import Line2D
fig,axes=plt.subplots(1,2,figsize=(12,4.6),constrained_layout=True)
colors={'sfm':'#2878b5','orca_plus':'#d45d26'};markers={42:'o',43:'s',44:'^'}
for seed in protocol['seeds']:
    for policy in colors:
        pair=[next(x for x in rows if x['seed']==seed and x['policy']==policy and x['profile']==profile) for profile in PROFILES]
        offset=(seed-43)*.06+(-.13 if policy=='sfm' else .13);xs=[i+offset for i in range(2)]
        for ax,key in zip(axes,['amcl_error_max_m','goal_error_world_m']):
            ax.plot(xs,[r[key] for r in pair],color=colors[policy],alpha=.45,lw=1)
            ax.scatter(xs,[r[key] for r in pair],color=colors[policy],marker=markers[seed],s=65)
for ax in axes:ax.set_xticks(range(2),['baseline','LF-prob + beam skip']);ax.grid(axis='y',alpha=.2)
axes[0].set_ylabel('Maximum published AMCL XY error [m]');axes[1].set_ylabel('Final actual distance to goal [m]');axes[1].axhline(.20,color='#555',ls='--',lw=1)
handles=[Line2D([0],[0],color=color,marker='o',lw=1,label=policy) for policy,color in colors.items()]+[Line2D([0],[0],color='#555',marker=marker,lw=0,label=f'seed {seed}') for seed,marker in markers.items()]
axes[0].legend(handles=handles,fontsize=8);fig.suptitle('12 paired diagnostic trials: 3 seeds, 2 policies, 2 AMCL profiles')
fig.savefig(report/'paired_comparison.png',dpi=160);plt.close(fig)
# Separate reported action success, actual XY accuracy, and geometric overlap.
aggregate=[]
for profile in PROFILES:
    for policy in colors:
        selected=[r for r in rows if r['profile']==profile and r['policy']==policy];assert len(selected)==3
        aggregate.append(dict(profile=profile,policy=policy,trials=3,reported_success=sum(r['status']=='SUCCEEDED' for r in selected),actual_success=sum(r['status']=='SUCCEEDED' and r['actual_goal_xy_within_tolerance'] for r in selected),false_success=sum(r['reported_success_outside_actual_xy_tolerance'] for r in selected),amcl_error_max_m=max(r['amcl_error_max_m'] for r in selected),min_robot_human_m=min(r['min_robot_human_footprint_m'] for r in selected),min_human_pair_m=min(r['min_human_pair_m'] for r in selected),human_pair_overlap_beyond_1cm_s=sum(r['human_pair_overlap_beyond_1cm_s'] for r in selected)))
(report/'aggregate.json').write_text(json.dumps(aggregate,indent=2,allow_nan=False))
lines=['# AMCL nhiều seed: 12 lượt so sánh ghép cặp','',
    f"HEAD lúc chạy `{protocol['head']}`; mã runner, mở rộng seed và analyzer mới chưa commit ở thời điểm đo. `protocol.json` giữ source SHA256 thực tế. Các commit trước được lưu trong `commit_checkpoint.json`.",'',
    'Chạy đúng 12 lượt: seed42/43/44 × AMCL gốc hoặc LF-prob+beam-skip × người SFM hoặc ORCA+. Mỗi ô một lượt, tối đa75 sim s, nghỉ15wall s, headless, GPU NVIDIA và shutdown WSL D3D12. Giữ mọi ABORTED/TIMEOUT, không retry để thay kết quả.','',
    '| AMCL | Người | Nav2 báo thành công | Thành công và XY thật ≤0.20m | Báo thành công nhưng ngoài ngưỡng |',
    '|---|---|---:|---:|---:|']
for r in aggregate:lines.append(f"| {r['profile']} | {r['policy']} | {r['reported_success']}/3 | {r['actual_success']}/3 | {r['false_success']} |")
lines+=['','Đây là số đếm của ba cảnh nhỏ, không phải ước lượng tỷ lệ thành công tổng quát. Hai profile thay cả laser model và beam skipping, nên chưa tách riêng tác dụng bỏ tia. Không chọn cấu hình mới làm mặc định chỉ từ cohort này.','',
    '| Seed | AMCL | Người | Nav2 báo | Sim s | Cách goal thật (m) | AMCL error max (m) | Khe robot–người (m) | Khe người–người (m) | Overlap người >1cm (s) |',
    '|---:|---|---|---|---:|---:|---:|---:|---:|---:|']
for r in rows:lines.append(f"| {r['seed']} | {r['profile']} | {r['policy']} | {r['status']} | {r['sim_seconds']:.3f} | {r['goal_error_world_m']:.3f} | {r['amcl_error_max_m']:.3f} | {r['min_robot_human_footprint_m']:.4f} | {r['min_human_pair_m']:.4f} | {r['human_pair_overlap_beyond_1cm_s']:.3f} |")
false_success=[r for r in rows if r['reported_success_outside_actual_xy_tolerance']]
if false_success:lines+=['','**Nav2 thành công nhưng chưa tới đích thật:** '+ '; '.join(f"seed{r['seed']}/{r['profile']}/{r['policy']}: cách {r['goal_error_world_m']:.3f}m" for r in false_success)+'. Các lượt này không được tính thành công thực tế.']
lines+=['','![So sánh từng cặp](paired_comparison.png)','',
    'Mỗi đường nối cùng seed và policy giữa hai cấu hình. Màu là policy, marker là seed; không phải quỹ đạo hoặc đường trung bình. Các ảnh/GIF trong thư mục seed/profile được thu từ camera Gazebo của từng lượt, có sampling/nén; biểu đồ này được vẽ từ dữ liệu đo.','',
    '## Bảo toàn điều kiện thử','',
    '- Seed42/43 giữ nguyên scene byte-for-byte so với fixture cũ. Seed44 sinh mới bằng cùng thuật toán bottleneck và biên jitter; template chỉ lấy thông số cố định radius/speed/robot từ seed42, không giả là đã chạy SocNavGym44. Kiểm tra initial overlap và khoảng trống route trong `fixture_checks.json`.','- Đảo thứ tự profile và policy ở seed43; seed42/44 chạy thứ tự ngược lại. Có đối chứng baseline mới cùng seed và policy. AMCL/MPPI/LiDAR/physics chưa được seed đồng bộ, nên vẫn có nhiễu và không coi là replay xác định.','- Native policy, robot controller, cảm biến và map giữ nguyên. Chỉ initial pose và profile AMCL được ghi vào YAML runtime; đọc lại service AMCL trước goal, so các tham số khác giữa mọi lượt.','- Metric XY thật lấy pose Gazebo; giá trị0.20m là ngưỡng đánh giá vị trí cuối, không kiểm chứng đầy đủ yaw hoặc mọi điều kiện stateful goal checker.','- Khoảng hở signed dùng footprint0.24×0.20m, người r0.36m, AABB máy. Giá trị âm là overlap dù dưới1cm. Không phải contact force hay bảo đảm giữa hai sample.','',
    '## Kiểm chứng và giới hạn','',
    f"Đối chiếu {sum(x['matches'] for x in checks)}/{sum(x['clouds'] for x in checks)} particle clouds; sai số XY giữa cụm native lớn nhất sau circular-mean lift và AMCL tối đa {max(x['cluster_match_error_max_m'] for x in checks):.6g}m. Đây là tái phân cụm cloud ghi được, không replay hidden filter hoặc phục hồi whole-turn state.",'',
    'Beam skipping chỉ chạy khi PF converged và có thể fallback dùng mọi tia. Các counts log bên dưới tính toàn phiên, gồm warmup; không đo mask từng scan. Không sửa thư viện AMCL native.','',
    '| Seed | Người | Profile | Cấp temp | Fallback mọi tia |',
    '|---:|---|---|---:|---:|']
for r in rows:lines.append(f"| {r['seed']} | {r['policy']} | {r['profile']} | {r['beam_skip_temp_allocation_log_count']} | {r['beam_skip_fallback_log_count']} |")
lines+=['','`protocol.json`, `progress.json`, `summary.json`, `aggregate.json`, `results.csv`, `amcl_readbacks.json`, `checks.json`, `manifest.json`, `validation.json` giữ nguồn, mọi outcome và hash raw. Log/diagnostics lớn nằm trong runtime/generated và bị gitignore. Các `*_circular_clusters.json` là bản cluster dùng trong báo cáo; plot cluster phụ do analyzer cũ tạo dùng principal yaw nên không dùng để kết luận chuyển mode quanh±pi.','',
    'Cohort chỉ xác định yếu tố định vị trong bottleneck ba người độc lập. Chưa đánh giá group/conversation, chưa nối ComplexityNav, chưa có World Model hoặc thay kiến trúc robot.','']
(report/'MULTISEED_REPORT.md').write_text('\n'.join(lines))
print(json.dumps(dict(status='PASS',trials=12,aggregate=aggregate,checks=json.loads((report/'checks.json').read_text())),indent=2))
