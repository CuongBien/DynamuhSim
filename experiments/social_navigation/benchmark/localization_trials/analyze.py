#!/usr/bin/env python3
"""Offline report for the fixed six-trial AMCL ablation; no reruns or promotion."""
import argparse,csv,hashlib,json,math,subprocess,sys
from collections import Counter
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
sys.path.insert(0,str(BENCH));sys.path.insert(0,str(BENCH/'stability'))
from recluster import recluster
from profiles import PROFILES,configure
import yaml
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();report=a.report_dir.resolve()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protocol=json.loads((report/'protocol.json').read_text());progress=json.loads((report/'progress.json').read_text())
assert progress['completed']==progress['planned']==6 and progress['stopped'] is None
assert protocol['profiles']==PROFILES and len(protocol['plan'])==6
for source,digest in protocol['source_sha256'].items():assert sha(REPO/source)==digest,source
assert all(not x['infrastructure_errors'] and not x['remaining_nodes'] for x in progress['attempts'])
for profile in PROFILES:
    directory=report/profile
    with (directory/'offline_analysis.log').open('w') as log:
        subprocess.run([sys.executable,str(BENCH/'navigation_diagnostics/analyze.py'),'--localization-ablation','--report-dir',str(directory)],stdout=log,stderr=subprocess.STDOUT,check=True,cwd=REPO)
rows=[];manifest=[];checks=[];readbacks=[]
for item in progress['attempts']:
    directory=report/item['profile'];trial_path=report/item['report'];trial=json.loads(trial_path.read_text());runtime=BENCH/trial['runtime']
    assert trial['commit']==protocol['head'] and trial['protocol_valid'] and trial['gazebo_exit']['clean']
    assert trial['localization_profile']==item['profile'] and trial['human_policy']==item['policy']
    expected=configure(yaml.safe_load((BENCH.parent/'warehouse/nav2_warehouse.yaml').read_text()),item['profile'],trial['robot_start'])
    assert yaml.safe_load((runtime/'nav2_scenario.yaml').read_text())==expected
    assert trial['localization_parameter_overrides']==PROFILES[item['profile']]
    readbacks.append(dict(profile=item['profile'],policy=item['policy'],values=trial['amcl_parameters_readback']))
    row=next(x for x in json.loads((directory/'summary.json').read_text()) if x['policy']==item['policy'])
    row.update(profile=item['profile'],sequence=item['sequence'],report=item['report'],gazebo_clean=True)
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
    checks.append(dict(profile=item['profile'],policy=item['policy'],clouds=len(circular),matches=len(errors),cluster_match_error_max_m=max(errors),counts=dict(counts),config_verified=True,parameters_readback_verified=True,gpu_verified=True,shutdown_verified=True,cleanup_verified=True))
    manifest.append(dict(profile=item['profile'],policy=item['policy'],trial_sha256=sha(trial_path),runtime=trial['runtime'],raw_sha256={f:sha(runtime/f) for f in ['diagnostics.jsonl','start_gate','scene.json','nav2_scenario.yaml','crowdsim_metadata.json','ogre2_session.log','worker.log']},launch_log_sha256=sha(directory/(trial['name']+'.log'))))
    rows.append(row)
baseline_readback=next(x['values'] for x in readbacks if x['profile']=='baseline')
for case in readbacks:
    for key,value in baseline_readback.items():
        if key not in ['laser_model_type','do_beamskip']:
            assert case['values'][key]==value,(case['profile'],key,value,case['values'][key])
(report/'amcl_readbacks.json').write_text(json.dumps(readbacks,indent=2))
summary=dict(trials=rows,scope=protocol['comparison_scope'],validation=checks)
(report/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False));(report/'manifest.json').write_text(json.dumps(dict(trials=manifest,analysis_source_sha256=sha(Path(__file__)),cluster_method='Independent circular-mean yaw lift, then installed libpf clustering; quaternion streams cannot recover hidden whole-turn state or PF converged flag.'),indent=2))
(report/'checks.json').write_text(json.dumps(dict(status='PASS',trials=len(rows),particle_clouds=sum(x['clouds'] for x in checks),matched_clouds=sum(x['matches'] for x in checks),max_cluster_match_error_m=max(x['cluster_match_error_max_m'] for x in checks),cases=checks),indent=2))
with (report/'results.csv').open('w',newline='') as f:
    keys=[k for k,v in rows[0].items() if not isinstance(v,(dict,list))];writer=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore',lineterminator='\n');writer.writeheader();writer.writerows(rows)
fig,axes=plt.subplots(1,2,figsize=(12,4.4),constrained_layout=True)
colors={'sfm':'#2878b5','orca_plus':'#d45d26'}
for i,profile in enumerate(PROFILES):
    for policy,offset in [('sfm',-.15),('orca_plus',.15)]:
        r=next(x for x in rows if x['profile']==profile and x['policy']==policy)
        axes[0].bar(i+offset,r['amcl_error_max_m'],width=.28,color=colors[policy],label=policy if i==0 else None)
        axes[1].scatter(i+offset,r['goal_error_world_m'],color=colors[policy],marker='o' if r['status']=='SUCCEEDED' and r['actual_goal_xy_within_tolerance'] else 'X',s=90)
        axes[1].annotate(r['status'],(i+offset,r['goal_error_world_m']),xytext=(0,7),textcoords='offset points',ha='center',fontsize=7)
for ax in axes:ax.set_xticks(range(3),list(PROFILES));ax.grid(axis='y',alpha=.2)
axes[0].set_ylabel('Maximum published AMCL XY error [m]');axes[0].legend();axes[1].set_ylabel('Final actual distance to goal [m]');fig.suptitle('AMCL pilot: one run per policy/profile, seed42 (not a success-rate estimate)');fig.savefig(report/'pilot_comparison.png',dpi=160);plt.close(fig)
lines=['# AMCL localization pilot — 6 lượt, seed42','',
f"Provenance: HEAD `{protocol['head']}` cùng mã thử nghiệm chưa commit ở thời điểm chạy; `protocol.json` giữ source hash thực tế. Không suy luận mọi thay đổi đã nằm trong HEAD này.",'',
'Giữ nguyên robot Mecanum, MPPI, LiDAR 3.5 m, IMU, bản đồ, ba người và native policy. Mỗi profile chạy một lần với mỗi policy; không chạy lại để thay kết quả. Tất cả dùng shutdown WSL D3D12 đã kiểm tra riêng. Đây là pilot nhỏ, chưa đủ kết luận cải thiện ổn định hoặc chọn cấu hình mặc định.','',
'| Profile | Người | Nav2 báo | Thời gian sim (s) | Sai số AMCL lớn nhất (m) | Cách goal thật (m) | Khe người–người nhỏ nhất (m) | Chồng lấn >1 cm (s) |',
'|---|---|---|---:|---:|---:|---:|---:|']
for r in rows:lines.append(f"| {r['profile']} | {r['policy']} | {r['status']} | {r['sim_seconds']:.3f} | {r['amcl_error_max_m']:.3f} | {r['goal_error_world_m']:.3f} | {r['min_human_pair_m']:.4f} | {r['human_pair_overlap_beyond_1cm_s']:.3f} |")
false_success=[r for r in rows if r['reported_success_outside_actual_xy_tolerance']]
if false_success:
    lines+=['','**Nav2 báo thành công nhưng vị trí thật chưa đạt ngưỡng XY 0.20 m:** '+ '; '.join(f"{r['profile']}/{r['policy']}: còn {r['goal_error_world_m']:.3f} m, AMCL error max {r['amcl_error_max_m']:.3f} m" for r in false_success)+'. Không tính các lượt này là tới đích chính xác.']
lines+=['','![Pilot comparison](pilot_comparison.png)','','Các tham số chỉ ghi vào YAML tạm của từng lượt; đối chiếu cấu trúc xác nhận mọi phần ngoài initial pose và profile AMCL giống cấu hình gốc. Đọc lại `/amcl/get_parameters` trước goal xác nhận tham số đang chạy, gồm laser model, beam skipping và initial pose. `transform_tolerance` thực tế vẫn theo override launch hiện có.','',
'- `baseline`: `likelihood_field` gốc.','- `lf_prob`: `likelihood_field_prob`, `do_beamskip=false`. Thay cả cách tính likelihood, nên phải so riêng với beam skipping.','- `lf_prob_beamskip`: cùng model prob, `do_beamskip=true`, distance=0.5 m, threshold=0.3, error_threshold=0.9.','',
'Native Nav2 1.3.13 chỉ bỏ tia khi cờ PF converged bật; quá nhiều tia bị bỏ sẽ quay về dùng toàn bộ. Readback không chứng minh mọi frame đều bỏ tia. Số dòng log cấp bộ nhớ/fallback chỉ là dấu hiệu chạy nhánh, không đo được mask mỗi scan. Không sửa thư viện AMCL để lấy trạng thái ẩn. [Mã nguồn đúng phiên bản](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_amcl/src/sensors/laser/likelihood_field_model_prob.cpp).','',
'| Profile | Người | Log cấp temp | Log dùng lại mọi tia | Khe robot–người (m) | Khe robot–máy (m) | Gazebo thoát |',
'|---|---|---:|---:|---:|---:|---|']
for r in rows:lines.append(f"| {r['profile']} | {r['policy']} | {r['beam_skip_temp_allocation_log_count']} | {r['beam_skip_fallback_log_count']} | {r['min_robot_human_footprint_m']:.4f} | {r['min_robot_machine_footprint_m']:.4f} | sạch |")
lines+=['',
'Khe hở được đo từ pose Gazebo dày theo footprint robot 0.24×0.20 m, người tròn r=0.36 m và AABB máy. Đây là đo hình học tại mẫu quan sát, không phải contact force hay bảo đảm không va chạm giữa hai mẫu. Dấu âm luôn được giữ, kể cả chồng lấn dưới ngưỡng báo thời gian 1 cm.','',
'Native particle cloud được phân cụm bằng libpf đang cài, sau circular-mean yaw lift độc lập để tránh seam ±pi. `*_circular_clusters.json` là kết quả dùng trong báo cáo này; các plot cluster trong thư mục profile do analyzer cũ tạo vẫn dùng principal yaw, chỉ dùng làm dữ liệu phụ, không suy luận chuyển mode từ chúng. `checks.json` giữ số cloud và sai số khớp với AMCL.','',
'`protocol.json` khóa thứ tự, source hash, cooldown 15 s và ngân sách 75 sim s. `manifest.json` giữ hash dữ liệu raw; `summary.json`, `results.csv` giữ đủ metric, gồm reroute, odometry, occupied pose, traversal và khoảng trống. Log/diagnostics lớn nằm trong generated và bị gitignore; hash không thay thế dữ liệu gốc.','',
'Không gộp pilot này với cohort stability cũ: các tiến trình AMCL/MPPI/LiDAR/physics không được seed đồng bộ và thứ tự profile chưa random hóa. Giữ cả ABORTED/TIMEOUT. Chưa áp dụng profile vào launch robot gốc; cần lặp thêm theo nhiều seed trước khi chọn.','']
(report/'LOCALIZATION_PILOT.md').write_text('\n'.join(lines))
print(json.dumps(dict(status='PASS',trials=6,results=[{k:r[k] for k in ['profile','policy','status','amcl_error_max_m','goal_error_world_m','min_human_pair_m']} for r in rows]),indent=2))
