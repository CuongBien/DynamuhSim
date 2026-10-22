#!/usr/bin/env python3
# Measured physical-task comparison; not an algorithm ranking.
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
import gzip,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent;REPO=SOCIAL.parent.parent;OUT=HERE/'reports';OLD=BENCH/'socnavgym_nav2_pilot'
load=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();plan=load(HERE/'protocol.json');summary=[]
for label,folder,name in [('SocNavGym v2',OLD,'people01'),('SocNavGym v2',OLD,'people02')]+[(r['model'],HERE,r['name']) for r in plan['plan']]:
    v=load(folder/'reports'/name/'validation.json');r=load(folder/'reports'/name/'result.json');assert v['status']=='AUDITED_NAV2_PILOT'
    summary.append(dict(name=name,framework=label,origin=str((folder/'reports'/name).relative_to(REPO)),**v))
    if folder==HERE:
        images=load(BENCH/'common_response/inputs/images.json');expected=images['dynamuhsim-crowdsimplus:round2' if r['model']=='crowd_orca_plus' else 'dynamuhsim-complexitynav:round3'];assert r['container_image']==expected
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
def canonical_world(raw):
    tree=ET.fromstring(raw);plugin=tree.find("world/plugin[@name='response_experiment::Adapter']");plugin.find('socket').text='SESSION_SOCKET';plugin.find('planning_dt').text='NATIVE_DT';return ET.tostring(tree)
expected=canonical_world(gzip.decompress((OLD/'reports/people01/warehouse.sdf.gz').read_bytes()));checks=[];prior=load(OLD/'reports/people01/result.json')
for row in summary:
    folder=REPO/row['origin'];r=load(folder/'result.json');assert canonical_world(gzip.decompress((folder/'warehouse.sdf.gz').read_bytes()))==expected
    assert r['runtime_nav_params_sha256']==prior['runtime_nav_params_sha256']
    for name in ['experiments/social_navigation/benchmark/common_response/inputs/crossing_near.json','experiments/social_navigation/warehouse/nav2_warehouse.yaml']:assert r['source_sha256'][name]==prior['source_sha256'][name]
    checks.append(dict(name=row['name'],world_equal_except_socket_and_native_dt=True,nav2_yaml_byte_identical=True))
installed=[]
for name in ['nav2_corridor.launch.py','corridor_tb3.launch.py','publish_episode_initial_pose.py']:
    src=REPO/'src/custom_corridor/launch'/name;dst=REPO/'install/custom_corridor/share/custom_corridor/launch'/name;assert sha(src)==sha(dst);installed.append(dict(name=name,sha256=sha(src),installed_equals_source=True))
(OUT/'physical_task_audit.json').write_text(json.dumps(dict(worlds=checks,installed_launch=installed,excluded_world_fields=['socket path','planning_dt .1/.25; native difference']),indent=2)+'\n')
fig,axes=plt.subplots(3,2,figsize=(12,10));colors=['#1565c0','#e65100'];labels=['SocNavGym v2','CrowdSimPlus ORCA+','ComplexityNav geometry']
for group in range(3):
    ax,sp=axes[group];rows=summary[group*2:group*2+2]
    for j,row in enumerate(rows):
        d=REPO/row['origin'];result=load(d/'result.json');poses=[json.loads(line) for line in gzip.decompress((d/'gazebo_poses.jsonl.gz').read_bytes()).decode().splitlines()];poses=[r for r in poses if result['goal_start_sim']<=r['time']<=result['action_end_sim'] and 'mecanum_robot' in r['poses']];t=np.array([r['time'] for r in poses]);xy=np.array([[r['poses']['mecanum_robot'][k] for k in ['x','y']] for r in poses]);ax.plot(xy[:,0],xy[:,1],color=colors[j],label=row['name']+(' reached' if row['actual_goal_reached'] else ' actual goal failed'));mask=np.diff(t)>1e-10;sp.plot((t[1:]-t[0])[mask],np.linalg.norm(np.diff(xy,axis=0),axis=1)[mask]/np.diff(t)[mask],color=colors[j],alpha=.8,label=row['name'])
    for h in load(BENCH/'common_response/inputs/crossing_near.json')['humans']:ax.add_patch(plt.Circle((h['x'],h['y']),.36,color='gray',alpha=.35))
    ax.scatter([-28,-20],[0,0],marker='x',c=['green','red'],s=60);ax.set(xlim=(-29,-18.5),ylim=(-3,3),aspect='equal',title=labels[group]+' - actual robot paths',xlabel='World x (m)',ylabel='World y (m)');ax.legend(fontsize=8);sp.set(title='Measured robot speed',xlabel='Simulation seconds after accepted goal',ylabel='m/s');sp.legend(fontsize=8);sp.grid(alpha=.2)
fig.suptitle('Same physical task; native dt/speed/observation differ. Small pilot, no ranking.',fontsize=12);fig.tight_layout();fig.savefig(OUT/'actual_paths_and_speed.png',dpi=140);plt.close(fig)
new=summary[2:];table='| Framework / lượt | Action | Đích thật | XY (cm) | Sim / wall (s) | Min robot–người (cm) |\n|---|---|---|---:|---:|---:|\n'
for row in summary:table+=f"|{row['framework']} / {row['name']}|{row['navigation_status']}|{'Đạt' if row['actual_goal_reached'] else 'Không đạt'}|{row['goal_xy_world_m']*100:.2f}|{row['active_seconds']:.2f} / {row['wall_active_seconds']:.1f}|{row['actual_swept_human_gaps_m']['robot']*100:.2f}|\n"
details=''
for row in new:
    g=row['actual_swept_human_gaps_m'];c=row['local_costmap_human_surface_evidence'];details+=f"- **{row['name']}**: min gap người–người {g['pair']*100:.2f} cm, người–tĩnh {g['static']*100:.2f} cm; sampled rotated footprint–người {row['actual_sampled_rotated_footprint_human_gap_m']*100:.2f} cm. Path {row['robot_path_m']:.3f} m, y range {row['robot_y_range_m'][0]:.3f}..{row['robot_y_range_m'][1]:.3f} m; slow-near {row['slow_while_near_human_s']:.3f} sim s. LiDAR {row['lidar_human_beams']} eligible beams, {row['lidar_fraction_within15cm']*100:.2f}% khớp trong15cm, IDs {row['lidar_matched_ids']}; occupied-ring {c['observations_with_occupied_ring']}/{c['eligible_human_observations']}. AMCL XY max {row['amcl_xy_error_m']['maximum']:.3f}m; RTF {row['observed_rtf']:.3f}.\n"
report=f'''# Pilot Nav2 cùng nhiệm vụ cho ba framework

Ngày22/10/2026, xuất phát từ commit `bb45f1b`. Đã chạy tuần tự4 lượt mới:2 CrowdSimPlus ORCA+,2 ComplexityNav geometry. Hai lượt SocNavGym v2 và empty control dùng dữ liệu đã commit, không chạy lại để chọn kết quả. Đây là pilot tích hợp với robot tự điều hướng trong Gazebo, không phải thống kê hoặc xếp hạng simulator.

{table}

Actual goal yêu cầu SUCCEEDED và world XY≤0.20m/yaw≤0.25rad tại pose ghi sau settle0.4wall s. **Crowd02 báo SUCCEEDED nhưng XY20.59cm: giữ không đạt**, không nâng ngưỡng hoặc chạy thay. Empty control cũ đạt trong15.72sim s. Goal outcome độc lập với clearance; không nghiệm thu đứng yên lâu ởđích.

## Cùng nhiệm vụ và khác biệt native

Nhà xưởng60×40m/42 boxes,6 người r0.36m, robot nominal disc r0.25m, start(−28,0,0),goal(−20,0,0),budget45sim s, fixture crossing_near/seed42. Map, Mecanum/LiDAR/IMU, MPPI/AMCL giữ nguyên. Runtime YAML chỉ đổi AMCL initial x từ−27 sang−28 như Soc cohort. Audit xác nhận YAML byte-identical giữa6 people runs; world XML giống nhau sau khi chỉ chuẩn hóa socket path/native planning dt. Installed launch khớp source. Physics0.0025s. /cmd_vel chỉ có controller_server/behavior_server theo graph kiểm, không robot velocity script.

Người đứng yên trong warmup, bắt đầu native sau accepted goal. Gate mỗi backend:10 transport ticks native_step0,20 bước sau gate khớp exact offline đã commit, clock lỗi terminal. Crowd dùng native ORCA+ act/constrain/step,dt0.1s,v_pref0.8m/s và bookkeeping6 người đã kiểm. CN dùng3ORCA+3SFM native intention **cộng roadmap/ORCA geometry projection**,dt0.25s,native intended speed chủ yếu0.5m/s,projection ceiling0.8m/s; không phải native geometry hoặc trained RGL. Không áp sửa motion Soc v2 chohai backend này; chỉ tái dùng transport/interpolation/terminal Pause plugin.

Native robot observation/intention assumptions vẫn khác: CSP observable position/velocity/radius; CN/Soc native inferred intention riêng, final geometry dùng velocity đo. Soc v2 có projection riêng. Vì vậy same physical task không phải equal internal observation/speed hoặc ablation thuật toán. Native waypoint/route lifecycle sau các lần tớiđích vẫn giữ nguyên.

![Pose Gazebo và tốc độ robot](actual_paths_and_speed.png)

Vòng tròn xám là người ban đầu; plot đường robot không vẽ toàn bộ human traces. Raw giữ đủ pose người.

## Kết quả đo mới

{details}

{sum(r['geometry_outcome']=='NO_OVERLAP_GT_5MM' for r in new)}/4 lượt mới không ghi overlap>5mm theo swept discs/AABBs giữa các pose samples. Nominal robot disc0.25m; rotated rectangle0.24×0.20m chỉ đo tại samples. Khoảng hở vài cm không chứng nhận social comfort. LiDAR chỉ đối chiếu tia đủ điều kiện trước static boxes, không phải recall6 người. Occupied-ring mô tả costmap cells≥65 gần bề mặt người, không chứng minh nguồn từng cell. Slow-near/path cong không tự chứng minh nhường do xã hội.

## Runtime và provenance

Ubuntu24.04/Jazzy/Python3.12.3 chạy robot/recorder; native CSP/CN Python3.8.13 trong Docker content-ID ghim,network none,2CPU/4GB/one BLAS thread. Không đổi Python hệ thống. NVIDIA D3D12 được kiểm preflight/Ogre ở4 lượt; shutdown sạch, không còn process/container thuộc phiên. Native/ORCA/Nav2/physics vẫn cần CPU. GUI/RTF/scheduling khác nhau; AMCL/MPPI/physics chưa joint-seeded. Không suy success rate hoặc xếp hạng runtime từ2 lượt/model.

Analyzer đã dừng tại crowd02 khi so pose cuối trong cleanup với pose kết luận trước cleanup. Pose tail lệch rất nhỏ nhưng vẫn ngoài20cm. Đã sửa **analyzer** để tìm đúng final_world_pose trong actual active stream. Không chạy lại simulator, sửa worker/Nav2/tolerance hoặc goal outcome. Assertion/source ban đầu và audit_returncode1 giữ trong audit_attempts/source_snapshot/initial_execution.json; final_audit riêng. Raw streams và gzip byte-identical giữ cả shutdown tail. Source runtime/worker/protocol hashes ởresult; source snapshot ban đầu và final manifest công bố riêng. Khi đóng gói, cohort.py thêm guard chống ghi đè receipt trước mọi lần chạy lại; source lúc chạy vẫn lưu.

Camera RGB640×480,1frame mỗi sim-second; GIF phát4× simulation time, **thu từ Gazebo thật, không AI**, không screen recording realtime. GUI yêu cầu ởcrowd01/complex01; chưa có xác nhận kéo/zoom riêng cho cohort này. Camera cố định không đảm bảo thấy mọi người suốt lượt.

![CrowdSimPlus — camera thật](crowd01/camera_preview_4x.gif)

![ComplexityNav geometry — camera thật](complex01/camera_preview_4x.gif)

## Nghiệm thu và bước tiếp theo

Cả3 nhánh đã có demo độc lập, adapter Gazebo và pilot Nav2; cohort này đối chiếu cùng physical task. Chưa gọi toàn bộ social navigation hoàn tất:6 cá nhân giao cắt, chưa nhóm/formation+Nav2 hoặc dense crowd nhiều seed. Không sửa baseline robot/Nav2/vendor, không World Model/Isaac.

Theo mục tiêu nhóm đi/đứng tương tác/formation, **SocNavGym lifecycle variant** vẫn là ứng viên chính dựa vào pilot lifecycle Gazebo trước; cần nhóm+Nav2 riêng. **CrowdSimPlus ORCA+** giữ baseline tránh người độc lập, chưa membership/conversation. **ComplexityNav geometry** là hybrid nghiên cứu choobstacle/không gian chật; không RGL checkpoint hoặc group model. Không chọn framework chỉ từgoal/min gap pilot này.

Bước tiếp có giới hạn: pilot SocNavGym nhóm+Nav2 với nhóm đi cùng và nhóm đứng trò chuyện; đo nhóm giữ/tan, goal thật và clearance. Sau đó chốt bộ demo/báo cáo lựa chọn theo năng lực đã đo. Không mở thêm framework hoặc ma trận AMCL.

[Protocol](../protocol.json) · [Số đo](summary.json) · [Physical task audit](physical_task_audit.json) · [Gate](gate01/validation.json) · [Commands/returncodes](initial_execution.json). Từng folder có result/validation,raw gzip,PNG/GIF.
'''
(OUT/'SHARED_NAV2_REPORT.md').write_text(report)
media=[]
for trial in plan['plan']:
    d=OUT/trial['name'];r=load(d/'result.json')
    for q in d.glob('*.png'):
        with Image.open(q) as im:im.load();assert im.size==(640,480)
    with Image.open(d/'camera_preview_4x.gif') as im:
        duration=0
        for i in range(im.n_frames):im.seek(i);im.load();duration+=im.info.get('duration',0)
        assert duration==r['media']['gif_duration_ms'];media.append(dict(name=trial['name'],frames=im.n_frames,duration_ms=duration,source='Actual Gazebo RGB camera'))
(OUT/'media_audit.json').write_text(json.dumps(media,indent=2)+'\n')
print(json.dumps(dict(status='COMPARISON_WRITTEN',new_actual_goals=sum(r['actual_goal_reached'] for r in new),new_trials=4),indent=2))
