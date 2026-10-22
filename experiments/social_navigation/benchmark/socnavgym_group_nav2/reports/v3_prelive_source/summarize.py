#!/usr/bin/env python3
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
import gzip,hashlib,json,math,xml.etree.ElementTree as ET
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;OUT=HERE/'reports';load=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();plan=load(HERE/'protocol.json');summary=[]
for trial in plan['plan']:
    d=OUT/trial['name'];v=load(d/'validation.json');assert v['status']=='AUDITED_NAV2_PILOT';summary.append(dict(name=trial['name'],**v))
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
# Robot and group geometry audited separately from the native group labels.
fig,axes=plt.subplots(2,2,figsize=(13,9));boxes=load(BENCH.parent/'warehouse/layout.json')['obstacles'];table='| Lượt / seed | Nav2 / đích thật | XY (cm) | Sim / wall (s) | Min robot–người (cm) | Nhóm đi ≥2s/0.5m | Nhóm đứng ≥2s |\n|---|---|---:|---:|---:|---|---|\n';details='';media=[]
for ax,row in zip(axes.flat,summary):
    name=row['name'];d=OUT/name;result=load(d/'result.json');poses=[json.loads(line) for line in gzip.decompress((d/'gazebo_poses.jsonl.gz').read_bytes()).decode().splitlines()];poses=[p for p in poses if result['goal_start_sim']<=p['time']<=row['goal_sample_sim'] and all(n in p['poses'] for n in ['mecanum_robot']+[f'common_{i}_proxy' for i in range(1,7)])];xy=np.array([[p['poses']['mecanum_robot'][k] for k in ['x','y']] for p in poses]);ax.plot(xy[:,0],xy[:,1],c='#1565c0',linewidth=2,label='Robot / Nav2')
    for hid in range(1,7):
        hxy=np.array([[p['poses'][f'common_{hid}_proxy'][k] for k in ['x','y']] for p in poses]);ax.plot(hxy[:,0],hxy[:,1],c='gray',alpha=.5,linewidth=.8);ax.add_patch(plt.Circle(hxy[0],.36,color='gray',alpha=.3))
    for b in boxes:
        if -30<b['x']< -18 and abs(b['y'])<6:ax.add_patch(plt.Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],color='#999999',alpha=.35))
    seen=set();segments=row['groups']['segments']
    for segment in segments:
        pp=[p for p in poses if segment['start']<=p['time']<=segment['end']]
        if len(pp)<2:continue
        cc=np.array([[np.mean([p['poses'][f'common_{i}_proxy'][k] for i in segment['members']]) for k in ['x','y']] for p in pp]);kind=segment['type'];ax.plot(cc[:,0],cc[:,1],c='#e65100' if kind=='moving' else '#2e7d32',linewidth=2,label=(kind+' centroid') if kind not in seen else None);seen.add(kind)
    ax.scatter([-28,-20],[0,0],c=['green','red'],marker='x',s=60);ax.set(xlim=(-29,-18.5),ylim=(-4,4),aspect='equal',title=f"{name}, seed{row['seed']}: {row['navigation_status']}, actual {'reached' if row['actual_goal_reached'] else 'failed'}",xlabel='World x (m)',ylabel='World y (m)');ax.legend(fontsize=8,loc='upper right')
    table+=f"|{name} / {row['seed']}|{row['navigation_status']} / {'Đạt' if row['actual_goal_reached'] else 'Không đạt'}|{row['goal_xy_world_m']*100:.2f}|{row['active_seconds']:.2f} / {row['wall_active_seconds']:.1f}|{row['actual_swept_human_gaps_m']['robot']*100:.2f}|{'Có' if row['groups']['moving_travel_covered'] else 'Chưa'}|{'Có' if row['groups']['standing_duration_covered'] else 'Chưa'}|\n"
    longest_move=max((s['actual_centroid_path_m'] for s in segments if s['type']=='moving'),default=0.);longest_stand=max((s['duration_s'] for s in segments if s['type']=='standing'),default=0.);diam=max((s['max_pair_diameter_m'] for s in segments),default=0.)
    details+=f"- **{name}**: đoạn nhóm đi dài nhất {longest_move:.3f}m; đoạn nhóm đứng dài nhất {longest_stand:.3f}s; đường kính cặp thành viên lớn nhất {diam:.3f}m. Có {len(row['groups']['membership_or_type_changes'])} mốc đổi membership/loại nhóm; group objects mới {row['groups']['new_group_objects']}. LiDAR {row['lidar_human_beams']} tia đủ điều kiện, {row['lidar_fraction_within15cm']*100:.2f}% khớp trong15cm, IDs{row['lidar_matched_ids']}; AMCL XY max {row['amcl_xy_error_m']['maximum']:.3f}m, RTF {row['observed_rtf']:.3f}.\n"
    times=result['media']['frame_times'];target=result['goal_start_sim']+4;frame=min(range(len(times)),key=lambda i:abs(times[i]-target))
    with Image.open(d/'camera_preview_4x.gif') as im:
        duration=0
        for i in range(im.n_frames):im.seek(i);im.load();duration+=im.info.get('duration',0)
        assert duration==result['media']['gif_duration_ms'];im.seek(frame);im.convert('RGB').save(d/'groups_camera.png');media.append(dict(name=name,frames=im.n_frames,duration_ms=duration,selected_png_time=times[frame]))
fig.suptitle('Actual Gazebo robot/human paths and native-membership centroids; gray circles = initial humans',fontsize=12);fig.tight_layout();fig.savefig(OUT/'group_actual_paths.png',dpi=140);plt.close(fig)
(OUT/'media_audit.json').write_text(json.dumps(media,indent=2)+'\n')
report=f'''# SocNavGym nhóm đi / đứng trò chuyện với Nav2

Ngày22/10/2026, bắt đầu từ commit `ebbd455`. Bốn lượt theo protocol sửa đổi được giữ toàn bộ; một lượt GUI ban đầu bị lỗi hạ tầng được lưu riêng. Đây là pilot nhóm+Nav2, không phải thống kê success rate hoặc chứng nhận tính tự nhiên.

{table}

Đích thật yêu cầu action SUCCEEDED và world XY≤0.20m/yaw≤0.25rad tại pose sau settle0.4wall s. Không thay ngưỡng, chạy thay failed goal hoặc lấy AMCL estimate làm ground truth. Empty control và6 cá nhân từ cohort trước chỉ làm tham chiếu, không phải matched-group ablation.

## Bài thử và các thay đổi được khai báo

Cùng robot Mecanum/LiDAR/IMU, Nav2 MPPI/AMCL, factory60×40m/42 boxes, start(−28,0,0),goal(−20,0,0),budget45sim s.6 người radius0.36m, nominal robot circle0.25m, physics0.0025s/native0.1s. Bắt đầu bằng2 nhóm3 người tại(−25.5,0)/(−22.5,0), radius bố trí0.72m; seed42/43 cho từng walking/standing. **Nhóm ban đầu là fixture tường minh, không phải formation tự phát**. Walking initial goal(−20.5,0); standing quay vào tâm. Sau đó native formation0.03/dispersal0.01 và native goals vẫn chạy.

Tái dùng ContinuousGroups giữ ID và liên tục khi commit nhóm, cùng response_v2 global ORCA/per-human convex projection/swept filter. Đây là variant sửa chuyển động, không native nguyên bản; Nav2 chỉ nhận LiDAR/costmap, không social group-aware controller. Standing conversation là bố trí đứng và hướng nhìn, không phát lời thoại; clip đi bộ giữ ở tốc độ0, chưa clip nói chuyện riêng.

Kernel cũ lấy mẫu trong8×4m local aisle. Khi đúng native group goal_radius1.44m, đổi đích nhóm có thể không tìm được chỗ trống và chờ30wall s. Probe offline03 được dừng có ghi nguyên nhân. Cấu hình mới riêng dùng vùng lấy mẫu thực60×40m/origin(0,0),giữ đủ42 walls và thuật toán native; không thêm tường giả, sửa vendor hoặc thay robot/Nav2. Goal sampling rộng không có nghĩa đã nghiệm thu người đi khắp nhà xưởng. Probe offline01/02 trước đó dùng bán kính goal cá nhân nhỏ hơn, chỉ giữ làm lịch sử fixture; không gộp thành bằng chứng native-group final.

Offline04:4 cấu hình×100 bước và exact20 repeat, swept geometry độc lập. Gate02:10 warmup transport ticks giữ nguyên toàn bộ native/human/group state, sau gate20 bước khớp exact offline04, clock lỗi terminal. Trong Gazebo chỉ mở gate khi accepted NavigateToPose; không publisher robot velocity script. YAML runtime chỉ đổi AMCL initial x−27→−28, robot/config/vendor/source cũ và staging của người dùng giữ nguyên.

![Quỹ đạo thật và tâm nhóm đo từ pose Gazebo](group_actual_paths.png)

Đồ thị giới hạn ROI quanh robot; human paths ra ngoài khung vẫn giữ trong dữ liệu và kiểm hình học. Tâm nhóm tính lại từ pose thật của các thành viên theo native membership, không lấy nhãn center làm bằng chứng chuyển động.

## Quan sát nhóm và cảm biến

{details}

Moving coverage nghĩa có một đoạn membership/loại liên tục≥2s và actual centroid path≥0.5m; standing coverage chỉ nghĩa thời lượng đứng nhóm≥2s. Đường kính, member peak speed và fraction quay vào tâm trong30° lưu từng segment ởvalidation/summary. Không đồng nhất coverage với cohesion tốt, độ tự nhiên hoặc robot đã nhường theo xã hội. Tan/đổi nhóm là hành vi được giữ, không lỗi bị xóa. Source group ID theo object trong một session; human IDs1..6 giữ xuyên thay đổi.

Clearance dùng swept discs/AABBs giữa actual pose samples; rotated robot footprint0.24×0.20m chỉ tại samples. Các số này không tự chứng nhận social comfort. LiDAR kiểm tia trước static occlusion; costmap occupied-ring là bằng chứng mô tả, không attribution/recall. Goal, geometry và group coverage được đánh giá riêng.

## Lượt lỗi gốc và runtime

**walking01 đầu tiên lỗi NVIDIA/WSL khi GUI khởi tạo Ogre/EGL**, chỉ một native active update, chưa đánh giá Nav2. Log có stack libnvwgf2umx.so/segmentation fault; giữ result/validation/raw gzip/source cũ. Cohort gốc đã dừng. Sau đó chốt protocol mới với nhãn_v2, khởi động server trước và GUI riêng sau khi sensor/AMCL/Nav2 sẵn sàng; đồng thời sửa fixture goal_radius và vùng sampling như trên. Đây là bộ đo sau thay đổi được khai báo, không retry âm thầm hoặc sửa outcome cũ. Chưa chứng minh đã loại bỏ mọi lỗi driver WSL.

GPU NVIDIA được kiểm preflight/Ogre cho các lượt nghiệm thu; worker giới hạn2 CPU/one BLAS thread, Nav2/physics/ORCA vẫn cần CPU. GUI vàRTF khác nhau, không joint-seed AMCL/MPPI/physics; không xếp hạng runtime hoặc suy tỷ lệ thành công tổng quát. Server/GUI kết thúc theo phiên, không kill tiến trình không liên quan. GUI được yêu cầu ở2 lượt, chưa có xác nhận kéo/zoom riêng của người dùng cho cohort này.

Ảnh/GIF từ RGB camera Gazebo thật640×480/1frame mỗi sim-second, phát4× simulation time; không AI và không quay màn hình realtime. Góc camera không thấy mọi người suốt lượt.

![Nhóm đi — camera Gazebo thật](walking01_v2/camera_preview_4x.gif)

![Nhóm đứng — camera Gazebo thật](standing01_v2/camera_preview_4x.gif)

## Quyết định demo và phần còn lại

Bảng trên là nghiệm thu thực tế của pilot; chưa gọi dense crowd/không gian chật hoặc nhiều seed đã đạt. SocNavGym variant phù hợp nhất để tiếp tục demo nhóm đi/đứng/lifecycle trong các nhánh đã khảo sát; CSP ORCA+ giữ baseline người độc lập, CN geometry giữ nhánh hybrid hình học chưa trained RGL. Không thay backend mặc định hoặc kiến trúc robot, không World Model/Isaac.

Bước tiếp theo là đóng gói checklist demo ngắn và bảng lựa chọn dựa trên báo cáo đã lưu; các failure/partial coverage vẫn hiển thị. Những mở rộng như dense crowd/narrow-aisle nhóm, in-place reset hoặc chất lượng animation là nhiệm vụ bổ sung, không tự gộp thành nghiệm thu pilot hiện tại.

[Protocol](../protocol.json) · [Số đo](summary.json) · [Offline cuối](offline04/validation.json) · [Gate cuối](gate02/validation.json) · [Lượt GUI lỗi](walking01/validation.json) · [Commands/returncodes mới](v2_execution.json). Raw pose/scan/odom/commands/plan/costmap/AMCL/TF/worker/world/config/log có gzip byte-identical trong từng folder.
'''
(OUT/'GROUP_NAV2_REPORT.md').write_text(report)
print(json.dumps(dict(status='GROUP_REPORT_WRITTEN',actual_goals=sum(v['actual_goal_reached'] for v in summary),trials=4,moving_covered=sum(v['groups']['moving_travel_covered'] for v in summary),standing_covered=sum(v['groups']['standing_duration_covered'] for v in summary)),indent=2))
