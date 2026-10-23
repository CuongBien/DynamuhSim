#!/usr/bin/env python3
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
s=json.loads((HERE/'reports/summary.json').read_text());a=json.loads((HERE/'reports/audit.json').read_text());ledger=json.loads((HERE/'reports/all_attempts_ledger.json').read_text())
rows='\n'.join(f"| {v['episode_id']} | {v['split']} | {v['navigation_status']} | {'Đạt' if v['capture_accepted'] else 'Cách ly: exit 255'} | {v['windows']} | {v['wm_valid']} |" for v in ledger)
splitrows='\n'.join(f"| {name} | {v['episodes']} | {v['windows']} | {v['wm_valid']} |" for name,v in s['splits'].items())
report=f"""# Batch đầu dữ liệu robot chuyển động cho World Model

**Đã thực hiện 10 lượt: 8 episode hợp lệ, 2 lượt cách ly.** Có {s['total_windows']:,} cửa sổ, trong đó {s['wm_observed_bev_valid_windows']:,} đủ điều kiện target BEV quan sát tương lai. Bộ này dùng được để **train thử WM dự báo BEV quan sát**, theo đúng split và mask. Chưa huấn luyện WM hoặc RL trong lượt này.

Robot chạy bằng Nav2 thật trong Gazebo. Giữ nguyên cấu hình moderate đã kiểm: start (−28,0), goal (−20,0), budget 90 giây mô phỏng, bán kính 0.25 m, padding 0.01 m, inflation 0.8 m, tốc độ vx 0.30/−0.15 và vy 0.20 m/s. Không script đường robot, không GT người vào planner, không đổi mặc định ROS/Nav2 hoặc source framework cũ. Render cảm biến dùng NVIDIA RTX 4060/D3D12; physics, Nav2, worker và export/audit vẫn dùng CPU với BLAS/OMP một luồng.

## Toàn bộ lượt thu

| Episode | Tập | Nav2 | Thu dữ liệu | Cửa sổ | WM BEV hợp lệ |
| --- | --- | --- | --- | ---: | ---: |
{rows}

Timeout được giữ nếu dữ liệu hợp lệ; không chạy lại để chọn kết quả tới đích. Hai lượt standing47 và walking49 đóng MCAP mã 0, không worker fault, nhưng Gazebo thoát mã 255. Chúng vẫn FAIL tiêu chí clean-exit và không có trong portable training corpus. Callback filter cân bằng lần lượt 1028/1028 và 1034/1034, cleanup rỗng; những bằng chứng này không thay thế clean-exit gate. **Lỗi shutdown còn tái diễn, chưa sửa dứt điểm.** Raw/log của cả hai lượt được giữ.

Pipeline ban đầu dừng ở lượt thứ năm. [Review01](quarantine_review01.json) cho phép chạy tiếp đúng năm lượt chưa thử bằng cùng frozen driver/settings, cách ly standing47, không retry hoặc thay thế. Lượt cuối walking49 cũng FAIL; [Review02](quarantine_review02.json) chỉ đóng sổ cuối với 8 accepted + 2 quarantine, không chạy thêm. Hai [diff accounting đầu](audit_quarantine_accounting.diff) và [cuối](audit_final_quarantine_accounting.diff) giữ rõ thay đổi số lượng; ngưỡng chấp nhận, capture/export/target math và raw cũ không đổi. Capture, export và continuation có source freeze riêng.

## Split và số lượng

| Tập | Episode hợp lệ | Cửa sổ xuất | WM BEV hợp lệ |
| --- | ---: | ---: | ---: |
{splitrows}

Chia trước thu theo toàn seed family: 45–47 train, 48 validation, 49 test. Các cửa sổ chồng lấn không bị chia sang tập khác. **Test chỉ còn cảnh standing49** vì walking49 bị cách ly, nên batch này không kiểm độc lập được walking trên test. Same warehouse/route/avatar vẫn xuất hiện ở các tập; đây là seed holdout, chưa phải holdout môi trường/người/simulator.

Đây là batch SocNavGym đầu tiên: sáu người, nhóm đứng hoặc đi ban đầu, native lifecycle và reactive projection hiện có. Nhóm có thể tan; không khẳng định cả sáu người trò chuyện suốt episode. CrowdSimPlus chưa thêm ở batch này, HuNavSim vẫn giữ riêng và chưa sử dụng. H1–H5 không được suy từ tên simulator hoặc tư thế.

## Dữ liệu đã có cho WM

Đã kiểm **{s['raw_depth_bitwise']:,} ảnh depth khớp MCAP từng byte** và {s['accepted_native_scan_messages']:,} native LaserScan trong 8 episode hợp lệ. MCAP giữ đủ 16 topic. Mỗi cửa sổ có lịch sử 2 giây/20 query ở 10 Hz: LiDAR, depth, IMU, odom, lệnh vx/vy/wz và goal từ AMCL có mask tuổi dữ liệu. Scan gốc 5 Hz được giữ theo mẫu đã khả dụng, không tạo scan giả ở 10 Hz. Camera thấp và kích thước robot giữ nguyên; RGB observer chỉ là camera debug bên ngoài, không đầu vào robot.

Normalizer xét {s['normalizer_candidate_windows']:,} anchor, giữ {s['total_windows']:,}, loại {s['normalizer_rejected_windows']:,} do thiếu hỗ trợ history/future label/action theo contract. Raw và lý do loại được giữ. Trong cửa sổ đã xuất, thêm 12 cửa sổ thiếu đủ target BEV được mask, không bù bằng frame tương lai hay GT trong input.

Input BEV dùng depth và odometry nhân quả để warp về anchor ước lượng. Target ở **+0.5/+1/+3 giây** dùng depth tương lai thực đã ghi, warp về hệ trục robot thật tại anchor; GT ego pose chỉ nằm ở nhánh LABEL. Grid 64×64, cell 0.1 m; unknown −1, observed free 0, occupied 1. Loss chỉ dùng cell/frame hợp lệ. Đây là occupancy thực quan sát, không toàn cảnh, social cost hoặc dự đoán do model sinh; input ước lượng và target trục thật vẫn có sai số căn chỉnh.

Accessor `.action_condition(i)` đọc riêng 30 lệnh tương lai đã chạy ở t…t+2.9 s để làm điều kiện hành động cho WM. `.observation(i)` không chứa future sensor/GT. Dữ liệu theo behavior policy Nav2, không đo phản thực cho mọi action và không tự coi là expert policy.

Human future/group GT được giữ ở LABEL để truy vết; association, H1–H5 và RL reward vẫn mask false. Reward NaN không có nghĩa reward bằng 0. Success/truncation được ghi riêng; chưa có reward/contact truth hoặc Gym step/reset để train RL online. Không cần classifier hoàn hảo mới ghi chuỗi cảm biến cho nhiệm vụ WM observed BEV.

## Kiểm chứng và sử dụng

8 unit tests trước export freeze đạt. [Audit cuối](audit.json) kiểm source/inventory SHA, thời gian/availability nhân quả, raw future actions/human labels, toàn bộ target pose/time/mask và whole-family split. Đã replay {a['geometry_grids_replayed']} grid ở first/middle/last của mỗi episode bằng pose/projection độc lập và raster đã chốt; không có history depth lượng tử 1 mm + action/odom trùng giữa các split. 7 episode train/validation đọc ở thư mục khác vẫn giữ observation khi xóa LABEL trên bản copy. Test API mặc định bị chặn; chỉ integrity/label QA tự động, không WM fitting/tuning/scoring hoặc xem/công bố ảnh test.

Native driver/normalizer tự tạo debug media/BEV preview ở raw, nhưng test media không được xem/chọn và không có trong portable input. Các tag `pilot_only`/overall training false của legacy normalizer được giữ để truy vết; split và **task eligibility mới** lấy từ dataset.json, episode.json và task masks. WM mask true ở test không cho phép train trên test. Xem [schema và API](SCHEMA.md).

[Ảnh train](media/wm_sample.png), [GIF Gazebo thật 4×](media/native_observer_train_4x.gif) và [window mẫu](../examples/window01/metadata.json) lấy cố định index 50 của standing45. **Không AI sinh ảnh/GIF, không có WM prediction**: cột tương lai là LABEL từ depth đã ghi. Gray = unknown, trắng = observed free, đỏ = observed occupied; cone tương lai dịch do camera tương lai được warp về anchor.

Full portable bundle ở `generated/dataset01/`, **{s['portable_dataset_logical_bytes']:,} byte (khoảng 3.01 GB)**. MCAP của cả 10 lượt là {s['raw_bag_bytes_all10_attempts']:,} byte; paired arrays/runtime khác cần backup riêng nếu muốn trọn bằng chứng. **Git không chứa full raw/bundle**; Git giữ code, protocol/freeze, audit/reports, native logs và một window mẫu. Copy toàn bộ dataset01 để loader chạy bằng Python + NumPy, không ROS/Gazebo/Torch; giữ raw để kiểm lại MCAP. [Inventory](dataset_inventory.json) lưu SHA/size. Các file đã có và staging người dùng được kiểm bảo toàn trong [preservation](preservation_before_commit.json).

Mốc thu/chuẩn hóa batch đầu đã xong với 2 quarantine rõ ràng. Bước tiếp theo là train thử WM observed BEV bằng train, chọn trên validation rồi cố định trước test standing49. Đây vẫn là pilot nhỏ, tương quan, cùng route/avatar và một backend; chưa nghiệm thu các head social-navigation khác, RL hay tổng quát qua simulator.
"""
(HERE/'reports/WM_BATCH_REPORT.md').write_text(report)
readme="""# Batch đầu dữ liệu robot chuyển động cho WM

Đã thực hiện **10 lượt: 8 episode hợp lệ, 2 cách ly**. Xuất **5.106 cửa sổ**, trong đó **5.094 target WM observed BEV hợp lệ**. Chưa train WM hoặc RL. Test chỉ còn standing49; walking49 bị cách ly vì lỗi shutdown, không chạy lại thay thế.

- [Báo cáo và mọi lượt đạt/lỗi](reports/WM_BATCH_REPORT.md)
- [Schema, file, shape, thời gian và masks](reports/SCHEMA.md)
- [Audit](reports/audit.json), [thống kê](reports/summary.json)
- [Một window để đọc](examples/window01/metadata.json)
- [Ảnh train thật](reports/media/wm_sample.png), [GIF Gazebo thật](reports/media/native_observer_train_4x.gif)

Bundle đầy đủ: `generated/dataset01/` (khoảng 3.01 GB); raw: `generated/runs/`, runtime: `generated/runtime/`. Cần backup riêng, full data không ở Git. Reader chỉ cần Python3 + NumPy. Capture/export đã chạy trên Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic, Python3.12, cảm biến render NVIDIA; không cài dependency hoặc sửa ROS/Nav2 mặc định.

## Đọc dữ liệu

Copy toàn bộ dataset01 và giữ đường dẫn tương đối:

```python
from pathlib import Path
import sys
root = Path('/home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/wm_episode_batch/generated/dataset01')
sys.path.insert(0, str(root / 'tools'))
from loader import Dataset

episode = Dataset(root).episode('wm_standing_45')
x = episode.observation(50)          # sensor history, không GT/future
condition = episode.action_condition(50) # future executed actions, WM condition riêng
y = episode.supervision(50)          # LABEL đọc riêng
mask = episode.task_masks(50)
print(x['depth_history'].shape)      # [20,240,424]
print(y['future_bev']['occupancy'].shape) # [3,64,64]
```

Chỉ fit trên train. Respect split dù task mask true; test API mặc định bị chặn. Depth NaN/Inf phải có finite/range mask trước preprocessing. Future occupancy −1 là unknown, chỉ tính loss trên cell_loss_valid. M1 track-aligned, H1–H5 và RL reward chưa có supervision hợp lệ; reward NaN không phải 0.

## Tái kiểm

```bash
TASK_DIR=experiments/social_navigation/benchmark/wm_episode_batch
bash "$TASK_DIR/run.sh" test
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 "$TASK_DIR/audit02.py"
```

Audit cần full bundle/raw đúng đường dẫn đã lưu. Capture/export gốc dừng ở standing47; review01 cho phép năm lượt chưa thử với cùng frozen driver, review02 đóng sổ cuối khi walking49 cũng shutdown exit255. Hai lượt lỗi giữ raw và bị loại; criteria/source math không thay. Một batch mới cần IDs/output/protocol/freeze mới, không ghi đè evidence. Script/freeze đầu và continuation được giữ để tái lập, không dùng rerun để thay FAIL.

Đã có dữ liệu để train thử WM dự báo BEV quan sát; chưa có reward/contact truth hoặc Gym step/reset để train RL online. Không cần mở thêm vòng detector trước khi thử WM này.
"""
(HERE/'README.md').write_text(readme)
note=f"""## Cập nhật — đã thu batch đầu dữ liệu chuyển động cho WM

[WM batch report](benchmark/wm_episode_batch/reports/WM_BATCH_REPORT.md): 10 lượt SocNavGym với 5 seed mới × standing/walking; **8 episode hợp lệ, 2 cách ly** vì Gazebo shutdown exit255. Không retry/thay thế/nới clean-exit gate. Có {s['total_windows']:,} cửa sổ, {s['wm_observed_bev_valid_windows']:,} target WM observed BEV hợp lệ; split 5 train / 2 validation / 1 test theo seed family. Test chỉ còn standing49. {s['raw_depth_bitwise']:,} depth khớp MCAP, 8 tests và audit causal/frame/portable/duplicate đạt.

Input sensor/action history 2 s; future observed BEV +0.5/+1/+3 s và executed-action condition đọc riêng. GT chỉ ở LABEL; H1–H5/association/RL reward masked. Chưa fit/tune WM, HuNavSim chưa dùng. Full bundle khoảng 3.01 GB/raw ở generated cần backup riêng; Git có code/freeze/reports/window mẫu và media ghi thật. Bước kế: train thử WM observed BEV trên train/validation, chưa RL online hoặc đổi kiến trúc robot. Mốc lưu: `feat: collect a masked sensor-action pilot dataset for world models`.

"""
for name in ['README.md','NEXT_STEPS.md','COMMIT_HISTORY.md','FRAMEWORK_COMPARISON.md']:
 file=HERE.parents[1]/name;text=file.read_text();begin=next(text.index(t) for t in ['## Cập nhật — batch đầu dữ liệu chuyển động cho WM đã thu','## Cập nhật — đã thu batch đầu dữ liệu chuyển động cho WM'] if t in text);end=text.index('\n## ',begin+4);file.write_text(text[:begin]+note+text[end+1:])
print('Final report and status rewritten with actual8/2 outcome')
