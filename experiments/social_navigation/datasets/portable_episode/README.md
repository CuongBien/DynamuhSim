# Portable episode — chuẩn hóa dữ liệu đã ghi

**Đã xuất823cửa sổ từ1episode thật**, không chạy lại Gazebo và chưa train World Model. [Báo cáo](../../../archive/social_navigation/reports/portable_episode/PORTABLE_EPISODE_REPORT.md), [audit raw độc lập](../../../archive/social_navigation/reports/portable_episode/audit02/audit.json), [một mẫu đầy đủ để xem](../../../archive/social_navigation/reports/portable_episode/examples), [ảnh minh họa dữ liệu thật](../../../archive/social_navigation/reports/portable_episode/sample_preview.png).

Bản dữ liệu đầy đủ local: `generated/episode01/`, khoảng539MB theo byte inventory (không phải823episode độc lập). Đây là bản sao mới từ early_avoidance_pilot/avoidance_moderate01 đã đạt; raw cũ và ROS/Nav2/simulator source không đổi. Mọi cửa sổ chồng lấn giữ split=pilot_only; không chia ngẫu nhiên các cửa sổ cùng episode/seed/scene sang train/validation. HuNavSim giữ testcuối.

## Đọc dữ liệu mà không cần ROS/Gazebo

Từ repo, chỉ cần Python + NumPy đã có:

```bash
TASK=experiments/social_navigation/datasets/portable_episode
python3 -I tools/probe_portable_episode.py "$TASK/generated/episode01"
```

Ví dụ trong một chương trình đọc bundle đã chuyển sang thư mục khác:

```python
from pathlib import Path
from dynamuhsim.datasets.portable import Episode
bundle = Path('/your/copied/episode01')

episode = Episode(bundle)
inputs = episode.observation(0)       # không có human GT / nhãn tương lai
labels = episode.supervision(0)       # gọi riêng, chỉ dùng làm supervision
masks = episode.task_masks(0)
```

`inputs['observations']` có query/header/availability timestamps và các validity masks. `depth_history` giữ20ảnh float32 với NaN/Inf gốc; `current_bev` là quan sát từ depth cuối khả dụng. Không tự xem maskfalse là người không xuất hiện hoặc là một nhãn âm.

**Thời gian không đồng bộ:**anchor đầu12.3s nhưng depth cuối khả dụng12.2s. Trong export này ảnh được chọn già100ms do rule khả dụng bảo thủ; không đo latency wall-clock của camera. BEV ở base_link tại lúc đo depth, **chưa ego-motion-compensate tới anchor**. Nhãn XY tương lai ở trục robot thật tại anchor. Goal194/823mẫu quá tuổi1s được zero-vector kèm goal_valid=false; không thay bằng goal từ ground truth.

Để chuyển dữ liệu, copy **toàn bộ generated/episode01/** và kiểm bundle_manifest.json SHA256. Reader chuẩn chỉ cần package DynamuhSim và NumPy, không cần ROS/simulator. SDF/launch trong snapshot còn path tài sản lịch sử; rerun mô phỏng cần remap assets/dependencies. Chưa thử Isaac Sim, không coi bản binary plugin snapshot là portable runtime.

## Đọc và kiểm bundle đã đóng

```bash
TASK=experiments/social_navigation/datasets/portable_episode
bash "$TASK/run.sh" smoke "$TASK/generated/episode01"
bash "$TASK/run.sh" audit --bundle "$TASK/generated/episode01" --report-dir "$TASK/generated/my_audit"
```

Reader/smoke dùng chung package chuẩn, không nạp code trong bundle; auditor MCAP cần ROS2 Jazzy. Ba writer một lần
`prepare.py`, `build.py`, `finalize.py` đã gỡ; [source trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/3b133626809a39991babe0e6fcc52d2572c2c03b/experiments/social_navigation/benchmark/portable_episode)
giữ quy trình tạo bundle lịch sử. Bundle, source snapshot, manifest và eligibility
không đổi. Thu episode mới dùng `tools/capture_episode.py`; đóng gói theo protocol
mới, không ghi lại episode01 hoặc chạy snapshot cũ trong luồng hiện hành.

## Những gì đã và chưa có

- M1: nhãn quỹ đạo người3s `[823,30,6,2]` và future executed actions `[823,30,3]`; chưa có sensor tracker/association nên mask track-aligned supervision vẫnfalse.
- M2: depth/K/TF và823BEV quan sát hiện tại; chưa có target BEV tương lai, maskfuture=false. BEV không phải costmap an toàn cho robot.
- M3: H1–H5 giữ nguyên schema, persona_index=−1/persona_valid=false; không tự gán người đứng thành H4.
- Tất cả training_eligible=false.823cửa sổ chỉ là QA định dạng từ1episode, không phải bộ train hoàn chỉnh hay dữ liệu phản thực cho mọi robot action.

Bước kế ưu tiên xác thực track/association từ cảm biến và chốt target BEV tương lai; sau đó kiểm độc lập nhỏ trước thu đa nguồn. Giữ provenance cấu hình Nav2 vừa chọn, depth proxy OFF/LiDAR proxy ON và bản sửa literal friction frame. Không mở simulator khác, đổi kiến trúc robot, train World Model hoặc chuyển Isaac ở bước này.

Cập nhật dọn 27/10/2026: đã gỡ các công cụ chỉ phục vụ cohort/report cũ: `preview.py`. [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/87b0c6eea9b54794db7b4cb794fae57c78f11c70/experiments/social_navigation/benchmark/portable_episode) giữ cách tái lập; report, engine, source freeze và outcome cũ không đổi.

## Source đọc/audit hiện hành

Reader dùng `dynamuhsim.datasets.portable.Episode`. Kiểm tra nhanh bằng
`python3 -I tools/probe_portable_episode.py /absolute/path/to/episode01` từ repo.
Auditor dùng `dynamuhsim.datasets.normalized_audit`, không import/exec tool từ
bundle. `audit.py --artifact-root /absolute/path/to/original/checkout` trỏ tới
raw gốc khi dữ liệu local vẫn ở layout cũ; `--report-dir` phải là thư mục mới.
Normalizing writer pilot cũ đã nghỉ dùng; xem
[lịch sử](../../../../docs/history/retired_capture_pipelines.md).
