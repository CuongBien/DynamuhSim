# Portable episode — chuẩn hóa dữ liệu đã ghi

**Đã xuất823cửa sổ từ1episode thật**, không chạy lại Gazebo và chưa train World Model. [Báo cáo](reports/PORTABLE_EPISODE_REPORT.md), [audit raw độc lập](reports/audit02/audit.json), [một mẫu đầy đủ để xem](reports/examples/), [ảnh minh họa dữ liệu thật](reports/sample_preview.png).

Bản dữ liệu đầy đủ local: `generated/episode01/`, khoảng539MB theo byte inventory (không phải823episode độc lập). Đây là bản sao mới từ early_avoidance_pilot/avoidance_moderate01 đã đạt; raw cũ và ROS/Nav2/simulator source không đổi. Mọi cửa sổ chồng lấn giữ split=pilot_only; không chia ngẫu nhiên các cửa sổ cùng episode/seed/scene sang train/validation. HuNavSim giữ testcuối.

## Đọc dữ liệu mà không cần ROS/Gazebo

Từ repo, chỉ cần Python + NumPy đã có:

```bash
TASK=experiments/social_navigation/benchmark/portable_episode
python3 -I "$TASK/generated/episode01/tools/smoke.py" "$TASK/generated/episode01"
```

Ví dụ trong một chương trình đọc bundle đã chuyển sang thư mục khác:

```python
from pathlib import Path
import sys
bundle = Path('/your/copied/episode01')
sys.path.insert(0, str(bundle / 'tools'))
from portable_loader import Episode

episode = Episode(bundle)
inputs = episode.observation(0)       # không có human GT / nhãn tương lai
labels = episode.supervision(0)       # gọi riêng, chỉ dùng làm supervision
masks = episode.task_masks(0)
```

`inputs['observations']` có query/header/availability timestamps và các validity masks. `depth_history` giữ20ảnh float32 với NaN/Inf gốc; `current_bev` là quan sát từ depth cuối khả dụng. Không tự xem maskfalse là người không xuất hiện hoặc là một nhãn âm.

**Thời gian không đồng bộ:**anchor đầu12.3s nhưng depth cuối khả dụng12.2s. Trong export này ảnh được chọn già100ms do rule khả dụng bảo thủ; không đo latency wall-clock của camera. BEV ở base_link tại lúc đo depth, **chưa ego-motion-compensate tới anchor**. Nhãn XY tương lai ở trục robot thật tại anchor. Goal194/823mẫu quá tuổi1s được zero-vector kèm goal_valid=false; không thay bằng goal từ ground truth.

Để chuyển dữ liệu, copy **toàn bộ generated/episode01/** và kiểm bundle_manifest.json SHA256. Array loader không cần repo/ROS/simulator. SDF/launch trong snapshot còn path tài sản lịch sử; rerun mô phỏng cần remap assets/dependencies. Chưa thử Isaac Sim, không coi bản binary plugin snapshot là portable runtime.

## Tạo lại từ raw hiện có, không mô phỏng lại

Chọn tên output mới; mọi bước từ chối ghi đè dữ liệu cũ:

```bash
TASK=experiments/social_navigation/benchmark/portable_episode
BUNDLE="$TASK/generated/my_episode_copy"
bash "$TASK/run.sh" prepare --output "$BUNDLE"
bash "$TASK/run.sh" build --bundle "$BUNDLE"
bash "$TASK/run.sh" finalize --bundle "$BUNDLE"
bash "$TASK/run.sh" audit --bundle "$BUNDLE" --report-dir "$TASK/generated/my_audit"
bash "$TASK/run.sh" smoke "$BUNDLE"
```

Build/audit source ROS2Jazzy để đọc MCAP; loader/smoke không dùng ROS. Build dùng bản copy byte-identical của exporter/contract/core cũ để giữ quy tắc sampling; không sửa các file cũ. Chuỗi thực tế của bundle01 chạy bản exporter đã copy trực tiếp, cùng implementation byte-identical. Thư mục raw/array lớn bị Gitignore; manifest, QA, code, hình và một mẫu nhỏ được lưu Git.

## Những gì đã và chưa có

- M1: nhãn quỹ đạo người3s `[823,30,6,2]` và future executed actions `[823,30,3]`; chưa có sensor tracker/association nên mask track-aligned supervision vẫnfalse.
- M2: depth/K/TF và823BEV quan sát hiện tại; chưa có target BEV tương lai, maskfuture=false. BEV không phải costmap an toàn cho robot.
- M3: H1–H5 giữ nguyên schema, persona_index=−1/persona_valid=false; không tự gán người đứng thành H4.
- Tất cả training_eligible=false.823cửa sổ chỉ là QA định dạng từ1episode, không phải bộ train hoàn chỉnh hay dữ liệu phản thực cho mọi robot action.

Bước kế ưu tiên xác thực track/association từ cảm biến và chốt target BEV tương lai; sau đó kiểm độc lập nhỏ trước thu đa nguồn. Giữ provenance cấu hình Nav2 vừa chọn, depth proxy OFF/LiDAR proxy ON và bản sửa literal friction frame. Không mở simulator khác, đổi kiến trúc robot, train World Model hoặc chuyển Isaac ở bước này.
