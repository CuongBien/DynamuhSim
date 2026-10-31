# Batch đầu dữ liệu robot chuyển động cho WM

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
