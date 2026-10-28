# Batch đầu dữ liệu robot chuyển động cho WM

Đã thực hiện **10 lượt: 8 episode hợp lệ, 2 cách ly**. Xuất **5.106 cửa sổ**, trong đó **5.094 target WM observed BEV hợp lệ**. Chưa train WM hoặc RL. Test chỉ còn standing49; walking49 bị cách ly vì lỗi shutdown, không chạy lại thay thế.

- [Báo cáo và mọi lượt đạt/lỗi](../../../archive/social_navigation/reports/wm_episode_batch/WM_BATCH_REPORT.md)
- [Schema, file, shape, thời gian và masks](../../../archive/social_navigation/reports/wm_episode_batch/SCHEMA.md)
- [Audit](../../../archive/social_navigation/reports/wm_episode_batch/audit.json), [thống kê](../../../archive/social_navigation/reports/wm_episode_batch/summary.json)
- [Một window để đọc](examples/window01/metadata.json)
- [Ảnh train thật](../../../archive/social_navigation/reports/wm_episode_batch/media/wm_sample.png), [GIF Gazebo thật](../../../archive/social_navigation/reports/wm_episode_batch/media/native_observer_train_4x.gif)

Bundle đầy đủ: `generated/dataset01/` (khoảng 3.01 GB); raw: `generated/runs/`, runtime: `generated/runtime/`. Cần backup riêng, full data không ở Git. Reader chỉ cần Python3 + NumPy. Capture/export đã chạy trên Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic, Python3.12, cảm biến render NVIDIA; không cài dependency hoặc sửa ROS/Nav2 mặc định.

## Đọc dữ liệu

Copy toàn bộ dataset01 và giữ đường dẫn tương đối:

```python
from pathlib import Path
import sys
root = Path('/home/cuongbien/DynamuhSim/experiments/social_navigation/datasets/wm_episode_batch/generated/dataset01')
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

## Tái kiểm độc lập

```bash
TASK_DIR=experiments/social_navigation/datasets/wm_episode_batch
bash "$TASK_DIR/run.sh" test
bash "$TASK_DIR/run.sh" audit \
  --dataset "$TASK_DIR/generated/dataset01" \
  --raw-root "$TASK_DIR/generated/runs" \
  --runtime-root "$PWD" \
  --output "$TASK_DIR/generated/audit_recheck_01"
```

`--runtime-root` là checkout chứa các đường dẫn runtime trong raw `result.json`.
Output phải là thư mục mới, ngoài dataset/raw/báo cáo lưu trữ và cùng filesystem
với dataset (audit dùng hardlink chỉ đọc cho phép thử tách nhãn). Không cần ROS
hoặc Gazebo. `audit.json` giữ schema và tiêu chí của sổ cuối 8/2;
`source_provenance.json` ghi nguồn bytes đã kiểm SHA256. Nếu source cũ đã gỡ,
audit đọc bytes từ commit benchmark cố định `9833565fa546d55cb0e553f8388bf4a703c59583`
hoặc `--source-root CHECKOUT`; không chạy source lấy từ lịch sử. Reader, protocol
và phép tính hình học đang dùng phải khớp hash frozen ban đầu. Thiếu nguồn hoặc
sai hash sẽ dừng, không sửa freeze để chấp nhận code mới.

Audit kiểm định dữ liệu, bao gồm tính toàn vẹn phần test và việc API mặc định
chặn test; không train hay đánh giá mô hình trên test. Dataset, ledger, quarantine,
freeze và báo cáo đã công bố giữ nguyên. Capture/export gốc dừng ở standing47;
review01 cho phép năm lượt chưa thử, review02 đóng sổ khi walking49 cũng lỗi
shutdown exit255. Hai lượt lỗi giữ raw, không chạy lại để thay FAIL.

## Source lịch sử và công việc mới

Cohort dataset01 đã đóng. Đã gỡ các writer một lần `capture.py`, `export.py`,
`follow_export.py`, hai auditor trùng `audit.py`/`audit02.py`, các script resume,
đóng sổ và backup. [Checkout trước lượt dọn này](https://github.com/CuongBien/DynamuhSim/tree/e02d8443a8242751443763690491c53d3b00bcfc/experiments/social_navigation/benchmark/wm_episode_batch)
giữ các writer/auditor vừa gỡ; [source resume](https://github.com/CuongBien/DynamuhSim/tree/62fed27adbfc87032ebfd5a240a49ca46080c19f/experiments/social_navigation/benchmark/wm_episode_batch),
[source đóng sổ](https://github.com/CuongBien/DynamuhSim/tree/637596b5aa4ba14ecc7b1bab092b12f6ac8f215c/experiments/social_navigation/benchmark/wm_episode_batch)
và source backup ở commit `9686d88256011d4a8ec3d0c7429d94e7f31be8bd` phục vụ tái lập
quy trình cũ. Khôi phục full data theo [hướng dẫn hiện hành](../../../../docs/START_HERE_CURRENT.md#5-khôi-phục-dữ-liệu-từ-drive).

Thu episode mới qua `python3 tools/capture_episode.py --help`; công cụ này không
xuất dataset WM. Batch WM mới cần IDs, output, protocol và freeze mới, cùng quy
trình export riêng; không append vào dataset01. Đã có dữ liệu thử WM dự báo BEV
quan sát; chưa có reward/contact truth hoặc Gym step/reset để train RL online.
