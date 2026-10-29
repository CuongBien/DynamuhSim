# Bắt đầu ở đây — những phần đang dùng

Cập nhật ngày **29/10/2026**. Công việc hiện tại: **kiểm tra điều hướng và sửa chuyển động nhóm người trước đợt thu dữ liệu mới**. `dataset01` là bộ pilot đã có, chưa thay đổi hoặc gán nhãn lại; World Model chưa train.

## 1. Hiện tại đã có gì?

| Hạng mục | Trạng thái hiện tại |
| --- | --- |
| Robot và môi trường thu dữ liệu | Mecanum + LiDAR 2D + depth + IMU, Nav2, nhà xưởng Gazebo; con người từ SocNavGym với adapter phản ứng hiện có |
| Bộ dữ liệu `dataset01` | 8 episode được chấp nhận; 2 lượt lỗi đã cách ly |
| Mẫu đủ điều kiện học dự báo BEV | **5.094**: train **2.763**, validation **1.652**, test **679** |
| World Model | Đã có dữ liệu pilot; **chưa train WM** |
| RL | Chưa có reward hợp lệ hoặc giao diện step/reset đã nghiệm thu |
| Xem lại camera | Có bản ghi thật và trang phát video với nút dừng/tua |
| Cửa sổ Gazebo trên Windows/WSLg | Lượt 29/10 đã được người dùng xác nhận thấy cảnh và kéo/zoom được; capture khoảng0,32×, replay nhẹ khoảng1× |

Đọc và train offline trên dữ liệu đã thu **không cần mở Gazebo hoặc chạy lại ROS/Nav2**. Bộ hiện tại cùng nhà xưởng, tuyến robot và mẫu người; chưa chứng minh khả năng tổng quát sang môi trường hoặc simulator khác. Test hiện chỉ có tình huống khởi tạo nhóm đứng.

## 2. Chỉ cần tìm những chỗ này

Các đường dẫn trong bảng tính từ gốc repository `/home/cuongbien/DynamuhSim`.

| Cần làm | Chỗ sử dụng |
| --- | --- |
| Đọc dữ liệu đầy đủ | `experiments/social_navigation/benchmark/wm_episode_batch/generated/dataset01/` |
| Reader không cần ROS, chỉ dùng NumPy | `dataset01/tools/loader.py` bên trong bộ dữ liệu |
| Xem file, shape, timestamp và mask chi tiết | [Schema dữ liệu](../experiments/archive/social_navigation/reports/wm_episode_batch/SCHEMA.md) |
| Xem kết quả thu và kiểm tra chất lượng | [Báo cáo batch](../experiments/archive/social_navigation/reports/wm_episode_batch/WM_BATCH_REPORT.md) |
| Xem lại camera và quỹ đạo đã ghi | `experiments/social_navigation/replay/wm_dataset_replay/` |
| Lấy bản sao đầy đủ | [Thư mục Google Drive](https://drive.google.com/drive/folders/1I3kfKbTm5X6Bj32RyPVck5kFSjmVgqnl) |

Code reader/target hiện ở `experiments/social_navigation/datasets/`, replay ở
`experiments/social_navigation/replay/`. Dữ liệu local cũ trong các ví dụ bên
dưới vẫn ở `benchmark/<stage>/generated`; đợt refactor không chuyển raw/dataset.
Khi restore vào chỗ khác, truyền `--dataset`/`--raw-root` hoặc đổi `root` cho đúng
vị trí thực. Receipt cũ vẫn giữ ID nguồn `benchmark/<stage>` và hash gốc.

**GitHub chứa code và báo cáo. Full dataset và raw MCAP nằm ở máy/Drive, không được tải về chỉ bằng `git clone`.**

## 3. Đọc thử một mẫu dữ liệu

Chạy Python trong Ubuntu/WSL có NumPy. Nếu khôi phục ở thư mục khác, sửa `root` cho đúng vị trí `dataset01`.

```python
from pathlib import Path
import sys

root = Path('/home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/wm_episode_batch/generated/dataset01')
sys.path.insert(0, str(root / 'tools'))
from loader import Dataset

episode = Dataset(root).episode('wm_standing_45')  # train
index = 50
x = episode.observation(index)
condition = episode.action_condition(index)
y = episode.supervision(index)
masks = episode.task_masks(index)

print('Depth:', x['depth_history'].shape)                    # (20, 240, 424)
print('Future actions:', condition['executed_action_future'].shape)  # (30, 3)
print('Future BEV:', y['future_bev']['occupancy'].shape)      # (3, 64, 64)
print('WM sample valid:', bool(masks['wm_observed_bev_train_valid']))
```

Một mẫu gồm **khoảng 2 giây quan sát trước thời điểm t**, chuỗi **3 giây lệnh robot đã thực thi**, và BEV thực tế sau **0,5 / 1 / 3 giây** để làm đáp án. BEV có ô trống, có vật cản và chưa quan sát được; đây là occupancy quan sát từ depth, chưa phải bản đồ chi phí xã hội.

Khi chuẩn bị train:

- Chỉ fit trên split `train`; dùng `validation` để chọn mô hình. Mask hợp lệ không cho phép dùng mẫu thuộc `test` để train.
- Dùng `wm_observed_bev_train_valid` và `executed_action_condition_valid` để lọc mẫu; dùng `cell_loss_valid` trong `y['future_bev']` để tính loss trên ô có đáp án hợp lệ.
- Depth có NaN/Inf: áp dụng finite/range mask trước khi đưa vào mạng. BEV giá trị `-1` là chưa quan sát được, không phải ô trống.
- Quan sát `x` và điều kiện hành động `condition` là hai phần đầu vào tách biệt. Không đưa GT người, đáp án tương lai hoặc ID simulator/seed vào đặc trưng quan sát.
- Chuỗi hành động là lệnh Nav2 thực sự đã chạy; bộ này chưa có đáp án cho mọi hành động giả định. Nhãn H1–H5, liên kết detection với ID người và reward RL hiện chưa hợp lệ.

## 4. Mở replay để xem ngay

Trong terminal Ubuntu/WSL:

```bash
cd /home/cuongbien/DynamuhSim
python3 experiments/social_navigation/replay/wm_dataset_replay/serve_recording.py --port 8765
```

Mở **http://localhost:8765/** trong trình duyệt Windows. Giữ terminal chạy khi xem; `Ctrl+C` để dừng. Nếu cổng đã bận, bỏ `--port 8765` và mở URL mới được in ra.

Trang có dừng, tua, tốc độ 1×/2×/4× và toàn màn hình. Video là **90 frame camera Gazebo ghi thật**, mặc định phát nhanh 4×; camera ngoài chỉ dùng minh họa, không phải đầu vào WM.

Gazebo pose replay và hướng dẫn build ở [README replay](../experiments/social_navigation/replay/wm_dataset_replay/README.md). Chỉ dùng khi cần kiểm tra 3D: nó nội suy XY/yaw đã ghi, không chạy lại Nav2/vật lý hay tái tạo cảm biến. Lượt 29/10 đã xem được GUI; chế độ capture còn chậm khoảng0,32×, replay nhẹ khoảng1×. Xem [kết quả demo mới](demos/2026-10-29/README.md).

## 5. Khôi phục dữ liệu từ Drive

1. Trong thư mục Drive, đọc `README_RESTORE.txt` và `parts_manifest.json`.
2. Tải đủ **14 file `DynamuhSim_wm_dataset01_backup_part001.zip` … `part014.zip`** được liệt kê trong manifest.
3. Giải nén tất cả vào **cùng một thư mục mới**, giữ đường dẫn con. Không nối các ZIP thành một file.
4. Dữ liệu portable nằm tại `wm_episode_batch/generated/dataset01/` trong thư mục vừa giải nén; trỏ `root` ở ví dụ Python tới đó.

`replay_and_backup_tools.zip` là gói công cụ bổ sung, không phải part thứ 15 của dữ liệu. Bản sao dữ liệu đã được giải nén thử và đối chiếu SHA256 từng file tại máy; metadata Drive đã đối chiếu tên/dung lượng. Chi tiết và phạm vi những file được sao lưu ở [báo cáo backup](../experiments/archive/social_navigation/reports/wm_episode_batch/BACKUP_DRIVE.md).

## Trạng thái mô phỏng mới nhất — 29/10

Các capture mới dùng geometry profile riêng: collision robot bao24×28cm, Nav2 radius17cm + padding1cm, URDF/TF dựng khớp SDF và camera x12cm. Model/cấu hình ROS mặc định vẫn giữ nguyên; dữ liệu cũ không bị sửa. Cấu hình MPPI horizon5s đã qua cả khe80cm và52,7cm trong fixture tĩnh, nhưng lượt nhà xưởng cũ cần recovery gần đích.

Đợt mới đã thử resampling MPPI và adapter nhóm người riêng biệt: **4/4 lượt Gazebo GPU tới đích, audit PASS, không recovery**. Cùng12 giây đầu vào robot, adapter giảm tổng góc quay73%; kiểm tra pose Gazebo trong các lượt tương tác cũng giảm quay rõ rệt. Chưa nghiệm thu hoàn toàn tính tự nhiên: cần thêm gặp ngược chiều/cắt ngang và nhiều seed. Chưa nâng lên mặc định hoặc đăng ký thành dataset train mới.

Xem [báo cáo hiện tại và video](demos/2026-10-29/endpoint_humans/README.md), [trang xem trực quan](demos/2026-10-29/endpoint_humans/index.html) và [khe hẹp](demos/2026-10-29/static_gap/README.md).

## 6. Việc tiếp theo, theo thứ tự

1. Kiểm tra nhóm người trong tình huống ngược chiều/cắt ngang và nhiều seed; kiểm tra recipe7 trên hai khe tĩnh trước khi chốt cấu hình thu mới.
2. Đóng băng cấu hình đã nghiệm thu và thu pilot mới khi cần mở rộng dữ liệu. Không trộn kết quả thử nghiệm với các mẫu train đã chấp nhận.
3. Với `dataset01` có sẵn, bước WM vẫn là baseline nhỏ dự báo BEV có điều kiện hành động: train trên train, chọn bằng validation, cố định rồi mới đánh giá test. Chưa thực hiện trainer trong vòng sửa mô phỏng này.
4. Chuẩn bị RL sau khi có reward và step/reset hợp lệ.

**Chưa có trainer WM đã triển khai/nghiệm thu trong bước hiện tại**, nên tài liệu chưa có lệnh train WM. Việc đổi simulator hoặc chuyển Isaac Sim chưa nằm trong vòng kiểm tra hiện tại.
