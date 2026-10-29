# Bắt đầu ở đây — những phần đang dùng

**Người tiếp nhận bắt đầu tại [HANDOFF_START.md](HANDOFF_START.md)**: setup theo mức,baseline guard,portable smoke và boundary MuJoCo. Phần bên dưới giữ hướng dẫn dataset/recipe đang có.


Cập nhật ngày **29/10/2026**. Công việc hiện tại: **đã hoàn thành pilot dữ liệu nhỏ và đủ mốc bàn giao backend MuJoCo**. `dataset01` là bộ pilot đã có, chưa thay đổi hoặc gán nhãn lại; World Model chưa train.

**Pilot bàn giao mới đã PASS:**2episode/517window,910depth bitwise và2068BEV replay;loader NumPy-only và LABEL independence đạt. [Mẫu/ZIP/report local](http://localhost:35315/handoff_pilot/index.html),[tài liệu bàn giao MuJoCo](HANDOFF_MUJOCO.md). Bộ này pilot_only,không train/val/test;dataset01 cũ vẫn giữ nguyên. Đã đủ mốc tạm dừng phase Gazebo; migration chưa bắt đầu.

## 1. Hiện tại đã có gì?

| Hạng mục | Trạng thái hiện tại |
| --- | --- |
| Robot và môi trường thu dữ liệu | Mecanum + LiDAR 2D + depth + IMU, Nav2, nhà xưởng Gazebo; con người từ SocNavGym với adapter phản ứng hiện có |
| Bộ dữ liệu `dataset01` | 8 episode được chấp nhận; 2 lượt lỗi đã cách ly |
| Pilot bàn giao mới |2episode/517window,raw/BEV/loader audit PASS;pilot_only,không cho phép train |
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

Kết quả mới nhất: [hai lượt bản sửa đầy đủ PASS](http://localhost:35315/transport_order/index.html). Đã đóng băng **factory_social_pilot_v1** cho thu pilot trong nhà xưởng hiện tại: nhóm trò chuyện giữ nguyên thành viên và nhóm đi bộ theo lifecycle tự nhiên. [Recipe và lệnh prepare/run/audit](demos/2026-10-29/transport_order/README.md), [manifest cấu hình](demos/2026-10-29/transport_order/collection_profile.json). Manifest là hồ sơ recipe; lệnh --config vẫn dùng geometry_v7_mppi_resample.json.

Nhóm trò chuyện đã giữ đủ hai nhóm trên seed46–48; kiểm chứng snapshot tuổi mục tiêu với truy vấn đến đích/hết hạn NOT_EXERCISED. Người không dịch chuyển trong các lượt này, nên chưa chứng minh active sidestep. Lượt walking vẫn dùng query verifier gốc, nhóm được phép tan/lập lại và cần metadata theo từng frame. Cả4 bridge ROS launch được dừng đúng phiên trước Gazebo; hai lượt mới full audit PASS, không recovery/escalation. Lỗi các cohort cũ vẫn giữ nguyên ở [báo cáo seed46–48](demos/2026-10-29/seed_regression/README.md) và [ba seed trò chuyện](demos/2026-10-29/persistent_conversation/README.md).

Các lệnh replay bên dưới xem **dataset01 cũ**, không tự bật profile người mới. dataset01 chưa bị sửa hay đăng ký thêm dữ liệu từ các lượt kiểm chứng này.

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

Robot/Nav2 đã qua khe80cm và52,7cm bằng profile riêng, giữ model/TF khớp SDF và MPPI horizon5s/resampling. Đồng hồ mục tiêu người đã có `sim_v1`, tính50s theo Gazebo từ lúc nhận goal, không tính thời gian chờ máy.

**Ứng viên người đang dùng để kiểm chứng: `group_cohesion_v2` + `sim_v1`.** Hai cảnh ngược chiều/cắt ngang seed45 giảm tốc độ quay78%/88%, giữ bán kính thành viên–tâm nhóm dưới1,23m/1,14m. Robot đều tới đích, không recovery; kiểm tra chuyển động đã công bố PASS cả2. Vẫn giữ per-person ORCA và kiểm tra va chạm liên tục. Đây là adapter tùy chỉnh, chưa phải nghiệm thu mọi hành vi tự nhiên.

**Vòng hành vi trước có audit1/2 PASS:** bridge ground_truth abort khi đóng lượt cắt ngang; lỗi gốc và audit FAIL vẫn giữ nguyên. **Vòng kiểm chứng shutdown mới đạt2/2 PASS**, sau khi dừng bridge đúng phiên lúc Gazebo đang pause, trước khi đóng server. Robot tới đích cả hai lượt, không recovery; nguồn người/guard, robot và Nav2 không đổi. Đây là bằng chứng giới hạn trên hai lượt seed45, chưa chứng minh hết race condition. Xem [báo cáo mới](demos/2026-10-29/bridge_shutdown/README.md). Chưa nâng mặc định ROS, chưa thay dataset01 hay đăng ký dữ liệu train mới. Các bản thử giảm quay nhưng tách nhóm và bản cohesion đầu tăng quay đều được lưu để đối chiếu.

Xem [báo cáo hiện tại/cách chạy đúng profile](demos/2026-10-29/group_cohesion_heading/README.md), [hai video và đồ thị](demos/2026-10-29/group_cohesion_heading/index.html), [lỗi đóng bridge](demos/2026-10-29/group_cohesion_heading/teardown_failure.json). Các nhóm đứng trò chuyện vẫn dùng pipeline cũ đã có; profile mới dành cho nhóm đi bộ.

## 6. Việc tiếp theo, theo thứ tự

1. Phase Gazebo hiện tại đã đạt mốc bàn giao:recipe factory_social_pilot_v1 + pilot end-to-end hai episode đã audit. Đọc [HANDOFF_MUJOCO.md](HANDOFF_MUJOCO.md) và chuyển gói portable; chưa cần thêm vòng tuning/thu batch lớn trong phase này.
2. Người tiếp nhận port backend MuJoCo theo clock/frame/action/sensor/label contract, rồi xuất hai cảnh tương đương và kiểm tra loader/BEV/masks. Sau khi backend mới đạt mới khai báo variation/seed/split cho batch;pilot này không phải train/validation/test.
3. Với `dataset01` có sẵn, bước WM vẫn là baseline nhỏ dự báo BEV có điều kiện hành động: train trên train, chọn bằng validation, cố định rồi mới đánh giá test. Chưa thực hiện trainer trong vòng sửa mô phỏng này.
4. Chuẩn bị RL sau khi có reward và step/reset hợp lệ.

**Chưa có trainer WM đã triển khai/nghiệm thu trong bước hiện tại**, nên tài liệu chưa có lệnh train WM. Việc port MuJoCo là phase bàn giao tiếp theo; chưa được thực hiện trong pilot dữ liệu này.
