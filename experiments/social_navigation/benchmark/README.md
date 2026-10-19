# Pilot RVO2 ↔ SocNavGym với Nav2 trong nhà xưởng

Harness này so sánh runtime RVO2 đang cài trong workspace với kernel nhóm SocNavGym của adapter, trên cùng Gazebo world, robot Mecanum, LiDAR, map và tham số MPPI. Không sửa code robot hoặc plugin RVO2 gốc.

## Chạy

```bash
cd /home/cuongbien/DynamuhSim
bash experiments/social_navigation/benchmark/run.sh trial --backend rvo2 --condition standing --seed 42 --gui
bash experiments/social_navigation/benchmark/run.sh trial --backend socnavgym --condition counterflow --seed 42 --gui
bash experiments/social_navigation/benchmark/run.sh matrix
experiments/social_navigation/.venv-socnavgym/bin/python experiments/social_navigation/benchmark/analyze.py
```

Chạy từng lệnh demo riêng, dừng bằng Ctrl+C trước khi mở lệnh tiếp theo. Cần workspace ROS đã build, venv SocNavGym và plugin adapter đã build theo [hướng dẫn adapter](../gazebo_adapter/README.md). `--gui` mở Gazebo; bản headless vẫn render camera quan sát để lưu ảnh/GIF.

Ma trận mặc định gồm một đối chứng không người và 8 lượt có người: standing/counterflow × RVO2/SocNavGym × seed 42/43. Chạy tuần tự. Các report có `protocol_valid: true` được tái sử dụng; dùng `matrix --rerun` để chạy lại và ghi đè report tương ứng. Mỗi trial luôn có thư mục `generated/` mới cho world/config/log; dữ liệu thô không bị ghi đè. Lệnh trial cùng backend/condition/seed sẽ ghi đè report tóm tắt tương ứng, kể cả khi thêm `--gui`; giữ bản đã commit trước khi chạy demo lại.

ROS domain mặc định 75; Gazebo partition riêng cho từng trial. Không chạy hai trial cùng ROS domain một lúc. Không mở thêm backend điều khiển cùng người. Harness chỉ dừng process group do chính nó tạo.

## Điều kiện chung

- Nhà xưởng 60×40 m; robot bắt đầu `(-27,0,0)`, goal `(-14,0,0)`, lối chính rộng.
- Standing: ba người quanh `(-23.5,0)`, giữ bố trí conversation ban đầu.
- Counterflow: ba người quanh `(-21,0)`, đi về `(-28,0)` rồi quay lại `(-21,0)`. Các backend giữ quy tắc chuyển goal native của mình.
- Initial pose và heading của từng người được sinh xác định theo seed; hash xác nhận cùng bộ pose được nạp vào cả hai backend. Proxy người radius 0.36 m, cao 1.7 m; robot radius danh định 0.25 m. Native reset có margin ±0.01 m; predicate người native dùng ellipse (xem đính chính bên dưới).
- Cùng file map và Nav2 MPPI parameters. Kiểm tra hash scene/map/parameters theo từng cặp; kiểm tra pose thật trước khi bắt đầu, không chỉ tin dữ liệu đầu vào.
- Người được giữ yên trong lúc AMCL/Nav2 khởi động. Chỉ mở gate khi `bt_navigator` active và goal đã được chấp nhận. Tất cả thời gian benchmark dùng simulation clock.
- Ngân sách goal mặc định 75 giây simulation, kèm giới hạn wall time khi simulator bị treo. ABORTED/TIMEOUT là kết quả điều hướng hợp lệ nếu hệ đo vẫn hoạt động; ERROR là lỗi harness/runtime cần xử lý riêng.

RVO2 là **binary hiện đang cài**, được lưu SHA256 trong từng report; không rebuild vào `build/` hoặc `install/` của project. Working tree source đang có thay đổi riêng của người dùng, nên không khẳng định binary đó tương đương source chưa build hiện tại.

RVO2 dùng H4 cho standing, H1 cho counterflow, tắt random persona. Các radius và preferred speed được khai báo riêng cho benchmark để khớp với scene SocNavGym; đây không phải toàn bộ cấu hình mặc định production. Nominal walking speed 0.8 m/s; noise, giới hạn và dynamics nội bộ vẫn native.

## Đọc kết quả

[Report](reports/BENCHMARK_REPORT.md), [CSV](reports/results.csv), [phân tích JSON](reports/analysis.json), [quỹ đạo](reports/paired_paths.png), [clearance theo thời gian](reports/clearance_curves.png).

Mỗi trial có JSON kết quả, trajectory JSON đo từ SceneBroadcaster, ảnh ở thời điểm gần robot nhất và GIF từ camera Gazebo. Dữ liệu benchmark của hai backend đều dùng pose model Gazebo, không so pose worker của một backend với pose simulator của backend kia.

Có hai quy ước clearance khác nhau:

1. Circle: khoảng cách tâm robot/người trừ 0.25 + 0.36 m. Đây là metric vòng tròn danh định, khác predicate ellipse native; không phải contact sensor.
2. Footprint: hình chữ nhật Nav2 0.24×0.20 m, xoay theo yaw robot, so với hình tròn proxy người 0.36 m. Đây là chỉ số chính về khoảng hở theo footprint điều hướng; chưa bao gồm toàn bộ collision geometry của robot.

Các duration overlap chính dùng ngưỡng **trên 1 cm** để loại sai số số học micromet. Giá trị strict `<0` ban đầu vẫn được giữ trong raw report. Thời gian overlap được xấp xỉ từ mẫu pose khoảng 10 Hz; chưa phải bằng chứng không có tiếp xúc giữa mọi physics step.

## Giới hạn pilot

Hai seed không đủ để xếp hạng thống kê. Seed chỉ điều khiển scene/human; MPPI noise và Gaussian noise LiDAR chưa được khóa chung. Standing là đối chứng có cùng bố trí người bất động; khác biệt nhỏ giữa hai lượt không nên quy cho khả năng hành vi xã hội.

RVO2 điều khiển ba người độc lập, còn SocNavGym có một group native. Cùng waypoint center không có nghĩa cùng thời điểm quay đầu, cùng quỹ đạo thành viên hay cùng update rate: RVO2 dùng physics step, SocNavGym dùng 0.1 s. Bản kernel này không chạy full RL episode hoặc formation/dispersal.

Source RVO2 hiện tại chỉ thêm tường vào ORCA ở corridor mode; benchmark giữ open-arena và dùng binary sẵn có. SocNavGym nhận 42 khối nhà xưởng. World vật lý giống nhau nhưng khả năng biết vật cản của solver không được khẳng định tương đương; lượt đầu chỉ ở lối chính rộng, không dùng để kết luận về toàn nhà xưởng/bottleneck.

Không sửa thêm avoidance/cohesion để che lỗi native trong lúc đo. Goal success phải được xem cùng clearance, overlap, drift và hình ảnh nhóm. Log shutdown Gazebo có thể chứa lỗi SIGTERM/segfault sau khi đo xong, tương tự các vòng trước.

## Seed 43 diagnostic replay

[Detailed report and run commands](diagnostic/reports/run01/DIAGNOSIS_REPORT.md): an unchanged-parameter replay reproduced proxy overlap. The native collision predicate uses a human ellipse, unlike the Gazebo cylinder; its robot radius also has default reset jitter. Read this correction before interpreting the pilot circle metrics as native contact geometry. Optional `--diagnostics` records passive sensor/TF/costmap telemetry and requires a separate `--report-dir`.

## Geometry normalization ablation

[Hướng dẫn native/circle_contact](geometry_comparison/README.md). Biến thể opt-in đổi predicate người sang circle khớp proxy và cố định radius robot danh định sau reset; không thêm né robot hoặc swept guard. Kết quả được lưu riêng, không ghi đè pilot.

## Robot-aware group experiment

[Hướng dẫn và kết quả](avoidance_comparison/README.md): `--robot-avoidance` bổ sung robot đo từ Gazebo vào group ORCA, chỉ trên circle_contact và trong phạm vi một moving group. Đây là adapter policy mới; native và circle_contact mặc định giữ nguyên.

## CrowdSimPlus container backend

[Adapter và cách chạy Gazebo](../crowdsimplus_adapter/README.md). `--backend crowdsimplus` dùng kernel người ORCA+ native trong Docker Python 3.8, robot vẫn do Nav2 điều khiển. Chỉ hỗ trợ fixture seed 42/43 và ba người ở vòng đầu; không có group/conversation model.

## CrowdSimPlus: native SFM versus ORCA+

`--human-policy sfm` chọn SFM upstream trong cùng container, không chỉnh lực hay
thêm luật giữ nhóm. `--human-policy orca_plus` là mặc định. [Hướng dẫn](../crowdsimplus_adapter/README.md#sfm-native-trong-gazebo)
và [kết quả](sfm_comparison/reports/run01/SFM_GAZEBO_REPORT.md).

Ma trận yêu cầu hai file `orca_plus_validation.json`/`sfm_validation.json` đã PASS
trong report directory, rồi chạy tuần tự bốn lượt cùng fixture seed 42/43:

```bash
bash experiments/social_navigation/benchmark/run.sh sfm-matrix \
  --report-dir experiments/social_navigation/benchmark/sfm_comparison/reports/run01
python3 experiments/social_navigation/benchmark/analyze_sfm.py \
  experiments/social_navigation/benchmark/sfm_comparison/reports/run01
```

Dùng directory mới khi muốn chạy lại, chuẩn bị fixture bằng `prepare_scene` rồi
chạy `crowdsimplus_adapter/validate.py --human-policy orca_plus|sfm` bên trong
container với `/runtime` riêng. `--gui` ở trial sẽ tự đóng khi lượt đo kết thúc;
đây là lượt benchmark có mục tiêu tự động, không phải demo chạy vô hạn.

## Fixture đích riêng cho từng người

[Hướng dẫn](individual_goals/README.md): `--goal-layout individual` giữ nguyên
vị trí ban đầu và native policy, tách endpoint của ba người để kiểm tra tuần
tra. `individual-goals-matrix` tự kiểm định kernel và goal completion trước
khi chạy SFM/ORCA+ seed 42/43. Mặc định vẫn là shared fixture cũ.

## Cảnh giao cắt SFM/ORCA+

Có opt-in `--condition crossing --goal-layout individual` cho CrowdSimPlus.
Ba người đi từ hai phía qua lối robot, với native policy giữ nguyên.
[Cách chạy, kiểm định và báo cáo](crossing/README.md).

## Lối hẹp SFM/ORCA+

Có thêm fixture `--condition bottleneck --goal-layout individual`, dùng khe
1,8×8 m hiện có trong nhà xưởng. [Cách chạy và kiểm định](bottleneck/README.md).

## Chẩn đoán AMCL từ các lượt lối hẹp

[Công cụ và report offline](localization_diagnosis/README.md) đối chiếu pose,
odom, scan, TF, local costmap và sensor score. Không chạy lại simulator hoặc
chỉnh tham số để thay các kết quả thất bại đã lưu.

## Ghi thêm global navigation

`--navigation-diagnostics` bật recorder thụ động cho global plan, global costmap và native particle cloud; tự bật `--diagnostics`. [Cách chạy và phạm vi](navigation_diagnostics/README.md), [hai lượt seed42](navigation_diagnostics/reports/run03/NAVIGATION_DIAGNOSTICS.md). Kết quả chẩn đoán giữ riêng với ma trận bottleneck gốc.

GPU/cleanup: runner kiểm hardware renderer trước khi khởi động và dọn child theo token phiên. [Cách kiểm tra](gpu_runtime/README.md), [kết quả RTX 4060 và xử lý ROS chạy sót](gpu_runtime/reports/run01/GPU_RUNTIME.md).
