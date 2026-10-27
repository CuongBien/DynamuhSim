# Native SFM/ORCA+ trong lối hẹp nhà xưởng

Runner ma trận và analyzer của cohort này đã nghỉ dùng. Báo cáo giữ nguyên;
[source và lệnh cũ](https://github.com/CuongBien/DynamuhSim/tree/9833565fa546d55cb0e553f8388bf4a703c59583/experiments/social_navigation/benchmark)
được ghim theo revision lịch sử. Các lệnh `trial`/kiểm định còn dưới đây vẫn được hỗ trợ.

Cảnh `bottleneck` dùng đúng khe giữa hai khối máy hiện có: x=12..20 m,
y=-0,9..+0,9 m, rộng 1,8 m, dài 8 m. Robot bắt đầu (8,0), nhận goal (24,5,0).
Ba người xuất phát bên phải, đi ngược chiều robot, có endpoint riêng ở hai
phía. Không ép làn, thời điểm nhường hoặc bắt Nav2 đi theo quỹ đạo định sẵn.

Map/world, robot, cảm biến và controller parameters giữ nguyên. Chỉ runtime
YAML copy thay AMCL initial pose cho spawn mới; file Nav2 gốc không sửa.
Native human timestep 0,1 s, radius 0,36 m, preferred speed 0,8 m/s và các
force/safety parameters giữ nguyên. Đây vẫn là ba người độc lập.

## Chạy demo

Từ terminal Ubuntu tại repository root; đóng phiên trước rồi chạy:

```bash
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend crowdsimplus --human-policy sfm --goal-layout individual \
  --condition bottleneck --seed 42 --gui --diagnostics \
  --report-dir "experiments/social_navigation/benchmark/generated/bottleneck_demo_$(date +%Y%m%d_%H%M%S)"
```

Đổi `--human-policy orca_plus` để xem đối chiếu; dùng directory mới mỗi lượt.
Robot tự nhận goal, GUI tự đóng khi lượt kết thúc. Không chạy hai trial cùng
ROS domain. Headless vẫn lưu camera RGB Gazebo/GIF, góc nhìn đặt tại lối hẹp.

## Protocol cohort lịch sử

Runner lịch sử kiểm định bốn policy/seed trước, rồi chạy một empty control và bốn
lượt có người tuần tự. Empty control phải tới đích và thực sự đi qua channel.
`--validate-only` chỉ chạy worker. Cache ghim image, native bridge, scene
builder, validator, kernel trước thay đổi và layout.

Probe native gồm parity 30 bước, regression crossing/counterflow, quan sát
20 sim s với robot ở xa và probe visibility 10 s với robot input định sẵn.
PASS xác nhận bridge/state hữu hạn, không có nghĩa mọi người phải đi qua
hoặc hành vi an toàn. Mọi kết quả đứng kẹt/overlap được giữ để đánh giá.

Đếm full traversal khi tâm đi từ x≤11,5 sang x≥20,5 m hoặc ngược lại, với
các mẫu trong core x=12..20 nằm trong y±0,9 m. Đi vòng ngoài strip được ghi
riêng là bypass. Số lần qua khe khác số lần tới endpoint. Clearance đo từ
pose Gazebo theo footprint, human circles và khối máy; không phải contact force.

[Báo cáo, GIF và giới hạn](reports/run01/BOTTLENECK_REPORT.md).
