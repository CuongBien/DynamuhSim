# Người đi cắt ngang lối robot: native SFM và ORCA+

Cảnh `crossing` có ba người độc lập, xuất phát tại ba vị trí x gần
-25,5/-24/-22,5 m và y luân phiên ±2,1 m. Mỗi người tuần tra sang phía đối
ngược rồi quay lại. Seed 42/43 đổi hướng xuất phát và jitter x ±0,1 m.
Đích chỉ biểu diễn ý định; không lập trình sẵn quỹ đạo, làn hoặc thời điểm
nhường robot. Robot bắt đầu (-27,0), nhận goal Nav2 (-14,0).

Cảnh mới thay cả initial poses/headings và endpoint so với counterflow cũ.
Giữ map/nhà xưởng, Mecanum/Nav2, native policy parameters, dt=0,1 s, human
radius=0,36 m, preferred speed=0,8 m/s. Không có group/conversation model.

## Chạy demo

Trong terminal Ubuntu từ repository root, đóng phiên cũ trước khi chạy:

```bash
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend crowdsimplus --human-policy sfm --goal-layout individual \
  --condition crossing --seed 42 --gui --diagnostics \
  --report-dir "experiments/social_navigation/benchmark/generated/crossing_demo_$(date +%Y%m%d_%H%M%S)"
```

Đổi `--human-policy orca_plus` để đối chiếu. Luôn dùng report directory mới.
Robot tự nhận goal; cửa sổ GUI đóng khi trial kết thúc. Bản headless vẫn lưu
ảnh/GIF từ camera RGB Gazebo. Các lượt báo cáo dùng headless, không tính là
kiểm thử thao tác GUI trực tiếp.

## Kiểm định và chạy lại benchmark

```bash
bash experiments/social_navigation/benchmark/run.sh crossing-matrix \
  --report-dir experiments/social_navigation/benchmark/crossing/reports/run02
python3 experiments/social_navigation/benchmark/analyze_sfm.py \
  experiments/social_navigation/benchmark/crossing/reports/run02 \
  --goal-layout individual --condition crossing
```

Runner kiểm tra đủ bốn policy/seed trước khi chạy tuần tự bốn lượt Gazebo.
`--validate-only` chỉ kiểm tra worker. Validation cache ghim source adapter,
fixture builder, validator, kernel trước thay đổi và Docker image.
Các probe gồm parity với upstream env.step, regression fixture đích riêng cũ,
tuần tra với robot cố định ở xa và phản ứng khi bật/tắt visibility robot.
Các probe không được cộng thành lượt Gazebo/Nav2.

Scene shared và individual counterflow cũ được so lại với fixture lưu trước
thay đổi; mode mới là opt-in và không ghi đè báo cáo cũ. Snapshot
`reports/run01/kernel_before.py` là adapter trước cảnh crossing, dùng regression,
không phải sửa source upstream. Native config phải giống byte-for-byte vòng SFM.

Đếm giao cắt từ pose người thực trong Gazebo, có hysteresis y ≤ -0,2 tới
≥ +0,2 m hoặc ngược lại. Đây là xác nhận đi qua tim lối dự kiến y=0;
robot có thể lệch khỏi tim lối, vì vậy không coi mọi giao cắt là gặp sát robot.
Khoảng hở robot–người dùng footprint xoay 0,24×0,20 m so proxy người tròn.

[Báo cáo và GIF](reports/run01/CROSSING_REPORT.md). Cảnh này ở lối chính rộng,
chưa phải thử bottleneck thật của nhà xưởng hoặc crowd đông.
