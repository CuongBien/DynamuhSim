# Demo 1 — Acceptance report

Ngày chạy: 2026-09-25

## Static geometry, map and topology — PASS

- PASS: Robot A → B (21 graph nodes)
- PASS: Human Room01 → Room12 (25 graph nodes)
- PASS: Human Stair → Room05 (10 graph nodes)
- PASS: Human Corridor → Room (5 graph nodes)
- Mọi graph edge được lấy mẫu mỗi 0,025 m với toàn footprint HuNav bán kính
  0,22 m và không cắt occupied cell.
- PASS: hai negative control (đi xuyên tường Room01 và xuyên lõi cầu thang)
  đều bị từ chối.
- Đủ 8 semantic zone types bắt buộc.

## Gazebo/Nav2 live baseline — PASS

- PASS: TF `map → odom → base_footprint`.
- PASS: `/odom`, `/scan`, `/map`, `/navigate_to_pose`.
- PASS: A → Room05.
- PASS: Room05 → CorridorC.
- PASS: CorridorC → JunctionCD.
- PASS: JunctionCD → JunctionDE.
- PASS: JunctionDE → JunctionEF.
- PASS: JunctionEF → Room12.
- PASS: Robot A → B end-to-end.
- PASS: cả 6 global plan đều footprint-clear trên occupancy grid.
- PASS: robot thật trong Gazebo đạt `(17.48, 7.35)` với goal Room12
  `(17.60, 7.30)` và action trả về `SUCCEEDED`.
- PASS: `/demo/gazebo_robot_pose`, `/demo/gazebo_robot_path` và
  `/demo/pose_sync_error` đồng bộ để đối chiếu trực tiếp trong RViz.
- PASS: Gazebo GUI resolve `robot` qua `/gui/follow` (`data: true`) và camera
  tự bám model thay vì đứng lại tại điểm spawn.
- PASS: test `/cmd_vel` làm pose Gazebo đổi từ `(-13.51, -8.15)` thành
  `(-13.23, -8.15)`; sai lệch Gazebo–TF sau test là 0,007 m.

Baseline dùng `school_floor_baseline.world`, cùng static geometry với world
chính nhưng bỏ proxy HuNav để actor chưa được điều khiển không chắn costmap.
Robot dùng vỏ primitive màu cam và mũi chỉ hướng xanh nên vẫn nhìn thấy nếu
mesh TurtleBot3 không được Gazebo resolve.

## HuNav one-human runtime — PASS

- Loader đọc scenario `demo_1_hunav.yaml`.
- Manager: `Successfully retrieved parameters from hunav_loader`.
- Manager: `BT nodes registered`.
- Adapter thấy `demo_primary`, robot và 122 static collision AABB.
- Target `demo_primary` thay đổi 0.075 m ngay trong hai mẫu đầu.

## HuNav two-human social-force runtime — PASS

Đo trong 30 giây với `demo_1_social.yaml`:

- 301 mẫu cho mỗi agent.
- `demo_primary`: 11.296 m.
- `demo_crossing`: 2.863 m.
- Khoảng cách nhỏ nhất: 1.078 m.
- Ngưỡng fail: 0.50 m (nhỏ hơn tổng khoảng cách an toàn quan sát được).

Behavior Tree `SchoolInteractiveTree` chứa `IsRobotClose` và native
`AvoidRobot`; scenario có các hệ số obstacle/social/other force khác 0.

