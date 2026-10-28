# Pilot Nav2 cùng nhiệm vụ — CrowdSimPlus / ComplexityNav

[Báo cáo, số đo và GIF](../../../archive/social_navigation/reports/shared_nav2_pilot/SHARED_NAV2_REPORT.md). Bốn lượt mới, cả4 action SUCCEEDED;3 tới đích thật, crowd02 vượt ngưỡng20cm nên giữ không đạt. Đối chiếu với2 SocNavGym v2 và empty control đã commit. Không gọi đây là thống kê success rate hoặc xếp hạng simulator.

Cùng nhà xưởng60×40m/42 boxes,6 người radius0.36m,start(−28,0,0),goal(−20,0,0),45sim s,MPPI/AMCL/map/robot/cảm biến gốc. Runtime YAML chỉ đổi AMCL initial x thành−28. Worker native đứng yên trongwarmup, chạy sau accepted goal; robot do Nav2 điều khiển. CSP native ORCA+dt0.1s; CN3ORCA+3SFM intention +geometry roadmap/projection dt0.25s, không trained RGL. Nội bộ observation/speed khác nhau, không equal-policy ablation.

Từ root repo trong Ubuntu/WSL, chọn report directory **mới**:

```bash
bash experiments/social_navigation/navigation/shared_nav2_pilot/run.sh live \
  --model crowd_orca_plus --gui \
  --report-dir experiments/social_navigation/navigation/shared_nav2_pilot/reports/demo_crowd_new
bash experiments/social_navigation/navigation/shared_nav2_pilot/run.sh analyze \
  --live experiments/social_navigation/navigation/shared_nav2_pilot/reports/demo_crowd_new

bash experiments/social_navigation/navigation/shared_nav2_pilot/run.sh live \
  --model complex_geometry_mixed --gui \
  --report-dir experiments/social_navigation/navigation/shared_nav2_pilot/reports/demo_complex_new
bash experiments/social_navigation/navigation/shared_nav2_pilot/run.sh analyze \
  --live experiments/social_navigation/navigation/shared_nav2_pilot/reports/demo_complex_new
```

Chạy từng lệnh live/analyze xong mới mở phiên kế. GUI tự đóng khi action xong/budget hết; không reset world. ROS domain91/partition riêng/lock chung, không chạy đồng thời experiment khác. Gate hai container đã PASS trong reports/gate01. Producer `run.sh gate` của cohort cố định đã nghỉ dùng; receipt vẫn được runtime kiểm tra, với [source đã ghim](https://github.com/CuongBien/DynamuhSim/tree/68b2dded70b1ae41361f2e1aa8dad593ae2111f6/experiments/social_navigation/benchmark) để tái lập. Hai script cohort/continuation đã gỡ khỏi entrypoint hiện hành; bản gốc còn ở `reports/final_source_snapshot/`, gồm continuation đúng2 lượt CN sau lỗi analyzer crowd02. Lượt mới dùng live/analyze ở trên hoặc [runtime Nav2 chung](../../../../docs/SHARED_NAV2_RUNTIME.md), với output/checkpoint mới.

Dependencies đã có: Ubuntu24.04,ROS2 Jazzy/workspace install,Docker và2 image content-ID trong common_response/inputs/images.json, plugin socnavgym_response_fix/build/libresponse_adapter.so. Runtime native Python3.8.13 cô lập, hostROS Python3.12.3;không đổi Python hệ thống. GPU NVIDIA được kiểm qua runtime_guard/Ogre; container2CPU/4GB/one BLAS thread, CPU vẫn xử lý ORCA/Nav2/physics. Nếu plugin thiếu chỉ build plugin experiment bằng run.sh build của response_fix; không build/sửa workspace robot.

Analyzer dùng venv-socnavgym có NumPy/Shapely/PIL/Matplotlib/PyYAML. Checkpoint bảo toàn Git/source trong generated là local ignored; máy khác cần checkpoint/cohort tương đương trước audit. Re-run dùngthưmục mới không tự gia nhập bảng cố định4 lượt đã đo. Công cụ finalize/package_audit của cohort cố định đã nghỉ dùng; bảng và provenance đã lưu giữ nguyên, với [source trước khi dọn](https://github.com/CuongBien/DynamuhSim/tree/fbfa33a6c5ed8b062afe7523073f24b77477b852/experiments/social_navigation/benchmark). Đích thật dựa pose Gazebo đúng mốc final_world_pose sau settle ngắn, action vàXY/yaw kiểm riêng. Analyzer ban đầu crowd02 bị assertion do tail pose trongcleanup; đã sửa đúng mốc, giữ attempt và outcome, không replay simulator.

Camera/GIF lấy RGB Gazebo thật,640×480,1frame/sim-second,phát4× simulation time; không AI hoặcscreen recording realtime. Raw streams/config/world/log được gzip byte-identical. [Physical task audit](../../../archive/social_navigation/reports/shared_nav2_pilot/physical_task_audit.json), [số đo](../../../archive/social_navigation/reports/shared_nav2_pilot/summary.json), [provenance](../../../archive/social_navigation/reports/shared_nav2_pilot/source_manifest.json) và final package manifest ghi rõ source/variant. Source_snapshot là snapshot trước sửa analyzer và thêm guard chống ghi đè receipt; final_source_snapshot là bản đóng gói sau đo.

Mốc này kiểm6 cá nhân giao cắt; nhóm đi/đứng/formation+Nav2 cần pilot SocNavGym riêng tiếp theo. Không chọn backend mặc định mới, không World Model/Isaac hoặc mở ma trận AMCL.
