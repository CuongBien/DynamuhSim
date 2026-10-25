# SocNavGym nhóm với robot chạy Nav2

[Báo cáo, số đo và GIF Gazebo thật](reports/GROUP_NAV2_REPORT.md). Pilot gồm hai nhóm ba người đi cùng và hai nhóm ba người đứng quay vào tâm, seed42/43; nhóm được phép đổi membership/tan theo native lifecycle. Nhóm ban đầu là fixture, không phải formation tự phát. Đích thật, clearance và group coverage được đánh giá riêng; xem outcome trong report thay vì chỉ dựa action SUCCEEDED.

Để demo trên Ubuntu/WSL từ root repo, dùng thư mục kết quả **mới**, chạy lần lượt:

```bash
bash experiments/social_navigation/benchmark/socnavgym_group_nav2/run.sh live \
  --scenario walking --seed 42 \
  --report-dir experiments/social_navigation/benchmark/socnavgym_group_nav2/reports/demo_walking_new
bash experiments/social_navigation/benchmark/socnavgym_group_nav2/run.sh analyze \
  --live experiments/social_navigation/benchmark/socnavgym_group_nav2/reports/demo_walking_new

bash experiments/social_navigation/benchmark/socnavgym_group_nav2/run.sh live \
  --scenario standing --seed 42 \
  --report-dir experiments/social_navigation/benchmark/socnavgym_group_nav2/reports/demo_standing_new
bash experiments/social_navigation/benchmark/socnavgym_group_nav2/run.sh analyze \
  --live experiments/social_navigation/benchmark/socnavgym_group_nav2/reports/demo_standing_new
```

Các lệnh trên chạy headless và ghi camera GPU thật. Có thể thêm --gui để mở cửa sổ, nhưng Qt/OpenGL vẫn lỗi ngẫu nhiên tronghai attempt; chưa nghiệm thu GUI ổn định. Khi dùng --gui, GUI mở sau khi server/sensor/AMCL/Nav2 sẵn sàng, tự đóng khi action kết thúc hoặc hết budget45sim s. Không chạy song song experiment khác dùng ROS domain91; không reset world trong session. Native được giữ nguyên trongwarmup, chỉ chạy sau accepted goal. Robot nhận lệnh từ Nav2, không script velocity. Renderer yêu cầu NVIDIA, không fallback CPU rendering. Native ORCA/Python/Nav2/physics vẫn dùng CPU; worker giới hạn2 CPU/one BLAS thread.

Dependencies hiện có: ROS2 Jazzy/workspace install, Python3.12 SocNavGym venv, NumPy/RVO2/Shapely/PIL/Matplotlib/PyYAML và plugin response_fix/build/libresponse_adapter.so đã kiểm. Không đổi Python hệ thống hoặc build workspace robot. Native config riêng config.yaml dùng actual factory60×40m, origin0,42 real walls; chỉ mở rộng vùng lấy mẫu của config lifecycle cũ8×4m. Native group member goal radius1.44m, human body radius0.36m. Walking dùng response v2; standing v3 thêm bounded sidestep0.2m/s cho thành viên nhóm đứng khi center distance robot<1.4m. Người đứng độc lập vẫn cố định. Đối chứng standing01_v2 bị terminal stop giữ nguyên; bản v3 qua replay interval cũ/reaction_gate, không nới floor2mm. Response variant sửa chuyển động để tránh hình học; không gọi đây là unmodified upstream.

Gate03/offline05 đã PASS cho source final. Nếu thay source/dependency, lập preflight/cohort mới với output mới; không dùng một PASS cũ làm bằng chứng cho code mới. Probe và gate đều từ chối ghi đè output:

```bash
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/benchmark/socnavgym_group_nav2/probe.py \
  --name my_offline_new --steps 100
bash experiments/social_navigation/benchmark/socnavgym_group_nav2/run.sh gate \
  --name my_gate_new --offline my_offline_new
```

Runner live hiện khóa preflight chính thức offline05/gate03 của cohort này. Nếu tạo phiên bản source mới phải quy hoạch/check gate mới trong một experiment riêng. Analyzer dùng local ignored checkpoint bảo toàn Git/source; máy khác cần checkpoint tương đương trước khi audit. Dùng thư mục demo mới không tự thêm lượt đó vào cohort bốn lượt đã đo.

Hai script cohort cố định đã gỡ khỏi entrypoint hiện hành; bản gốc còn ở `reports/final_source/`. Receipts GUI v3 và cặp standing headless vẫn giữ trong v3_execution.json và standing_headless_execution.json. Dùng các lệnh live/analyze ở trên hoặc [runtime Nav2 chung](../../../../docs/SHARED_NAV2_RUNTIME.md) cho lượt mới. Hai walking đã đo giữ v2_execution.json và từ chối chạy khi receipt đã tồn tại. Walking01 lỗi GUI gốc được giữ riêng, không xóa; protocol mới công bố thay đổi startup,goal_radius và sampling trước bốn lượt_v2. Source snapshots trước/sau thay đổi được giữ. Native reward/obs/RL training, RGL, World Model/Isaac và controller mặc định không thuộc pilot.

Camera PNG/GIF là RGB Gazebo thật,640×480/1frame mỗi sim-second,phát4× simulation time; không AI hoặc screen recording realtime. Standing conversation chỉ biểu diễn đứng/hướng nhìn, chưa speech/standing animation riêng. Khả năng cảm giác tự nhiên vẫn cần người xem đánh giá.
