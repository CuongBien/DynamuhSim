# Ranh giới backend cần giữ khi port

Machine-readable specification: `configs/handoff/backend_contract.json`. Đây là tài liệu của giao diện đang có,không phải một MuJoCo implementation mới hoặc abstraction class đã chạy.

| Phần | Source chính | Vai trò khi bàn giao |
| --- | --- | --- |
| Dữ liệu portable |`datasets/handoff_pilot`,wm_episode_batch/loader.py |Reader/arrays độc lập simulator;giữ bytes/schema đã audit |
| Sampling/BEV |`python/dynamuhsim/datasets/normalization.py`,perception_target_pilot/sensor.py,geometry.py,wm_episode_batch/targets.py |Dùng lại math và timing contract;exporter MCAP là adapter input riêng |
| Ý định/lifecycle người |`python/dynamuhsim/simulation/group_*`,goal_clock.py,conversation.py;navigation/socnavgym_* kernel/worker |Logic vẫn dựa SocNavGym và guard hiện có;adapter MuJoCo cung cấp robot state và áp human state |
| Hình học/tình huống |warehouse/layout.json,source_run/assets và runtime world/nav2 config |Dùng làm nguồn geometry/fixture đã nghiệm thu;chuyển cách dựng world sang backend mới |
| Gazebo/ROS orchestration |runtime/nav2_trial.py,capture*.py,simulation/capture_launch.py;response_fix/adapter.cc |Phần phụ thuộc Gazebo/ROS,không phải portable reader |
| WSL/render/shutdown |runtime/gpu.py,ordered_stop.py,shutdown.py,transport_shutdown.py |Giữ cho backend tham chiếu;MuJoCo cần kiểm tra startup/stop/render riêng |
| Bằng chứng |docs/demos,experiments/archive,source freezes |Giữ immutable;không di chuyển chỉ để gom folder |

Tên thư mục `simulation/` hoặc `runtime/` không có nghĩa mọi file trong đó đều độc lập engine;hãy theo ownership ở bảng này. Đường dẫn đầy đủ của stage là `experiments/social_navigation/datasets/` hoặc `navigation/`.

## Hai frame vận tốc khác nhau

- **Robot command**: vx,vy,wz trong planar body axes base_link/base_footprint đã kiểm aligned;đơn vị m/s,m/s,rad/s.
- **Human worker robot input**:x,y,yaw trong world;vx,vy,omega là finite difference của measured world pose qua native ticks,không phải body command. Adapter C++ hiện tính trực tiếp `(x-prevX)/dt`,`(y-prevY)/dt` và wrapped yaw difference.

Gửi thẳng body vx/vy vào trường world velocity sẽ đổi phản ứng tránh robot khi robot quay. Người port cần biến đổi theo đúng frame và ghi cả commanded/executed/measured streams đúng vai trò.

## Nhịp thời gian và pose người

Request worker là `frame=world,epoch=0,time=t,dt=0.1,robot={...}`. Reply mang `time=t,state_time=t+0.1,before,humans,groups_before,groups`. Backend hiện nội suy before→humans trong interval[t,t+0.1],với shortest wrapped yaw. Metadata groups_before thuộc interval hiện tại. **Scene LABEL là actual applied poses**,không stamp future proposal thành pose hiện tại.

Pause không step human model;start gate là accepted navigation goal time. Worker hiện hỗ trợ một epoch0 liên tục;reset cần episode/worker session mới. Clock discontinuity,NaN/Inf hoặc lỗi identity làm terminal restart_required;không tự extrapolate/retry. Ready được công bố nguyên khối trước khi consumer thấy path.

## Những contract nghiệm thu cần giữ

History20query10Hz,availability≤query≤anchor;future executed action condition30interval và LABEL riêng. BEV current dùng causal odom;future BEV dùng GT ego anchor LABEL ONLY. Đơn vị và axes/calibration đầy đủ ở HANDOFF_MUJOCO.md. H1–H5/association/reward hiện masked;backend mới không tự làm chúng trở thành nhãn hợp lệ.

Persistent conversation là mode chọn rõ,tắt dispersal cho designated triplets bằng native flag. Natural walking cho phép tan/lập nhóm;đọc metadata mỗi frame. Snapshot-clock contract chỉ áp cho conversation được freeze;query-clock verifier gốc vẫn dùng cho walking. Không nâng coverage NOT_EXERCISED thành PASS khi chuyển backend.

## Phần chưa triển khai

Chưa có MJCF,MuJoCo renderer/ray/IMU collector,reset driver hoặc bridge MuJoCo. Người nhận tạo chúng theo boundary và checklist trong HANDOFF_MUJOCO.md,rồi xuất hai episode mới để so schema/time/frame/masks. Đây là bước tiếp sau chuẩn bị bàn giao,không gộp vào cleanup hiện tại.

## Source closure và versioning

`runtime/nav2_trial.py::source_files` hiện đưa toàn bộ `python/dynamuhsim/**/*.py` vào closure capture. Bổ sung module trong package đó có thể khiến `verify_run` của một run đã prepare đòi thêm source mới. Khi port,giữ checkout baseline để tái lập và tạo source freeze/version mới cho lượt mới;không sửa manifest cũ để nhận source mới. Raw audit provenance có thể đối chiếu source ở recorded Git commit.

Vòng bàn giao này đặt wrapper trong `tools/` và metadata trong `configs/handoff/`,nên giữ nguyên closure/runtime đã nghiệm thu. Baseline guard kiểm116file chọn rõ;đó không phải một signed security policy hay chứng nhận implementation MuJoCo tương lai.
