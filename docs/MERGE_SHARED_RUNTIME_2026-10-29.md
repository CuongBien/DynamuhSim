# Kiểm tra và merge shared-navigation-runtime — 29/10/2026

Đã fast-forward nhánh test/gazebo-harmonic-benchmark từ 9833565 đến 71a9158 (96 commit). Không có xung đột lịch sử. Bản refactor giữ các chức năng hiện hành qua API chung; không giữ mọi script sản xuất cohort đã đóng. Chưa train WM/RL, chưa đổi tập dữ liệu.

## Chức năng đã đối chiếu

| Phần trước đây | Chỗ dùng sau refactor / kết quả |
| --- | --- |
| Mecanum, LiDAR/depth/IMU, Nav2 và world | src/custom_corridor + src/custom_corridor_plugins; hai ROS package build thành công |
| SocNavGym nhóm đứng/đi, CrowdSimPlus, ComplexityNav | experiments/social_navigation/navigation; shared worker/profile và các gate giữ theo backend. IPC/physical outcomes cũ ở docs/verification/backend_acceptance_2026-10-28.json; không chạy lại physical E2E trong review này |
| Thu episode mới | tools/capture_episode.py + configs/capture; không dùng lại các cohort writer đã đóng |
| Tracker và phân biệt người/vật | experiments/social_navigation/perception + control_bundles; bundle/source bindings PASS, identity/refinement và corpus audit PASS. Không đánh giá lại model trên test |
| WM dataset01 | Canonical reader/auditor trong python/dynamuhsim; audit lại 8 episode, 5.106 windows, 5.094 hợp lệ, 6.696 depth frame đúng bit; split, test seal và nhãn giữ nguyên |
| Video/quỹ đạo replay, backup | experiments/social_navigation/replay/wm_dataset_replay; dữ liệu/Drive link và report lịch sử được giữ. Runtime root có thể truyền tường minh |
| School/HuNav và dataset tooling | 99 test PASS; không chạy HuNav simulation mới |
| Report/raw/model/freeze cũ | Report chuyển vào experiments/archive; dữ liệu ignored giữ tại benchmark/<stage>/generated. Không có thay đổi tracked trong archive sau review |

## Kiểm chứng chạy trong lần merge

- Root: 235 test, 233 PASS + 2 skip môi trường; chạy ở nhánh refactor sạch và checkout sau merge.
- School: 75 runner + 13 scenario + 11 production = 99 PASS. Lần gọi production đầu thiếu PYTHONPATH được giữ log; chạy đúng môi trường PASS.
- Stage: identity 14, refinement 13, WM contract 8, fixture 2 PASS; swept/warehouse geometry/standing reaction PASS. Projection writer không ghi đè output cũ; cohort gate không được chạy thiếu tham số.
- Audit WM và corpus19frames/132proposals PASS; control bundle PASS. Lần audit corpus thiếu ignored native library đã được chẩn đoán và chạy lại sau mapping dependency thành công; không nới ngưỡng/hash.
- Build ROS: colcon build --base-paths src --packages-select custom_corridor custom_corridor_plugins --executor sequential PASS. Phải giới hạn base-paths vì có worktree kiểm thử lồng trong experiments/generated.

## Sửa lỗi tích hợp

1. Bổ sung ignore cho generated/build trong các capability mới, bao gồm local symlink. Tạo 29 liên kết cục bộ từ đường dẫn mới tới dữ liệu/dependency ignored sẵn có; không chuyển, sao chép hay ghi lại raw. Mapping cụ thể nằm trong JSON kiểm chứng. Đây là cấu hình riêng checkout, không phải asset được mang theo bởi clone.
2. Lệnh test fixture depth-proxy vẫn import assembler đã bị retire. Bỏ import và test của producer đã đóng, giữ và chạy hai test hiện hành về mask, collision và native LiDAR. Không hồi sinh assembler hoặc bỏ kiểm tra fixture còn sử dụng.

Standing-reaction test cũ tự ghi reaction_gate.json với compute_ms mới. Kết quả mới đã được chép vào evidence ignored rồi report lịch sử được phục hồi đúng byte từ commit; không dùng rerun để sửa receipt cũ.

## Thay đổi chưa commit trước merge

Đã lưu nguyên trạng vào stash và ref cục bộ refs/local-preservation/pre-shared-runtime-merge-20261029. Ref này **không được publish**. Giữ cả index/staging, tracked worktree edits và 7 file untracked; kiểm tra OID toàn bộ 2.681 paths đã thay đổi không có mismatch. Index patch khớp chính xác; patch text worktree có khác trình bày sau đổi gitattributes, nên kiểm tra object bytes là chuẩn.

Không áp lại các thay đổi cũ lên code refactor: có cả xoá file và sửa world/robot riêng, có thể chồng lên implementation mới. Chúng còn đầy đủ trong bản khôi phục, không được đưa vào commit merge. Xem preservation fields trong JSON. Không chạy stash pop trực tiếp lên nhánh mới; khôi phục sang checkout riêng tại base khi cần lấy lại từng thay đổi.

## Giới hạn giữ nguyên

- Recipe v3 vẫn có standing47 TIMEOUT và walking48 TIMEOUT trong các lượt mở rộng đã ghi; không được xem là tất cả seed đều đạt goal.
- WSLg GUI chưa được người dùng nghiệm thu; ảnh renderer nội bộ không chứng minh cửa sổ Windows hiện ra.
- Chưa chạy mới toàn bộ Gazebo/Docker/HuNav sau merge, chưa train WM hoặc RL. Kết luận là giữ được chức năng/contract đã kiểm tra, không phải bảo đảm mọi nhánh runtime đều hoàn hảo.

Evidence: experiments/social_navigation/benchmark/generated/merge_review_20261029/ (ignored logs và snapshot cục bộ). [Kết quả có cấu trúc](verification/merge_shared_runtime_2026-10-29.json). [Hướng dẫn đang dùng](START_HERE_CURRENT.md).
