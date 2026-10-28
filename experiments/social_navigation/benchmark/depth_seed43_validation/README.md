# Kiểm chứng depth tracker trên seed mới 43

Đã thu **một recording thật mới**, seed 43, bằng Gazebo/NVIDIA. Thu dữ liệu và audit đạt; **navigation TIMEOUT và gate tracking kết hợp FAIL**, không đổi kết quả thành pass. [Báo cáo](../../../archive/social_navigation/reports/depth_seed43_validation/SEED43_REPORT.md), [acceptance](../../../archive/social_navigation/reports/depth_seed43_validation/acceptance.json), [audit raw](../../../archive/social_navigation/reports/depth_seed43_validation/raw_audit.json), [metric audit](../../../archive/social_navigation/reports/depth_seed43_validation/audit01/audit.json), [ảnh dữ liệu thật](../../../archive/social_navigation/reports/depth_seed43_validation/seed43_comparison.png), [GIF Gazebo thật](../../../archive/social_navigation/reports/depth_seed43_validation/capture/camera_preview_4x.gif).

- 1051 ảnh depth khớp raw từng byte; 802 cửa sổ từ một episode, split internal_validation_only.
- Frozen inference trước capture, không tune theo seed 43. Shape-only/combined giữ đủ 1983 paired rows, giảm ứng viên chưa ghép. Combined giảm unpaired 900→268, paired ratio 68.78→88.09%.
- Fragmentation proxy 9→9, một lần đổi GT ghép hình học vẫn còn; chưa chứng minh cải thiện re-ID. Gate yêu cầu giảm fragmentation nên **FAIL**.
- Robot timeout 90.025 sim s, sai goal 3.7013 m; không sửa Nav2 hoặc chạy lại để lấy kết quả đẹp. Semantic/association/H1–H5/training masks vẫn false.

## Thu và tái kiểm chứng

Capture dùng Ubuntu 24.04 + ROS 2 Jazzy, Python 3.12 và venv SocNavGym hiện có. GPU preflight và actual Ogre log đều là NVIDIA RTX 4060 D3D12; giữ startup preload/ordered pause-stop theo parent. Không mở GUI ở lượt này; ảnh/GIF do camera observer Gazebo ghi thật. Physics/Nav2 vẫn dùng CPU cho phần việc của chúng; không tuyên bố toàn bộ mô phỏng chạy GPU.

Hai script capture/preflight một lần đã gỡ sau khi recording seed43 đóng;
[source trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/3b133626809a39991babe0e6fcc52d2572c2c03b/experiments/social_navigation/benchmark/depth_seed43_validation)
giữ driver preparation và nguồn chính xác. Không thay recording hoặc TIMEOUT.

```bash
TASK=experiments/social_navigation/benchmark/depth_seed43_validation
bash "$TASK/run.sh" capture-audit --artifact-root /PATH/TO/RECORDED_CHECKOUT --output "$TASK/generated/my_capture_audit"
```

Artifact root chứa raw/runtime/driver đã ghi; implementation auditor chạy từ
checkout hiện hành. Helper sources đối chiếu SHA256 từ artifact root hoặc Git
`9833565`, không thực thi source lấy từ lịch sử. Episode mới thu qua
`tools/capture_episode.py` với protocol/output mới.

Bộ artifact đã đo vẫn giữ `normalized/`, frozen engine, sensor replay và evaluation. `capture_audit.py` và `audit.py` còn tái kiểm bộ artifact cố định; runner so sánh/source extension, diagnostic, preview và công cụ chốt report của cohort đã hoàn tất được gỡ khỏi HEAD. Tái lập chuỗi cũ dùng source revision và inputs đã ghi, không ghi lại outcome cũ. Raw MCAP/NPY/JSONL lớn vẫn ở generated/ và không thay đổi.

Dữ liệu mới kiểm được việc đổi seed trong cùng layout/simulator, chưa chứng minh qua layout/backend. GT chỉ ở nhánh label/diagnostic; không đưa vào tracker hoặc suy H1–H5. HuNavSim chưa dùng. Nếu dùng case seed 43 để tune tracker ở bước sau, nó trở thành regression/development cho bản mới; cần seed chưa xem khác cho validation tiếp theo.

Bước kế: xử lý active matching khi quan sát thiếu/ghép mơ hồ bằng sensor-only, dùng case đổi pairing GT5→GT6 làm regression. Chưa thu batch hoặc train; baseline cũ vẫn giữ. Navigation timeout là kết quả riêng chưa được sửa trong lượt perception này.

Cập nhật dọn 27/10/2026: đã gỡ các công cụ chỉ phục vụ cohort/report cũ: `diagnose_identity.py`, `finalize_reports.py`, `preview.py`, `compare.py`. [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/87b0c6eea9b54794db7b4cb794fae57c78f11c70/experiments/social_navigation/benchmark/depth_seed43_validation) giữ cách tái lập; report, engine, source freeze và outcome cũ không đổi.
