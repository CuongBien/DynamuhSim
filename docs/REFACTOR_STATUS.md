# Trạng thái refactor/clean

**Phạm vi:** source project hiện hành trên nhánh `codex/shared-navigation-runtime`.
**Trạng thái ngày 29/10/2026:** hoàn tất checklist refactor/clean đã rà trên source hiện hành.
Bản source sạch đã qua các kiểm tra ghi trong biên bản. Không tính xử lý lỗi goal,
train model hay thêm tính năng là việc clean còn lại.

| Checklist | Kết quả |
|---|---|
| Kiểm kê source tự quản toàn repo | 459 file; mỗi file có vai trò được ghi trong inventory kiểm chứng |
| Gỡ các chuỗi cohort đã được API mới thay thế | Dataset pilot, depth-proxy lifecycle, early-avoidance đã gỡ trọn cây code |
| Gom phần còn dùng | Một normalizer/contract, một auditor normalized/portable, một parser SDF, một bộ dựng lệnh worker |
| Giữ giao diện và logic | Golden sáu command worker, math/source pins, contract và audit dữ liệu |
| Loại dependency vào code trong bundle | Reader/auditor dùng source chuẩn; bundle tools chỉ còn là bằng chứng byte |
| Rà source còn giữ và cập nhật entrypoint | Không còn mục source chưa phân loại hoặc caller code trỏ tới ba cây đã gỡ |
| Kiểm chứng và bảo toàn | Tests, CLI, build wheel, native parser, ba dataset audits; receipts/models/seals và checkout gốc được đối chiếu |

Đợt chốt này giảm ròng **1176 dòng source**, tính cả tests,
không tính JSON/YAML/data/media và không tính file di chuyển là code bị xoá.
Social-navigation còn 26 stage có code: navigation 12, perception 10, datasets 3,
replay 1. [Các chuỗi đã nghỉ dùng](history/retired_capture_pipelines.md).

## Vì sao vẫn còn các folder/file tên tương tự?

- `src/`: ROS launch, resource generators, plugins và School runtime có consumer/build riêng.
- Gazebo suites và trajectory V2: các phép đo/collector hiện hành có contract khác nhau.
- Navigation/perception: adapter/backend, thuật toán và fixture khác nhau; wrapper mỏng giữ CLI/source identity, implementation chung ở `python/dynamuhsim/`.
- Reader/auditor, model math và source đã freeze: còn nằm trong dependency/source-hash closure. Ví dụ `common_response/model_worker.py` được worker đọc đúng hash và dùng Runner class; file verifier của tracker giữ nguyên byte theo engine pins.
- Vendor/RVO2/DR-SPAAM, source snapshots và report/model artifacts: dependency/bằng chứng có chủ đích, không phải các bản implementation đang phát triển song song.
- CLI corridor và general navigation được giữ vì có option/metric riêng; không hợp nhất các định nghĩa khác nhau chỉ để giảm tên file.

Việc rà AST/consumer và kiểm chứng trên đây là tiêu chí chốt refactor, không phải
khẳng định mọi nhánh động đều đã được chạy hay mọi phần mềm không thể refactor
thêm trong tương lai. [Biên bản kiểm chứng cuối](verification/refactor_completion_2026-10-29.json).

[Inventory source: vai trò, số dòng và SHA256 của từng file](verification/refactor_source_inventory_2026-10-29.csv).
