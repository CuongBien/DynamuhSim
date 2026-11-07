# Quy tắc làm việc trong NarrowNav Research

## Đọc trước khi sửa

1. Đọc [README](README.md), [ranh giới module](docs/architecture/01_MODULE_BOUNDARIES.md), [luồng dữ liệu](docs/architecture/02_DATA_FLOW.md) và [quy tắc phụ thuộc](docs/architecture/03_DEPENDENCY_RULES.md).
2. Đọc [schema state/action](docs/schemas/STATE_AND_ACTION_SCHEMA.md), [schema episode](docs/schemas/EPISODE_SCHEMA.md) và giao thức liên quan trong `docs/protocols/` trước khi thiết kế interface hoặc thực nghiệm.
3. Đọc [ADR](docs/decisions/ADR-0001-mujoco-only.md), đối chiếu [lộ trình](docs/ROADMAP.md) và kiểm tra nhánh, index, working tree. Phân biệt mã đang có với thiết kế dự kiến. Giữ nguyên mọi thay đổi ngoài phạm vi task.

## Module và nơi đặt tệp

| Vị trí | Trách nhiệm |
| --- | --- |
| `src/narrow_nav/core/` | Kiểu dữ liệu, validation dùng chung và interface độc lập với engine. |
| `configs/` | YAML có phiên bản schema. Các nhóm Hydra hiện có là `simulation/`, `worlds/`, `scenarios/`, `humans/`, `robots/` và `experiments/`. Thêm nhóm sensor khi triển khai cảm biến. |
| `sim/mujoco/` | MJCF world, model, sensor và asset. Đây là cây tài nguyên, không có Python. |
| `src/narrow_nav/simulation/` | Điều phối reset/step, giải cấu hình mô phỏng, giới hạn lệnh, phân tuyến dữ liệu và adapter MuJoCo tại `simulation/mujoco/`. |
| `src/narrow_nav/humans/` | Backend điều hướng người theo interface của `core`. Mỗi backend đề xuất chuyển động và duy trì trạng thái riêng. |
| `tests/` | Kiểm thử đơn vị, tích hợp, mô phỏng và hồi quy. Fixture nhỏ, có nguồn gốc, nằm dưới `tests/fixtures/`. |
| `scripts/simulation/`, `experiments/` | Entry point lắp ghép module và định nghĩa thí nghiệm. Đặt mã dùng lại trong package và YAML cấu hình dưới `configs/`. |
| `runs/<run_id>/` | Output sinh ra theo lần chạy, gồm cấu hình đã resolve, manifest, log và kết quả thô. |

Xem hợp đồng từng module và vị trí của module tương lai trong [ranh giới module](docs/architecture/01_MODULE_BOUNDARIES.md).

## Kiến trúc và dữ liệu

- `core` là tầng thấp nhất. `simulation` và `humans` dùng interface từ `core`. Entry point tạo backend và adapter rồi truyền vào bộ điều phối. `configs/` và `sim/mujoco/` cung cấp dữ liệu qua bộ nạp, không tham gia đồ thị import Python.
- Chỉ adapter `simulation/mujoco/` trong mã sản phẩm được dùng MuJoCo API và đối tượng native. Controller nhận kiểu dữ liệu của `core`. Cấm truyền model, data, chỉ số joint/geom hoặc buffer sống của engine qua interface.
- Mỗi run đối chứng độc lập chọn đúng một backend human navigation cho tất cả người. Chỉ phối hợp thuật toán trong backend hybrid được khai báo thành một điều kiện thí nghiệm riêng, với quy tắc lựa chọn hoặc phối hợp ghi trong cấu hình.
- Tách `Observation` và `GroundTruthState` ở cả kiểu dữ liệu và kênh lưu trữ. Chỉ cấp `Observation` từ luồng cảm biến cho robot policy và các module dùng lúc suy luận. Context mô phỏng dành cho backend người, nhãn ngoại tuyến và số đo kiểm định có đường truy cập riêng.
- Dùng `snake_case` cho tệp/module Python, YAML, hàm, field schema và ID cấu hình. Dùng `PascalCase` cho class/interface và hậu tố mô tả vai trò như `State`, `Policy`, `Backend` hoặc `Adapter`. Tên tệp tài liệu theo các nhóm hiện có trong `docs/`. Mỗi hợp đồng trao đổi công bố `schema_version` và đơn vị/frame cho trường vật lý.
- Thay đổi interface phải cập nhật schema, test của bên cung cấp và bên gọi, và tài liệu trong cùng thay đổi. Ghi ADR trong `docs/decisions/` trước khi đổi ranh giới, quyền truy cập dữ liệu, hướng phụ thuộc hoặc engine.
- Chỉ thêm dependency khi cần cho task hiện tại và dependency sẵn có không đáp ứng. Ghi mục đích, module sở hữu và phiên bản dùng để tái lập. Tách dependency của module tương lai khỏi lõi mô phỏng.

## Kiểm chứng và bảo vệ dữ liệu

- Chạy kiểm tra liên quan từ entry point chính thức, ghi đúng lệnh và kết quả. Báo riêng kiểm tra lỗi, bị bỏ qua hoặc chưa có. Với thay đổi chỉ gồm tài liệu, kiểm tra liên kết, sơ đồ, hợp đồng liên tài liệu và diff.
- Trước khi kết thúc, kiểm tra `git status --short`, `git diff --check` và nội dung `git diff`. Đọc cả nội dung tệp untracked thuộc task vì `git diff` mặc định bỏ qua chúng. Chỉ stage/commit khi được yêu cầu và chỉ chọn tệp liên quan. Viết commit message nêu thay đổi có thể kiểm chứng.
- Không sửa, xóa hoặc lấy lại mã và asset từ nhánh khác để làm nền cho task. Bảo vệ thay đổi sẵn có của người dùng và các gitlink. Không đưa secret, dữ liệu cảm biến thô, log, output, checkpoint hay dữ liệu nhận dạng cá nhân vào Git. Chỉ đưa fixture nhỏ đã được rà soát vào test.
- Mỗi episode ghi `episode_id`, `run_id`, `episode_seed`, phiên bản schema, commit và trạng thái dirty, cấu hình đã resolve cùng fingerprint, phiên bản phần mềm/backend/MuJoCo, fingerprint MJCF/asset và kết quả kết thúc. Tách các luồng ngẫu nhiên theo mục đích theo [schema episode](docs/schemas/EPISODE_SCHEMA.md).
