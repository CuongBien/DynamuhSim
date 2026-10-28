# Gazebo benchmark — phần hiện hành

`core/` giữ world/scenario, policy, geometry, process scope, telemetry và profiler.
Các suite còn lại gồm A3 velocity/repeatability/Nav2/replay/reset, MPPI Step1/2/3
và wall clearance, P4 CPU filter, năm phép đo scaling/controller/cold-start/sensor,
cùng hai demo tương tác corridor/hospital. Chạy từ root checkout; xem
[entrypoint chính](../../README.md) và [phạm vi clean](../../docs/CLEANUP_SCOPE.md).

Các cohort A3.3 đời đầu, 224-trial datagen, pilot300, plot/report generators và
các harness scaling cũ đã nghỉ dùng. `results/`, `reports/` và source lịch sử
vẫn giữ kết quả/phương pháp đã đo; tái lập cần đúng source và input của lượt đó:
[source trước khi nghỉ dùng](https://github.com/CuongBien/DynamuhSim/tree/82870eea0e351427c25f6764b4c1bc2f145e676e/experiments/gazebo_benchmark).

## Input cố định và output sinh mới

Bảy baseline SDF ở `configs/fixtures/` được giữ nguyên byte từ snapshot cũ;
chín consumer A3/reset/replay/A4/cold-start đã chuyển sang đây và resolve đúng
checkout. `configs/generated/` chỉ là output runtime bị gitignore, được các
generator tạo khi chạy. 1.505 output cũ và hai cây experiment chỉ còn report
được lưu theo [index lịch sử](../../docs/history/closed_experiment_payloads.md).
Không cần hydrate snapshot cũ để chạy generator hoặc đọc baseline hiện hành.
