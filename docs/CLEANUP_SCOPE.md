# Phạm vi clean đã chốt — 27/10/2026

Đợt rà toàn source Git hiện hành đã kiểm consumer import/path/hash của 60 harness
Gazebo/persona/pipeline, đối chiếu nhánh benchmark `9833565` và source trước dọn
`82870ee`. Đã nghỉ dùng 35 file source, 10.729 dòng: 33 Python và hai C++.
Đây là code được xoá khỏi HEAD; dataset/media không được tính vào số dòng giảm.

| Nhóm đã nghỉ dùng | File | Lý do |
|---|---:|---|
| A3.3 đời đầu, advanced/paired analysis và plot/leakage cohort cũ | 6 | Cohort hoàn thành; bài A3, replay/reset và MPPI hiện hành được giữ |
| Datagen 224-trial và report/plot riêng | 3 | Bộ đo cũ đã đóng; P1–P4 trong core và P4 CPU test được giữ |
| MPPI plot tổng hợp cũ | 1 | Figure/report đã lưu; bốn runner ablation vẫn giữ |
| Pilot300 generator/evaluator/recalculation/replay | 7 | Cohort cố định đã đóng, không là producer của WM hiện tại |
| Scaling/cold-start/uncapped/storage harness cũ | 7 | Giữ năm công cụ đo còn tái sử dụng và core profiler |
| Persona oracle/observability generator/evaluator | 8 | Không có consumer ngoài nhóm hoặc build/freeze hiện hành |
| LiDAR pipeline Step1 | 3 | Harness cohort đầu đã hoàn thành |

Không còn caller ngoài nhóm bị xoá, không có source pin hiện hành bị bỏ.
Tham chiếu trùng tên `generate_markdown_report` trong A4 là hàm cục bộ, không
import script bị xoá. Source lịch sử vẫn ở
[revision đã ghim](https://github.com/CuongBien/DynamuhSim/tree/82870eea0e351427c25f6764b4c1bc2f145e676e/experiments). Báo cáo, figures, labels, split,
model, test seal và các outcome đã ghi giữ nguyên byte.

Tại commit `f856a36`, toàn nhánh so với `test/gazebo-harmonic-benchmark` ở `9833565`: source project
**+9.619 / −30.223, giảm ròng 20.604 dòng**. Phạm vi này loại data/config/media,
source copy đã freeze trong report/result/control bundles, vendor và gitlinks.

## Phần chủ động giữ lại

| Khu vực | Vai trò |
|---|---|
| `python/dynamuhsim/`, `tools/`, `evaluation/` và baseline root | Runtime/API, phân tích, artifact/hydration và đánh giá hiện hành |
| `src/` cùng School scenario/dataset runner/production | Robot, world/assets, native plugins và luồng School |
| Gazebo `core/` và 25 script suite | A3 kiểm tra/reset/replay; MPPI; P4; scaling/sensor; hai demo |
| Social adapters/common workers, projection, native kernels, groups | Runtime và test/hook còn có consumer |
| Capture API và audit policies | Episode mới, provenance, recorder/TF/friction/tail và goal audit |
| WM/depth/classifier/lidar/portable pipelines đã freeze | Source/provenance/export/reader/auditor còn trong dependency closure |
| `trajectory_risk_v0` | Collector V2/QA hiện hành; tên thư mục không phải bằng chứng hết dùng |
| Compatibility entrypoints mỏng | Giữ lệnh theo stage; implementation đã gom về module chung |
| Vendor, control bundles, source snapshots, archived receipts | Dependency hoặc bằng chứng bất biến; các bản sao này có chủ đích |

Các evaluator, reset/replay và phép đo sensor còn giữ có phương pháp riêng.
Đợt clean harness cũ được chốt ở phạm vi này; thay thuật toán, controller,
threshold hoặc nguồn dữ liệu cần một công việc phát triển/kiểm chứng riêng.

[Biên bản kiểm chứng](CLEANUP_ACCEPTANCE_2026-10-26.md) ghi kết quả và giới hạn.

## Chốt rà soát sau refactor launch — 27/10/2026

Đã rà 501 file source tracked (430 Python, 50 shell, phần còn lại C++),
16 đường dẫn registry và 13 nhóm file trùng byte. Không thấy import project
hoặc đường dẫn Python được phân giải tĩnh nào còn gọi source đã xoá. Không có
hàm trùng AST từ 15 dòng ngoài các engine/upstream snapshot đã ghim.
Bản sao frozen/vendor, loader portable và entrypoint tương thích mỏng có vai trò
cụ thể, được giữ; đây không phải bằng chứng mọi nhánh động đều đã được phủ.

Rà dependency phát hiện và sửa 11 khai báo ROS runtime bị thiếu, cùng các tên
dependency native chưa đúng rosdep Jazzy. `rosdep check`, build mới hai package,
282 tests, 33 CLI và kiểm tra installed runtime đều PASS; mã máy `.text` và
thư viện liên kết của cả bốn plugin giống bản trước sửa. Tổng source đã giảm
ròng **21.289 dòng** so với `9833565`, theo phạm vi ghi trong
[receipt chốt cleanup](verification/cleanup_closure_2026-10-27.json).

Phần cleanup trong các luồng đã rà được chốt tại đây. Việc tiếp theo là acceptance
điều hướng end-to-end và chẩn đoán các ca đã FAIL/TIMEOUT, thuộc kiểm chứng chức
năng riêng; không sửa kết quả cũ hoặc merge vào nhánh benchmark.
