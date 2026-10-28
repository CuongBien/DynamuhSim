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

Nghiệm thu navigation ngày 28/10 đã thực hiện và sửa hai vấn đề teardown;
[receipt](verification/cleanup_navigation_2026-10-28.json) phân biệt 7/7 audit
dữ liệu/process PASS với 5/7 goal thật PASS. Hai ca capture còn TIMEOUT hoặc
báo SUCCEEDED khi world XY vượt ngưỡng; bước tiếp theo là chẩn đoán chức năng
AMCL/goal checker và moderate navigation, giữ nguyên các outcome đã ghi.

## Cấu trúc experiments đã xử lý ngày 28/10

Cây benchmark còn 29 thư mục code/config; 13 thư mục vỏ đã bỏ. Report/control
được chuyển sang archive, helper GPU/shutdown dùng package chuẩn. Hai alias
helper chỉ giữ nguồn freeze cũ; 23 alias reports giữ đường dẫn đọc/hydrate.
Đây là phần dọn cấu trúc còn thiếu ở lần chốt trước, đã làm gộp trong một đợt.
[Receipt layout và kiểm chứng](verification/benchmark_layout_2026-10-28.json).

## Chốt cohort/reader/auditor — 28/10/2026

Ba gói cuối đã hoàn tất: corpus depth, rà writer cohort còn lại và nghiệm thu
source/CLI/dữ liệu. [Biên bản](verification/corpus_cleanup_2026-10-28.json)
ghi 12 source bị gỡ trong lượt này; lượt trước đã gỡ 5 entrypoint WM cũ.
Không còn hạng mục xoá/gộp đã xác định đang chờ trong ba gói này.

- Corpus dùng một auditor theo sổ cuối: 132 proposal/19 frame, 21 missing-LiDAR
  giữ mask; các hàm gán nhãn/proposal dùng chung, không import exporter cohort.
- Gỡ writer portable và các script thu/seal/holdout seed43–44 một lần.
  `freeze.py` của refinement/identity giữ nguyên vì verifier còn được engine
  import và sealed source pin hash; wrapper không còn mở lệnh freeze/seal.
- Giữ target builder tổng quát, classifier train/inference, reader, tracker,
  runtime, native plugins và contract tests có consumer; snapshot/vendor/control
  giữ vai trò provenance hoặc dependency. Số thư mục không phải số việc còn lại.
- 355 test PASS; 33 CLI chính và 24 mode stage PASS. 392 Python parse và 50 shell
  syntax PASS; không thấy hàm benchmark trùng AST từ 15 dòng ngoài phạm vi
  snapshot/vendor/test đã loại. Corpus/WM/capture audits khớp toàn bộ JSON cũ;
  1.814 báo cáo và checkout/index/patch chính nguyên vẹn.

Phạm vi dọn mã đã rà được chốt. Đây không phải chứng minh mọi nhánh động đều
đã được thử. Build/native navigation trước đó giữ revision riêng; lượt này
không đổi simulator/controller hoặc tạo lại dataset. Các công việc sản phẩm
như baseline WM và hiển thị WSLg có scope riêng; không tính thành clean còn nợ.
Nhánh benchmark giữ tại `9833565`, không merge thay người dùng.

## Phần cấu trúc còn lại — cập nhật 28/10/2026

Các mục “chốt” phía trên mô tả những luồng code và batch writer đã rà, chưa phải
nghiệm thu toàn bộ cách tổ chức thư mục. Sau rà cây Git đã push, bổ sung phần
cấu trúc còn thiếu:

| Việc | Trạng thái |
|---|---|
| 1.505 file Gazebo generated trộn với input benchmark | Đã archive/restore kiểm đầy đủ; giữ 7 baseline nguyên byte tại `configs/fixtures`, chuyển 9 consumer; output mới bị ignore |
| Hai cây persona/Step1 chỉ còn report | Đã externalize 29 file; index/history và cách khôi phục được ghi riêng |
| Gom 29 stage social-navigation theo chức năng | Đã chuyển sang navigation 13 / perception 11 / datasets 4 / replay 1; chuyển thêm 13 Nav2 helper; không giữ alias thư mục code cũ |
| Nghiệm thu sau khi gom stage | Đã chạy test/CLI, build 8 target, audit đầy đủ WM/corpus, control parity và capture native/Nav2; chi tiết receipt bên dưới |

[Payload index](artifacts/closed_payloads_2026-10-28.json) và
[hướng dẫn lịch sử](history/closed_experiment_payloads.md). Không dùng số dòng
SDF/JSON/data bị bỏ để báo mức giảm code. Checkout gốc và benchmark branch
không bị merge hoặc reset.

## Gom stage theo chức năng — 28/10/2026

[Receipt cuối](verification/capability_layout_2026-10-28.json) ghi kiểm chứng cho
cấu trúc mới. Giữ cùng độ sâu thư mục để các relative root và native CMake còn
đúng; cập nhật caller chéo nhóm qua một resolver duy nhất. Các ID đã freeze
không đổi, source thực thi vẫn bắt buộc khớp hash. Không dùng bản sao cache cũ
để che file hiện hành bị thiếu hoặc bị sửa.

Đây là lượt sắp xếp code có consumer, không báo file di chuyển thành code bị
xoá hoặc cộng dồn số dòng từ generated payload. Những stage còn giữ có vai trò
riêng; raw ignored, source frozen/vendor và báo cáo nguyên bản không bị xoá chỉ
để giảm số thư mục. Các mục cấu trúc đã xác định trong bảng trên đã xử lý;
không có batch gom stage nào còn chờ trong phạm vi này. Công việc phát triển WM,
nghiệm thu thêm seed/backend và hiển thị WSLg thuộc phạm vi riêng.

## Bổ sung nghiệm thu backend sau gom stage — 28/10/2026

Đường group capture đã qua smoke ở lượt trước; lượt này khởi động worker thật
đã bắt được các chỗ sót ở backend cá nhân/Docker: đường `--common`, preflight
geometry/contract và mount protocol nằm sau alias report. Worker cá nhân nạp
kernel/motion hiện hành trước Runner đã khóa để source ignored cũ không lấn
source mới. Registry trỏ đúng implementation hiện hành. Runner, motion math,
inputs và image pins giữ nguyên; prefix số 20 bước không đổi sau chốt import.

[Biên bản backend/seed](verification/backend_acceptance_2026-10-28.json) ghi các
lỗi tái hiện trước sửa, kiểm source sạch/cache cũ và các kết quả mới. Batch
standing/walking seed47–49 có 6/6 integrity PASS nhưng chỉ 4/6 goal PASS.
Standing47 và walking48 TIMEOUT là việc điều hướng cần chẩn đoán tiếp; không
được dùng audit dữ liệu hoặc cleanup làm bằng chứng rằng mọi goal đã đạt.
Đợt này không điều chỉnh controller, budget, threshold hoặc dataset/test seal.
