# Nghiệm thu đợt dọn repository — 26/10/2026

**Đạt build, install, CLI và các phép đối chiếu đã liệt kê.** Code được nghiệm thu tại `91dae28bb990769a3330852d9173101d8066138b`, trên nhánh `codex/shared-navigation-runtime`.

[Biên bản gọn và hash bằng chứng local](verification/cleanup_acceptance_2026-10-26.json).

## Kết quả dọn

Từ mốc `9f4a9ef` đến revision code trên: **62 file thay đổi, +748/−4.710 dòng, giảm ròng 3.962 dòng và bỏ 14 file**. Số này tính cả test và tài liệu, chưa tính chính biên bản này. Đó là phần dọn sau khi chốt ưu tiên giảm code; không dùng số dòng dataset/media đã externalize làm số dòng code được giảm. Lịch sử Git vẫn được giữ.

Đã bỏ các collector V1, pilot/diagnostic đã hoàn tất, wrapper canary và bản sao C++ không được build; gộp vòng IPC của bốn worker, lifecycle tiến trình, helper telemetry/CSV và các entrypoint tương đương. Danh sách file bị bỏ có thể xem bằng:

```bash
git diff --diff-filter=D --name-only 9f4a9ef 91dae28
```

Các `worker.py`/`trial.py` tương thích còn giữ tên ở stage để lệnh cũ tiếp tục dùng được; implementation chung được chỉ ra tại [runtime Nav2](SHARED_NAV2_RUNTIME.md) và [registry](experiment_registry.json).

## Các lỗi đóng gói được xử lý khi nghiệm thu

- Cài `tools/process_runtime.sh` cạnh helper School, dùng chung đường resolve trong source checkout, installed share và container bundle. Trước sửa, probe installed School thất bại vì thiếu helper.
- Loại bytecode/cache Python khỏi install; prefix copy trước sửa có 50 file cache. Audit `--fresh` phát hiện cache trong prefix mới; audit mặc định vẫn dùng được sau khi Python tự sinh cache khi chạy.
- Bổ sung 11 runtime dependencies mà launch thực sự dùng; audit kiểm tra helper được cài đủ và đúng Git blob.

## Phạm vi kiểm chứng

| Kiểm tra | Kết quả |
|---|---|
| Bộ test hiện tại | 277 đạt: root 178, scenario 13, dataset runner 75, production 11 |
| Build ROS2 Jazzy | Hai package trong prefix symlink mới; thêm bản copy asset riêng |
| Asset audit theo Git revision | Cả hai kiểu install đạt; mỗi kiểu 214 asset, trong đó 169 world file, tổng 17.462.878 byte |
| Installed runtime | Năm launch `--show-args`, ba robot description, bốn plugin đủ dependency, ba executable |
| School installed share/container bundle | Hai kiểu install × ba chế độ installer, Docker được mock; helper được chạy bằng tiến trình CPU |
| CLI chính | 27 lệnh help đạt, chặn ROS init và subprocess ngoài |
| Bảo toàn | 1.814 control/model/snapshot giữ nguyên; checkout và index chính không đổi |

Các phép đối chiếu ở những đợt trước gồm 2.240 ca CSV, 192 cặp worker IPC, 10 cặp prefix model thật (sáu cặp trong image Python3.8 đã pin), và các ca ngắt/timeout/flush tiến trình. Kết quả chi tiết nằm trong các thư mục `benchmark/generated/*cleanup_20261026/` tại máy; Git giữ biên bản gọn.

Lệnh build/audit để lặp lại nằm ở [WORLD_ASSETS](WORLD_ASSETS.md). Dùng prefix mới và `PYTHONDONTWRITEBYTECODE=1` cho smoke check trước audit `--fresh`; CMake không xóa cache hay payload đã tồn tại trong prefix cũ.

## Những phần giữ lại có chủ đích

Report, snapshot, source đã freeze, mô hình/heldout/seal và thư viện upstream phục vụ tái lập lịch sử. Các evaluator hoặc suite khác phương pháp vẫn giữ protocol riêng. World `arena_dataset_bu.sdf` dùng MultiHuman khác world RVO2 hiện tại nên vẫn là đầu vào hợp lệ; xem hướng dẫn plugin. Các dữ liệu, split, nhãn, threshold và kết quả cũ giữ nguyên.

## Giới hạn tại thời điểm nghiệm thu build/install/CLI

**Chưa chạy lại E2E vật lý Gazebo/Nav2 tại revision nghiệm thu này.** Build/CLI, prefix model CPU và kết quả native ngày 24/10 ở revision cũ có phạm vi riêng; không dùng chúng để đánh dấu lại success rate, an toàn vật lý hay hiển thị WSLg hiện tại.

Đợt dọn và nghiệm thu build/install/CLI được chốt tại đây. Bước kiểm chứng hành vi tiếp theo là smoke test Gazebo/Nav2 trong domain/partition và output riêng, giữ nguyên kết quả cũ.

## E2E bổ sung trên revision 838a76a

[Biên bản và hash bằng chứng E2E](verification/cleanup_e2e_2026-10-26.json). Năm lượt headless thực trên ROS2 Jazzy/Gazebo, NVIDIA D3D12, prefix đã nghiệm thu, output/HOME/cache riêng. Giữ native dt, physics 0,0025s và budget 45s; không thay controller, threshold, model hoặc dữ liệu cũ.

| Ca cố định | Nav2 action | XY thật (m) | Yaw thật (rad) | Đạt đích vật lý |
|---|---|---:|---:|---|
| Không người | SUCCEEDED | 0,118 | 0,032 | Có |
| SocNavGym riêng lẻ | SUCCEEDED | 0,150 | 0,239 | Có |
| CrowdSimPlus ORCA+ | SUCCEEDED | 0,160 | 0,079 | Có |
| ComplexityNav geometry | SUCCEEDED | 0,145 | 0,198 | Có |
| Nhóm đứng seed42 | SUCCEEDED | 0,221 | 0,218 | **Không** |

Cả năm vượt audit độc lập về source/config, 42 AABB, pose/sensor/control streams, costmap, geometry và cleanup; các ca có người giữ start gate/native timestep; không ghi nhận overlap lớn hơn 5mm theo proxy/disc của phép đo. Các khoảng cách này không phải chứng nhận an toàn. Bốn ca đạt ngưỡng đích thật; ca nhóm đứng giữ nguyên thất bại XY>0,20m, không chạy lại để thay kết quả.

Audit phát hiện analyzer cá nhân dùng pose cuối stream sau teardown thay vì mẫu đã chốt trong result. Đã sửa theo cách hai analyzer shared/group đang dùng: tìm đúng pose được ghi trong phase active, giữ toàn bộ assertion cũ và ngưỡng, thêm hồi quy cho pose đạt/không đạt và thiếu mẫu. **278 test đạt.** Dữ liệu physics không chạy lại sau sửa analyzer.

Môi trường phân tích dùng venv có Shapely; wrapper audit giữ nguyên các assertion, chỉ thay checkpoint sang snapshot mới và khôi phục import path của stage. Hai thay đổi offline (analyzer/test) được ghi hash riêng; snapshot đầu, lỗi chuẩn bị và audit thất bại đầu tiên vẫn được giữ tại máy.

Với nhóm đứng, AMCL gần thời điểm chốt có sai số tới goal 0,190m; pose Gazebo cùng timestamp là 0,228m, chênh AMCL–Gazebo khoảng 0,048m. Đây là gợi ý điều tra localization/goal-checker, chưa khẳng định nguyên nhân duy nhất. Goal checker vẫn giữ XY=0,20m và yaw=0,25rad.

Phần dọn và smoke E2E được chốt với kết quả hỗn hợp trên. Công việc hành vi tiếp theo là phân tích ca nhóm đứng từ bản ghi hiện có, tách lệch localization, stateful goal-checker và chuyển động sau action; mọi thay đổi controller cần một kế hoạch thử nghiệm mới.
## Gỡ workflow lifecycle/demo đã hoàn tất

Đối chiếu nhánh đã push `test/gazebo-harmonic-benchmark` tại `9833565` và consumer hiện hành, đã **xoá 23 file, 1.358 dòng code/build** khỏi HEAD:

| Workflow lịch sử | File bị xoá | Phần giữ lại |
|---|---:|---|
| `socnavgym_lifecycle` (`2480158`) | 3 | README chỉ tới mã và báo cáo lịch sử |
| `socnavgym_lifecycle_fix` (`779c338`) | 5 | `continuous_groups.py`, regression swept geometry |
| `socnavgym_warehouse_lifecycle` (`b3f0022`) | 11 | kernel/motion/geometry/config và regression geometry |
| `group_gazebo_demo` (`e749fb5`) | 4 | README chỉ tới mã/báo cáo cũ và lệnh Nav2/replay hiện tại |

Gỡ cả chuỗi runner/worker/launch/analyzer đã đóng; không chuyển sang một thư mục legacy hoặc thêm wrapper. Preflight `offline02` bị gỡ cùng harness velocity-script của nó; preflight response/Nav2 hiện hành giữ nguyên. Các README từng stage dẫn tới code trước khi gỡ tại `270330e` và commit thí nghiệm gốc. Tái lập lịch sử cần source revision/manifest của kết quả đó.

**Kiểm chứng:** 278 test đạt (179 root, 13 scenario, 75 runner, 11 production), thêm lần chạy lại sáu test measurement sau khi thu hẹp wiring test về hai analyzer còn tồn tại; 27 CLI đạt với ROS init/subprocess bị chặn; swept geometry, warehouse geometry, current preflight và control/classifier probe đạt. Các fixture số đo lifecycle vẫn được kiểm tra; chỉ bỏ wiring tới analyzer đã xoá. Log thiếu dependency/import path ở lần chọn môi trường đầu được giữ; lần chạy phù hợp không cần cài thêm thư viện.

273 hash nguồn từ năm lượt E2E ở trên, gồm 70 file riêng biệt, vẫn khớp; không chạy lại physics trong đợt chỉ xoá harness này. 1.814 control/model/snapshot và checkout/index chính giữ nguyên. Bằng chứng local tại `benchmark/generated/retired_lifecycle_cleanup_20261026/`.

Bước dọn tiếp theo: rà chuỗi cohort localization/stability đã hoàn tất, phân biệt runner/analyzer chỉ còn dùng với report lịch sử và các helper `profiles`/geometry/capture vẫn được benchmark hiện hành gọi. Điều tra ca nhóm đứng ở mục trước vẫn còn là công việc hành vi riêng; chưa chỉnh controller hoặc nới ngưỡng.

## Gỡ chuỗi cohort localization/stability đã hoàn tất

Đối chiếu cùng nhánh benchmark đã push tại `9833565`, đã **xoá 21 file, 1.553 dòng code** của chuỗi nghiên cứu cũ; không thêm wrapper hoặc chuyển code sang thư mục legacy:

| Stage / commit gốc | File bị xoá | Phần giữ lại ở HEAD |
|---|---:|---|
| `localization_diagnosis` / `1186e1d` | 7 | README chỉ tới source và report lịch sử |
| `navigation_diagnostics` / `e8bf1c0` | 3 | `capture.py` còn được benchmark trial gọi |
| `stability` / `25e4157` | 5 | README chỉ tới source và report lịch sử |
| `localization_trials` / `d0a7bbb` | 3 | `profiles.py` còn được trial gọi; reference pin cũ |
| `localization_validation` / `a2ef6af` | 3 | Fixture receipt và README lịch sử |

Gỡ trọn chuỗi runner/analyzer/plot/validator cùng native-score, raycast và clustering helper không còn consumer. Hai helper runtime capture/profiles, `benchmark/trial.py`, footprint geometry và baseline YAML giữ nguyên byte. Các README dẫn tới code trước khi gỡ tại `e150d6b`; HEAD vẫn ghi diagnostics nhưng không còn analyzer chuyên biệt của các cohort cũ. Tái lập phân tích lịch sử cần revision/manifest/raw input của chính cohort đó. Không sửa kết quả 2/4, 5/6 hoặc thay ABORTED/TIMEOUT bằng lượt mới.

**Kiểm chứng:** 278 test đạt (179 root, 13 scenario, 75 runner, 11 production); 28 CLI đạt, gồm benchmark trial, với ROS init/subprocess bị chặn. Đối chiếu helper capture trước/sau bằng 16 callback message ROS thật và hai graph snapshots mỗi phiên bản trên tám topic: payload khớp, grid nén không mất dữ liệu, GID thiếu vẫn null. AMCL có 12 ca configure và 45 ca service readback mỗi phiên bản, gồm 36 ca lỗi; output/exception khớp và client luôn được dọn. Các probe dùng node/client giả, không khởi tạo ROS hoặc chạy physics.

273 hash nguồn của năm lượt E2E trước, gồm 70 file riêng biệt, vẫn khớp; 1.814 control/model/snapshot và checkout/index chính giữ nguyên. Bằng chứng local tại `benchmark/generated/retired_localization_cleanup_20261026/`. Chưa chạy thêm physics hoặc chỉnh controller/ngưỡng trong đợt này.

Bước dọn tiếp theo: kiểm tra runner/probe contract và geometry ComplexityNav cùng các offline replay đã đóng; giữ kernel/roadmap/input mà worker hiện tại còn dùng. Điều tra hành vi ca nhóm đứng vẫn là việc riêng sau phần dọn.
