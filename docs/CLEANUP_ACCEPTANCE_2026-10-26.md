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

## Giới hạn và công việc tiếp theo

**Chưa chạy lại E2E vật lý Gazebo/Nav2 tại revision nghiệm thu này.** Build/CLI, prefix model CPU và kết quả native ngày 24/10 ở revision cũ có phạm vi riêng; không dùng chúng để đánh dấu lại success rate, an toàn vật lý hay hiển thị WSLg hiện tại.

Đợt dọn và nghiệm thu build/install/CLI được chốt tại đây. Bước kiểm chứng hành vi tiếp theo là smoke test Gazebo/Nav2 trong domain/partition và output riêng, giữ nguyên kết quả cũ.
