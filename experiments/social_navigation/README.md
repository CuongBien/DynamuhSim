# Social navigation — các đường chạy hiện tại

Code được chia theo [navigation](navigation/README.md), [perception](perception/README.md), [datasets](datasets/README.md) và [replay](replay/README.md). 29 stage giữ hợp đồng riêng; implementation tái sử dụng nằm trong `python/dynamuhsim/`. `benchmark` chỉ giữ shell runtime chung và alias source lịch sử.

[START_HERE_CURRENT](../../docs/START_HERE_CURRENT.md) hướng dẫn đọc dataset WM, masks và replay. Đây là điểm bắt đầu cho công việc WM hiện tại; không cần chạy lại các vòng khảo sát simulator ban đầu.

| Công việc | Entry point / hướng dẫn |
|---|---|
| Đọc và xem dataset01 | [WM workflow](../../docs/START_HERE_CURRENT.md) |
| Nav2 với individual/group hoặc backend đã pin | [Shared Nav2 runtime](../../docs/SHARED_NAV2_RUNTIME.md), `tools/nav2_trial.py` |
| Thu một episode mới | [Capture API](../../docs/CAPTURE_RUNTIME.md), `tools/capture_episode.py` |
| Audit capture và artifact | [Analysis/artifacts](../../docs/ANALYSIS_AND_ARTIFACTS.md) |
| Classifier/control đã freeze | [Sensor controls](../../docs/SENSOR_CONTROLS.md) |
| Ghi thêm global plan/costmap/particle cloud | [Navigation capture](navigation/navigation_diagnostics/README.md) |
| Input/preflight cho native worker | [Common response](navigation/common_response/README.md) |

Dataset pilot có 8 episode được chấp nhận và 2 lượt cách ly; WM chưa được train/nghiệm thu. Nhãn H1–H5/association/RL reward còn bị mask. Dùng train để fit, validation để chọn cấu hình và giữ test theo protocol. Full dataset/raw không đi kèm `git clone`; hướng dẫn backup/restore nằm ở workflow trên.

## Chuẩn bị runtime

- Vendor/venv SocNavGym: `setup_socnavgym.sh` và `requirements-socnavgym.txt` còn được giữ; dùng trên checkout/môi trường cần chuẩn bị, không chạy chỉ để đọc dữ liệu offline.
- Image CrowdSimPlus/ComplexityNav: [round2](round2/README.md), [round3](round3/README.md). Các script đó chỉ build khi truyền `build`; runtime hiện hành vẫn ghim content ID riêng.
- Warehouse assets và ROS launch: `generate_warehouse.py`, `warehouse/`; hướng dẫn install/build ở [WORLD_ASSETS](../../docs/WORLD_ASSETS.md).

Launcher warehouse vẫn giữ default và launch arguments:

```bash
bash experiments/social_navigation/run.sh warehouse gui:=true
```

Cần ROS Jazzy và overlay của đúng checkout đã build. Hình ảnh/log headless không xác nhận cửa sổ WSLg đang hiển thị trên màn hình.

## Các vòng khảo sát đã kết thúc

Runner/analyzer/report writer SocNavGym ban đầu, verifier scene cũ, scene/matrix của round2/round3 và 54 config phát sinh đã được gỡ khỏi HEAD. Các mode `run.sh socnavgym`/`verify` của thư mục này cũng được gỡ; các công cụ kiểm định adapter đời đầu cũng đã nghỉ dùng. Adapter SocNavGym giữ `build`/`demo`; native worker và nguồn fixture hiện hành vẫn được giữ.

[Khảo sát ban đầu](https://github.com/CuongBien/DynamuhSim/commit/91f36207e0b7250b36e120554b99ac81be084c8f) · [Code/config trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/ed208d24baab7d685ae1b6a8ac2b4140a73ab40e/experiments/social_navigation) · [Round2](round2/README.md) · [Round3](round3/README.md).

[COMMIT_HISTORY](COMMIT_HISTORY.md), [FRAMEWORK_COMPARISON](FRAMEWORK_COMPARISON.md) và report/snapshot/seal cũ giữ provenance và các outcome ban đầu. Những dòng kế hoạch/trạng thái ở thời điểm đo là lịch sử, không phải hướng dẫn chạy mới. Tái lập cần source revision/manifest/input của chính phép đo đó; [report storage](../../docs/REPORT_STORAGE.md) mô tả hydrate bằng chứng đã externalize.

[Biên bản cleanup và kiểm chứng](../../docs/CLEANUP_ACCEPTANCE_2026-10-26.md) ghi rõ các file đã bỏ, giới hạn và kết quả E2E hỗn hợp; không dùng cleanup để thay outcome lỗi hoặc chứng nhận an toàn.
