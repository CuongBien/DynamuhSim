# Các chuỗi capture/cohort đã nghỉ dùng

Source trước khi gỡ được giữ tại [revision 7b50aa0](https://github.com/CuongBien/DynamuhSim/tree/7b50aa0a357ba91904f8d7b4dd0eb9a46921be00/experiments/social_navigation).
Tái lập writer lịch sử cần checkout/input của revision đã ghi trong manifest.
Các source ID cũ trong receipt giữ nguyên; resolver đối chiếu đúng hash từ source
hiện hành hoặc đúng commit lịch sử, không thực thi bản code nằm trong report/bundle.

## Dataset pilot

Chuỗi `dataset_pilot` là producer episode đầu tiên, trước `dataset01` và capture
API. Runner, build/audit shim, camera/recording/launch riêng và bản config trùng
đã gỡ. Dataset/raw local không bị di chuyển hay xoá.

- Thu mới: `tools/capture_episode.py` và `configs/capture/`.
- Toán timing/projection/BEV: `dynamuhsim.datasets.normalization`, giữ nguyên hash.
- Contract: `python/dynamuhsim/datasets/normalization_contract.json`, nguyên byte.
- Audit normalized episode: `dynamuhsim.datasets.normalized_audit`.
- Reader portable: `dynamuhsim.datasets.portable.Episode`; probe tại `tools/probe_portable_episode.py`.
- [Report pilot gốc](../../experiments/archive/social_navigation/reports/dataset_pilot/PILOT_REPORT.md).

## Depth-proxy lifecycle

Driver factory/quiesce/pair shim và analyzer/auditor chỉ dành cho cohort
motion01/paused01 đã nghỉ dùng. Runtime hiện hành dùng
`runtime/capture_recording.py` và `runtime/shutdown.py`. Regression của trạng thái
projection không khả thi được giữ trong `tests/test_group_projection_regressions.py`.
[Report gốc](../../experiments/archive/social_navigation/reports/depth_proxy_lifecycle/).

## Early-avoidance pilot

Driver-preparation, config v1/moderate riêng, analyzer/audit của cohort cố định
đã nghỉ dùng. Recipe mới đọc từ `configs/capture/`, triển khai trong
`simulation/capture.py`; parser SDF dùng chung ở
`python/dynamuhsim/runtime/native/check_friction.cc`. Protocol gốc nguyên byte
được giữ ở `configs/history/`; [report gốc](../../experiments/archive/social_navigation/reports/early_avoidance_pilot/).

Ba thư mục stage code không còn trên cây Git hiện hành. Các thư mục `generated`
ở máy nếu còn chỉ là raw/cache ignored. Báo cáo, models, labels, split/quarantine
và test seals vẫn ở archive/dataset ban đầu. Tên cũ trong manifest không phải
chỉ dẫn còn chạy producer đó ở HEAD.
