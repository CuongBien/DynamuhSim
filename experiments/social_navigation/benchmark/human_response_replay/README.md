# Human-response replay — cohort lịch sử

Runner điều phối, hai runner backend, analyzer và validator cho cohort `b3f59e2` đã được gỡ khỏi HEAD; không còn consumer hiện hành của các file này.

[Commit gốc](https://github.com/CuongBien/DynamuhSim/commit/b3f59e2aa5f43826f4936a7eae3cb9a842339170) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/f3e54812c3149df050db339eeb8cdc490614b648/experiments/social_navigation/benchmark/human_response_replay) · [Report lịch sử](../../../../docs/REPORT_STORAGE.md).

Giữ 6 CrowdSimPlus probes, 18 SocNavGym ablations và mọi outcome/overlap gốc. Robot input được định trước; group membership là fixture. Diagram GIF là OFFLINE REPLAY / NOT GAZEBO CAMERA, không phải camera hoặc benchmark closed-loop Nav2.

Tái lập dùng source revision/manifest/raw inputs của cohort. Các [common inputs/worker](../common_response/README.md) còn dùng cho pipeline mới; xem lại dataset đã thu bằng [WM replay](../wm_dataset_replay/README.md).
