# Benchmark hiện hành

Cây này còn **29 thư mục thực nghiệm có code/config hiện hành**, thay vì 44
thư mục lẫn lịch sử. 13 thư mục chỉ còn vỏ đã bỏ; hai helper cũ trở thành alias
source freeze. Các cây `reports` là alias tới archive, không chứa bản sao nữa.

| Cần làm | Điểm vào / thành phần |
|---|---|
| Nav2 cá nhân, nhóm và backend cô lập | `tools/nav2_trial.py`; `socnavgym_nav2_pilot`, `socnavgym_group_nav2`, `shared_nav2_pilot` |
| Capture mới | `tools/capture_episode.py`, `configs/capture/`, `python/dynamuhsim/runtime/` |
| Worker, adapter và kernel còn được dùng | `common_gazebo`, `common_response`, `socnavgym_response_fix`, `complexitynav_contract`, `complexitynav_geometry`, lifecycle helpers |
| Depth/LiDAR/classifier | `depth_*`, `lidar_person_pilot`, `perception_target_pilot`; các pipeline/freeze đã chốt giữ hợp đồng riêng |
| Đọc/export dataset, replay | `wm_episode_batch`, `wm_dataset_replay`, `portable_episode`, compatibility `dataset_pilot` |
| Chẩn đoán còn hỗ trợ | Root `run.sh trial`, `navigation_diagnostics`, `localization_trials`, camera/proxy suites |
| GPU, ownership cleanup và ordered shutdown | `python/dynamuhsim/runtime/gpu.py`, `shutdown.py`, `native/gl_probe.cpp`; `tools/check_gpu_runtime.py --output NEW_DIRECTORY` |
| Report/control lịch sử | [Archive](../../archive/README.md) và [hydrate](../../../docs/REPORT_MEDIA.md) |
| Cohort không còn chạy | [Danh mục đã nghỉ dùng](../../../docs/history/retired_benchmarks.md) |

Các `trial.py`, `analyze.py`, `worker.py` tương thích chỉ chuyển tiếp tới code dùng
chung khi experiment đã được migration; trùng tên file không đồng nghĩa còn
bản sao vòng chạy. Source frozen/upstream dùng đối chứng vẫn giữ nguyên byte.

Chạy mới theo [shared Nav2](../../../docs/SHARED_NAV2_RUNTIME.md) và
[capture API](../../../docs/CAPTURE_RUNTIME.md), luôn dùng output directory mới.
Không đổi controller, ngưỡng goal, model, split/nhãn hoặc kết quả cũ trong cleanup.
