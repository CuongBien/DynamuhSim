# Navigation

Điều hướng Nav2, worker/backend, lifecycle và chẩn đoán. Có 12 stage.

| Stage | Hướng dẫn |
|---|---|
| `common_gazebo` | [common_gazebo](common_gazebo/README.md) |
| `common_response` | [common_response](common_response/README.md) |
| `complexitynav_contract` | [complexitynav_contract](complexitynav_contract/README.md) |
| `complexitynav_geometry` | [complexitynav_geometry](complexitynav_geometry/README.md) |
| `localization_trials` | [localization_trials](localization_trials/README.md) |
| `navigation_diagnostics` | [navigation_diagnostics](navigation_diagnostics/README.md) |
| `shared_nav2_pilot` | [shared_nav2_pilot](shared_nav2_pilot/README.md) |
| `socnavgym_group_nav2` | [socnavgym_group_nav2](socnavgym_group_nav2/README.md) |
| `socnavgym_lifecycle_fix` | [socnavgym_lifecycle_fix](socnavgym_lifecycle_fix/README.md) |
| `socnavgym_nav2_pilot` | [socnavgym_nav2_pilot](socnavgym_nav2_pilot/README.md) |
| `socnavgym_response_fix` | [socnavgym_response_fix](socnavgym_response_fix/README.md) |
| `socnavgym_warehouse_lifecycle` | [socnavgym_warehouse_lifecycle](socnavgym_warehouse_lifecycle/README.md) |

Implementation dùng chung ở `python/dynamuhsim/`; wrapper/profile riêng giữ hợp đồng của từng phép đo. Source snapshots, manifests và report đã freeze giữ nguyên bằng chứng gốc.

Chạy mới qua `tools/nav2_trial.py` hoặc `tools/capture_episode.py` từ root repo. Helper Nav2 trước đây ở root benchmark nay nằm ngay trong thư mục này. [Runtime](../../../docs/SHARED_NAV2_RUNTIME.md).

[Toàn bộ social-navigation](../README.md) · [Kiểm chứng chuyển cấu trúc](../../../docs/verification/capability_layout_2026-10-28.json).

Các cohort đã đóng được ghi tại [lịch sử capture](../../../docs/history/retired_capture_pipelines.md); report cũ đọc trực tiếp từ archive.
