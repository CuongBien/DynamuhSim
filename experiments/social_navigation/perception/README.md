# Perception

Camera/depth/LiDAR, classifier và tracker đã ghim protocol. Có 11 stage.

| Stage | Hướng dẫn |
|---|---|
| `camera_height_ab` | [camera_height_ab](camera_height_ab/README.md) |
| `depth_identity_guard` | [depth_identity_guard](depth_identity_guard/README.md) |
| `depth_perception_corpus` | [depth_perception_corpus](depth_perception_corpus/README.md) |
| `depth_person_classifier` | [depth_person_classifier](depth_person_classifier/README.md) |
| `depth_proxy_ablation` | [depth_proxy_ablation](depth_proxy_ablation/README.md) |
| `depth_proxy_lifecycle` | [depth_proxy_lifecycle](depth_proxy_lifecycle/README.md) |
| `depth_seed43_validation` | [depth_seed43_validation](depth_seed43_validation/README.md) |
| `depth_surface_guard` | [depth_surface_guard](depth_surface_guard/README.md) |
| `depth_track_refinement` | [depth_track_refinement](depth_track_refinement/README.md) |
| `depth_visibility_pilot` | [depth_visibility_pilot](depth_visibility_pilot/README.md) |
| `lidar_person_pilot` | [lidar_person_pilot](lidar_person_pilot/README.md) |

Implementation dùng chung ở `python/dynamuhsim/`; wrapper/profile riêng giữ hợp đồng của từng phép đo. Source snapshots, manifests và report đã freeze giữ nguyên bằng chứng gốc.

Consumer mới dùng [Sensor controls](../../../docs/SENSOR_CONTROLS.md). Tái lập pipeline freeze cũ cần source revision/input đúng manifest; việc chuyển thư mục không cấp quyền mở test hoặc train lại.

[Toàn bộ social-navigation](../README.md) · [Kiểm chứng chuyển cấu trúc](../../../docs/verification/capability_layout_2026-10-28.json).
