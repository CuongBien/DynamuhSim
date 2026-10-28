# Datasets

Reader, supervision/target, portable contract và dataset pilot. Có 4 stage.

| Stage | Hướng dẫn |
|---|---|
| `dataset_pilot` | [dataset_pilot](dataset_pilot/README.md) |
| `perception_target_pilot` | [perception_target_pilot](perception_target_pilot/README.md) |
| `portable_episode` | [portable_episode](portable_episode/README.md) |
| `wm_episode_batch` | [wm_episode_batch](wm_episode_batch/README.md) |

Implementation dùng chung ở `python/dynamuhsim/`; wrapper/profile riêng giữ hợp đồng của từng phép đo. Source snapshots, manifests và report đã freeze giữ nguyên bằng chứng gốc.

Full raw/dataset không đi kèm Git. Dữ liệu local cũ vẫn ở vị trí `benchmark/<stage>/generated`; truyền đường dẫn tường minh khi đọc/audit. [WM workflow](../../../docs/START_HERE_CURRENT.md).

[Toàn bộ social-navigation](../README.md) · [Kiểm chứng chuyển cấu trúc](../../../docs/verification/capability_layout_2026-10-28.json).
