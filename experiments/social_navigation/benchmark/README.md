# Hạ tầng tương thích benchmark

26 stage code hiện hành nằm ở bốn khu vực chức năng. Ba chuỗi capture/cohort cũ đã nghỉ dùng; không còn code stage tại đây.

| Khu vực | Vai trò |
|---|---|
| [Navigation](../navigation/README.md) | 12 stage điều hướng, worker, adapter, lifecycle và chẩn đoán; 13 helper Nav2 chung |
| [Perception](../perception/README.md) | 10 stage camera/depth/LiDAR, classifier và tracker |
| [Datasets](../datasets/README.md) | 3 stage reader, target, portable và dataset pilot |
| [Replay](../replay/README.md) | Xem lại dataset/recording |

`task_runtime.sh` là implementation shell dùng chung. Mỗi nhóm liên kết tới
file này; wrapper stage đã freeze giữ nguyên byte. `gpu_runtime`/`shutdown_fix`
là hai alias source lịch sử, không phải implementation thứ hai.

Tên `benchmark/<stage>` trong receipt/manifest đã freeze vẫn là ID nguồn cũ.
`python/dynamuhsim/workspace.py` ánh xạ sang source hiện hành, rồi verifier
vẫn kiểm đúng hash; code thiếu hoặc sai hash không được thay bằng cache cũ.
Report alias nằm cạnh stage mới và trỏ tới archive nguyên bản.

Các thư mục `generated` cũ ở máy chỉ là dữ liệu ignored, không nằm trong cây Git
mới và không bị di chuyển/xoá trong đợt này. Dùng đường dẫn dữ liệu tường minh
khi đọc raw/dataset ở vị trí cũ. [WM workflow](../../../docs/START_HERE_CURRENT.md)
và [receipt kiểm chứng](../../../docs/verification/capability_layout_2026-10-28.json).
