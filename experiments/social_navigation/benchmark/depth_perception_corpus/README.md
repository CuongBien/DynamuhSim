# Pilot dataset cho perception LiDAR + depth

Đã thu7cảnh GPU, xuất19frame/132proposal có nhãn (64người,68môi trường). Đây là bộ nhỏ thử pipeline phân biệt người/vật; chưa có model được nghiệm thu hay bộ train World Model. Không RGB, không đổi robot/ROS/Nav2 mặc định.

- [Báo cáo và giới hạn](../../../archive/social_navigation/reports/depth_perception_corpus/CORPUS_REPORT.md)
- [Schema và cách đọc sample](../../../archive/social_navigation/reports/depth_perception_corpus/SCHEMA.md)
- [Audit cuối](../../../archive/social_navigation/reports/depth_perception_corpus/audit.json), [portable audit](../../../archive/social_navigation/reports/depth_perception_corpus/portable_audit.json)
- [Mẫu train đọc độc lập](examples/train_p003/manifest.json)
- [GIF từ depth đã ghi thật](../../../archive/social_navigation/reports/depth_perception_corpus/media/depth_labels.gif)

Full bundle ở `generated/corpus01/`; raw ở `generated/runs/`. Hai thư mục này bị gitignore, cần backup riêng. Git giữ example train p003, code, nguồn freeze và báo cáo; test không có preview/score model ở mốc này.

## Chạy offline trong Ubuntu/WSL

Từ root repo, dùng Python hiện tại với NumPy/SciPy; không cài gì mới:

```bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
TASK_DIR=experiments/social_navigation/benchmark/depth_perception_corpus
bash "$TASK_DIR/run.sh" test
python3 "$TASK_DIR/reports/audit02.py" --corpus "$TASK_DIR/generated/corpus01" --output "$TASK_DIR/generated/recheck.json"
python3 -I "$TASK_DIR/reports/verify_portable.py" --corpus "$TASK_DIR/generated/corpus01" --output "$TASK_DIR/generated/portable_recheck.json"
```

Auditor01 `run.sh audit` giữ nguyên để tái hiện lỗi giả định mọi sample phải có scan; **auditor02 là command nghiệm thu**. Loader đã hỗ trợ missing mask trước thu:21proposal/3frame đầu không có scan khả dụng đúng thời điểm, giữNaN/validfalse thay vì lấy scan tương lai. Không bỏ các sample này một cách âm thầm khi train.

```python
import sys
sys.path.insert(0, "experiments/social_navigation/benchmark/depth_perception_corpus")
from loader import load_input, load_label
root = "experiments/social_navigation/benchmark/depth_perception_corpus/generated/corpus01"
x = load_input(root, "p003", 0)
y = load_label(root, "p003", 0)   # LABEL supervision đọc riêng
print(x["points_xyz"].shape, x["scan_valid"], y["class_index"])
```

Có thể copy `examples/train_p003/` sang máy khác và import loader bên trong, chỉ cần NumPy. Đọc test mặc định bị chặn; chỉ dùng `allow_test=True` cho giai đoạn đã chốt hoặc integrity QA, không tune từ test.

## Thu lại một cảnh mới để chẩn đoán

Có ROS2Jazzy/GazeboHarmonic và dependency benchmark cũ trong workspace:

```bash
TASK_DIR=experiments/social_navigation/benchmark/depth_perception_corpus
bash "$TASK_DIR/run.sh" build
bash "$TASK_DIR/run.sh" capture --scene p003 --output "$TASK_DIR/generated/manual_motion01"
```

Output phải chưa tồn tại, chạy từng lượt. Capture tự source Jazzy/install, dùng runtime partition/domain91 lock và GPU preflight; dừng nếu không đúng NVIDIA hoặc không đạt dữ liệu/cleanup. Không dùng killall. Chuyển động ở đây được lập trình cho diagnostic, không phải policy social nav. Build lại có thể đổi binary SHA: không ghi đè freeze cũ hoặc tái diễn giải corpus01; một đợt thu mới cần protocol/freeze phiên bản mới. Export official7scene bằng `run.sh export --output <new-directory>` chỉ khi source hashes khớp freeze.

Ảnh/GIF: script final `reports/preview02.py --corpus <bundle> --output <new-media-dir> --scene p003`; chỉ train/validation, cần Matplotlib/Pillow. Đây là replay depth và label/point projection, không dùng image generation.
