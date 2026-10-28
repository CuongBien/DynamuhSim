# Pilot classifier người/vật từ sensor geometry

Đã hoàn thành4 fit CUDA, chọn `depth_linear`/threshold0.5 trên validation rồi chấm test một lần:23/24 proposal người đúng,0/24 proposal môi trường bị nhận nhầm. Đây là classifier của những cụm đã có trong bộ nhỏ cùng avatar; không phải nghiệm thu detector đầy đủ/ROS rollout.

- [Báo cáo, tiêu chí và lỗi còn lại](../../../archive/social_navigation/reports/depth_person_classifier/CLASSIFIER_REPORT.md)
- [Trọng số và runtime NumPy](artifacts)
- [Ảnh/GIF dự đoán trên depth validation thật](../../../archive/social_navigation/reports/depth_person_classifier/media/validation_predictions.gif)
- [Test result](../../../archive/social_navigation/reports/depth_person_classifier/test_result.json), [audit](../../../archive/social_navigation/reports/depth_person_classifier/audit.json)

`protocol.json` và `frozen.json` chốt trước fit. Toàn bộ fit/score/checkpoint output nhỏ được giữ trong reports; selected release ở artifacts. Raw corpus vẫn nằm ở stage trước `depth_perception_corpus/generated/corpus01`, cần backup riêng.

## Đọc model độc lập, không ROS/Torch

Copy toàn bộ `artifacts/` sang nơi khác; chỉ cần Python3+NumPy. API nhận một proposal sensor-only của loader corpus cũ và metadata scan:

```python
import sys, json
from pathlib import Path
repo = Path('/home/cuongbien/DynamuhSim')
bench = repo / 'experiments/social_navigation/benchmark'
sys.path.insert(0, str(bench / 'depth_person_classifier/artifacts'))
from predict import Classifier
sys.path.insert(0, str(bench / 'depth_perception_corpus'))
from loader import load_input
corpus = bench / 'depth_perception_corpus/generated/corpus01'
sample = load_input(corpus, 'p003', 0)
geometry = json.loads((corpus / 'inputs/p003/lidar_geometry_SENSOR_ONLY.json').read_text())
result = Classifier().predict(sample, geometry)
print(result)   # class_index1=person,0=environment; experimental score, not calibrated
```

Không gọi `load_label` trong đường inference. Model không tự tìm người từ toàn ảnh; cần geometry proposal. Nguồn calibration/units phải phù hợp profile gốc; không tự mang nguyên model này sang robot/camera khác rồi coi đã được validation.

## Kiểm và tái lập

Từ root repo Ubuntu/WSL, dùng venv CUDA đã có, không cài dependency mới:

```bash
TASK_DIR=experiments/social_navigation/perception/depth_person_classifier
bash "$TASK_DIR/run.sh" test
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -I "$TASK_DIR/reports/audit.py"
```

Audit đọc snapshot reports và corpus gốc cần được khôi phục từ backup nếu generated không còn. Kiểm portable dùng NumPy, không Torch/ROS.4 model và68 threshold choice được chấm lại số học từ frozen outputs; không train hoặc chọn lại theo test.

Để tái lập fit ở thư mục output mới, cần corpus đầy đủ và source hashes không đổi:

```bash
bash "$TASK_DIR/run.sh" train --output "$TASK_DIR/generated/reproduction01"
bash "$TASK_DIR/run.sh" evaluate --run "$TASK_DIR/generated/reproduction01"
```

Đây là tái lập experiment đã công bố, không một held-out test mới. Chỉ evaluate sau khi fit đã đóng băng model/threshold. `evaluate` lần hai trên cùng output bị seal chặn; không xóa seal để tune. Bất kỳ cải tiến nào dựa trên testp006/p007 phải có tập test mới.4 cấu hình/seed17/600 step là kế hoạch thí nghiệm đã hoàn thành, không tự mở thêm vòng tuning.

Cả fit và test numerical check dùng GPU NVIDIA; artifact inference dùng NumPy và không đổi ROS/Nav2 mặc định. LiDAR chưa cải thiện kết quả so với depth ở bộ nhỏ này; robot và corpus vẫn giữ cả hai cảm biến, không RGB. Mốc tiếp theo quay lại thu episode robot di chuyển phục vụ WM/RL, mask supervision chưa đủ tin cậy thay vì chờ detector hoàn hảo.
