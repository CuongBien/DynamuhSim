# Pilot track cảm biến và target BEV tương lai

Đã chạy offline trên cùng episode standing seed 42: **823 cửa sổ, 1013 ảnh depth**, không thu thêm episode hoặc chạy lại Gazebo. [Báo cáo nghiệm thu](reports/PERCEPTION_TARGET_REPORT.md), [audit cuối](reports/audit02/audit.json), [12 test](reports/contract_tests.txt), [ảnh dữ liệu thật](reports/sample_preview.png), [mẫu sidecar](reports/examples/).

**Kết luận:** hình học/thời gian target BEV đạt ở pilot; nhận diện và giữ ID người **chưa đạt điều kiện train**. Đây là tracker cụm điểm hình học từ depth, chưa phải person detector. Không tự mở mask association hoặc H1–H5.

## Chạy lại offline

Python 3.12 hiện tại có NumPy/SciPy/Matplotlib. Không cần ROS, Gazebo, model weights hoặc download. Mỗi bước tạo thư mục mới, từ chối ghi đè; wrapper giới hạn BLAS/OMP một luồng.

```bash
TASK=experiments/social_navigation/benchmark/perception_target_pilot
BUNDLE=experiments/social_navigation/benchmark/portable_episode/generated/episode01
bash "$TASK/run.sh" test
bash "$TASK/run.sh" track --bundle "$BUNDLE" --output "$TASK/generated/my_sensor"
bash "$TASK/run.sh" labels --bundle "$BUNDLE" --sensor "$TASK/generated/my_sensor" --output "$TASK/generated/my_labels"
bash "$TASK/run.sh" audit --bundle "$BUNDLE" --sensor "$TASK/generated/my_sensor" --labels "$TASK/generated/my_labels" --output "$TASK/generated/my_audit"
```

Lượt đã thực hiện dùng `generated/sensor01` và `generated/labels01`. Đây là **sidecar riêng**, portable episode gốc và mask gốc giữ nguyên. Khi di chuyển sidecar, giữ kèm bundle gốc và kiểm hash/source manifest; không coi sidecar một mình là raw đầy đủ.

## Đọc mẫu

```python
import numpy as np
inputs = np.load('reports/examples/sensor_inputs.npz', allow_pickle=False)
labels = np.load('reports/examples/future_bev_labels.npz', allow_pickle=False)
# Input: xy_vxy [20,16,4], observed_valid [20,16], current_bev [64,64]
# Label: occupancy [3,64,64], cell_observed_valid [3,64,64]
```

Vùng unknown=-1 phải mask loss; slot/timestep thiếu quan sát được zero kèm valid=false. `association_hints_LABEL_ONLY.npz` chứa nhãn ghép GT dùng chẩn đoán, không đưa vào input. Mọi semantic/association acceptance và training eligibility vẫn false.

BEV tương lai của pilot dùng **+0.5/+1/+3 giây**, trong khung robot cố định tại anchor, chỉ chiếm dụng thực quan sát từ depth. GT ego pose chỉ dùng để dựng LABEL. BEV input dùng odom nhân quả và vẫn có sai số ước lượng; không dùng GT để chỉnh input. Không tạo bản đồ toàn cảnh, vùng khuất, social cost hoặc nhãn H1–H5.

Bước kế: kiểm chứng detector chỉ từ depth phân biệt người/vật thể, đánh giá occlusion và đứt ID trên episode độc lập. Chỉ mở association sau khi có bằng chứng đủ; giữ HuNavSim cho test cuối, chưa train hoặc thu batch.
