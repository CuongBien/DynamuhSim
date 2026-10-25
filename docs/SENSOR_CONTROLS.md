# Sensor control bundle và API inference

API mới đọc model/freeze từ `control_bundles/sensor_pilot_v1`, không dùng `reports/` làm kho model. Registry `configs/control_registry.json` pin SHA256 manifest; manifest pin từng file bên trong. Bundle có sáu tệp / 26.068 byte payload, cộng manifest 2.898 byte. Đây là artifact model đã chốt để dùng lại, không phải một run train/test mới.

[Biên bản parity và portability](verification/control_bundle_2026-10-25.json) · [Registry](../configs/control_registry.json) · [Bundle manifest](../control_bundles/sensor_pilot_v1/manifest.json).

## Entry point hiện tại

| Mục đích | API/lệnh |
|---|---|
| Load bundle được pin | `dynamuhsim.controls.bundle.registered_bundle(repo)` |
| Verify engine identity/refinement | `bundle.verify_engine("identity", repo)` / `"refinement"` |
| Trích 30 feature từ một proposal point cloud | `dynamuhsim.perception.depth_classifier.depth_features(points_xyz)` |
| Inference model đã freeze | `FrozenDepthClassifier(bundle)` |
| Kiểm hash bundle và binding source engine | `python3 tools/verify_control_bundle.py --output NEW.json` |
| Probe mặc định bằng API mới | `python3 tools/probe_report_controls.py --output NEW.json` |
| Đối chiếu reader cũ rõ ràng | `python3 tools/probe_report_controls.py --consumer legacy --output NEW.json` |

Ví dụ trong source checkout:

```python
from pathlib import Path
import numpy as np
from dynamuhsim.controls.bundle import registered_bundle
from dynamuhsim.perception.depth_classifier import FrozenDepthClassifier

repo = Path.cwd()
classifier = FrozenDepthClassifier(registered_bundle(repo))
proposals = [np.array([[1., 0., .5], [1.1, .1, 1.2]])]  # ví dụ sensor tổng hợp
result = classifier.predict_points(proposals)  # list các mảng N x3, sensor-only
# Hoặc classifier.predict(features): mảng float N x30 đúng thứ tự feature.
print(result["score"], result["prediction"], result["threshold"])
```

Cài package với extra `perception` để có NumPy, hoặc dùng PYTHONPATH trỏ `python/` khi làm việc trong repo. `proposals` phải là các cụm điểm đã lấy từ sensor; API không tạo proposal từ ảnh/video toàn khung. Prediction true là phía person của classifier pilot, threshold0.5 đã chọn trước test. Không có phương thức fit, tìm threshold hay mở test split. Chỉ hỗ trợ đúng model `depth_linear`, hidden0, depth-only đang được pin; không tự dùng candidate/fusion khác.

Feature order, phép chuẩn hóa và forward pass giữ cùng thứ tự tính với bản freeze. Input sai shape/dtype, NaN/Inf hoặc overflow bị từ chối. Model arrays được kiểm schema, shape, float32, finite và scale dương, rồi giữ bản read-only trong bộ nhớ; thay file trên đĩa không thay model đã load, và lần load mới sẽ phát hiện hash sai.

## Dùng độc lập với reports

```python
from dynamuhsim.controls.bundle import ControlBundle
from dynamuhsim.perception.depth_classifier import FrozenDepthClassifier

bundle = ControlBundle("/PATH/TO/sensor_pilot_v1", expected_manifest_sha256)
classifier = FrozenDepthClassifier(bundle)
```

Mang theo bundle, SHA manifest từ registry được tin cậy và package Python/NumPy. Inference không cần experiments, Git, Torch, ROS, training log, corpus hoặc reports. Hai verifier engine vẫn cần source tree tương ứng để so hash; chúng không chạy/import snapshot hoặc mở dataset được ghi trong freeze.

Metadata, source_freeze và evaluation_receipt giữ đúng byte từ commit đã pin. Training JSON và Torch weights được kiểm hash khi export, rồi chỉ ghi provenance trong manifest; chúng không là file cần đọc ở runtime NumPy. Implementation API mới được kiểm parity với source cũ; không gọi nó là source đã nằm trong manifest train lịch sử.

## Dấu test là trạng thái lịch sử

`classifier/evaluation_receipt.json` là bản sao chứng cứ lần test cũ. Nó **không phải counter mới** và không cấp quyền đánh giá lại. Dấu có hiệu lực với script cũ vẫn là `depth_person_classifier/reports/test_opened.json`; file đó, model cũ và toàn bộ 1.814 control files đã giữ nguyên byte. Bundle có tên/layout riêng, không được đưa vào `evaluate.py --run` như một thư mục run mới.

Các script capture/train/evaluate đã freeze không bị chỉnh sửa để tránh phá source hash và quy trình một lần test. Consumer mới dùng API ở trên; legacy mode chỉ để đối chiếu/replay đúng phiên bản. Chưa thể xóa ba thư mục report chỉ vì probe mặc định đã chuyển sang bundle: còn các consumer lịch sử chưa migrate.

## Export tái lập

```bash
python3 tools/export_control_bundle.py \
  --ref a6fd939e877759703431f66a67cc50ac9e85c3c4 \
  --output /PATH/TO/NEW_BUNDLE
```

Output phải mới. Export đọc committed Git objects, không lấy file dirty/untracked, không train hoặc mở corpus. Nó kiểm binding model/source freeze/test receipt, provenance training/Torch weights và 16 source file engine tại revision đó. Bản export thứ hai đã đối chiếu giống byte với bundle canonical. Registry phải pin đúng SHA manifest được review; loader không tự đổi registry hoặc chấp nhận model thay thế cùng tên.

## Kết quả kiểm chứng 25/10/2026

**118 tests PASS.** Có parity 30 feature trên cloud suy biến/ngẫu nhiên, logits/score/prediction trên 127 hàng synthetic cho từng dtype float32 và float64; thêm test input hỏng, manifest/file sai hash, model array sai schema, normalization không hợp lệ, symlink/path traversal, export từ Git thay vì file dirty và binding engine source.

Native probe cũ/mới cho toàn bộ functional fields giống nhau: hai engine verify10+6 source files, model hash, receipt hash, logits của ba hàng synthetic và output tracker bốn frame đều giữ nguyên. Probe mặc định chủ động chặn open vào ba thư mục report classifier/identity/refinement; không có lần đọc hoặc thử đọc các prefix đó. Runtime gate report vẫn được đọc theo hợp đồng hiện hành; không tuyên bố mọi consumer đã rời reports.

Một process riêng chạy classifier từ bản package+bundle sao chép trong thư mục trống, chặn đọc repo gốc/process/network và import Torch/ROS: PASS với hai proposal tổng hợp. Không có experiments hay `.git` trong thư mục đó. Bản sao kiểm thử đã được dọn, log và kết quả retained.

Không train/test lại, không tune threshold, không dùng raw/corpus/label để kiểm accuracy, không chạy Gazebo/Nav2 và không thêm tuyên bố nhận diện thực tế. Checkout/index chính, report cũ, dataset01, split/nhãn/freeze/outcome giữ nguyên. Bước này tách ba đường đọc mặc định; không xóa thêm payload hoặc giảm clone history.
