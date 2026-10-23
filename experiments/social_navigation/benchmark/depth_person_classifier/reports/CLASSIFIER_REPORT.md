# Huấn luyện thử bộ phân biệt người/vật — 23/10/2026

**Đã hoàn thành mốc huấn luyện và kiểm test pilot.** Mô hình chọn trên validation là `depth_linear`, ngưỡng 0.5. Trên test cố định: **23/24 cụm người đúng, 0/24 cụm môi trường bị nhận nhầm người**, còn 1 false negative. Gate pilot đặt trước fit đạt. Đây là phân loại những proposal đã tồn tại trong corpus nhỏ, không phải nghiệm thu detector phát hiện đủ mọi người trong cảnh hoặc độ tin cậy ngoài mô phỏng.

Code, trọng số, normalization, toàn bộ kết quả bốn cấu hình và lần test được lưu trong Git. Không chạy Gazebo mới hoặc thay mặc định ROS/Nav2 ở mốc này. Kết quả và raw corpus trước đó không đổi. Mốc này có thể đóng; bước kế tiếp của dự án quay lại thu episode robot chuyển động phục vụ WM, với những nhãn chưa đủ tin cậy được mask. Không yêu cầu classifier này hoàn hảo mới ghi dữ liệu cảm biến cho WM.

## Thiết kế chốt trước fit

[Protocol](../protocol.json), [source freeze](../frozen.json) được ghi sau10 unit tests và trước mọi fit/chấm validation/test.13 file nguồn gồm feature/data/model/inference/training/evaluation/metrics/test/loader và calibration robot;67 file của corpus cũng bị ràng buộc SHA. Checkpoint repo `deecebd`, không thay corpus hoặc split. Test policy của mốc mới cho phép đánh giá một lần winner đã cố định; policy corpus cũ mô tả giai đoạn thu trước đó, khi chưa dùng test cho model.

Dữ liệu train 54 proposal từp001–p003, validation 30 từp004–p005, test 48 từp006–p007. Họ bố cục A/B/C nằm trong các split riêng; frame liên tiếp cùng cảnh không chia sang tập khác. Một mesh người/nhà xưởng, chỉ 19 frame xuất, không coi132 proposal là132 người hay episode độc lập.

30 feature depth mô tả log số điểm, độ trải XYZ, quantile chiều cao, bán kính tương đối XY, covariance và profile sáu dải chiều cao. Bỏ tuyệt đối XY, pixel/bbox center, scene ID, thời gian và mọi GT khỏi feature model. Chiều cao trong base_link được giữ. Số điểm/mật độ vẫn có thể phản ánh khoảng cách và renderer; loại XY trực tiếp không loại hết shortcut của bộ nhỏ. Candidate extraction kế thừa corpus, không chỉnh sau chấm test.

Bản fusion thêm8 feature từ điểm native LiDAR nằm trong0.6 m quanh medianXY của proposal depth. base_scan trùng XY vàyaw với base_link theo SDF gốc. Bỏ tia cuối trùng0/2π; không nội suy. Cần≥3 điểm local hợp lệ. Thiếu scan/patch thì nhánh LiDAR residual bị gate0, không tự gắn environment và không bỏ sample. Normalization mean/std chỉ fit trên train; LiDAR chỉ thống kê patch hợp lệ.21 sample của corpus thiếu scan nhân quả vẫn giữ trong phép đo, kể cả khi model thắng chỉ dùng depth.

Bốn cấu hình cố định: linear hoặc tanhMLP16, dùng depth hoặc depth+LiDAR. PyTorch2.7.1+cu128/CUDA12.8 trong venv detector cũ, **không cài hoặc đổi dependency**. Seed17, FP32/TF32off, deterministic algorithms, full-batch600 step AdamW lr0.01/weight_decay0.01. Tổng train mất18.83s wall gồm fit/validation/save, không phải benchmark latency inference. Parameter và gradient của cả bốn cấu hình thực ởCUDA0/NVIDIA RTX4060; parameter thay đổi và train loss giảm được ghi trong training.json/trace/plot. NumPy/IO/audit dùng CPU một luồng để giữ tải thấp.

Loss cân bằng tổng trọng số từng train scene, từng frame trong scene, từng class có mặt trong frame; tránh40proposal của cảnh động chi phối hai cảnh tĩnh. Không early stopping, không chọn epoch/seed, không augmentation từ counterfactual. Validation chọn một trong4×17threshold theo thứ tự tiêu chí đã ghi; tie chọn ít tham số hơn và threshold gần0.5. Test không tham gia normalization, gradient, chọn cấu hình hoặc ngưỡng.

## Validation và lựa chọn

| Cấu hình | Tham số train | Threshold chọn | Người đúng | Môi trường→người | Balanced accuracy |
| --- | ---: | ---: | ---: | ---: | ---: |
| depth_linear | 31 | 0.50 | 11/12 | 0/18 | 95.83% |
| fusion_linear | 40 | 0.50 | 11/12 | 0/18 | 95.83% |
| depth_mlp16 | 513 | 0.50 | 11/12 | 0/18 | 95.83% |
| fusion_mlp16 | 594 | 0.50 | 11/12 | 0/18 | 95.83% |

Bốn bản đạt cùng kết quả tốt nhất. Linear depth31 tham số được chọn do đơn giản nhất; fusion chưa đem lại lợi ích đo được trên bộ này. Điều đó không kết luận LiDAR vô ích cho robot hoặc dữ liệu thực. Robot/dataset vẫn giữ LiDAR+depth, không RGB. All-person reference validation có18 false-positive; đây chỉ là reference tầm thường, không phải so sánh trực tiếp với DR-SPAAM trên cùng corpus.

Ngưỡng/model/feature/normalization được cố định bằng `selected_frozen.json` và SHA trọng số trước khi tạo `test_opened.json`. [Trọng số NumPy](../artifacts/selected_model.npz) chỉ1635byte, đi kèm feature/inference code và Torch state dictionary để tái lập. Không đổi threshold sau khi xem lỗi test.

## Test duy nhất của winner

| Tập | TP | FN | FP | TN | Recall người | Precision người | Balanced accuracy |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Validation | 11 | 1 | 0 | 18 | 91.67% | 100% | 95.83% |
| Test p006 tĩnh | 4 | 0 | 0 | 4 | 100% | 100% | 100% |
| Test p007 động | 19 | 1 | 0 | 20 | 95% | 100% | 97.5% |
| Test tổng | 23 | 1 | 0 | 24 | 95.83% | 100% | 97.92% |

Gate global recall≥90%, FPR≤10%, balanced accuracy≥90%; từng scene recall≥75%, FPR≤25%. Test đạt các ngưỡng này.0/24false-positive trên vài frame tương quan không chứng minh sẽ không nhận nhầm ngoài tập này. Recall ở đây có mẫu số là24proposal người có nhãn, **không bao gồm người geometry proposal bỏ sót, ngoài FOV hoặc không nhìn thấy**. Chưa có end-to-end detector recall/AP.

`evaluate.py` tạo seal bằng exclusive-create trước khi mở test; lệnh lần hai bị file seal chặn trước khi đọc lại test. Một winner được chấm, không chấm ba cấu hình khác trên test. Audit sau đó phát lại chính model/threshold đã chốt để kiểm số học và provenance, không phải lượt chọn model mới. Test family C hiện đã được dùng; mọi tune tiếp từ lỗi này cần test mới, không được tiếp tục gọi C là unseen test.

## Lỗi được giữ và hình minh họa

Validationp005/index9/frame1: score0.0051 nhưng nhãn người; chỉ thấy24.91%silhouette in-view,21 điểm, zmax0.72m. Testp007/index22/frame2: score0.0809 nhưng nhãn người; thấy30.39%,23 điểm, zmax0.95m. Cả hai là instance 2 LABEL và không có local LiDAR patch đủ điều kiện; scan bản thân vẫn khả dụng. Đó là quan sát hậu kiểm, không chứng minh nguyên nhân duy nhất. Không hạ threshold hoặc sửa model từ hai lỗi này. Sigmoid score **chưa calibrated**: điểm thấp không có nghĩa chắc chắn là vật.

[Ảnh lỗi validation](media/validation_predictions.png), [GIF5frame validation](media/validation_predictions.gif) dùng depth native đã ghi, kèm dự đoán thật của trọng số đã chốt. Màu xanh dương=đúng người, xanh lá=đúng môi trường, cam=bỏ sót người, đỏ=nhận nhầm người. Màu chấm lỗi đọc LABEL riêng, không truyền vào model. PNG chọn frame có lỗi validation đầu tiên; GIF giữ đủ5frame. Không có preview test và không AI sinh ảnh/GIF. [Loss/validation plot](media/training_validation.png) lấy trực tiếp trace và score fit.

## Kiểm chứng và phạm vi nghiệm thu

10 tests trước fit: hình dạng/finite feature, bất biến dịchXY và thứ tự điểm, GT không ảnh hưởngfeature, missing scan, local native patch, weighting, metric/mask, test mặc định bị chặn và4 CUDA–NumPy parity/gated-missing behavior. [Unit test record](unit_tests.json).

[Audit độc lập](audit.json) tính lại68validation choices và lựa chọn winner, normalization từ train, kiểm hash source/corpus/trọng số và đối chiếu portable runtime với132prediction đã lưu. Maxscoreerror7.6e-08; portable replay không import Torch/ROS/Gazebo. Test CUDA–NumPy logiterror1.91e-06, quyết định threshold khớp. Audit chỉ kiểm lại frozen result, không fit/tune.15,361 file tracked hiện diện trước mốc không đổi,12 staged entry người dùng/outside diff giữ nguyên; [preservation](preservation_before_commit.json).

**Nghiệm thu: hoàn thành pilot train/validation/frozen-test và model artifact chạy độc lập.** Không nghiệm thu cross-avatar/real-world/cross-simulator, tracking/re-ID, H1–H5, hành vi social navigation hay mức đủ dữ liệu WM/RL. Không tích hợp ROS/Gazebo realtime từ mốc này. HuNavSim vẫn chưa dùng. Những gateFAIL/TIMEOUT cũ giữ nguyên, không thay thành success từ kết quả classifier này.
