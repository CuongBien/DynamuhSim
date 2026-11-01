# Đối chiếu hai báo cáo được cung cấp — 30/10/2026

**Kết quả quan trọng:** chạy nguyên detector API của commit gốc 70bcda0 trên cùng 910 frame, cùng recorded CameraInfo/TF và causal odometry cho số detection/XY/ID/point-count/confirmed trùng bản sau merge ở mọi frame. Không có bằng chứng core detector suy giảm do merge trong lượt đối chiếu này. Không mở lại classifier test hoặc chạy Gazebo.

| Recording | Frames | Output gốc | Output sau merge | Frame khác nhau | Max sai khác XY |
|---|---:|---:|---:|---:|---:|
| Conversation |464|173|173|0|0m|
| Walking |446|144|144|0|0m|

Đây là đối chiếu detector API với calibration đúng và cùng sensor inputs; không tuyên bố original hard-coded ROS wrapper hoặc mọi chế độ runtime đều tương đương. Metadata/semantic flags mới không được so byte-identical.

## 1. Báo cáo đa môi trường: script chưa đo điều được phần kết luận tuyên bố

Báo cáo gọi 30 ca là dữ liệu trực tiếp từ 4 world Gazebo, zero-shot 100%, zero false positives và sẵn sàng cấp PPO. Trong code được bàn giao:

- make_human dùng NumPy lấy điểm ngẫu nhiên trong hình trụ radius 0,22m, height 1,70m, mặc định 350 điểm. Bench/cart/wall là hình khối tự sinh. Tên actor/world có thể được dùng để mô tả hoặc lấy cảm hứng kịch bản; script không đọc SDF/YAML, chạy world hoặc nhận depth camera để tạo những point clouds này.
- Các ca “đi bộ/cắt ngang/rẽ góc” trong 30 scene phần lớn chỉ là một point cloud tĩnh ở một vị trí. Không chứng minh phản ứng động/che khuất qua render cảm biến trong các world tương ứng.
- TP=min(detected_count,expected_count). Không matching vị trí GT. Một người bị bỏ sót nhưng một đồ vật được phát hiện thay thế vẫn có thể có TP=1 trong scene một người.
- Người hình trụ đủ chiều cao với mật độ gần cố định khác point cloud camera chỉ thấy bề mặt, có góc nhìn/che khuất, số điểm theo khoảng cách và nhiều NaN/Inf.
- Timing đo trên point cloud sinh sẵn; chưa gồm nhận/decode/project depth hoặc ROS/RViz. Không suy ra FPS end-to-end hoặc hiệu năng CPU nhúng từ đó.

100% của đúng bộ synthetic smoke này có thể tái lập; ngay cả lượt sửa matching vị trí của chúng ta vẫn PASS30/30 trên chính các point clouds này. Do đó vấn đề không chỉ ở count-only metric, mà còn ở độ đơn giản/phạm vi input. Kết luận zero-shot trong 4 world, khử FP tuyệt đối và readiness cho PPO/WM vượt phạm vi bằng chứng này.

Nguồn gốc: [make_human](https://github.com/CuongBien/DynamuhSim/blob/70bcda0e26ff86b3389a49295feda3ef72958b1d/experiments/social_navigation/benchmark/depth_identity_guard/evaluate_multi_environments.py#L32), [cảnh mẫu](https://github.com/CuongBien/DynamuhSim/blob/70bcda0e26ff86b3389a49295feda3ef72958b1d/experiments/social_navigation/benchmark/depth_identity_guard/evaluate_multi_environments.py#L111), [count-only TP](https://github.com/CuongBien/DynamuhSim/blob/70bcda0e26ff86b3389a49295feda3ef72958b1d/experiments/social_navigation/benchmark/depth_identity_guard/evaluate_multi_environments.py#L373).

## 2. Báo cáo kỹ thuật: số classifier/tracker hẹp hơn end-to-end detection

Kết quả classifier 23 TP/1 FN/24 TN/0 FP, balanced accuracy 97,92%, đúng với artifact test_result.json của 48 proposal. Đây là phân loại patch đã có trong một corpus nhỏ, một warehouse/một mesh; không đo khả năng gom được mọi người từ toàn ảnh hoặc generalization sang avatar/sensor/world mới. Chọn depth_linear nghĩa là winning model dùng depth, dù code có thêm 8 feature LiDAR cho các fusion candidate khác.

Tracker IdentityGuard có các gate riêng đã PASS (sửa case short-gap trên seed43, không suy giảm seed44). Điều này không chứng minh track đó là người hay bộ detector không bỏ sót. Artifact acceptance vẫn ghi legacy_full_gate_seed43=false, legacy_full_gate_seed44=false, semantic_person_validated=false, association_accepted=false, training_ready=false và navigation TIMEOUT. Các số đó được giữ nguyên, không phủ nhận PASS của gate ID hẹp hơn.

SHA/seal và chia train/test giúp kiểm soát quy trình trong phạm vi đã định nghĩa; chúng không tự chứng minh dữ liệu đủ đa dạng hoặc loại bỏ mọi shortcut. Các test đã mở không trở lại thành unseen test nhờ đổi tên.

## 3. Tại sao pixel thật làm detector thất bại?

Prior nhân 0,15 khi cụm thấp/rộng vốn đã có trong code gốc. Khi nhìn partial human, nó có thể biến classifier score 0,76–1,00 thành 0,11–0,15, dưới threshold 0,45. Test người hình trụ đủ thân không bộc lộ tình huống này tốt như depth thật. Clustering/planar filters còn loại các proposal trước classifier.

Ablation đã đo: conversation coverage proxy 82,6% (2D cũ) /56,1% (3D no prior) /8,2% (3D prior); walking 90,7% /61,4% /7,6%. **Các tỷ lệ này không cùng định nghĩa với 100% count-only hoặc 97,92% patch balanced accuracy; không so chúng như cùng một accuracy giảm sau merge.**

Kết luận: giữ những kết quả classifier/identity nhỏ đúng phạm vi; đổi cách gọi 30 scene thành synthetic smoke, và dùng depth recordings/instance labels ngoài miền để nghiệm thu full pipeline. Không đánh giá động cơ hoặc ý định của người viết báo cáo từ các chênh lệch này.

[Raw replay result](result.json) · [12 source SHA pins](original_source_pins.json) · [script đã chạy](reproduce.py) · [log](run.log).

Script as-measured dùng snapshot /tmp/depth-detector-review-1sfeuh4f; pins ghi commit và tất cả 12 file import cần tái dựng bằng git show/archive. Normalized đầu vào và post-merge reference vẫn ởgenerated ngoàiGit của workspace gốc. Chạy với OMP/OPENBLAS/MKL threads=1; không Torch/GPU hoặc model fitting.
