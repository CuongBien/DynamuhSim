# Review nhánh feat/depth-person-detector — 30/10/2026

**Kết luận: có ích như một detector thử nghiệm, cần sửa trước khi tích hợp vào runtime hiện tại. Chưa merge.**

- Nhánh được đọc: `70bcda0e26ff86b3389a49295feda3ef72958b1d`.
- Nhánh hiện tại: `test/gazebo-harmonic-benchmark`, HEAD `4d0a4b3`.
- Merge base: `9833565fa546d55cb0e553f8388bf4a703c59583`.
- Phần mới của nhánh: một commit, 8 file, 1.611 dòng thêm; không thay world, Nav2, weights classifier hay thuật toán IdentityGuard đã có.
- Chỉ fetch/read và chạy snapshot trong /tmp. Không checkout, actual merge, cài dependency, mở simulator hoặc huấn luyện.

## Bạn đã làm những gì?

| Thành phần | Thay đổi thực tế |
|---|---|
| GeneralizedPersonDetector | Thay proposal 2D grid bằng clustering 3D dùng cKDTree, radius 0,18 m, mật độ tối thiểu 18 điểm; giới hạn khoảng cách/chiều cao. |
| Tách người và đồ vật | Với cụm rộng, thử lấy phần quanh tâm thân trên; loại mặt phẳng mỏng; điều chỉnh score bằng chiều cao, chiều rộng, tỷ lệ hình học. |
| Classifier | Dùng lại selected_model.npz của depth_linear đã có; không có train mới hoặc weights mới. Ngưỡng cuối 0,45 và các prior là thay đổi rule quyết định. |
| Tracking | Nối detections vào IdentityGuard có sẵn, chuyển base_link sang odom và giữ ID theo cơ chế cũ. |
| Pipeline | Có API nhận point cloud hoặc back-project depth qua K và optical-to-base transform. |
| ROS node | Thêm depth_person_tracker_node.py, subscribe depth và odom, publish MarkerArray + PoseArray để quan sát. |
| Benchmark | 5 unit tests; 12 cảnh tổng hợp kèm chuỗi một người 8 frame; thêm 30 cảnh mang tên Hospital Easy/Medium/Hard và School Arena. |

**“Nhiều môi trường” ở đây là point cloud hình trụ/hộp/mặt phẳng sinh bằng NumPy. Không có world Gazebo mới hoặc chạy cảm biến trong các world đó.** Các tên đi bộ/giao cắt/đẩy xe trong benchmark 30 cảnh chủ yếu là mô tả một ảnh điểm tĩnh, không phải episode chuyển động.

Mô hình selected hiện tại có modality=depth. ROS node không subscribe LiDAR; các hàm có đối số scan tùy chọn nhưng lượt chạy này không chứng minh fusion LiDAR + depth. Việc lọc sàn dùng ngưỡng z cố định; không có ước lượng mặt phẳng sàn thích nghi như mô tả đầu file.

## Kiểm chứng đã chạy

| Kiểm tra | Kết quả và giới hạn |
|---|---|
| test_integrated_pipeline.py | 5/5 PASS trên Python 3.12 + NumPy/SciPy hiện tại. |
| benchmark_scenarios.py | 12/12 cảnh PASS theo metric gốc; chuỗi một người giữ ID. |
| evaluate_multi_environments.py | 30/30 cảnh PASS, script in precision/recall/F1=100%. Đây là kiểm tra đếm trên dữ liệu tổng hợp. |
| 6 frame depth đã ghi của pilot hiện tại | Chạy được với calibration ghi thật; số proposal 0/1/0 và 0/0/0. Không đối chiếu GT nên không suy ra recall từ số này. |
| Performance | Log lưu lần chạy cuối: khoảng 4,55 ms/test nhỏ, 8,37 ms trung bình bộ 30 cảnh; lần chạy trước tương ứng 9,88 và 10,60 ms. Sáu frame thật khoảng 5,26–33,29 ms. Chưa có chứng cứ end-to-end <2,5 ms hay p95 ở tải thực. |
| Import theo layout mới | PASS khi đặt các file pipeline/detector đúng thư mục perception trong bản tạm. ROS node vẫn hard-code benchmark cũ. |
| Merge preview | git merge-tree trả exit 1: nhiều xung đột vị trí file do refactor. Không làm actual merge. |

Các test benchmark nhỏ đều nhận point cloud sinh sẵn; timing không bao gồm nhận/giải mã depth, ROS communication và vẽ RViz. Test có mô tả “<5 ms” nhưng assertion thực tế cho phép 15 ms. Log tại đây là kết quả của lần review, không phải report thực nghiệm do tác giả nhánh commit thêm.

## Các điểm cần sửa, theo mức ưu tiên

### P1 — Calibration ROS node sai với robot đang sử dụng

Node dòng 54–69 hard-code D435i 640×480, fx=fy=386, cx=320/cy=240 và camera x=0,08/z=0,28 m. Pilot hiện tại là 424×240, fx≈fy≈223,40, cx=212/cy=120, x=0,12/z=0,08 m. Back-projection, độ cao cụm và vị trí người vì vậy sai khi dùng node nguyên trạng.

Cần đọc CameraInfo và TF đúng frame, kiểm kích thước ảnh/calibration trước khi nhận kết quả. API process_depth_frame đã cho phép truyền calibration; vấn đề nằm ở ROS wrapper mặc định.

### P1 — Odometry không được ghép thời gian với depth

Node dòng 98–107 chỉ ghi pose mới nhất, dòng 131–139 dùng pose đó với timestamp của depth, không giữ odom stamp/frame/age. Depth đến trễ có thể dùng pose tương lai hoặc quá cũ, làm track trôi và vi phạm tính nhân quả khi thu data.

Cần buffer odom, chọn measurement đã có với stamp <= depth stamp và giới hạn tuổi; xử lý reset/thời gian trùng hoặc lùi bằng episode/tracker mới. IdentityGuard hiện sẽ raise với timestamp không tăng, node chưa xử lý tình huống này.

### P1 — Lỗi inference có thể biến thành nhãn “người”

Detector chính dòng 182–187, bản artifact dòng 121–125 bắt mọi Exception rồi gán score=0,5. Với prior 1,15 và threshold 0,45, một cụm hình người bị lỗi classifier vẫn trở thành is_person=True, score=0,575. Đã tái hiện bằng classifier cố ý raise RuntimeError.

Cần báo lỗi/invalid rõ ràng và không quảng bá fallback thành detection người hợp lệ. Đây là lỗi âm thầm, đặc biệt nguy hiểm nếu dùng kết quả làm nhãn dữ liệu.

### P1 — Sai đơn vị depth 16UC1 khi có CvBridge

Node dòng 116–124 chỉ chia 1000 ở nhánh fallback. CvBridge passthrough giữ uint16 mm; đưa ảnh 2 m vào thì pipeline nhận 2000 và bộ project loại vì vượt 5 m. Đã chạy actual on_depth method với stub bridge/pipeline, không khởi động ROS.

Cần chuẩn hóa theo encoding ở cả hai nhánh, kiểm step/endian nếu giữ raw fallback. Lỗi 16UC1 không chứng minh lượt Gazebo 32FC1 hiện tại gặp lỗi này; nó chặn tuyên bố hỗ trợ sensor 16UC1.

### P2 — Benchmark chưa đủ để kết luận khả năng tổng quát

Trong evaluate_multi_environments.py dòng 369–381 và benchmark_scenarios.py dòng 219–232, TP=min(số phát hiện, số mong đợi). Không matching vị trí; phát hiện nhầm một đồ vật trong cảnh có một người vẫn có thể được tính là TP. Benchmark không reset tracker giữa từng cảnh độc lập; kết quả mỗi cảnh cũng không phải kiểm thử identity qua quỹ đạo dài.

ID switch dùng số ID duy nhất trừ một; trường hợp ID 1→2→1 không được đếm đúng số chuyển. Chuỗi in rate có phần “0.0%” ghi cứng. Không có so sánh detector cũ/mới trên cùng raw recordings, không có multi-seed held-out sensor evaluation mới. Các audit/report classifier cũ không tự chứng nhận detector có proposal/prior mới.

Cần giữ bộ tổng hợp như smoke test, rồi bổ sung matching vị trí một-một có gate, FP/FN thật, ID switches/fragmentation theo chuỗi, latency p50/p95 trên depth đã ghi. Không tuning lại trên test đã mở của classifier cũ.

### P2 — ROS package và output ID chưa hoàn thiện

- Script chưa được thêm vào install(PROGRAMS), launch hoặc dependency khai báo mới (SciPy, visualization_msgs); không chạy mặc định qua ros2 run/launch của project.
- Script hard-code experiments/social_navigation/benchmark, cần đổi sang layout perception và cơ chế tìm resource khi cài package. Không khôi phục toàn bộ thư mục cũ chỉ để node import được.
- PoseArray không chứa ID, confidence, velocity_valid hoặc ambiguity, nên chưa đủ làm interface person tracks cho dataset/WM. Đây mới là output hiển thị pose.
- Marker dùng abs(track_id); ID unknown=-1 trùng ID=1 và nhiều unknown trùng nhau, có thể làm hiển thị bị ghi đè.
- Hai bản generalized_detector trùng logic; pipeline hiện import bản artifacts. Cần chọn một nguồn implementation để tránh sửa nhầm bản.

## Đánh giá và đề xuất merge

Hướng đi hợp lý cho một baseline hình học nhẹ, không cần thêm RGB/PyTorch. Có thể tái sử dụng phần detector/pipeline độc lập khi chuyển engine, miễn calibration và thời gian đầu vào đúng. Các heuristic còn có khả năng bỏ sót người bị che, người ngồi hoặc người chỉ hiện một phần do camera thấp; cần đo bằng dữ liệu sensor, không dựa vào hình trụ đủ thân.

**Không nên merge nguyên trạng vào luồng nghiệm thu hoặc đưa output làm ground truth.** Đề xuất một vòng tích hợp gọn:

1. Mang code mới vào đúng perception layout; giữ nguyên archive, dataset và baseline đã bàn giao. Không làm theo gợi ý merge-tree đưa code mới vào thư mục report lịch sử.
2. Sửa calibration, đơn vị depth, timestamp/odom và fallback inference; bổ sung interface ID/validity nếu dùng cho data.
3. Chạy detector cũ/mới trên cùng vài recordings đã có, sửa metric matching; không cần mở rộng world hoặc thu data mới ngay.
4. Khi các gate đạt, merge dạng tùy chọn thử nghiệm, chưa thay detector mặc định. Chỉ cập nhật gói bàn giao bằng một version mới; không sửa checksum của pilot cũ.

Lượt review này chỉ thêm tài liệu và receipts. Runtime, Nav2, model, raw data và freezes đã nghiệm thu được giữ nguyên. Chưa mở Gazebo/RViz hay đánh giá installed ROS node.

## Nguồn và cách đọc log

- [Commit được review](https://github.com/CuongBien/DynamuhSim/commit/70bcda0e26ff86b3389a49295feda3ef72958b1d)
- [ROS wrapper](https://github.com/CuongBien/DynamuhSim/blob/70bcda0e26ff86b3389a49295feda3ef72958b1d/src/custom_corridor/scripts/depth_person_tracker_node.py)
- [Detector thực tế pipeline import](https://github.com/CuongBien/DynamuhSim/blob/70bcda0e26ff86b3389a49295feda3ef72958b1d/experiments/social_navigation/benchmark/depth_person_classifier/artifacts/generalized_detector.py)
- [Metric 30 cảnh](https://github.com/CuongBien/DynamuhSim/blob/70bcda0e26ff86b3389a49295feda3ef72958b1d/experiments/social_navigation/benchmark/depth_identity_guard/evaluate_multi_environments.py#L369)

Thư mục này có result.json, ba log test/benchmark, merge_tree.log và reproduce_findings.py. Reproducer nhận --snapshot trỏ đến git archive của commit được review và chỉ chạy hai lỗi có kiểm soát; không dùng ROS runtime hay nhãn GT.
