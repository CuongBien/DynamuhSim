# Bàn giao công việc detector người — 30/10/2026

## 1. Đọc phần này trước

Công việc đang bàn giao là **cải thiện khả năng phân biệt người/vật từ depth, hướng tới tổng quát hóa sang cảnh và mẫu người mới**. Đã hoàn thành phân tích nguyên nhân detector mới kém hơn bản cũ. Chưa train ứng viên mới vì chưa có dữ liệu đủ độc lập và nhãn instance đã kiểm.

Việc đầu tiên người nhận cần làm: **kiểm kê/chọn các human mesh độc lập thật sự, kiểm tra nhãn instance từ cảm biến, và chuẩn bị collection train/validation/test**. Chưa cần lặp lại ablation hoặc sửa luật theo từng cảnh. Vòng tiếp theo phải có điểm dừng rõ trước khi mở test.

MuJoCo là luồng riêng. Nhận công việc detector không đồng nghĩa nhận port simulator; không merge chéo hai luồng khi chưa đối chiếu contract.

## 2. Code ở đâu? Nhánh nào là đúng?

Repository: https://github.com/CuongBien/DynamuhSim

| Luồng | Nhánh | Mốc đối chiếu ngày 30/10 | Vai trò |
|---|---|---|---|
| Công việc này | feat/depth-detector-generalization | 908ead7a8f37f86765e0236ee8512a99c435d492 | Ablation, audit báo cáo, candidate XYZ và benchmark preparation |
| Chuyển backend | feat/mujoco-backend | b777e646621c25ec3312f20164f9a0be63a0586a | Runbook/workspace bàn giao; chưa triển khai MuJoCo |
| Tham chiếu Gazebo | test/gazebo-harmonic-benchmark | dd7a2b03d9fe40841052c0db6c7dc52619e13e67 | Baseline tích hợp và báo cáo detector v2 |
| Code bạn khác gửi | feat/depth-person-detector | Commit nguồn 70bcda0e26ff86b3389a49295feda3ef72958b1d | Đã merge trước đó, không cần merge lại |

Nhánh detector tách từ dd7a2b0, không chứa riêng các commit chuẩn bị MuJoCo b777e64. Hai nhánh cùng tồn tại, không phải một nhánh là toàn bộ thay đổi của nhánh kia.

Trên máy tác giả, checkout chính `/home/cuongbien/DynamuhSim` đang ở feat/mujoco-backend. Checkout làm detector là worktree khác:

```text
/home/cuongbien/DynamuhSim/experiments/social_navigation/perception/depth_identity_guard/generated/generalization_worktree
```

Đừng sửa detector trong checkout chính rồi tưởng đang làm trên nhánh detector. Worktree nằm dưới generated nhưng có Git branch thật; commit đã push vẫn có trên GitHub. Người nhận không cần tái tạo đường dẫn dài này; clone nhánh vào thư mục bình thường được.

```bash
git clone --branch feat/depth-detector-generalization --single-branch https://github.com/CuongBien/DynamuhSim.git DynamuhSim-detector
cd DynamuhSim-detector
git status --short --branch
git rev-parse HEAD
```

SHA ở trên là mốc đối chiếu; nếu nhánh có commit mới, đọc diff từ 908ead7 thay vì reset/xóa thay đổi. Không đặt tên nhánh mới có prefix codex/; người dùng yêu cầu dùng tên như feat/....

Tài liệu bàn giao này, LOCAL_REPORTS.md và tools/build_report_hub.py được thêm trong commit bàn giao sau 908ead7 trên chính nhánh feat/depth-detector-generalization. Người nhận clone đầu nhánh sẽ có chúng; nếu checkout đúng mốc 908ead7 thì chưa có. Xem SHA commit bàn giao bằng git log -1. Bundle localhost là bản sao ba file này để tiện chuyển, không phải bản sao toàn repository hoặc dataset.

## 3. Robot và các ràng buộc cần giữ

- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.3, Gazebo Harmonic; máy tham chiếu có RTX 4060 Laptop.
- Robot Mecanum; perception dùng LiDAR 2D + depth, odometry và calibration/TF. Không thêm RGB.
- Nav2 hiện dùng NavFn/Dijkstra và MPPI Omni, AMCL định vị. Chưa dùng WM/RL điều khiển robot.
- Detector mới là opt-in experimental. Không đổi default ROS/Nav2, kiến trúc robot hoặc dữ liệu tham chiếu để chạy thử.
- Capture/native simulation và neural model nên dùng GPU; NumPy/SciPy đặt OMP/OPENBLAS/MKL threads=1 khi replay. Smoke CUDA không có nghĩa toàn bộ xử lý đã chạy GPU.
- Giữ FAIL/ERROR cũ, source/input hashes, seal và báo cáo. Mỗi run/output mới cần đường dẫn mới, không ghi đè.
- Chưa có semantic/persona H1–H5/association được nghiệm thu; không dùng các nhãn đó như ground truth đã sẵn sàng. Chưa train WM hoặc RL.

## 4. Đã làm gì và kết luận nào được chứng minh?

### Tích hợp code bạn gửi

Đã merge detector vào nhánh tham chiếu, sửa đầu vào CameraInfo/TF, depth 16UC1/32FC1, causal odom, reset theo thời gian, diagnostics và cài resources riêng. Node thử nghiệm ở `src/custom_corridor/scripts/depth_person_tracker_node.py`, launch opt-in ở `src/custom_corridor/launch/depth_person_tracker.launch.py`.

Output dưới `/perception/experimental_depth/` gồm tracks_json, markers và poses. Đây không phải nguồn obstacle mặc định của Nav2. ROS transport smoke đã PASS; chất lượng semantic và live Gazebo detector chưa được chấp nhận.

### Vì sao bản mới kém?

Weights depth_linear cũ không được train lại. Proposal 3D mới có thêm các bước lọc hình học và prior nhân score. Rule cụm thấp/rộng có thể nhân 0,15: người chỉ thấy một phần có score 0,76–1,00 bị giảm xuống 0,11–0,15, thấp hơn ngưỡng 0,45. Có cả mất proposal trước classifier nên chỉ train lại classifier không thể sửa mọi lỗi.

Đã chạy bốn cấu hình đóng băng trên cùng hai recordings, 910 frame; 905 frame đủ odom/GT để đánh giá:

| Episode | Cũ 2D + classifier 0,50 | 3D + prior 0,45 | 3D bỏ prior 0,45 | 3D bỏ prior 0,50 |
|---|---:|---:|---:|---:|
| Conversation | 82,6% | 8,2% | 56,1% | 55,7% |
| Walking | 90,7% | 7,6% | 61,4% | 61,2% |

Đây là coverage proxy ghép vị trí GT với hỗ trợ depth, **không phải semantic recall**. Bỏ prior giúp rõ nhưng vẫn FAIL gate giữ >=95% paired của baseline ở cả hai episode. Không có cấu hình mới nào được promote. Prior làm mất thêm 831 lượt ghép GT/frame; các lượt không phải mẫu độc lập thống kê.

### Vì sao báo cáo của người gửi có vẻ rất tốt?

Bộ 30 ca Hospital/School gốc là point cloud người hình trụ và đồ vật khối tự sinh, không phải 30 captures depth từ các world đó. Metric gốc đếm `TP=min(detected,expected)`. Những ca đó vẫn PASS sau khi sửa matching vị trí, nên vấn đề còn nằm ở độ đơn giản/phạm vi bài test, không chỉ công thức metric.

23 TP/1 FN/24 TN/0 FP của classifier cũ là kết quả thật trên 48 patch trong corpus nhỏ, cùng world/mesh; khác full-scene detection và generalization. Các loại tỷ lệ này không được so như một accuracy duy nhất.

Đã replay detector API nguyên commit 70bcda0 và sau merge với cùng calibration đúng/input: 910/910 frame trùng số detection, XY, ID, point-count và confirmed. Không có bằng chứng merge làm giảm core detector trong phép thử này. Không suy rộng thành mọi ROS runtime đều tương đương: wrapper gốc từng có calibration hard-code sai.

### Kiểm chứng gần nhất

Suite tại commit implementation 43153c7: 338 tests OK, 2 conditional skips; 38 contract smoke nằm trong phạm vi suite, không cộng thành tổng khác. 116 protected baseline files giữ nguyên. Các receipt nằm trong docs/verification, không phải lời hứa chất lượng cho model chưa train.

Chi tiết: [Ablation và quyết định](DEPTH_GENERALIZATION.md), [bằng chứng ablation](verification/depth_generalization_2026-10-30/README.md), [audit báo cáo nguồn](verification/source_report_audit_2026-10-30/README.md).

## 5. Hướng đã chọn và những gì chưa tồn tại

Giữ proposal geometry 2D cũ có recall tốt hơn, lấy toàn bộ XYZ quan sát của patch, rồi thử head PointNet-style nhỏ học người/vật. “2D proposal” không có nghĩa bỏ dữ liệu 3D hoặc chuyển sang ảnh RGB.

Tại `experiments/social_navigation/perception/depth_generalization/`:

| File | Trạng thái / mục đích |
|---|---|
| protocol.json, ablation.py | Ablation đóng băng đã chạy; không quét threshold để làm đẹp kết quả |
| candidate_decision.json | Quyết định provider và phần giữ/bỏ |
| candidate.py | Head PyTorch riêng, 18.881 parameters; input XYZ + valid mask |
| probe_candidate.py | Chỉ forward/backward CUDA, 5 check PASS, 0 optimizer step |
| training_recipe.json | Recipe PREPARED_NOT_RUN; chưa có trained weights của head |
| benchmark_manifest.json | 8 case PLANNED/NOT_COLLECTED, chưa có benchmark đa miền mới |
| benchmark_readiness.py | Metadata/hash/leakage checker; hiện NOT_READY |
| source_review.json | Rà soát dataset/model theo nguồn tác giả; chưa download |
| report.py | Render báo cáo mới trên output ablation |

Head này không phải checkpoint person pretrained từ PointNet gốc. Chưa có trainer end-to-end, pipeline pixel ownership đã nghiệm thu cho collection mới, hay accuracy của head. Các phần đó còn phải triển khai/kiểm chứng. Không thể chạy một lệnh rồi coi như đã hoàn tất train.

Giới hạn cốt lõi: classifier patch không lấy lại người đã mất/gộp ở proposal stage. Luôn đo riêng proposal recall và classifier, đồng thời đánh giá toàn pipeline.

## 6. Dữ liệu hiện có và cách chuyển sang máy khác

Trên máy tác giả, prefix dữ liệu gốc là `/home/cuongbien/DynamuhSim`; không phải bên trong detector worktree.

| Artifact | Đường dẫn dưới prefix gốc | Ý nghĩa |
|---|---|---|
| Normalized pilot | experiments/social_navigation/datasets/handoff_pilot/generated/pilot01/normalized | pilot_conversation, pilot_walking; dùng development/regression |
| ZIP portable | experiments/social_navigation/datasets/handoff_pilot/generated/package01/handoff_data_pilot01.zip | 41 file, 31.706.091 byte; không chứa raw MCAP |
| Replay reference | experiments/social_navigation/perception/depth_identity_guard/generated/integration_20261030_01/comparison | Output trước ablation dùng đối chiếu |
| Web/media | experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer | Toàn bộ báo cáo localhost, ảnh/video/JSON |

Trong mỗi episode normalized có depth.npy, observations.npz, calibration.json, samples.npz, labels.npz, human_states.jsonl, events.json và qa.json. Tổng hai episode có 910 depth frames, 517 windows. labels/GT tách riêng; không đưa vào input detector. ZIP SHA256:

```text
b64bdb394f4a2ddabc6d97f2e9ccd3704dd9c6b0855e5d52fb11c6eeaea23ee2
```

Kết quả raw ablation nằm dưới `experiments/social_navigation/perception/depth_generalization/generated/ablation01` của **detector worktree**. Receipts versioned là bản nhỏ trong docs/verification. Raw MCAP ở capture gốc; phải chuyển riêng nếu cần, không có trong ZIP.

Các thư mục generated và media trên máy không được Git mang theo. Hai pilot/corpus cũ đã mở để xem lỗi: chỉ dùng regression/dev, không đổi tên thành unseen test. Pilot này vẫn pilot_only, training_authorized=false; không phải dataset train WM/RL hoàn chỉnh.

Cần gửi cho người nhận: URL+SHA nhánh, tài liệu này, ZIP pilot; thêm replay reference nếu muốn tái lập ablation đầy đủ, và nguyên thư mục viewer nếu muốn xem toàn bộ web offline. Không gửi riêng index.html vì sẽ mất ảnh/video/JSON. Bundle bàn giao nhỏ không thay cho các artifact này.

## 7. Các bước tiếp theo theo thứ tự

### Bước A — Chọn assets và kiểm tra khả năng thu nhãn

Inventory đã xác nhận walk, walk_blue, walk_orange, walk_red, stand và push_trolley dùng cùng geometry fingerprint. Đổi màu hoặc animation không tạo body family mới. Hash khác do pose bake cũng không đủ.

Chọn tối thiểu các family độc lập A1, A2, B1, C1 theo manifest; ghi nguồn, license, rest geometry, kích thước và family. Chuẩn bị layout riêng cho train/validation/test. Các tên factory/hospital/school/narrow hiện chỉ là kế hoạch, chưa có captures mới để nghiệm thu.

Kiểm tra mô hình nhìn được từ depth thấp, occlusion, khoảng cách và khả năng sinh pixel-instance ownership cùng timestamp/calibration. LiDAR local đang thấy collision cylinder, không phải articulated legs; không coi leg detector pretrained là tương thích tự động.

**Điểm dừng đầu tiên:** inventory/source/license + sơ đồ split + bằng chứng depth/owner-mask đồng bộ trong một smoke development nhỏ. Không cần train hoặc mở nội dung test để đạt bước này. Nếu backend chưa có owner masks, ghi rõ và làm adapter nhãn trước; không thay bằng nearest-GT-center.

### Bước B — Thu mini benchmark và QA nhãn

Giữ 8 case trong manifest: train factory/hospital với người và controls; validation school với người/control; test narrow với người/control. Scene geometry, body family và episode không trùng giữa splits. Không random-split frame liền kề.

Thu depth + LiDAR + odom + calibration, GT ownership tách riêng; lưu scene/avatar/sensor/source hashes và provenance. Mỗi split có no-person/clutter để đo báo nhầm. Proposal thiếu người vẫn phải được tính như lỗi detector.

Manifest/validator yêu cầu depth.npy, observations.npz, calibration.json, person_instance_LABEL_ONLY.npz và hashes; validator hiện kiểm metadata/hash, không kiểm đầy đủ schema, nội dung mask, semantic QA hay toàn bộ modalities. Người nhận phải hoàn tất schema của nhãn và QA/đồng bộ, không chỉ tạo file đúng tên. Existing pilot không có pixel-instance ownership đã kiểm cho benchmark mới.

Giữ test niêm phong sau khi thu và QA dữ liệu. Không xem detector predictions hoặc chỉnh theo lỗi test trước model freeze. Không tuyên bố “tổng quát hóa” chỉ vì đổi simulator nhưng dùng cùng geometry/sensor rendering.

### Bước C — Một lượt train/validation có kiểm soát

Sau khi QA đạt, triển khai trainer theo recipe, giữ provider cố định ở mọi split. Seed17, CUDA, max512 points/patch, batch32, AdamW lr0,001, max30 epochs, early stop5. Không thêm GT/persona/scene/backend ID hay future input vào model.

Positive label theo ownership purity >=0,90 và >=5 person points; negative chỉ khi toàn bộ points có ownership là nonperson; patch mơ hồ bỏ qua. Augmentation chỉ train. Chọn epoch/threshold bằng validation macro-F1 toàn scene; tie theo FPR thấp hơn rồi threshold gần0,5. Recipe có threshold grid cố định, không mở test để chọn.

Đóng băng weights/provider/calibration/preprocessing/threshold và source/data hashes; sau đó mới đánh giá test một lần. Recipe và readiness hiện đều khóa training_authorized=false. Chỉ cập nhật quyền/trạng thái bằng receipt curation/model-freeze thật, không gạt cờ để bỏ qua kiểm tra thiếu dữ liệu.

### Bước D — Báo cáo kết quả rồi dừng để quyết định

So sánh cùng input: baseline cũ, 3D prior, 3D no-prior và candidate đã train. Trên bộ mới có semantic labels, báo proposal recall, patch metrics, full-detector precision/recall/F1, FP ở cảnh không người, phân theo scene/body/distance/occlusion, latency end-to-end và lỗi điển hình. Nêu rõ điều kiện visibility và tiêu chí matching đã chốt trước test; không thay ground truth bằng geometric proxy cũ.

Không có ngưỡng nghiệm thu semantic test mới đã chốt. Người nhận cần đề xuất/freeze chúng từ mục tiêu vận hành và development trước khi mở test; không tái dùng gate paired-retention cũ như chứng minh semantic quality.

Báo cáo localhost phải có khung dự đoán người, ID/score trên depth thật, GT tham chiếu riêng, lỗi bỏ sót/báo nhầm, đủ các kết quả FAIL. Kết luận giữ/bỏ head hoặc cần thay proposal; chưa tự tích hợp default hay bắt đầu WM/RL. Đây là điểm dừng vòng detector nhỏ, không mở vòng chỉnh luật vô hạn.

## 8. Lệnh onboarding và kiểm tra

Từ root checkout detector, Python3.12 với dependencies theo docs/HANDOFF_SETUP.md. Không pip vào Python hệ thống hay cài Torch/ROS vào venv của nhau. NumPy/SciPy dùng cho detector; render thêm Matplotlib/Pillow; Torch CUDA dùng venv riêng. Môi trường CUDA đã probe trên máy tác giả: Torch2.7.1+cu128, CUDA12.8, RTX4060; máy nhận cần kiểm lại, không mặc định tương thích mọi driver.

```bash
python3 tools/handoff.py baseline
python3 tools/handoff.py doctor --scope data
python3 tools/handoff.py smoke-contract
PYTHONPATH=python:tests OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 -m unittest test_depth_person_integration test_depth_generalization test_depth_benchmark_readiness
```

Có ZIP pilot thì kiểm restore trong thư mục mới chưa tồn tại:

```bash
python3 tools/handoff.py smoke-data --archive /ABS/PATH/handoff_data_pilot01.zip --work-dir /ABS/PATH/NEW_RESTORE_DIR
```

Kiểm readiness với data root mới và output chưa tồn tại:

```bash
TASK=experiments/social_navigation/perception/depth_generalization
python3 "$TASK/benchmark_readiness.py" --manifest "$TASK/benchmark_manifest.json" --data-root /ABS/PATH/NEW_DATA --output "$TASK/generated/receiver_readiness01.json"
```

NOT_READY/exit2 là kết quả dự kiến khi chưa thu dữ liệu; không sửa checker để biến thành PASS. Readiness READY cũng không tự chứng minh chất lượng nhãn hoặc cho phép mở test. Cách replay ablation nếu cần điều tra mới nằm trong DEPTH_GENERALIZATION.md; không bắt buộc lặp lại 910 frames để bắt đầu assets.

## 9. Xem báo cáo và ranh giới MuJoCo

- [Trang chính tất cả báo cáo](http://localhost:35315/).
- [Khung detector trên depth](http://localhost:35315/depth_person_detector_v2/index.html#detector).
- [Ablation và benchmark chuẩn bị](http://localhost:35315/depth_generalization_v1/index.html).
- [Audit báo cáo nguồn](http://localhost:35315/source_report_audit_v1/index.html).
- [Cách mở server, vị trí media và mục lục](LOCAL_REPORTS.md).

localhost là máy đang chạy server, không phải link public: người nhận ở máy khác phải có bản sao viewer và mở server ở máy họ. Ảnh/overlay từ depth Gazebo đã ghi, không phải ảnh AI; GIF là slideshow, không phải video realtime.

MuJoCo chưa triển khai backend trong các mốc trên. Người làm port dùng nhánh feat/mujoco-backend, đọc docs/MUJOCO_HANDOVER.md và docs/HANDOFF_START.md **trên nhánh đó**. Detector nên nhận depth/XYZ + calibration + causal odom qua contract chung; không phụ thuộc tên model/world để dự đoán người. Sau khi adapter MuJoCo đạt contract mới cân nhắc bổ sung backend test, không bắt detector chờ port hoặc giả định có thể copy Gazebo sensor plugin sang MuJoCo.

## 10. Tin nhắn giao việc có thể gửi ngay

Bạn nhận phần detector trên nhánh feat/depth-detector-generalization, mốc 908ead7. Hãy đọc tài liệu bàn giao này rồi DEPTH_GENERALIZATION.md và audit báo cáo nguồn. Ablation đã xong: prior và proposal 3D làm mất người; original API và sau merge khớp trên910 frame. Hướng tiếp theo là provider 2D cũ + head học XYZ, chưa train.

Bắt đầu bằng assets thật sự khác morphology và kiểm pixel-instance labels; gửi inventory, split plan và một smoke depth/mask trước khi thu batch/train. Giữ LiDAR+depth, không RGB; giữ ROS/Nav2 default và nhánh MuJoCo độc lập. Pilot cũ chỉ regression, không phải unseen test. Mọi run có output mới, báo cáo localhost có bbox/score và giữ cả lỗi. Sau một vòng train/validation rồi freeze/test, báo kết luận và dừng để quyết định, chưa làm WM/RL.
