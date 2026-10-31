# Review và tích hợp detector cập nhật — 31/10/2026

Nguồn: hai commit bab853ea và 2ffd7c75218778f328d692724f382f623aab7139 trên feat/depth-detector-generalization; baseline trước nhận 879aaf41229250de8eb4288d3105f12322f15dda. Đã fast-forward vào riêng checkout detector, không merge vào feat/mujoco-backend. Các file cuong.txt và tools/mcp_server/ của người dùng trong checkout chính không bị động tới.

## Kết luận

Nhận code và giữ artifacts mới để thử nghiệm; **không promote weights hoặc world mới**. Weights linear mới có ích ở walking nhưng giảm conversation. PointNet không đánh giá lại được vì checkpoint/receipts không có trong Git hoặc local. Vòng này không train, không mở Gazebo, không đọc lại test classifier cũ hoặc chạy predictions trên test synthetic mới.

Báo cáo có bbox: http://localhost:35315/incoming_detector_review_v1/index.html.

| Cấu hình | Conversation coverage / paired / unpaired | Walking coverage / paired / unpaired |
|---|---|---|
| Cũ 2D0,50 | 82,6% /1059/90 | 90,7% /488/160 |
| Weights mới 2D0,50 | 70,5% /801/50 | 91,4% /490/133 |
| Weights mới 2D0,85 | 65,3% /741/43 | 89,2% /478/116 |
| Weights mới 3Dprior0,45 | 4,9% /55/69 | 6,3% /34/92 |

910frame replay,905frame đủ odom/GT,910baseline rows khớp reference. Coverage là hỗ trợ depth + ghép hình học, không phải semantic recall. Unpaired không đồng nghĩa chắc chắn FP. Conversation fragment proxy2→4/7; walking2→0/0. Hai ngưỡng cố định trước replay, không sweep. Cả hai 2D mới PASS gate cục bộ walking nhưng FAIL conversation; 3D mới FAIL cả hai. Gate >=95% paired, không tăng unpaired/ID-switch/fragmentation; không phải gate generalization. Mỗi variant dùng tracker riêng, source/input/output hashes giữ nguyên; seal tất cả sensor outputs trước khi đọc GT. Không có so sánh latency giữa variant vì shared instrumentation.

## Phần bổ sung đáng giữ

- Trainer head XYZ, trainer linear, builder depth/mask/LiDAR và QA/render đã có, trước đây chỉ có candidate và recipe.
- Weights mới là linear30features,31parameters; không phải weights PointNet. API frozen Classifier tải được và chạy NumPy.
- Bản nguồn đổi5world sang mesh người và thêm collision; vẫn giữ trong Git history để thử opt-in sau, chưa nghiệm thu native sensor/collision của các world này.
- Snapshot wm_restore thêm báo cáo/loader/sample WM, nằm ngoài phạm vi nghiệm thu detector. Receipt nguồn nói8accepted/2quarantine và3GB ngoài Git; chỉ ghi nhận receipt, chưa restore/audit full bundle mới hoặc trainWM/RL.

## Vấn đề bằng chứng/phương pháp

1. build_mini_benchmark.py dựng80frame bằng renderer NumPy/SciPy, floor/wall/box thủ công, human point splat + minimum_filter; không phải captures từ4worldGazebo. Đừng gọi đây là robot sensor benchmark đã generalize.
2. CASES_SPEC dùng walk_blue/orange/red/stand, cùng geometry-array fingerprint và height1,716m. A1/A2/B1/C1 là labels khác, không phải morphology độc lập. smoke_step_a.py liệt kê source/license/vertex-count dưới dạng literal, không chứng minh đã nhập assets đó.
3. Scene hashes được gán sẵn trong code; school/narrow dài63hex, không phải64. Hash của file dữ liệu thực có thể đúng; nó không xác minh source-family độc lập. Readiness READY là metadata/hash pass với training/test_authorized=false, không phải quality pass.
4. frozen_candidate_weights.pt, model_freeze.json, training_curation_receipt.json, test_evaluation_receipt.json không có trong commit/local. Không tuyên bố tái lập được PointNet hoặc kết luận head đó thất bại từ artifacts chưa có.
5. evaluate_step_d.py nguồn tải3D baseline từ default đã bị thay bằng linear mới, nên baseline/ablation không còn đúng weights cũ; có thể tính nhiềuTP cho1người; timing chưa gồm projection/ROS,3D đo thêm extraction lặp. Gate được khởi tạo sau khi metrics đã chạy, không có receipt freeze chứng minh pre-test. Train selection là patch-pooled macroF1, không phải whole-scene macroF1 theo recipe. Các chữ ký test-once/curation_verified được script tự ghi, chưa tự chứng minh governance.
6. Trainer linear ghi đè default trước evaluation và set validation_gate=true vô điều kiện. Không tìm thấy checkpoint+receipt để xác minh lựa chọn threshold0,85 bằng validation trước test; không khẳng định chắc chắn có test tuning, nhưng chưa đủ chứng cứ nói không có.

## Sửa khi tích hợp

- artifacts/selected_model.npz và selected_frozen.json khôi phục byte baseline879aaf4. Model mới chuyển sang depth_person_classifier/artifacts_experimental_2ffd7c75; giữ NPZ nguyên SHA, giữ metadata nguồn ở incoming_selected_frozen.json, metadata active experimental validation_gate=false/promotion=false.
- Năm world mặc định khôi phục byte baseline. Thay đổi nguồn vẫn ở2ffd7c75, không xóa lịch sử. Không thay default ROS/Nav2.
- training_recipe.json giữ recipe chưa nghiệm thu; bản tuyên bố training phía nguồn được giữ training_recipe_incoming_2ffd7c75.json. Đó không phải cấp phép fit trên dữ liệu thiếu độc lập.
- train_and_deploy_robust_detector.py bắt buộc --output-dir mới, chặn default/legacy, ghi receipt experimental tại output; không tự deploy. evaluate_step_d.py baseline3D dùng artifacts_legacy và linear mới dùng experimental. Matching đổi sang Hungarian one-to-one; scope timing/status được ghi lại là procedural smoke, không acceptance. Những sửa này không tạo lại training receipts hoặc chứng minh test sealed.
- Thêm6regression tests cho leak morphology, giữ model cũ và output-dir guard. Không có sự thay thế ground truth hay tiêu chuẩn để biến FAIL thành PASS.

## Kiểm chứng và cách tái lập

Full suite nguồn338OK/2skip; sau tích hợp344OK/2skip; xem log và acceptance trong docs/verification/incoming_detector_review_2026-10-31/. Các StartingGazebo/timeout trong log là mock failure-path tests, không có simulator run.116protected filesPASS; nămworld/defaultweights còn kiểm riêng vì không nằm hết trong116guard.

Từ root checkout nhánh detector; dữ liệu/raw/reference phải chuyển riêng như DETECTOR_HANDOVER.md. Chỉ NumPy/SciPy cho replay, không Torch/GPU training. Output mới chưa tồn tại:

```bash
TASK=experiments/social_navigation/perception/depth_generalization
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 "$TASK/review_incoming.py" --normalized-root /ABS/pilot01/normalized --reference /ABS/integration_20261030_01/comparison --output "$TASK/generated/receiver_review01/replay"
```

Renderer còn cần Matplotlib/Pillow, log suite và baseline ở thư mục cha run, ví dụ incoming_review_20261031_01; xem generated của máy tác giả hoặc tạo log kiểm tra của máy nhận. `render_incoming_review.py --run RUN/replay --output NEW_VIEWER_FOLDER` chỉ vẽ4frame cố định/episode và kiểm counts/XY/scores với seal. Không dùng GT tạo bbox.

Bước tiếp theo: cung cấp assets thực độc lập với provenance và owner-mask QA, checkpoint+receipts còn thiếu; thiết kế/freeze semantic gates trước test mới. Không cần lặp ablation hoặc tiếp tục chỉnh luật theo từng scene. Các oldpilot và synthetic test đã được công bố/đánh giá không được đổi tên thành unseen test.
