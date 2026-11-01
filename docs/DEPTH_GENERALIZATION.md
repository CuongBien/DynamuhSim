# Vòng đánh giá detector — nhánh feat/depth-detector-generalization

Báo cáo trực quan: http://localhost:35315/depth_generalization_v1/index.html

Nhánh/worktree riêng từ dd7a2b0, không sửa feat/mujoco-backend. Vòng này gồm một ablation đóng băng, chuẩn bị benchmark ngoài miền và chốt một ứng viên train thử. Không mở Gazebo hoặc huấn luyện trên dữ liệu hiện có.

## Kết luận số liệu

| Episode | 2D cũ, 0,50 | 3D giữ prior, 0,45 | 3D bỏ prior, 0,45 | 3D bỏ prior, 0,50 control |
|---|---:|---:|---:|---:|
| Conversation | 82,6% | 8,2% | 56,1% | 55,7% |
| Walking | 90,7% | 7,6% | 61,4% | 61,2% |

Các số là coverage hình học trên GT có hỗ trợ depth, không phải semantic recall. 910 frame replay, 905 đủ odom/GT; 1.820 dòng output cũ được tái lập đúng. Control 0,50 giúp tránh quy việc giảm threshold cho tác dụng bỏ prior. Cả hai nhánh no-prior đều FAIL gate giữ >=95% paired của baseline ở cả hai episode; conversation còn tăng fragmentation.

Stage counts (lượt GT/frame, cùng body/world, không độc lập thống kê):

- Conversation: hỗ trợ1134 → proposal3D802 → planar713 → classifier không prior636 → có prior93.
- Walking: hỗ trợ536 → proposal3D497 → planar363 → classifier không prior329 → có prior41.
- Prior làm mất thêm831 lượt ghép. Proposal3D bao gồm envelope/density/size/torso rules; không gọi đây là clustering thuần. Lọc có thể làm mất hoặc phục hồi pairing do ambiguity, JSON báo cả hai, không coi các cột là upper bound toán học.

## Ứng viên đã chốt

Giữ proposal 2D recall cao và toàn bộ XYZ quan sát; thử head học đặc trưng PointNet-style nhỏ thay cho 30 feature thủ công/luật nhân score. Đây vẫn là mô hình sử dụng thông tin 3D, không phải bỏ depth/3D. Head được viết riêng bằng Torch, 18.881 tham số; không phải implementation/checkpoint người pretrained của PointNet gốc.

Đã chạy forward/backward trên RTX4060, kiểm permutation invariance, padding mask, empty patch và weights không đổi. **0 optimizer step, chưa có accuracy của head.** Proposal mất/gộp người vẫn là giới hạn; kết quả local không chứng minh head sẽ tổng quát hóa.

[Kiến trúc PointNet của tác giả](https://github.com/charlesq34/pointnet) là tham khảo phương pháp; repo gốc dùng Python2.7/TF1, không cài vào môi trường Jazzy. Mã inference của nhánh hiện tại không bị thay.

## Benchmark đa miền: contract đã có, dữ liệu chưa đủ

`benchmark_manifest.json` khai báo8 case, scene/avatar morphology disjoint giữa train/validation/test, mỗi split có no-person/clutter control. Validator kiểm metadata/hash, trùng scene/morphology/episode, file thiếu và loại nhãn; không tự cho phép train/test. Trạng thái hiện tại **NOT_READY**, các case mới chưa thu. Đây là thiết kế benchmark nhỏ, không phải đã nghiệm thu generalization.

Inventory geometry-array cho thấy walk, walk_blue, walk_orange, walk_red, stand và push_trolley dùng cùng fingerprint. Đổi màu/clip animation không tạo test người độc lập. Hash khác cũng chưa tự chứng minh morphology độc lập: phải quản lý family của source mesh/rest geometry, không lấy pose đã bake làm danh tính mới.

Hai pilot và corpus nhỏ cũ chỉ là development/regression; test classifier cũ đã mở không được tái dùng như unseen test. Pixel owner/instance mask đã kiểm mới là nhãn; không dùng tâm GT gần nhất hoặc dự đoán hiện tại làm semantic truth. H1–H5/association/WM/RL giữ chưa được bật.

### Dataset/model kiểm theo nguồn tác giả

Xem `source_review.json` để có URL và scope:

- DR-SPAAM: giữ comparator LiDAR; lượt cũ chạy CUDA nhưng semantic gate FAIL. LiDAR local nhìn collision cylinder, khác articulated leg returns.
- FROG/DROW: nguồn real 2D LiDAR riêng, không thay cho dữ liệu depth của head XYZ. Cấu hình sensor khác, cần adapter/đánh giá riêng.
- SDT: depth/thermal có person boxes, synthetic và real nhưng camera đặt cao. Chưa tải; còn kiểm K/units/partial-view và khả năng tạo nhãn patch sạch từ boxes.
- LFE-PPN: author runtime Humble/Ubuntu22.04; chưa xác nhận Jazzy. Không cài .deb hay thay hệ thống.
- Nghiên cứu synthetic depth robot person detection: ủng hộ dữ liệu đa dạng/noise và kiểm real warehouse; không coi paper là checkpoint đã chạy trên robot này.

Không có model/dataset nào trong bảng được tuyên bố zero-shot PASS ở vòng này. Benchmark MuJoCo được bổ sung sau khi backend/sensor contract đạt; không chặn bàn giao backend.

## Cách tái lập

Trong worktree nhánh này, truyền path dữ liệu gốc rõ ràng; output phải mới. Các công cụ dùng NumPy/SciPy, render thêm Matplotlib/Pillow. Không cài dependency mới.

```bash
TASK=experiments/social_navigation/perception/depth_generalization
DATA=/home/cuongbien/DynamuhSim/experiments/social_navigation/datasets/handoff_pilot/generated/pilot01/normalized
REF=/home/cuongbien/DynamuhSim/experiments/social_navigation/perception/depth_identity_guard/generated/integration_20261030_01/comparison
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
 python3 "$TASK/ablation.py" --normalized-root "$DATA" --reference "$REF" --output "$TASK/generated/my_new_run"
python3 "$TASK/benchmark_readiness.py" --manifest "$TASK/benchmark_manifest.json" --data-root /PATH/TO/NEW_DOMAINS --output "$TASK/generated/my_readiness.json"
```

Readiness exit2 là NOT_READY được dự kiến khi dữ liệu chưa có. `report.py --run ... --output ...` tạo một trang mới, có bbox đo thật cho ba cấu hình; không ghi đè report cũ.

`probe_candidate.py` cần venv CUDA riêng đã có, chỉ smoke forward/backward, không fit. Không gọi Python hệ thống để tự cài Torch hoặc model.

Bằng chứng versioned: [verification](verification/depth_generalization_2026-10-30/README.md). Raw JSONL/images đầy đủ ở generated của worktree, ngoài Git.
