# Cải tiến track depth — so sánh ba lượt ghi

**Cải thiện hình học trên hai lượt depth mesh; không đạt trên dữ liệu proxy cũ. Chưa thay baseline hoặc bật train.** [Báo cáo](reports/REFINEMENT_REPORT.md), [quyết định nghiệm thu](reports/acceptance.json), [audit](reports/audit01/audit.json), [13 test](reports/contract_tests.txt), [hình so sánh dữ liệu thật](reports/profile_comparison.png).

Code thử gồm lọc bề mặt đứng cao dạng cột và nối lại ID sau mất quan sát ngắn nếu vị trí/hình dạng đủ rõ. Đây vẫn là candidate tracker theo hình học, chưa phải person detector được xác thực. Bốn ablation dùng cùng code/tham số cố định: baseline, shape_only, tracking_only, combined.

| Lượt ghi | Ghép hình học baseline → combined | Phân mảnh ID baseline → combined | Kết luận |
|---|---|---|---|
| Phát triển, depth mesh OFF proxy | 77.74% → 93.71% | 13 → 10 | Cải thiện tại cảnh phát triển |
| Bổ sung cùng depth mesh | 75.62% → 90.09% | 10 → 8 | Chẩn đoán bổ sung; navigation cũ vẫn TIMEOUT |
| Kiểm tra ban đầu, depth proxy cũ | 91.38% → 92.34%, nhưng paired rows 997 → 603 | 3 → 3 | **FAIL**, mất gần 40% paired rows |

Tỉ lệ ghép không phải accuracy/precision/recall. Tham số không sửa sau khi xem validation. Lượt mesh bổ sung được chọn sau failure để tìm hiểu domain; không thay thế validation đã fail hoặc trở thành held-out test. Cả ba cùng simulator/seed/nhóm cảnh. HuNavSim không được dùng.

## Chạy lại offline

Python 3.12, NumPy/SciPy; Matplotlib cho ảnh. Wrapper giới hạn BLAS/OMP/MKL một luồng. Không cần ROS/Gazebo cho inference và evaluation. Chọn thư mục output mới; các lệnh không ghi đè raw hay kết quả cũ.

```bash
TASK=experiments/social_navigation/benchmark/depth_track_refinement
bash "$TASK/run.sh" test
bash "$TASK/run.sh" freeze --output "$TASK/generated/my_frozen.json"
bash "$TASK/run.sh" replay --frozen "$TASK/generated/my_frozen.json" --dataset development --output "$TASK/generated/my_development"
bash "$TASK/run.sh" replay --frozen "$TASK/generated/my_frozen.json" --dataset validation --output "$TASK/generated/my_validation"
bash "$TASK/run.sh" evaluate --frozen "$TASK/generated/my_frozen.json" --dataset development --sensor "$TASK/generated/my_development" --output "$TASK/generated/my_development_evaluation"
bash "$TASK/run.sh" evaluate --frozen "$TASK/generated/my_frozen.json" --dataset validation --sensor "$TASK/generated/my_validation" --output "$TASK/generated/my_validation_evaluation"
```

`frozen01.json` lưu hash engine/config/input trước replay hai lượt đầu; `frozen02.json` thêm nguồn mesh-repeat, **engine và config y hệt**. Các bản receipt được lưu Git tại reports/. Mã snapshot baseline byte-identical với stage trước.

Để chuẩn hóa lại lượt mesh bổ sung, chỉ bước đọc MCAP cần ROS Jazzy:

```bash
TASK=experiments/social_navigation/benchmark/depth_track_refinement
COPY="$TASK/generated/my_fixed_source"
bash "$TASK/run.sh" prepare-repeat --output "$COPY"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 experiments/social_navigation/benchmark/portable_episode/generated/episode01/tools/legacy/build.py --episode "$COPY" --output "$COPY/normalized"
python3 "$TASK/extend_freeze.py" --source "$TASK/generated/my_frozen.json" --normalized "$COPY/normalized" --output "$TASK/generated/my_extended.json"
python3 "$TASK/extra_repeat.py" replay --frozen "$TASK/generated/my_extended.json" --output "$TASK/generated/my_mesh_repeat"
python3 "$TASK/extra_repeat.py" evaluate --frozen "$TASK/generated/my_extended.json" --sensor "$TASK/generated/my_mesh_repeat" --output "$TASK/generated/my_mesh_evaluation"
```

Normalizer PASS chỉ là integrity/format, không biến TIMEOUT thành navigation success. Lượt thật dùng `generated/fixed_source/normalized`: 1037 ảnh raw, 802 cửa sổ; giữ toàn bộ lỗi/trạng thái nguồn. Dataset lớn và JSONL từng frame ở generated/, bị Gitignore; summaries/hashes/code/hình được commit.

Audit và preview tái kiểm chứng bộ artifact cụ thể đã lưu:

```bash
bash "$TASK/run.sh" audit --output "$TASK/generated/my_audit"
python3 "$TASK/diagnose_profiles.py" --output "$TASK/generated/my_profile_evidence"
python3 "$TASK/preview.py" --output "$TASK/generated/my_previews"
```

Quyết định: refinement là thử nghiệm riêng, không tự áp dụng trên profile depth khác/không rõ. Các mask semantic, association, H1–H5 và training vẫn false. Bước kế là episode mới với depth mesh và seed/layout khác, có tiêu chí visibility/semantic, trước khi xác nhận person detector hoặc thu batch train.
