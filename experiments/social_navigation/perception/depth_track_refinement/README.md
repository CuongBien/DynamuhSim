# Cải tiến track depth — so sánh ba lượt ghi

**Cải thiện hình học trên hai lượt depth mesh; không đạt trên dữ liệu proxy cũ. Chưa thay baseline hoặc bật train.** [Báo cáo](../../../archive/social_navigation/reports/depth_track_refinement/REFINEMENT_REPORT.md), [quyết định nghiệm thu](../../../archive/social_navigation/reports/depth_track_refinement/acceptance.json), [audit](../../../archive/social_navigation/reports/depth_track_refinement/audit01/audit.json), [13 test](../../../archive/social_navigation/reports/depth_track_refinement/contract_tests.txt), [hình so sánh dữ liệu thật](../../../archive/social_navigation/reports/depth_track_refinement/profile_comparison.png).

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
TASK=experiments/social_navigation/perception/depth_track_refinement
bash "$TASK/run.sh" test
FROZEN="$TASK/reports/frozen01.json"
bash "$TASK/run.sh" replay --frozen "$FROZEN" --dataset development --output "$TASK/generated/my_development"
bash "$TASK/run.sh" replay --frozen "$FROZEN" --dataset validation --output "$TASK/generated/my_validation"
bash "$TASK/run.sh" evaluate --frozen "$FROZEN" --dataset development --sensor "$TASK/generated/my_development" --output "$TASK/generated/my_development_evaluation"
bash "$TASK/run.sh" evaluate --frozen "$FROZEN" --dataset validation --sensor "$TASK/generated/my_validation" --output "$TASK/generated/my_validation_evaluation"
```

`frozen01.json` lưu hash engine/config/input trước replay hai lượt đầu; `frozen02.json` thêm nguồn mesh-repeat, **engine và config y hệt**. Các bản receipt được lưu Git tại reports/. Mã snapshot baseline byte-identical với stage trước.

Lượt mesh-repeat và các outcome TIMEOUT/FAIL vẫn giữ nguyên. Công cụ copy recording cố định, wrapper extra-repeat, profile diagnostic và preview của cohort đó đã được gỡ khỏi HEAD. `extend_freeze.py` một lần đã gỡ; engine/config/replay/evaluator và auditor còn được giữ; các pin WM/classifier không đổi. Chuỗi lịch sử dùng source revision/manifest và raw inputs đã ghi.

Audit còn dùng:

```bash
bash "$TASK/run.sh" audit --output "$TASK/generated/my_audit"
```

Quyết định: refinement là thử nghiệm riêng, không tự áp dụng trên profile depth khác/không rõ. Các mask semantic, association, H1–H5 và training vẫn false. Bước kế là episode mới với depth mesh và seed/layout khác, có tiêu chí visibility/semantic, trước khi xác nhận person detector hoặc thu batch train.

Cập nhật dọn 27/10/2026: đã gỡ các công cụ chỉ phục vụ cohort/report cũ: `diagnose_profiles.py`, `extra_repeat.py`, `prepare_repeat.py`, `preview.py`. [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/87b0c6eea9b54794db7b4cb794fae57c78f11c70/experiments/social_navigation/benchmark/depth_track_refinement) giữ cách tái lập; report, engine, source freeze và outcome cũ không đổi.

`freeze.py` giữ nguyên vì các consumer hiện hành import verifier từ đây; không còn lệnh freeze trong wrapper. Đợt dữ liệu mới cần protocol/source identity riêng. Source extension lịch sử nằm tại commit `3b133626809a39991babe0e6fcc52d2572c2c03b`; các receipt hiện có không đổi.
