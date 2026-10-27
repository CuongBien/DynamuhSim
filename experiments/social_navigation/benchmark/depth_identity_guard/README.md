# Causal identity guard — regression và seed44

**Đã chặn case đổi pairing GT5→GT6 trên seed43, giữ đủ paired rows và không tăng phân mảnh trên regression. Seed44 mới không suy giảm. Chưa nghiệm thu person detector/train hoặc navigation.** [Báo cáo](../../../archive/social_navigation/reports/depth_identity_guard/IDENTITY_GUARD_REPORT.md), [acceptance](../../../archive/social_navigation/reports/depth_identity_guard/acceptance.json), [audit độc lập](../../../archive/social_navigation/reports/depth_identity_guard/audit01/audit.json), [14 behavior tests](test_guard.py), [trace case thật](../../../archive/social_navigation/reports/depth_identity_guard/regression_identity_trace.png), [ảnh đối chiếu](../../../archive/social_navigation/reports/depth_identity_guard/identity_comparison.png).

Bản cuối là candidate05; candidate01–04 chưa đạt được giữ đủ code/hash/kết quả ở reports/candidates/. Không sửa threshold sau khi xem seed44. Bộ lọc hình dạng vẫn là prior thử nghiệm trên depth mesh; lỗi với proxy cũ giữ nguyên. Tracker vẫn chạy offline, không điều khiển robot hoặc thay default ROS.

## Cách chạy

Offline cần Python 3.12 + NumPy/SciPy. Wrapper giới hạn BLAS/OMP/MKL một luồng. Dùng output mới; không ghi đè dữ liệu cũ.

```bash
TASK=experiments/social_navigation/benchmark/depth_identity_guard
bash "$TASK/run.sh" test
bash "$TASK/run.sh" replay --frozen "$TASK/reports/final_engine/frozen.json" --dataset regression43 --output "$TASK/generated/my_sensor"
bash "$TASK/run.sh" evaluate --frozen "$TASK/reports/final_engine/frozen.json" --dataset regression43 --sensor "$TASK/generated/my_sensor" --output "$TASK/generated/my_evaluation"
```

Nguồn regression: mesh42, repeat42, regression43. proxy_stress dùng kiểm giới hạn profile, không chứng nhận semantic. Seed43 đã trở thành dữ liệu phát triển của bản mới, không gọi lại là held-out.

`seal.py` chỉ chốt candidate sau khi ba gate regression đạt; ghi source/config/gate hashes. `capture44.py` chỉ chạy một lần khi sealed source còn nguyên và preflight Nvidia/native friction đạt. Thay đổi driver chỉ gồm vị trí runtime và cho phép input seed44; camera/world/Nav2/native worker giữ nguyên.

```bash
# Các lệnh này đã chạy. Tên cố định sẽ từ chối ghi đè lần thu hiện có.
bash "$TASK/run.sh" seal --candidate "$TASK/generated/candidate05.json" --tag 05
bash "$TASK/run.sh" preflight
bash "$TASK/run.sh" capture44
```

Capture/normalize/raw-audit dùng ROS 2 Jazzy + overlay hiện có. Ví dụ đọc lại raw đã đóng vào output mới:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
EPISODE="$TASK/generated/runs/identity_seed44_01"
python3 experiments/social_navigation/benchmark/portable_episode/generated/episode01/tools/legacy/build.py --episode "$EPISODE" --output "$EPISODE/my_normalized"
```

Bộ đã đo dùng normalized/ trong episode, generated/final_with_seed44.json, generated/seed44_44/ và generated/seed44_eval44/. `holdout.py` thêm source mới mà không đổi engine/gate. Đọc lại phép so sánh vào output mới:

```bash
bash "$TASK/run.sh" replay --frozen "$TASK/reports/final_with_seed44.json" --dataset seed44 --output "$TASK/generated/my_seed44_sensor"
bash "$TASK/run.sh" evaluate --frozen "$TASK/reports/final_with_seed44.json" --dataset seed44 --sensor "$TASK/generated/my_seed44_sensor" --output "$TASK/generated/my_seed44_evaluation"
bash "$TASK/run.sh" audit --output "$TASK/generated/my_audit"
```

Audit dùng các path artifact đã đo của stage này. Báo cáo nhỏ, code và media được commit; raw MCAP, NPY, JSONL lớn ở generated bị Gitignore và vẫn chỉ nằm local.

## Schema quan sát chưa rõ ID

Mỗi cụm depth có một proposal_index. Nếu ghép mơ hồ: track_id=−1, confirmed=false, identity_observed=false, velocity_valid=false. Vị trí đo vẫn được lưu, không xoá detection hoặc nội suy/backfill từ tương lai. Nhiều proposal có thể cùng ID−1; chỉ các ID≥0 mới là track riêng. Model/loader sau này phải dùng mask, không coi ID−1 là một người chung.

Không nhầm identity_observed/confirmed với semantic person hoặc association GT dùng train. Các mask semantic/association/H1–H5/training vẫn false. GT chỉ ở evaluator/plot. Mốc nghiệm thu identity-safety của stage này yêu cầu giữ ≥95% paired rows so với parent, zero short-gap GT-pair changes, không tăng fragmentation và không có known-wrong dormant re-ID. **Gate đầy đủ cũ yêu cầu giảm fragmentation vẫn báo riêng**, không đổi FAIL thành PASS.

Trên seed44, cả parent và guard đều không đổi pairing và có cùng fragmentation 7, paired rows 2587: kiểm không suy giảm, chưa chứng minh guard cải thiện trên seed này. Navigation nguồn vẫn TIMEOUT, goal còn 4.012 m. Bước kế cần kiểm nhãn visibility/person instance và semantic association trước batch/train; giữ HuNavSim test cuối. Lỗi điều hướng là việc riêng chưa sửa trong lượt này.

Cập nhật dọn 27/10/2026: đã gỡ các công cụ chỉ phục vụ cohort/report cũ: `diagnose_case.py`, `finalize_reports.py`, `preview.py`. [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/87b0c6eea9b54794db7b4cb794fae57c78f11c70/experiments/social_navigation/benchmark/depth_identity_guard) giữ cách tái lập; report, engine, source freeze và outcome cũ không đổi.
