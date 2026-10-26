# CrowdSimPlus runtime image — round2

Dockerfile, requirements và `.dockerignore` vẫn được giữ nguyên để dựng image dùng bởi adapter/worker hiện tại. `run.sh` chỉ còn chế độ build tường minh:

```bash
bash experiments/social_navigation/round2/run.sh build
```

Tag build vẫn là `dynamuhsim-crowdsimplus:round2`. Các lượt Nav2/IPC đã nghiệm thu dùng image content ID đã pin trong `benchmark/common_response/inputs/images.json`; dựng lại tag không tự thay pin hoặc chứng minh image mới giống byte với image cũ.

Runner scene/matrix, analyzer, validator checkpoint, extractor native demo và 24 config phát sinh từ cohort cũ đã được xoá. `demo`, `scene`, `matrix` và gọi shell không có mode đều trả usage; không tự chạy lại hoặc ghi đè report.

[Thí nghiệm gốc](https://github.com/CuongBien/DynamuhSim/commit/1dbf832950d0d87d8b275dab645b05913fe98da8) · [Code/config trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/ed208d24baab7d685ae1b6a8ac2b4140a73ab40e/experiments/social_navigation/round2) · [Report lịch sử](../../../docs/REPORT_STORAGE.md).

Giữ kết quả 24 lượt và các timeout/giới hạn gốc. Đây là khảo sát 2D DWA, không phải MPPI Mecanum/Gazebo hoặc mô hình nhóm. Tái lập cần revision/manifest và inputs của cohort đó. Lượt hiện hành theo [shared Nav2 runtime](../../../docs/SHARED_NAV2_RUNTIME.md).
