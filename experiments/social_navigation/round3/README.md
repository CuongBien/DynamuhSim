# ComplexityNav runtime image — round3

Dockerfile, requirements và `.dockerignore` giữ nguyên. `run.sh build` dùng base image round2 nếu có; nếu thiếu thì gọi build round2 trước, cùng hành vi build cũ:

```bash
bash experiments/social_navigation/round3/run.sh build
```

Tag build vẫn là `dynamuhsim-complexitynav:round3`; worker hiện hành dùng content ID đã pin. Build lại tag không tự đổi pin hay chứng minh tái tạo bit-for-bit; xem [shared Nav2](../../../docs/SHARED_NAV2_RUNTIME.md).

Runner scene/matrix, analyzer, checkpoint validator, hai file launcher/config demo CV-MPC và 30 config matrix phát sinh đã được xoá. Các file demo không nằm trong Docker COPY hoặc runtime human adapter. `demo`, `scene`, `matrix` và shell không có mode đều trả usage.

[Thí nghiệm gốc](https://github.com/CuongBien/DynamuhSim/commit/a9397f2e16ef2a81668cbb159aa7376e1c77a08b) · [Code/config trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/ed208d24baab7d685ae1b6a8ac2b4140a73ab40e/experiments/social_navigation/round3) · [Report lịch sử](../../../docs/REPORT_STORAGE.md).

Kết quả 30 lượt, sampling/render limitations và các failure gốc giữ nguyên. Demo CV-MPC đã hoàn tất không phải trained RGL; trajectory clustering không phải group formation/hội thoại. Tái lập dùng revision/manifest/raw inputs cũ. Human backend hiện hành dùng [geometry kernel/roadmap](../benchmark/complexitynav_geometry/README.md).
