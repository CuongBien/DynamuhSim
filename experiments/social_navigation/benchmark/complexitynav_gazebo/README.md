# ComplexityNav Gazebo pilot — đã hoàn tất

Build/adapter, worker/launch, live verifier, transport test, analyzer, preflight và shell dispatcher của pilot `c9dffe2` đã được gỡ khỏi HEAD. Runtime Nav2 hiện hành dùng worker/adapter chung, không gọi harness pilot này. `fixture.json` giữ nguyên như đầu vào lịch sử.

[Commit gốc](https://github.com/CuongBien/DynamuhSim/commit/c9dffe25eebd8d9a9b87506dd15d4e01dae6cb04) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/f3e54812c3149df050db339eeb8cdc490614b648/experiments/social_navigation/benchmark/complexitynav_gazebo) · [Report lịch sử](../../../../docs/REPORT_STORAGE.md).

Preflight cũ kiểm hash probe geometry của cùng cohort; nó được gỡ cùng toàn bộ harness phụ thuộc, không nới gate của runtime mới. Giữ nguyên report, source snapshots, các fault/terminal outcomes và giới hạn pilot. Tái lập cần revision/manifest/raw runtime lịch sử.

Để chạy mới dùng [shared Nav2](../../../../docs/SHARED_NAV2_RUNTIME.md), model `complex_geometry_mixed`; kernel/roadmap nằm ở [geometry runtime](../complexitynav_geometry/README.md).
