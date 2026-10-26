# ComplexityNav geometry runtime

`kernel.py` và `roadmap.py` vẫn được worker ComplexityNav hiện tại dùng; giữ nguyên byte. Preflight dependency hiện hành nằm ở `../common_response/preflight.py` và không đổi.

Đây là biến thể thay đổi hành vi rõ ràng: roadmap intention, global disc ORCA và swept geometry filter trên 42 AABBs. Với SFM intention, lớp cuối vẫn là ORCA; không gọi đó là SFM native hoặc suy thành chứng nhận an toàn.

Runner/probe/analyzer của cohort offline `be816c3` đã được gỡ. [Commit gốc](https://github.com/CuongBien/DynamuhSim/commit/be816c3da3a440064da8ef7046d37b03e3d642bb) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/f3e54812c3149df050db339eeb8cdc490614b648/experiments/social_navigation/benchmark/complexitynav_geometry) · [Report lịch sử](../../../../docs/REPORT_STORAGE.md).

Các kết quả gốc 6/6 cảnh offline và các giới hạn seed/fixture giữ nguyên. Tái lập cohort cũ cần revision/manifest và raw inputs của chính cohort đó; không dùng bằng chứng offline để tự nhận đã đo lại Gazebo.

Lượt chạy mới dùng [common inputs/preflight](../common_response/README.md) và [shared Nav2 runtime](../../../../docs/SHARED_NAV2_RUNTIME.md), với image/backend đã pin và native timestep giữ nguyên.
