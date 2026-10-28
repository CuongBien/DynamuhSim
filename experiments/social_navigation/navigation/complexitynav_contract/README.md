# ComplexityNav contract kernel

`kernel.py` vẫn được preflight của `common_response` sao chép thành dependency `native_contract.py` cho backend ComplexityNav. File giữ nguyên byte: world frame, native dt 0.25s, epoch/reset/clock/input validation và native human update contract.

Runner/probe/analyzer của thí nghiệm contract `b0b3c33` đã được gỡ khỏi HEAD. [Commit gốc](https://github.com/CuongBien/DynamuhSim/commit/b0b3c336e9246a6d1e970f72aa6e1ba025f59ad7) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/f3e54812c3149df050db339eeb8cdc490614b648/experiments/social_navigation/benchmark/complexitynav_contract) · [Report lịch sử](../../../../docs/REPORT_STORAGE.md).

Kết quả cũ vẫn giữ: centralized native ORCA/SFM không dùng nội vật cản và hai fixture đi xuyên kệ. Contract đạt không phải geometry đạt; kernel này không điều khiển robot ROS và không có trained RGL. Tái lập phép thử cũ cần source revision/manifest/raw inputs đã ghi.

Backend mới dùng thêm [geometry kernel](../complexitynav_geometry/README.md), [locked inputs/preflight](../common_response/README.md) và [shared Nav2 runtime](../../../../docs/SHARED_NAV2_RUNTIME.md).
