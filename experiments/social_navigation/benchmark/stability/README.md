# Stability — cohort lịch sử đã hoàn tất

Cohort `25e4157` gồm ba lượt mỗi policy SFM/ORCA+, cùng fixture seed42. Các runner, analyzer, plot theo ca cố định, reclustering và validator đã được gỡ khỏi HEAD cùng chuỗi phân tích localization không còn consumer.

[Thí nghiệm gốc](https://github.com/CuongBien/DynamuhSim/commit/25e41573a70665af10052161efb39f83500e4269) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/e150d6b8c0d89293dc9d50df2f776e3e697c69b6/experiments/social_navigation/benchmark/stability) · [Khôi phục report](../../../../docs/REPORT_STORAGE.md).

Giữ các outcome, ABORTED/TIMEOUT và giới hạn ban đầu: seed fixture không đồng bộ AMCL/MPPI/physics; mẫu nhỏ không chứng minh failure rate hay xếp hạng an toàn. Circular-mean yaw lift trước native libpf clustering là phép phân tích cloud đã ghi, không khôi phục hidden-filter state.

Để tái lập lịch sử, dùng source revision/manifest và raw inputs của cohort đó. [Navigation capture](../navigation_diagnostics/README.md) vẫn hoạt động cho lượt mới; [runtime chung](../../../../docs/SHARED_NAV2_RUNTIME.md) là đường chạy Nav2 hiện tại.
