# Chẩn đoán localization — thí nghiệm lịch sử

Bộ phân tích offline năm lượt bottleneck đã hoàn tất tại `1186e1d`. Runner phân tích, raycast/map helper, native score cross-check, công cụ lấy reference/plot và validator của đợt đó đã được gỡ khỏi HEAD; không còn consumer trong pipeline hiện hành.

[Thí nghiệm gốc](https://github.com/CuongBien/DynamuhSim/commit/1186e1db70a1faa03b94cf958c525a6073a65029) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/e150d6b8c0d89293dc9d50df2f776e3e697c69b6/experiments/social_navigation/benchmark/localization_diagnosis) · [Khôi phục report](../../../../docs/REPORT_STORAGE.md).

Các kết quả gốc, gồm 2/4 goal đạt của bốn ca có người, vẫn giữ nguyên. Sensor score là phép chẩn đoán likelihood-field, không phải replay toàn particle filter. Tái lập cần source revision/manifest và raw runtime đã ghi của kết quả lịch sử.

Ghi dữ liệu mới bằng [navigation capture](../navigation_diagnostics/README.md); chạy Nav2/capture hiện tại theo [runtime chung](../../../../docs/SHARED_NAV2_RUNTIME.md) và [capture API](../../../../docs/CAPTURE_RUNTIME.md).
