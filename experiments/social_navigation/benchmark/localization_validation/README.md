# AMCL multiseed — cohort lịch sử đã hoàn tất

Runner/analyzer/validator cho 12 lượt seed42/43/44 tại `a2ef6af` đã được gỡ khỏi HEAD. `fixture_checks.json` giữ nguyên receipt fixture/route clearance; đây là bằng chứng lịch sử, không phải kết quả mới được đánh giá lại.

[Thí nghiệm gốc](https://github.com/CuongBien/DynamuhSim/commit/a2ef6afa14c7775d0214c51f382c3bc770944fff) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/e150d6b8c0d89293dc9d50df2f776e3e697c69b6/experiments/social_navigation/benchmark/localization_validation) · [Khôi phục report](../../../../docs/REPORT_STORAGE.md).

Kết quả gốc vẫn là 5/6 goal thật cho mỗi profile: baseline SFM 2/3, ORCA+ 3/3; LF-prob+beam-skip SFM 3/3, ORCA+ 2/3. Giữ hai failure và các overlap đã đo; không dùng goal success để kết luận không có va chạm hoặc tự đổi baseline.

Tái lập cần source revision/manifest và raw inputs của cohort đó. Các AMCL helper runtime còn dùng nằm ở [profiles](../localization_trials/README.md); xem [navigation capture](../navigation_diagnostics/README.md) cho dữ liệu mới.
