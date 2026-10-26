# AMCL profiles và pilot lịch sử

`profiles.py` vẫn được `benchmark/trial.py` dùng để cấu hình YAML runtime và đọc lại tham số AMCL qua service. File này giữ nguyên byte; không đổi baseline YAML hoặc tự chọn profile mới.

| Profile | Override runtime |
|---|---|
| `baseline` | Không override laser model |
| `lf_prob` | `likelihood_field_prob`, beam-skip false |
| `lf_prob_beamskip` | `likelihood_field_prob`, beam-skip true; distance 0.5, threshold 0.3, error threshold 0.9 |

Runner/analyzer/validator của pilot sáu lượt `d0a7bbb` đã được gỡ. `reference.json` giữ pin nguồn của phép phân tích lịch sử; không phải dependency của `profiles.py`.

[Thí nghiệm gốc](https://github.com/CuongBien/DynamuhSim/commit/d0a7bbb2d8c26546b44491d65f0d2be92228e799) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/e150d6b8c0d89293dc9d50df2f776e3e697c69b6/experiments/social_navigation/benchmark/localization_trials) · [Khôi phục report](../../../../docs/REPORT_STORAGE.md).

Giữ nguyên mọi outcome, gồm LF-prob/ORCA báo SUCCEEDED khi XY thật còn 2.597m. Một lượt mỗi ô không chứng minh profile nào tốt hơn. Tái lập dùng revision/manifest đã ghi; xem [navigation capture](../navigation_diagnostics/README.md) cho cách bật profile ở trial mới.
