# PBL6 Bước 4 — Scenario coverage V1

| Nhóm | File chính | Scenario | Kiểm tra | Lỗi còn lại |
| --- | --- | --- | --- | --- |
| 4.1 Inspect | src/scenario_generator.py, src/models.py, src/validators.py | S00–S04 | Đã đối chiếu schema, template, graph, zone, HuNav và Runner | Không |
| 4.2 Individual | src/individual_temporal.py, templates/S05–S09 | S05–S09 | 20 seed × 3 density/family PASS | Không |
| 4.3 Temporal + geometry | src/individual_temporal.py, templates/S10–S15, human_sampler.py, bridge | S10–S15 | 20 seed × 3 density/family PASS; S10 runtime PAUSE/RESUME | Không |
| 4.4 Group | src/group_mixed.py, templates/S16–S19 | S16–S19 | 20 seed × 3 density/family PASS | Không |
| 4.5 Mixed traffic | src/group_mixed.py, templates/S20–S23 | S20–S23 | 20 seed × 3 density/family PASS | Không |
| 4.6 Validator + deterministic | src/validators.py, tests/test_coverage_v1.py | S05–S23 | 1.140 mẫu PASS; 13 generator tests PASS | Không |
| 4.7 Runtime | runtime_smoke_v1.py, runtime_smoke_v1.csv | S00–S23 + 7 family đại diện ở medium/high | 38/38 episode riêng biệt qua Runner; 41 lượt, 40 PASS, 1 FAIL ban đầu rồi retry PASS | Một lần AMCL/scan-time TF khởi động không kịp ở S15 |
| 4.8 Coverage | scenario_coverage.py, scenario_coverage.csv/yaml, generated_v1/ | S00–S23 | 72/72 artifact PASS; mỗi family có low/medium/high | Không |

Timeout trong smoke là phân loại hợp lệ do giới hạn quan sát 3 giây sau khi Nav2 nhận goal. Hạ tầng được tính PASS khi loader, HuNav, Gazebo, Nav2 và cleanup hoàn tất. Lần đầu S15 lỗi AMCL/scan-time TF trong lúc khởi động; chạy lại cùng episode PASS. Báo cáo YAML giữ runtime_failed_attempts = 1 để thể hiện lượt lỗi đó.

S10 chạy riêng với thời gian quan sát 15 giây: bridge ghi PAUSE event_agent_1 elapsed=2.33 và RESUME event_agent_1 elapsed=8.93.

Bộ test dataset_runner: 75/75 PASS. Không thay đổi mã Runner/Logger/Builder. Checkout hiện tại không có module hoặc test Logger/Builder của Bước 3B, nên chưa thể xác nhận regression riêng cho 3B.

## Lệnh tái hiện

Từ thư mục demo_1:

    python3 -m unittest discover -s scenario_generator/tests -v
    python3 -m unittest discover -s dataset_runner/tests -v
    python3 scenario_generator/scenario_generator.py --scenario all --num-episodes 24 --seed-start 42000 --density low --output /tmp/pbl6-v1-check
    python3 scenario_generator/scenario_coverage.py --input scenario_generator/generated_v1 --output scenario_generator --runtime-results scenario_generator/runtime_smoke_v1.csv

Lệnh runtime và seed của batch đại diện có trong README.md.
