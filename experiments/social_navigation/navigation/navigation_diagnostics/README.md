# Ghi navigation diagnostics

`capture.py` vẫn được `navigation/trial.py --navigation-diagnostics` gọi; file giữ nguyên byte. Chế độ này tự bật `--diagnostics`, chỉ thêm subscriber và graph snapshots. Nó không thêm goal, publisher hoặc đổi tham số robot/Nav2.

Từ repository root, source môi trường ROS/Gazebo và dùng một output mới, chạy tuần tự:

```bash
bash experiments/social_navigation/navigation/run.sh trial \
  --backend crowdsimplus --human-policy sfm --goal-layout individual \
  --condition bottleneck --seed 42 --navigation-diagnostics \
  --report-dir /absolute/new/navigation-report
```

Có thể thêm `--localization-profile lf_prob` hoặc `lf_prob_beamskip` cho ablation opt-in; mặc định `baseline`. [Profiles](../localization_trials/README.md) giữ configure/readback riêng; không sửa baseline YAML. `--gui` cần môi trường hiển thị phù hợp, không suy nghiệm thu GUI từ headless logs. Không chạy đồng thời trial khác trong cùng domain.

| Kênh | Payload |
|---|---|
| `/plan`, `/global_costmap/costmap` | Native path; full grid int8 nén zlib/base64 không mất dữ liệu |
| `/particle_cloud` | Native ParticleCloud với pose/weight |
| Navigate/follow-path/spin/backup status, `/rosout` | Status/log nhận được; terminal update có thể thiếu khi teardown |

Graph snapshots lưu publisher/QoS/GID trước goal và shutdown; MessageInfo thiếu trường thì để null, không suy identity trong khoảng giữa. Callback time/count được ghi vì subscriber vẫn có thể ảnh hưởng scheduling.

Analyzer, native clustering helper và validator hardcoded cho hai lượt seed42 cũ đã được gỡ cùng chuỗi cohort localization/stability. Dữ liệu mới vẫn được ghi; HEAD không còn cung cấp analyzer chuyên biệt của cohort cũ.

[Thí nghiệm gốc](https://github.com/CuongBien/DynamuhSim/commit/e8bf1c0c8fed8ed3cb3a53f2f5a9b933b622432d) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/e150d6b8c0d89293dc9d50df2f776e3e697c69b6/experiments/social_navigation/benchmark/navigation_diagnostics) · [Khôi phục report](../../../../docs/REPORT_STORAGE.md).

Tái lập phân tích cũ dùng revision/manifest và raw runtime tương ứng. Không thay outcome lỗi gốc bằng các diagnostic rerun thành công. Xem [Nav2 runtime chung](../../../../docs/SHARED_NAV2_RUNTIME.md) và [audit capture hiện tại](../../../../docs/ANALYSIS_AND_ARTIFACTS.md) cho các pipeline đang dùng.
