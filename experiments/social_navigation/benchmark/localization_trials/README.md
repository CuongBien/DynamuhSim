# Isolated AMCL localization pilot

Chạy tuần tự đúng sáu lượt: baseline SFM/ORCA, LF-prob ORCA/SFM, LF-prob+beam-skip SFM/ORCA. Seed42, mỗi lượt tối đa75 sim s, nghỉ15wall s; dừng cohort nếu hạ tầng/rendering/cleanup sai. Giữ ABORTED và TIMEOUT; không chạy lại để thay kết quả.

```bash
python3 experiments/social_navigation/benchmark/localization_trials/run.py \
  --report-dir experiments/social_navigation/benchmark/localization_trials/reports/my_run
# Chỉ phân tích sau khi cả 6 lượt kết thúc:
python3 experiments/social_navigation/benchmark/localization_trials/analyze.py \
  --report-dir experiments/social_navigation/benchmark/localization_trials/reports/my_run
```

Thực hiện từ repo trong Ubuntu/WSL; runner tự source ROS và workspace qua `benchmark/run.sh`. Chọn thư mục mới, Docker đang chạy và không có phiên experiment khác. Không chạy nhiều simulator đồng thời.

[Đợt run01](reports/run01/LOCALIZATION_PILOT.md) gồm protocol, readback tham số AMCL thực tế trước goal, kết quả điều hướng, pose thật, cloud native, khoảng trống và log shutdown. Các bản ghi lớn không nằm trong Git; `manifest.json` giữ SHA và runtime tương ứng.

Profiles chỉ thay YAML runtime:

| Profile | laser_model_type | do_beamskip | Tham số thêm |
|---|---|---|---|
| baseline | likelihood_field gốc | không override | Không |
| lf_prob | likelihood_field_prob | false | Không |
| lf_prob_beamskip | likelihood_field_prob | true | distance0.5, threshold0.3, error_threshold0.9 |

Initial pose bottleneck8,0,0 được áp dụng như cohort trước. Deep comparison xác nhận phần còn lại như robot controller, costmap và motion model không đổi. Launch gốc vẫn có transform_tolerance override0.5s; readback ghi giá trị thực tế, không tự đổi về YAML1.0s.

Beam skipping chỉ có trong `likelihood_field_prob` và bị native AMCL1.3.13 tắt khi PF chưa converged; cấu hình bật không có nghĩa mọi scan đều được bỏ tia. Giữ ablation LF-prob không skipping để tách thay đổi model khỏi cơ chế bỏ tia. [Nguồn đúng phiên bản](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_amcl/src/sensors/laser/likelihood_field_model_prob.cpp).

Mỗi ô một lượt, không seed đồng bộ AMCL/MPPI/physics và không random hóa đầy đủ thứ tự: kết quả chỉ định hướng thử nghiệm tiếp. Chưa chọn cấu hình thay baseline. Native human policy, robot ROS source và map không bị sửa. Shutdown dùng workaround opt-in [WSL D3D12](../shutdown_fix/README.md).

Kết quả run01: hai lượt LF-prob+beam-skip có sai lệch XY thật cuối0.119m (SFM) và0.140m (ORCA). LF-prob/ORCA báo SUCCEEDED nhưng còn2.597m tới goal; không coi là tới đích chính xác. Sáu lượt không đủ chứng minh cải thiện ổn định. Dấu tròn trong chart: Nav2 thành công và XY thật cuối trong0.20m; dấu X: ABORTED hoặc nằm ngoài ngưỡng đó.

Counts log beam-skip là tổng log cả warmup và đo: SFM1 lần cấp temp/23 lần fallback; ORCA1/25. Không xác định mask ở mỗi scan từ các counts này. Khoảng hở người–người nhỏ nhất âm0.0050m ở SFM+beam-skip (5mm), dù thời gian chồng lấn vượt1cm bằng0; không diễn giải thành không có chồng lấn.

Kiểm chứng sau phân tích:

```bash
python3 experiments/social_navigation/benchmark/localization_trials/validate.py \
  --report-dir experiments/social_navigation/benchmark/localization_trials/reports/run01
```

`validation.json`: source/raw hashes đúng,1009 cloud được đối chiếu, Git index có sẵn và worktree ngoài vùng thử nghiệm giữ nguyên; không còn owned ROS/Gazebo process hay CrowdSimPlus container.
