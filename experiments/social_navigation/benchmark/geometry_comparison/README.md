# Đối chứng hình học contact SocNavGym trong Gazebo

Runner ma trận và analyzer của cohort này đã nghỉ dùng. Báo cáo giữ nguyên;
[source và lệnh cũ](https://github.com/CuongBien/DynamuhSim/tree/9833565fa546d55cb0e553f8388bf4a703c59583/experiments/social_navigation/benchmark)
được ghim theo revision lịch sử. Các lệnh `trial`/kiểm định còn dưới đây vẫn được hỗ trợ.

Hai lựa chọn đều dùng native `GroupKernel` và cùng proxy tròn, scene, map, Nav2, LiDAR, robot:

| Lựa chọn | Predicate người | Radius robot trong native | Né robot trong group ORCA | Contact response |
|---|---|---|---|---|
| `native` (mặc định) | Ellipse trục cố định 0,12 × 0,36 m | Nominal 0,25 ± margin reset mặc định | Không | Native dừng nhóm |
| `circle_contact` (opt-in) | Circle radius 0,36 m; circle–circle và circle–oriented-box | Cố định 0,25 m sau reset | Không | Native dừng nhóm |

Biến thể chỉ gắn predicate lên từng instance người của worker thử nghiệm. Không sửa class global, file vendor, plugin C++, robot hoặc config Nav2. Đổi radius sau reset để giữ nguyên số lần lấy mẫu RNG và bố trí ban đầu. `geometry.json` ghi cả radius reset ban đầu và radius hiệu lực; hash scene không chứa policy nên có thể kiểm tra scene giống nhau giữa hai lựa chọn.

Đây là ablation hình học ở adapter, **không phải SocNavGym nguyên bản hay mô hình tránh robot mới**. Contact vẫn kiểm tra trước bước native 0,1 s; không thêm swept guard, dịch pose ra ngoài, clipping, tách người khỏi nhau hoặc điều khiển robot. Vì vậy đổi predicate không bảo đảm hết overlap. Human–human separation và group cohesion vẫn phụ thuộc native. Wall force response cũng giữ nguyên native; chỉ predicate khớp proxy tròn và khối tĩnh.

Radius robot 0,25 m là envelope danh định dùng trong benchmark, không thay collision links hoặc footprint vật lý của Mecanum. Báo cáo clearance robot chính dùng rectangle cấu hình Nav2 0,24×0,20 m; metric padded 0,26×0,22 m và circle danh định được ghi riêng.

## Chạy

Kiểm định hình học tại repository root, dùng output file mới:

```bash
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/benchmark/validate_circle_contact.py \
  --output experiments/social_navigation/benchmark/generated/geometry_validation.json
```

Xem riêng biến thể trên GUI:

```bash
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend socnavgym --geometry circle_contact --condition counterflow --seed 43 \
  --gui --diagnostics \
  --report-dir experiments/social_navigation/benchmark/geometry_comparison/reports/gui01
```

`--geometry` mặc định là `native`. Biến thể yêu cầu `--backend socnavgym` và `--report-dir` riêng; trial từ chối ghi đè kết quả đã có. Ma trận lịch sử tái sử dụng trial protocol hợp lệ trong cùng thư mục, chạy tuần tự và đổi thứ tự variant theo parity seed. Dùng directory mới cho trial; giữ dữ liệu lỗi để điều tra.

Hai phía cùng bật passive diagnostics. Raw captures nằm trong `benchmark/generated/` (ignored); report giữ hash/path. Standing là negative control, khác biệt nhỏ về đường robot không phải bằng chứng khác biệt hành vi xã hội. Seed người chưa cố định đồng thời mẫu MPPI, AMCL và noise LiDAR; hai seed không đủ xếp hạng thống kê.

[Kết quả run01](reports/run01/GEOMETRY_REPORT.md).

## Replay cùng robot input

Phép kiểm tra bổ sung tách biến thiên Nav2 khỏi phản ứng contact: dùng cùng chuỗi pose/vận tốc robot đã ghi, chạy hai worker native/circle_contact. Native phải tái hiện đúng trace đã lưu trước khi so sánh.

```bash
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/benchmark/replay_contact_geometry.py \
  --trial experiments/social_navigation/benchmark/diagnostic/reports/run01/socnavgym_counterflow_s43.json \
  --output experiments/social_navigation/benchmark/geometry_comparison/reports/replay02
```

Đây là replay open loop, không phải một lượt robot điều hướng Gazebo mới. So sánh kín vòng cũng chịu ảnh hưởng thời gian tính toán: predicate circle giải tích nhanh hơn native Shapely, làm RTF và lịch timer khác nhau. Báo cáo ghi cả RTF và worker compute time; không gán toàn bộ thay đổi đường Nav2 cho geometry.
