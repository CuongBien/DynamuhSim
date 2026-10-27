# Nhóm người ORCA nhận biết robot — adapter thử nghiệm

Runner ma trận và analyzer của cohort này đã nghỉ dùng. Báo cáo giữ nguyên;
[source và lệnh cũ](https://github.com/CuongBien/DynamuhSim/tree/9833565fa546d55cb0e553f8388bf4a703c59583/experiments/social_navigation/benchmark)
được ghim theo revision lịch sử. Các lệnh `trial`/kiểm định còn dưới đây vẫn được hỗ trợ.

`--robot-avoidance` là lựa chọn opt-in trên SocNavGym adapter. Cả đối chứng và biến thể đều dùng `--geometry circle_contact`; đây là thay đổi hành vi sau khi đã chuẩn hóa contact, không phải bản SocNavGym gốc.

Biến thể giữ nguyên thiết lập group ORCA native, thêm một agent robot với vị trí và vận tốc world thực đo từ Gazebo ở mỗi tick. Nó chỉ lấy vận tốc nhóm từ solver. Vận tốc robot do solver đề xuất bị bỏ qua; Nav2 vẫn là bộ điều khiển robot duy nhất. Native smoothing, giới hạn quay của người, nhiễu, waypoint, collision response và robot/LiDAR không đổi.

Phạm vi hiện tại: đúng một nhóm đang đi, không có người độc lập hoặc nhóm khác. Runtime từ chối population ngoài phạm vi thay vì âm thầm bỏ qua. Cảnh standing giữ nguyên native để làm negative control; chưa thêm người đang trò chuyện nhường đường.

## Chạy

Từ repository root, dùng output directory mới cho mỗi lần:

```bash
# Kiểm chứng solver: khi ở xa phải khớp native, khi ở gần phải phản ứng trước contact
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/benchmark/validate_robot_orca.py \
  --output experiments/social_navigation/benchmark/avoidance_comparison/reports/run02/solver_validation.json

# Replay cùng robot input với native, circle_contact và robot_orca
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/benchmark/replay_contact_geometry.py \
  --trial experiments/social_navigation/benchmark/diagnostic/reports/run01/socnavgym_counterflow_s43.json \
  --include-robot-orca \
  --output experiments/social_navigation/benchmark/avoidance_comparison/reports/run02
```

Ma trận lịch sử: counterflow seed 42/43 cho hai policy, standing seed 43 cho hai policy. Native/circle_contact không bị ghi đè. Runner cũ có thể tiếp tục từ trial hợp lệ trong cùng thư mục; kết quả lỗi được giữ để điều tra, không ghi đè bằng lượt khác.

Xem GUI riêng:

```bash
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend socnavgym --geometry circle_contact --robot-avoidance \
  --condition counterflow --seed 43 --gui --diagnostics \
  --report-dir experiments/social_navigation/benchmark/avoidance_comparison/reports/gui01
```

Bỏ `--robot-avoidance` để xem đối chứng cùng geometry. Không chạy hai trial trong cùng ROS domain đồng thời. World base không có actor controller khác điều khiển cùng người.

## Cách diễn giải

Đây là ORCA reciprocal với robot agent nhưng không áp dụng nghiệm robot; không có bảo đảm từ giả định reciprocal trong triển khai này. Agent nhóm vẫn là một circle lớn native radius 1,44 m, không phải ORCA riêng cho mỗi thành viên. Current velocity của agent nhóm vẫn reset 0 theo native; smoothing 0,9, giới hạn quay và nhiễu ở tầng người có thể làm pose thực khác nghiệm agent.

Người nhận robot ground truth từ simulator; robot vẫn dùng cảm biến của nó. Chưa có swept guard, sửa pose, pair separation, social-force mới hoặc huấn luyện. Hai seed chưa đủ đánh giá an toàn thống kê hay tính tự nhiên trong đám đông. Xem clearance footprint, human-pair overlap, factory clearance, cohort radius và ảnh/GIF cùng goal success.

[Kết quả run01](reports/run01/AVOIDANCE_REPORT.md).
